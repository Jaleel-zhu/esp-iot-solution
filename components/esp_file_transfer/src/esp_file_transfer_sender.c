/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_file_transfer_internal.h"

#include <limits.h>
#include <stdlib.h>
#include <string.h>

static void sender_fail(ft_instance_t *instance, uint16_t reason, esp_err_t detail,
                        bool notify_peer)
{
    ft_abort_active(instance, reason, detail, notify_peer, false);
}

static void sender_progress(ft_instance_t *instance)
{
    ft_context_t *ctx = instance->active_ctx;
    if (!ctx) {
        return;
    }
    uint8_t percent = ctx->file_size == 0 ? 100 :
                      (uint8_t)((ctx->bytes_transferred * 100) / ctx->file_size);
    if (!ctx->progress_reported || percent == 100 ||
            percent >= (uint8_t)(ctx->last_reported_percent + 5)) {
        ctx->progress_reported = true;
        ctx->last_reported_percent = percent;
        ft_emit_event(instance, ESP_FT_EVENT_PROGRESS);
    }
}

static void handle_final(ft_instance_t *instance, const ft_final_confirm_t *final)
{
    ft_context_t *ctx = instance->active_ctx;
    if (!ctx || final->transfer_id != ctx->transfer_id) {
        return;
    }
    ft_timer_disarm(instance);
    if (final->status != ESP_FT_WIRE_STATUS_OK) {
        sender_fail(instance, final->reason_code, ESP_FAIL, false);
        return;
    }
    if (final->received_size != ctx->file_size) {
        sender_fail(instance, ESP_FT_REASON_INVALID_MESSAGE, ESP_ERR_INVALID_SIZE, true);
        return;
    }
    if (memcmp(final->received_sha256, ctx->expected_sha256, 32) != 0) {
        sender_fail(instance, ESP_FT_REASON_HASH_MISMATCH, ESP_ERR_INVALID_CRC, true);
        return;
    }
    memcpy(ctx->saved_name, final->saved_name, sizeof(ctx->saved_name));
    if (!ctx->progress_reported || ctx->last_reported_percent != 100) {
        ctx->bytes_transferred = ctx->file_size;
        sender_progress(instance);
    }
    ft_finish_transfer(instance, ESP_FT_EVENT_COMPLETED, ESP_FT_REASON_OK, ESP_OK);
}

static void use_early_final(ft_instance_t *instance)
{
    ft_context_t *ctx = instance->active_ctx;
    if (!ctx || !ctx->early_final_valid) {
        return;
    }
    ft_final_confirm_t final = ctx->early_final;
    ctx->early_final_valid = false;
    handle_final(instance, &final);
}

static void send_next_block(ft_instance_t *instance)
{
    ft_context_t *ctx = instance->active_ctx;
    if (!ctx || ctx->current_block >= ctx->total_blocks) {
        return;
    }
    uint64_t remaining = ctx->file_size - ctx->bytes_transferred;
    size_t expected = remaining > ctx->block_size ? ctx->block_size : (size_t)remaining;
    uint8_t *data = ctx->block_buffer + ESP_FT_DATA_REQ_HDR_LEN;
    size_t count = fread(data, 1, expected, ctx->file);
    if (count != expected) {
        sender_fail(instance, ESP_FT_REASON_FILE_READ_FAILED,
                    ferror(ctx->file) ? ESP_FAIL : ESP_ERR_INVALID_SIZE, true);
        return;
    }
    esp_err_t err = ft_data_send_request(instance, ctx, ctx->current_block, count);
    if (err != ESP_OK) {
        sender_fail(instance, ESP_FT_REASON_DATA_SEND_FAILED, err, true);
        return;
    }
    ctx->state = TRANSFER_STATE_SENDING_DATA;
    ft_snapshot_update(instance);
    esp_err_t timer_err = ft_timer_arm(instance, FT_TIMER_BLOCK_RSP,
                                       ESP_FT_BLOCK_RSP_TIMEOUT_US);
    if (timer_err != ESP_OK) {
        sender_fail(instance, ESP_FT_REASON_INTERNAL_ERROR, timer_err, true);
    }
}

esp_err_t ft_sender_start(ft_instance_t *instance, const char *src_path, const char *remote_name)
{
    if (!instance || !src_path) {
        return ESP_ERR_INVALID_ARG;
    }
    if (instance->active_ctx) {
        return ESP_FT_ERR_BUSY;
    }
    esp_err_t err = ft_refresh_capabilities(instance);
    if (err != ESP_OK) {
        return err;
    }
    const char *file_name = remote_name ? remote_name : ft_fs_basename(src_path);
    if (!ft_fs_name_valid(file_name)) {
        return ESP_FT_ERR_REMOTE_NAME_INVALID;
    }
    uint64_t file_size;
    err = ft_fs_source_info(src_path, &file_size);
    if (err != ESP_OK) {
        return err;
    }
    if (file_size > instance->max_file_size) {
        return ESP_FT_ERR_FILE_TOO_LARGE;
    }
    uint64_t blocks = file_size == 0 ? 0 : 1 + (file_size - 1) / instance->block_size;
    if (blocks > UINT32_MAX) {
        return ESP_FT_ERR_FILE_TOO_LARGE;
    }

    FILE *file = NULL;
    err = ft_fs_open_source(src_path, &file);
    if (err != ESP_OK) {
        return err;
    }
    ft_context_t *ctx = calloc(1, sizeof(*ctx));
    if (!ctx) {
        fclose(file);
        return ESP_ERR_NO_MEM;
    }
    ctx->source_path = strdup(src_path);
    ctx->block_buffer = malloc(instance->block_size + ESP_FT_DATA_REQ_HDR_LEN);
    if (!ctx->source_path || !ctx->block_buffer) {
        free(ctx->source_path);
        free(ctx->block_buffer);
        free(ctx);
        fclose(file);
        return ESP_ERR_NO_MEM;
    }
    err = ft_hash_create(&ctx->hash);
    if (err != ESP_OK) {
        free(ctx->source_path);
        free(ctx->block_buffer);
        free(ctx);
        fclose(file);
        return err;
    }

    ctx->role = ESP_FILE_TRANSFER_ROLE_SENDER;
    ctx->state = TRANSFER_STATE_PREPARING;
    ctx->link = instance->gmp_link;
    ctx->transfer_id = ft_next_transfer_id(instance);
    ctx->file_size = file_size;
    ctx->block_size = instance->block_size;
    ctx->total_blocks = (uint32_t)blocks;
    ctx->file = file;
    ctx->block_buffer_size = instance->block_size;
    strcpy(ctx->file_name, file_name);
    instance->active_ctx = ctx;
    ft_snapshot_update(instance);
    ft_emit_event(instance, ESP_FT_EVENT_STARTED);
    return ESP_OK;
}

void ft_sender_prepare(ft_instance_t *instance)
{
    ft_context_t *ctx = instance ? instance->active_ctx : NULL;
    if (!ctx || ctx->role != ESP_FILE_TRANSFER_ROLE_SENDER ||
            ctx->state != TRANSFER_STATE_PREPARING) {
        return;
    }
    while (!feof(ctx->file)) {
        if (atomic_load(&instance->user_abort_pending) ||
                atomic_load(&instance->link_down_pending) ||
                atomic_load(&instance->transport_error_pending)) {
            return;
        }
        uint8_t *data = ctx->block_buffer + ESP_FT_DATA_REQ_HDR_LEN;
        size_t count = fread(data, 1, ctx->block_buffer_size, ctx->file);
        if (count && ft_hash_update(ctx->hash, data, count) != ESP_OK) {
            sender_fail(instance, ESP_FT_REASON_FILE_READ_FAILED, ESP_FAIL, false);
            return;
        }
        if (count < ctx->block_buffer_size && ferror(ctx->file)) {
            sender_fail(instance, ESP_FT_REASON_FILE_READ_FAILED, ESP_FAIL, false);
            return;
        }
    }
    if (ft_hash_finish(ctx->hash, ctx->expected_sha256) != ESP_OK) {
        sender_fail(instance, ESP_FT_REASON_FILE_READ_FAILED, ESP_FAIL, false);
        return;
    }
    uint64_t current_size;
    if (ft_fs_source_info(ctx->source_path, &current_size) != ESP_OK ||
            current_size != ctx->file_size || fseek(ctx->file, 0, SEEK_SET) != 0) {
        sender_fail(instance, ESP_FT_REASON_FILE_READ_FAILED, ESP_ERR_INVALID_SIZE, false);
        return;
    }

    ft_meta_request_t metadata = {
        .transfer_id = ctx->transfer_id,
        .file_size = ctx->file_size,
        .block_size = ctx->block_size,
        .total_blocks = ctx->total_blocks,
    };
    memcpy(metadata.sha256, ctx->expected_sha256, 32);
    strcpy(metadata.file_name, ctx->file_name);
    uint8_t payload[ESP_FT_META_REQ_MAX_LEN];
    size_t payload_len;
    esp_err_t err = ft_protocol_encode_meta_request(&metadata, payload, sizeof(payload),
                                                    &payload_len);
    if (err != ESP_OK) {
        sender_fail(instance, ESP_FT_REASON_DATA_SEND_FAILED, err, false);
        return;
    }
    if (ft_process_pending_termination(instance)) {
        return;
    }
    err = ft_gmp_send_request(instance, ctx, ESP_FT_CMD_TRANSFER_META,
                              payload, payload_len, true);
    if (err != ESP_OK) {
        sender_fail(instance, ESP_FT_REASON_DATA_SEND_FAILED, err, false);
        return;
    }
    ctx->metadata_sent = true;
    ctx->state = TRANSFER_STATE_WAIT_META_RSP;
    ft_snapshot_update(instance);
    ft_emit_event(instance, ESP_FT_EVENT_META_SENT);
    err = ft_timer_arm(instance, FT_TIMER_META_RSP, ESP_FT_META_RSP_TIMEOUT_US);
    if (err != ESP_OK) {
        sender_fail(instance, ESP_FT_REASON_INTERNAL_ERROR, err, true);
    }
}

static void handle_meta_response(ft_instance_t *instance, const ft_gmp_packet_t *packet)
{
    ft_context_t *ctx = instance->active_ctx;
    if (!ft_gmp_pending_matches(ctx, packet, ESP_FT_CMD_TRANSFER_META)) {
        return;
    }
    ft_timer_disarm(instance);
    ctx->pending_valid = false;
    if (packet->status != ESP_GMP_STATUS_OK) {
        uint16_t reason = packet->status == ESP_GMP_STATUS_NOT_SUPPORTED
                          ? ESP_FT_REASON_CAPABILITY_MISMATCH
                          : ESP_FT_REASON_DATA_SEND_FAILED;
        sender_fail(instance, reason, ESP_FAIL, true);
        return;
    }
    ft_meta_response_t response;
    if (ft_protocol_decode_meta_response(packet->payload, packet->payload_len, &response) != ESP_OK ||
            response.transfer_id != ctx->transfer_id) {
        sender_fail(instance, ESP_FT_REASON_INVALID_MESSAGE, ESP_ERR_INVALID_RESPONSE, true);
        return;
    }
    if (response.status == ESP_FT_WIRE_STATUS_REJECTED) {
        ctx->reason_code = response.reason_code;
        ft_emit_event(instance, ESP_FT_EVENT_PEER_REJECTED);
        sender_fail(instance, response.reason_code, ESP_FAIL, false);
        return;
    }
    if (response.accepted_block_size != ctx->block_size) {
        sender_fail(instance, ESP_FT_REASON_CAPABILITY_MISMATCH,
                    ESP_ERR_INVALID_RESPONSE, true);
        return;
    }
    ft_emit_event(instance, ESP_FT_EVENT_PEER_ACCEPTED);
    if (ctx->total_blocks == 0) {
        ctx->state = TRANSFER_STATE_WAIT_FINAL_CONFIRM;
        ft_snapshot_update(instance);
        if (ctx->early_final_valid) {
            use_early_final(instance);
        } else {
            esp_err_t err = ft_timer_arm(instance, FT_TIMER_FINAL, ESP_FT_FINAL_TIMEOUT_US);
            if (err != ESP_OK) {
                sender_fail(instance, ESP_FT_REASON_INTERNAL_ERROR, err, true);
            }
        }
    } else {
        send_next_block(instance);
    }
}

static void handle_data_response(ft_instance_t *instance, const ft_gmp_packet_t *packet)
{
    ft_context_t *ctx = instance->active_ctx;
    if (!ft_gmp_pending_matches(ctx, packet, ESP_FT_CMD_DATA_BLOCK)) {
        return;
    }
    ft_timer_disarm(instance);
    ctx->pending_valid = false;
    if (packet->status != ESP_GMP_STATUS_OK) {
        sender_fail(instance, ESP_FT_REASON_DATA_SEND_FAILED, ESP_FAIL, true);
        return;
    }
    ft_data_response_t response;
    if (ft_protocol_decode_data_response(packet->payload, packet->payload_len, &response) != ESP_OK ||
            response.transfer_id != ctx->transfer_id ||
            response.block_index != ctx->current_block) {
        sender_fail(instance, ESP_FT_REASON_INVALID_MESSAGE, ESP_ERR_INVALID_RESPONSE, true);
        return;
    }
    if (response.status != ESP_FT_WIRE_STATUS_OK) {
        sender_fail(instance, response.reason_code, ESP_FAIL, true);
        return;
    }
    uint64_t remaining = ctx->file_size - ctx->bytes_transferred;
    ctx->bytes_transferred += remaining > ctx->block_size ? ctx->block_size : remaining;
    ++ctx->current_block;
    sender_progress(instance);
    if (ctx->current_block == ctx->total_blocks) {
        ctx->state = TRANSFER_STATE_WAIT_FINAL_CONFIRM;
        ft_snapshot_update(instance);
        if (ctx->early_final_valid) {
            use_early_final(instance);
        } else {
            esp_err_t err = ft_timer_arm(instance, FT_TIMER_FINAL, ESP_FT_FINAL_TIMEOUT_US);
            if (err != ESP_OK) {
                sender_fail(instance, ESP_FT_REASON_INTERNAL_ERROR, err, true);
            }
        }
    } else {
        send_next_block(instance);
    }
}

static void handle_final_packet(ft_instance_t *instance, const ft_gmp_packet_t *packet)
{
    ft_context_t *ctx = instance->active_ctx;
    if (!ctx) {
        return;
    }
    if (packet->payload_len < sizeof(uint32_t)) {
        return;
    }
    uint32_t transfer_id = ((uint32_t)packet->payload[0] << 24) |
                           ((uint32_t)packet->payload[1] << 16) |
                           ((uint32_t)packet->payload[2] << 8) |
                           packet->payload[3];
    if (transfer_id != ctx->transfer_id) {
        return;
    }
    if (packet->status != ESP_GMP_STATUS_OK) {
        sender_fail(instance, ESP_FT_REASON_INVALID_MESSAGE, ESP_ERR_INVALID_RESPONSE, true);
        return;
    }
    ft_final_confirm_t final;
    if (ft_protocol_decode_final_confirm(packet->payload, packet->payload_len, &final) != ESP_OK ||
            final.transfer_id != ctx->transfer_id) {
        sender_fail(instance, ESP_FT_REASON_INVALID_MESSAGE, ESP_ERR_INVALID_RESPONSE, true);
        return;
    }
    if (ctx->state == TRANSFER_STATE_WAIT_FINAL_CONFIRM && !ctx->pending_valid) {
        handle_final(instance, &final);
    } else if ((ctx->state == TRANSFER_STATE_WAIT_META_RSP && ctx->file_size == 0) ||
               (ctx->state == TRANSFER_STATE_SENDING_DATA &&
                ctx->current_block + 1 == ctx->total_blocks)) {
        if (!ctx->early_final_valid) {
            ctx->early_final = final;
            ctx->early_final_valid = true;
        }
    } else if (!ctx->early_final_valid) {
        sender_fail(instance, ESP_FT_REASON_INVALID_MESSAGE, ESP_ERR_INVALID_STATE, true);
    }
}

void ft_sender_handle_packet(ft_instance_t *instance, const ft_gmp_packet_t *packet)
{
    if (!instance || !instance->active_ctx || !packet) {
        return;
    }
    if (packet->command_id == ESP_FT_CMD_TRANSFER_META &&
            packet->op == ESP_GMP_OP_WRITE_RSP) {
        handle_meta_response(instance, packet);
    } else if (packet->command_id == ESP_FT_CMD_DATA_BLOCK &&
               packet->op == ESP_GMP_OP_WRITE_RSP) {
        handle_data_response(instance, packet);
    } else if (packet->command_id == ESP_FT_CMD_FINAL_CONFIRM &&
               packet->op == ESP_GMP_OP_WRITE_REQ) {
        handle_final_packet(instance, packet);
    }
}

void ft_sender_handle_timeout(ft_instance_t *instance, ft_timer_kind_t kind)
{
    ft_context_t *ctx = instance ? instance->active_ctx : NULL;
    if (!ctx || ctx->role != ESP_FILE_TRANSFER_ROLE_SENDER) {
        return;
    }
    bool expected = (kind == FT_TIMER_META_RSP && ctx->state == TRANSFER_STATE_WAIT_META_RSP) ||
                    (kind == FT_TIMER_BLOCK_RSP && ctx->state == TRANSFER_STATE_SENDING_DATA) ||
                    (kind == FT_TIMER_FINAL && ctx->state == TRANSFER_STATE_WAIT_FINAL_CONFIRM);
    if (expected) {
        sender_fail(instance, ESP_FT_REASON_TIMEOUT, ESP_ERR_TIMEOUT, true);
    }
}

void ft_handle_peer_abort(ft_instance_t *instance, const ft_gmp_packet_t *packet)
{
    if (!instance || !instance->active_ctx || !packet) {
        return;
    }
    ft_abort_t abort_msg;
    if (ft_protocol_decode_abort(packet->payload, packet->payload_len, &abort_msg) != ESP_OK ||
            abort_msg.transfer_id != instance->active_ctx->transfer_id) {
        return;
    }
    ft_abort_active(instance, abort_msg.reason_code, ESP_OK, false, true);
}
