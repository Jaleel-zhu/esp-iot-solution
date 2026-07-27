/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_file_transfer_internal.h"

#include <stdlib.h>
#include <string.h>

static ft_instance_t s_ft;

ft_instance_t *ft_instance_get(void)
{
    return &s_ft;
}

esp_err_t ft_refresh_capabilities(ft_instance_t *instance)
{
    if (!instance || !instance->gmp_link) {
        return ESP_ERR_INVALID_STATE;
    }
    size_t effective = esp_gmp_max_payload_effective(instance->gmp_link);
    if (effective < ESP_FT_MIN_EFFECTIVE_PAYLOAD) {
        return ESP_FT_ERR_CAPABILITY_MISMATCH;
    }
    size_t block_limit = effective - ESP_FT_DATA_REQ_HDR_LEN;
    if (block_limit > ESP_FT_DEFAULT_MAX_BLOCK_SIZE) {
        block_limit = ESP_FT_DEFAULT_MAX_BLOCK_SIZE;
    }
    size_t block_size = instance->configured_block_size
                        ? instance->configured_block_size
                        : block_limit;
    if (block_size == 0 || block_size > UINT32_MAX ||
            block_size > block_limit) {
        return ESP_FT_ERR_CAPABILITY_MISMATCH;
    }
    instance->effective_payload = effective;
    instance->block_size = block_size;
    return ESP_OK;
}

bool ft_async_enter(ft_instance_t *instance)
{
    if (!instance || !atomic_load(&instance->accepting_events)) {
        return false;
    }
    atomic_fetch_add(&instance->hook_ref_count, 1);
    if (!atomic_load(&instance->accepting_events)) {
        atomic_fetch_sub(&instance->hook_ref_count, 1);
        return false;
    }
    return true;
}

void ft_async_exit(ft_instance_t *instance)
{
    atomic_fetch_sub(&instance->hook_ref_count, 1);
}

void ft_worker_wake(ft_instance_t *instance)
{
    if (instance && instance->worker_task) {
        xTaskNotifyGive(instance->worker_task);
    }
}

uint16_t ft_next_sequence(ft_instance_t *instance)
{
    uint16_t value = instance->next_sequence++;
    if (value == 0) {
        value = instance->next_sequence++;
    }
    if (instance->next_sequence == 0) {
        instance->next_sequence = 1;
    }
    return value;
}

uint32_t ft_next_transfer_id(ft_instance_t *instance)
{
    uint32_t value = instance->next_transfer_id++;
    if (value == 0) {
        value = instance->next_transfer_id++;
    }
    if (instance->next_transfer_id == 0) {
        instance->next_transfer_id = 1;
    }
    return value;
}

static void timeout_callback(void *arg)
{
    ft_instance_t *instance = arg;
    if (!ft_async_enter(instance)) {
        return;
    }
    ft_worker_event_t event = {
        .type = FT_WORK_TIMEOUT,
    };
    portENTER_CRITICAL(&instance->timeout_lock);
    event.data.timeout = instance->timeout_tuple;
    portEXIT_CRITICAL(&instance->timeout_lock);
    if (event.data.timeout.kind == FT_TIMER_NONE) {
        ft_async_exit(instance);
        return;
    }
    if (xQueueSend(instance->event_queue, &event, 0) != pdTRUE) {
        portENTER_CRITICAL(&instance->timeout_lock);
        instance->timeout_pending_tuple = event.data.timeout;
        portEXIT_CRITICAL(&instance->timeout_lock);
        atomic_store(&instance->timeout_pending, true);
    }
    ft_worker_wake(instance);
    ft_async_exit(instance);
}

esp_err_t ft_timer_arm(ft_instance_t *instance, ft_timer_kind_t kind, uint64_t timeout_us)
{
    if (!instance || !instance->timer || kind == FT_TIMER_NONE || !instance->active_ctx) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_timer_stop(instance->timer);
    portENTER_CRITICAL(&instance->timeout_lock);
    if (++instance->timer_generation == 0) {
        ++instance->timer_generation;
    }
    instance->timer_kind = kind;
    instance->timer_transfer_id = instance->active_ctx->transfer_id;
    instance->timeout_tuple.kind = kind;
    instance->timeout_tuple.generation = instance->timer_generation;
    instance->timeout_tuple.transfer_id = instance->timer_transfer_id;
    instance->timeout_pending_tuple = (ft_timeout_tuple_t) {
        0
    };
    atomic_store(&instance->timeout_pending, false);
    portEXIT_CRITICAL(&instance->timeout_lock);
    esp_err_t err = esp_timer_start_once(instance->timer, timeout_us);
    if (err != ESP_OK) {
        ft_timer_disarm(instance);
    }
    return err;
}

void ft_timer_disarm(ft_instance_t *instance)
{
    if (!instance || !instance->timer) {
        return;
    }
    esp_timer_stop(instance->timer);
    portENTER_CRITICAL(&instance->timeout_lock);
    if (++instance->timer_generation == 0) {
        ++instance->timer_generation;
    }
    instance->timer_kind = FT_TIMER_NONE;
    instance->timer_transfer_id = 0;
    instance->timeout_tuple = (ft_timeout_tuple_t) {
        0
    };
    instance->timeout_pending_tuple = (ft_timeout_tuple_t) {
        0
    };
    atomic_store(&instance->timeout_pending, false);
    portEXIT_CRITICAL(&instance->timeout_lock);
}

void ft_snapshot_update(ft_instance_t *instance)
{
    if (!instance || !instance->snapshot_mutex) {
        return;
    }
    xSemaphoreTake(instance->snapshot_mutex, portMAX_DELAY);
    ft_context_t *ctx = instance->active_ctx;
    memset(&instance->snapshot, 0, sizeof(instance->snapshot));
    if (!ctx) {
        instance->snapshot.role = ESP_FILE_TRANSFER_ROLE_NONE;
        instance->snapshot.state = ESP_FILE_TRANSFER_STATE_IDLE;
    } else {
        instance->snapshot.in_progress = true;
        instance->snapshot.role = ctx->role;
        instance->snapshot.state = (esp_file_transfer_state_t)(ctx->state + 1);
        memcpy(instance->snapshot.file_name, ctx->file_name, sizeof(instance->snapshot.file_name));
        instance->snapshot.bytes_transferred = ctx->bytes_transferred;
        instance->snapshot.total_bytes = ctx->file_size;
        instance->snapshot.percent = ctx->file_size == 0 && ctx->state == TRANSFER_STATE_FINALIZING
                                     ? 100
                                     : (ctx->file_size == 0 ? 0 :
                                        (uint8_t)((ctx->bytes_transferred * 100) / ctx->file_size));
        instance->snapshot.reason_code = ctx->reason_code;
    }
    xSemaphoreGive(instance->snapshot_mutex);
}

void ft_emit_event(ft_instance_t *instance, esp_file_transfer_event_id_t event_id)
{
    if (!instance || !instance->active_ctx || !instance->event_cb) {
        return;
    }
    ft_context_t *ctx = instance->active_ctx;
    uint8_t percent = ctx->file_size == 0
                      ? ((event_id == ESP_FT_EVENT_PROGRESS ||
                          event_id == ESP_FT_EVENT_COMPLETED ||
                          event_id == ESP_FT_EVENT_VERIFYING) ? 100 : 0)
                      : (uint8_t)((ctx->bytes_transferred * 100) / ctx->file_size);
    esp_file_transfer_event_t event = {
        .event_id = event_id,
        .role = ctx->role,
        .file_name = ctx->file_name,
        .bytes_transferred = ctx->bytes_transferred,
        .total_bytes = ctx->file_size,
        .percent = percent,
        .reason_code = ctx->reason_code,
        .saved_name = ctx->saved_name[0] ? ctx->saved_name : NULL,
        .detail = ctx->detail,
    };
    ft_snapshot_update(instance);
    instance->event_cb(&event, instance->event_ctx);
}

void ft_emit_rejected_event(ft_instance_t *instance, const char *file_name,
                            uint64_t file_size, uint16_t reason, esp_err_t detail)
{
    if (!instance || !instance->event_cb) {
        return;
    }
    esp_file_transfer_event_t event = {
        .event_id = ESP_FT_EVENT_META_RECEIVED,
        .role = ESP_FILE_TRANSFER_ROLE_RECEIVER,
        .file_name = file_name,
        .total_bytes = file_size,
        .reason_code = ESP_FT_REASON_OK,
        .detail = ESP_OK,
    };
    instance->event_cb(&event, instance->event_ctx);
    event.event_id = ESP_FT_EVENT_FAILED;
    event.reason_code = reason;
    event.detail = detail;
    instance->event_cb(&event, instance->event_ctx);
}

void ft_finish_transfer(ft_instance_t *instance, esp_file_transfer_event_id_t terminal_event,
                        uint16_t reason, esp_err_t detail)
{
    if (!instance || !instance->active_ctx || instance->active_ctx->terminal_emitted) {
        return;
    }
    ft_context_t *ctx = instance->active_ctx;
    ctx->terminal_emitted = true;
    ctx->reason_code = reason;
    ctx->detail = detail;
    if (terminal_event == ESP_FT_EVENT_FAILED) {
        ctx->state = TRANSFER_STATE_ERROR;
    }
    ft_timer_disarm(instance);
    if (ctx->file) {
        fclose(ctx->file);
        ctx->file = NULL;
    }
    if (ctx->role == ESP_FILE_TRANSFER_ROLE_RECEIVER && !ctx->target_committed) {
        ft_fs_remove(ctx->temp_path);
    }
    ft_hash_destroy(ctx->hash);
    ctx->hash = NULL;
    free(ctx->block_buffer);
    ctx->block_buffer = NULL;
    ft_emit_event(instance, terminal_event);
    free(ctx->source_path);
    free(ctx->temp_path);
    free(ctx->target_path);
    free(ctx);
    instance->active_ctx = NULL;
    ft_snapshot_update(instance);
}

void ft_abort_active(ft_instance_t *instance, uint16_t reason, esp_err_t detail,
                     bool notify_peer, bool cancelled)
{
    if (!instance || !instance->active_ctx) {
        return;
    }
    ft_timer_disarm(instance);
    instance->active_ctx->pending_valid = false;
    if (notify_peer) {
        ft_gmp_send_abort(instance, instance->active_ctx, reason);
    }
    ft_finish_transfer(instance, cancelled ? ESP_FT_EVENT_CANCELLED : ESP_FT_EVENT_FAILED,
                       reason, detail);
}

static bool timeout_current(ft_instance_t *instance, const ft_timeout_tuple_t *timeout)
{
    if (!instance->active_ctx) {
        return false;
    }
    bool current;
    portENTER_CRITICAL(&instance->timeout_lock);
    current = timeout->kind != FT_TIMER_NONE &&
              timeout->kind == instance->timer_kind &&
              timeout->generation == instance->timer_generation &&
              timeout->transfer_id == instance->timer_transfer_id &&
              timeout->transfer_id == instance->active_ctx->transfer_id;
    portEXIT_CRITICAL(&instance->timeout_lock);
    return current;
}

static void handle_timeout(ft_instance_t *instance, const ft_timeout_tuple_t *timeout)
{
    if (!timeout_current(instance, timeout)) {
        return;
    }
    if (instance->active_ctx->role == ESP_FILE_TRANSFER_ROLE_SENDER) {
        ft_sender_handle_timeout(instance, timeout->kind);
    } else {
        ft_receiver_handle_timeout(instance, timeout->kind);
    }
}

bool ft_process_pending_termination(ft_instance_t *instance)
{
    if (atomic_exchange(&instance->link_down_pending, false)) {
        bool had_transport_error = atomic_exchange(&instance->transport_error_pending, false);
        esp_err_t detail = had_transport_error ? atomic_load(&instance->transport_error) :
                           ESP_ERR_INVALID_STATE;
        ft_abort_active(instance, ESP_FT_REASON_LINK_ERROR, detail, false, false);
        return true;
    }
    if (atomic_exchange(&instance->transport_error_pending, false)) {
        esp_err_t error = atomic_load(&instance->transport_error);
        ft_abort_active(instance, ESP_FT_REASON_DATA_SEND_FAILED, error, true, false);
        return true;
    }
    if (atomic_exchange(&instance->user_abort_pending, false)) {
        if (instance->active_ctx) {
            ft_abort_active(instance, ESP_FT_REASON_ABORTED, ESP_OK, true, true);
            atomic_store(&instance->user_abort_completed, true);
            return true;
        }
    }
    return false;
}

static void process_pending_flags(ft_instance_t *instance)
{
    if (ft_process_pending_termination(instance)) {
        return;
    }
    if (atomic_exchange(&instance->timeout_pending, false)) {
        ft_timeout_tuple_t timeout;
        portENTER_CRITICAL(&instance->timeout_lock);
        timeout = instance->timeout_pending_tuple;
        instance->timeout_pending_tuple = (ft_timeout_tuple_t) {
            0
        };
        portEXIT_CRITICAL(&instance->timeout_lock);
        handle_timeout(instance, &timeout);
    }
}

static void handle_packet(ft_instance_t *instance, const ft_gmp_packet_t *packet)
{
    if (packet->command_id == ESP_FT_CMD_ABORT && packet->op == ESP_GMP_OP_WRITE_REQ) {
        ft_handle_peer_abort(instance, packet);
    } else if (packet->command_id == ESP_FT_CMD_TRANSFER_META &&
               packet->op == ESP_GMP_OP_WRITE_REQ) {
        ft_receiver_handle_meta(instance, packet);
    } else if (packet->command_id == ESP_FT_CMD_DATA_BLOCK &&
               packet->op == ESP_GMP_OP_WRITE_REQ) {
        ft_receiver_handle_data(instance, packet);
    } else if (instance->active_ctx &&
               instance->active_ctx->role == ESP_FILE_TRANSFER_ROLE_SENDER) {
        ft_sender_handle_packet(instance, packet);
    } else if (packet->op == ESP_GMP_OP_WRITE_REQ &&
               (packet->command_id == ESP_FT_CMD_TRANSFER_META ||
                packet->command_id == ESP_FT_CMD_DATA_BLOCK)) {
        ft_gmp_send_response(instance, packet, ESP_GMP_STATUS_BAD_STATE, NULL, 0);
    }
}

static void free_event(ft_worker_event_t *event)
{
    if (event->type == FT_WORK_SEND_CMD) {
        free(event->data.send.src_path);
        free(event->data.send.remote_name);
    } else if (event->type == FT_WORK_GMP_PACKET) {
        free(event->data.packet.payload);
    }
}

static void complete_reply(ft_command_reply_t *reply, esp_err_t result)
{
    if (reply->result) {
        *reply->result = result;
    }
    if (reply->done) {
        xSemaphoreGive(reply->done);
    }
}

static bool receive_next_event(ft_instance_t *instance, ft_worker_event_t *event)
{
    if (xQueueReceive(instance->urgent_queue, event, 0) == pdTRUE) {
        return true;
    }
    return xQueueReceive(instance->event_queue, event, 0) == pdTRUE;
}

static void worker_task(void *arg)
{
    ft_instance_t *instance = arg;
    bool running = true;
    while (running) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        process_pending_flags(instance);
        ft_worker_event_t event;
        while (receive_next_event(instance, &event)) {
            switch (event.type) {
            case FT_WORK_SEND_CMD: {
                esp_err_t err = ft_sender_start(instance, event.data.send.src_path,
                                                event.data.send.remote_name);
                complete_reply(&event.data.send.reply, err);
                if (err == ESP_OK) {
                    ft_sender_prepare(instance);
                }
                break;
            }
            case FT_WORK_ABORT_CMD:
                if (atomic_exchange(&instance->user_abort_completed, false)) {
                    complete_reply(&event.data.abort.reply, ESP_OK);
                } else if (instance->active_ctx) {
                    ft_abort_active(instance, ESP_FT_REASON_ABORTED, ESP_OK, true, true);
                    complete_reply(&event.data.abort.reply, ESP_OK);
                } else {
                    complete_reply(&event.data.abort.reply, ESP_ERR_INVALID_STATE);
                }
                break;
            case FT_WORK_GMP_PACKET:
                handle_packet(instance, &event.data.packet);
                break;
            case FT_WORK_TIMEOUT:
                handle_timeout(instance, &event.data.timeout);
                break;
            case FT_WORK_DEINIT:
                if (instance->active_ctx) {
                    ft_abort_active(instance, ESP_FT_REASON_ABORTED, ESP_OK, true, true);
                }
                ft_timer_disarm(instance);
                complete_reply(&event.data.deinit.reply, ESP_OK);
                running = false;
                break;
            default:
                break;
            }
            free_event(&event);
            process_pending_flags(instance);
            if (!running) {
                break;
            }
        }
    }
    instance->worker_task = NULL;
    vTaskDelete(NULL);
}

static void reset_instance_dynamic(ft_instance_t *instance)
{
    free(instance->recv_dir);
    instance->recv_dir = NULL;
    instance->gmp_link = NULL;
    instance->event_cb = NULL;
    instance->event_ctx = NULL;
    instance->initialized = false;
}

esp_err_t esp_file_transfer_init(const esp_file_transfer_config_t *config)
{
    ft_instance_t *instance = &s_ft;
    if (!config || !config->recv_dir || !config->gmp_link || !config->event_cb) {
        return ESP_ERR_INVALID_ARG;
    }
    if (instance->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    size_t effective = esp_gmp_max_payload_effective(config->gmp_link);
    if (effective < ESP_FT_MIN_EFFECTIVE_PAYLOAD) {
        return ESP_FT_ERR_CAPABILITY_MISMATCH;
    }
    size_t block_limit = effective - ESP_FT_DATA_REQ_HDR_LEN;
    if (block_limit > ESP_FT_DEFAULT_MAX_BLOCK_SIZE) {
        block_limit = ESP_FT_DEFAULT_MAX_BLOCK_SIZE;
    }
    size_t block_size = config->block_size ? config->block_size : block_limit;
    if (block_size == 0 || block_size > UINT32_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (block_size > block_limit) {
        return config->block_size ? ESP_ERR_INVALID_ARG : ESP_FT_ERR_CAPABILITY_MISMATCH;
    }
    esp_err_t err = ft_fs_prepare(config->recv_dir);
    if (err != ESP_OK) {
        return err;
    }

    memset(instance, 0, sizeof(*instance));
    instance->timeout_lock = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;
    instance->recv_dir = strdup(config->recv_dir);
    if (!instance->recv_dir) {
        return ESP_ERR_NO_MEM;
    }
    instance->gmp_link = config->gmp_link;
    instance->max_file_size = config->max_file_size ? config->max_file_size :
                              ESP_FT_DEFAULT_MAX_FILE_SIZE;
    instance->configured_block_size = config->block_size;
    instance->block_size = block_size;
    instance->effective_payload = effective;
    instance->event_cb = config->event_cb;
    instance->event_ctx = config->event_ctx;
    instance->next_sequence = 1;
    instance->next_transfer_id = 1;
    atomic_init(&instance->accepting_events, false);
    atomic_init(&instance->user_abort_pending, false);
    atomic_init(&instance->user_abort_completed, false);
    atomic_init(&instance->link_down_pending, false);
    atomic_init(&instance->transport_error_pending, false);
    atomic_init(&instance->timeout_pending, false);
    atomic_init(&instance->transport_error, ESP_OK);
    atomic_init(&instance->hook_ref_count, 0);

    instance->snapshot_mutex = xSemaphoreCreateMutex();
    instance->abort_mutex = xSemaphoreCreateMutex();
    instance->event_queue = xQueueCreate(ESP_FT_EVENT_QUEUE_LEN, sizeof(ft_worker_event_t));
    instance->urgent_queue = xQueueCreate(ESP_FT_URGENT_QUEUE_LEN, sizeof(ft_worker_event_t));
    if (!instance->snapshot_mutex || !instance->abort_mutex ||
            !instance->event_queue || !instance->urgent_queue) {
        err = ESP_ERR_NO_MEM;
        goto fail;
    }
    const esp_timer_create_args_t timer_args = {
        .callback = timeout_callback,
        .arg = instance,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "ft_business",
    };
    err = esp_timer_create(&timer_args, &instance->timer);
    if (err != ESP_OK) {
        goto fail;
    }
    if (xTaskCreate(worker_task, "file_transfer", ESP_FT_WORKER_STACK_SIZE, instance,
                    ESP_FT_WORKER_PRIORITY, &instance->worker_task) != pdPASS) {
        err = ESP_ERR_NO_MEM;
        goto fail;
    }
    instance->initialized = true;
    ft_snapshot_update(instance);
    ft_fs_cleanup_parts(instance->recv_dir);
    atomic_store(&instance->accepting_events, true);
    return ESP_OK;

fail:
    if (instance->timer) {
        esp_timer_delete(instance->timer);
        instance->timer = NULL;
    }
    if (instance->event_queue) {
        vQueueDelete(instance->event_queue);
        instance->event_queue = NULL;
    }
    if (instance->urgent_queue) {
        vQueueDelete(instance->urgent_queue);
        instance->urgent_queue = NULL;
    }
    if (instance->snapshot_mutex) {
        vSemaphoreDelete(instance->snapshot_mutex);
        instance->snapshot_mutex = NULL;
    }
    if (instance->abort_mutex) {
        vSemaphoreDelete(instance->abort_mutex);
        instance->abort_mutex = NULL;
    }
    reset_instance_dynamic(instance);
    return err;
}

esp_err_t esp_file_transfer_deinit(void)
{
    ft_instance_t *instance = &s_ft;
    if (!instance->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xTaskGetCurrentTaskHandle() == instance->worker_task) {
        return ESP_ERR_INVALID_STATE;
    }
    atomic_store(&instance->accepting_events, false);
    while (atomic_load(&instance->hook_ref_count) != 0) {
        taskYIELD();
    }
    SemaphoreHandle_t done = xSemaphoreCreateBinary();
    if (!done) {
        atomic_store(&instance->accepting_events, true);
        return ESP_ERR_NO_MEM;
    }
    esp_err_t result = ESP_FAIL;
    ft_worker_event_t event = {
        .type = FT_WORK_DEINIT,
        .data.deinit.reply = {
            .done = done,
            .result = &result,
        },
    };
    xQueueSend(instance->event_queue, &event, portMAX_DELAY);
    ft_worker_wake(instance);
    xSemaphoreTake(done, portMAX_DELAY);
    vSemaphoreDelete(done);
    while (instance->worker_task) {
        taskYIELD();
    }
    esp_timer_delete(instance->timer);
    instance->timer = NULL;
    ft_worker_event_t pending;
    while (xQueueReceive(instance->event_queue, &pending, 0) == pdTRUE) {
        free_event(&pending);
        if (pending.type == FT_WORK_SEND_CMD) {
            complete_reply(&pending.data.send.reply, ESP_ERR_INVALID_STATE);
        } else if (pending.type == FT_WORK_ABORT_CMD) {
            complete_reply(&pending.data.abort.reply, ESP_ERR_INVALID_STATE);
        }
    }
    while (xQueueReceive(instance->urgent_queue, &pending, 0) == pdTRUE) {
        free_event(&pending);
    }
    vQueueDelete(instance->event_queue);
    vQueueDelete(instance->urgent_queue);
    vSemaphoreDelete(instance->snapshot_mutex);
    vSemaphoreDelete(instance->abort_mutex);
    instance->event_queue = NULL;
    instance->urgent_queue = NULL;
    instance->snapshot_mutex = NULL;
    instance->abort_mutex = NULL;
    reset_instance_dynamic(instance);
    return result;
}

esp_err_t esp_file_transfer_send(const esp_file_transfer_send_param_t *param)
{
    ft_instance_t *instance = &s_ft;
    if (!param || !param->src_path) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!instance->initialized || !atomic_load(&instance->accepting_events)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xTaskGetCurrentTaskHandle() == instance->worker_task) {
        return ESP_ERR_INVALID_STATE;
    }
    ft_worker_event_t event = {
        .type = FT_WORK_SEND_CMD,
    };
    event.data.send.src_path = strdup(param->src_path);
    event.data.send.remote_name = param->remote_name ? strdup(param->remote_name) : NULL;
    event.data.send.reply.done = xSemaphoreCreateBinary();
    esp_err_t result = ESP_FAIL;
    event.data.send.reply.result = &result;
    if (!event.data.send.src_path || (param->remote_name && !event.data.send.remote_name) ||
            !event.data.send.reply.done) {
        free(event.data.send.src_path);
        free(event.data.send.remote_name);
        if (event.data.send.reply.done) {
            vSemaphoreDelete(event.data.send.reply.done);
        }
        return ESP_ERR_NO_MEM;
    }
    if (xQueueSend(instance->event_queue, &event, 0) != pdTRUE) {
        free(event.data.send.src_path);
        free(event.data.send.remote_name);
        vSemaphoreDelete(event.data.send.reply.done);
        return ESP_FT_ERR_QUEUE_FULL;
    }
    ft_worker_wake(instance);
    xSemaphoreTake(event.data.send.reply.done, portMAX_DELAY);
    vSemaphoreDelete(event.data.send.reply.done);
    return result;
}

esp_err_t esp_file_transfer_abort(void)
{
    ft_instance_t *instance = &s_ft;
    if (!instance->initialized || !atomic_load(&instance->accepting_events)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xTaskGetCurrentTaskHandle() == instance->worker_task) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(instance->abort_mutex, portMAX_DELAY);
    esp_file_transfer_status_t status;
    esp_err_t status_err = esp_file_transfer_get_status(&status);
    if (status_err != ESP_OK || !status.in_progress) {
        xSemaphoreGive(instance->abort_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    SemaphoreHandle_t done = xSemaphoreCreateBinary();
    if (!done) {
        xSemaphoreGive(instance->abort_mutex);
        return ESP_ERR_NO_MEM;
    }
    esp_err_t result = ESP_FAIL;
    ft_worker_event_t event = {
        .type = FT_WORK_ABORT_CMD,
        .data.abort.reply = {
            .done = done,
            .result = &result,
        },
    };
    atomic_store(&instance->user_abort_completed, false);
    atomic_store(&instance->user_abort_pending, true);
    xQueueSend(instance->event_queue, &event, portMAX_DELAY);
    ft_worker_wake(instance);
    xSemaphoreTake(done, portMAX_DELAY);
    vSemaphoreDelete(done);
    xSemaphoreGive(instance->abort_mutex);
    return result;
}

esp_err_t esp_file_transfer_get_status(esp_file_transfer_status_t *status)
{
    ft_instance_t *instance = &s_ft;
    if (!status) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!instance->initialized || !instance->snapshot_mutex) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(instance->snapshot_mutex, portMAX_DELAY);
    *status = instance->snapshot;
    xSemaphoreGive(instance->snapshot_mutex);
    return ESP_OK;
}
