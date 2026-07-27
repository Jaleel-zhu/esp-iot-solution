/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_file_transfer_internal.h"
#include "esp_file_transfer_protocol.h"

esp_err_t ft_data_send_request(ft_instance_t *instance, ft_context_t *ctx,
                               uint32_t block_index, size_t data_len)
{
    if (!instance || !ctx || !ctx->block_buffer || data_len == 0 ||
            data_len > ctx->block_buffer_size || data_len > UINT32_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    size_t payload_len = ESP_FT_DATA_REQ_HDR_LEN + data_len;
    if (payload_len > instance->effective_payload) {
        return ESP_ERR_INVALID_SIZE;
    }
    ft_protocol_encode_data_request_header(ctx->block_buffer, ctx->transfer_id, block_index,
                                           (uint32_t)data_len);
    return ft_gmp_send_request(instance, ctx, ESP_FT_CMD_DATA_BLOCK,
                               ctx->block_buffer, payload_len, true);
}

esp_err_t ft_data_send_response(ft_instance_t *instance, const ft_gmp_packet_t *request,
                                uint32_t transfer_id, uint32_t block_index,
                                uint8_t status, uint16_t reason)
{
    ft_data_response_t response = {
        .transfer_id = transfer_id,
        .block_index = block_index,
        .status = status,
        .reason_code = reason,
    };
    uint8_t payload[ESP_FT_DATA_RSP_LEN];
    esp_err_t err = ft_protocol_encode_data_response(&response, payload, sizeof(payload));
    if (err != ESP_OK) {
        return err;
    }
    return ft_gmp_send_response(instance, request, ESP_GMP_STATUS_OK, payload, sizeof(payload));
}
