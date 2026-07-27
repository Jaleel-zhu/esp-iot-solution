/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_gmp.h"

#ifdef __cplusplus
extern "C" {
#endif

/** GMP command group assigned to the file transfer profile. */
#define ESP_GMP_GRP_FILE_TRANSFER 0x08

/** Base for component-specific esp_err_t values. */
#define ESP_ERR_FILE_TRANSFER_BASE        0x7500
#define ESP_FT_ERR_OPEN_FAILED            (ESP_ERR_FILE_TRANSFER_BASE + 0x01)
#define ESP_FT_ERR_REMOTE_NAME_INVALID    (ESP_ERR_FILE_TRANSFER_BASE + 0x02)
#define ESP_FT_ERR_BUSY                   (ESP_ERR_FILE_TRANSFER_BASE + 0x03)
#define ESP_FT_ERR_FILE_TOO_LARGE         (ESP_ERR_FILE_TRANSFER_BASE + 0x04)
#define ESP_FT_ERR_CAPABILITY_MISMATCH    (ESP_ERR_FILE_TRANSFER_BASE + 0x05)
#define ESP_FT_ERR_QUEUE_FULL             (ESP_ERR_FILE_TRANSFER_BASE + 0x06)

/** Local role in the current transfer. */
typedef enum {
    ESP_FILE_TRANSFER_ROLE_NONE = 0,
    ESP_FILE_TRANSFER_ROLE_SENDER,
    ESP_FILE_TRANSFER_ROLE_RECEIVER,
} esp_file_transfer_role_t;

/** File transfer event identifiers. */
typedef enum {
    ESP_FT_EVENT_STARTED = 0,
    ESP_FT_EVENT_META_SENT,
    ESP_FT_EVENT_META_RECEIVED,
    ESP_FT_EVENT_PEER_ACCEPTED,
    ESP_FT_EVENT_PEER_REJECTED,
    ESP_FT_EVENT_PROGRESS,
    ESP_FT_EVENT_VERIFYING,
    ESP_FT_EVENT_COMPLETED,
    ESP_FT_EVENT_FAILED,
    ESP_FT_EVENT_CANCELLED,
} esp_file_transfer_event_id_t;

/** Public transfer state snapshot. */
typedef enum {
    ESP_FILE_TRANSFER_STATE_IDLE = 0,
    ESP_FILE_TRANSFER_STATE_PREPARING,
    ESP_FILE_TRANSFER_STATE_WAIT_META_RSP,
    ESP_FILE_TRANSFER_STATE_WAIT_DATA_BLOCK,
    ESP_FILE_TRANSFER_STATE_SENDING_DATA,
    ESP_FILE_TRANSFER_STATE_WAIT_FINAL_CONFIRM,
    ESP_FILE_TRANSFER_STATE_FINALIZING,
    ESP_FILE_TRANSFER_STATE_ERROR,
} esp_file_transfer_state_t;

/** Event delivered to the application callback. */
typedef struct {
    esp_file_transfer_event_id_t event_id;
    esp_file_transfer_role_t role;
    const char *file_name;
    uint64_t bytes_transferred;
    uint64_t total_bytes;
    uint8_t percent;
    uint16_t reason_code;
    const char *saved_name;
    esp_err_t detail;
} esp_file_transfer_event_t;

/**
 * @brief Application event callback.
 *
 * The event and its string fields remain valid only for the duration of the
 * callback.
 */
typedef void (*esp_file_transfer_event_cb_t)(const esp_file_transfer_event_t *event,
                                             void *event_ctx);

/** Component initialization configuration. */
typedef struct {
    const char *recv_dir;
    esp_gmp_link_t gmp_link;
    size_t max_file_size;
    size_t block_size;
    esp_file_transfer_event_cb_t event_cb;
    void *event_ctx;
} esp_file_transfer_config_t;

/** Parameters for starting an outbound transfer. */
typedef struct {
    const char *src_path;
    const char *remote_name;
} esp_file_transfer_send_param_t;

/** Consistent snapshot returned by esp_file_transfer_get_status(). */
typedef struct {
    bool in_progress;
    esp_file_transfer_role_t role;
    esp_file_transfer_state_t state;
    char file_name[33];
    uint64_t bytes_transferred;
    uint64_t total_bytes;
    uint8_t percent;
    uint16_t reason_code;
} esp_file_transfer_status_t;

/**
 * @brief Initialize the singleton file transfer component.
 */
esp_err_t esp_file_transfer_init(const esp_file_transfer_config_t *config);

/**
 * @brief Deinitialize the singleton file transfer component.
 */
esp_err_t esp_file_transfer_deinit(void);

/**
 * @brief Start an asynchronous outbound file transfer.
 */
esp_err_t esp_file_transfer_send(const esp_file_transfer_send_param_t *param);

/**
 * @brief Cancel the active transfer.
 */
esp_err_t esp_file_transfer_abort(void);

/**
 * @brief Get the current transfer status snapshot.
 */
esp_err_t esp_file_transfer_get_status(esp_file_transfer_status_t *status);

/**
 * @brief Dispatch a GMP file transfer packet.
 *
 * This hook never takes ownership of pkt->frame_buf and always returns false.
 */
bool esp_file_transfer_on_packet(const esp_gmp_rx_t *pkt);

/**
 * @brief Notify the component before the configured GMP link is unregistered.
 */
void esp_file_transfer_on_link_down(esp_gmp_link_t link);

/**
 * @brief Notify the component of an unrecoverable transport error.
 */
void esp_file_transfer_on_transport_error(esp_gmp_link_t link, esp_err_t error);

#ifdef __cplusplus
}
#endif
