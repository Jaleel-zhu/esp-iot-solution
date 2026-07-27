/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * GMP OTA client (host): OS_CAP_QUERY + CONTROL/DATA upload over esp_gmp.
 */

#pragma once

#include "esp_err.h"
#include "esp_gmp.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    esp_gmp_link_t link;
    uint16_t chunk_size;
    uint32_t max_gmp_payload;
} gmp_ota_client_caps_t;

/**
 * Read @p len bytes at @p offset into @p buf; set @p out_len to bytes read.
 */
typedef esp_err_t (*gmp_ota_client_read_fn_t)(void *ctx, size_t offset, uint8_t *buf, size_t len, size_t *out_len);

/**
 * Run full OTA upload from a reader callback: start → DATA chunks → finish(SHA256).
 * DATA chunks use a pipelined stop-and-wait (up to GMP_OTA_CLIENT_PIPE_DEPTH in flight).
 * The common client keeps process-wide response state and is not thread-safe;
 * run at most one upload/query flow at a time.
 */
esp_err_t gmp_ota_client_upload_stream(esp_gmp_link_t link, size_t image_len, gmp_ota_client_read_fn_t read_fn, void *read_ctx);

/**
 * Run full OTA upload from a contiguous buffer.
 */
esp_err_t gmp_ota_client_upload(esp_gmp_link_t link, const uint8_t *image, size_t image_len);

/** Query device capabilities via OS_CAP_QUERY; not thread-safe with upload. */
esp_err_t gmp_ota_client_query_caps(esp_gmp_link_t link, gmp_ota_client_caps_t *out);

/** Install GMP response handler; call once after esp_gmp_init(). */
void gmp_ota_client_install_handler(void);

/** Request in-flight upload to abort; safe from disconnect callbacks. */
void gmp_ota_client_request_cancel(void);

/** Clear cancel flag before starting a new upload. */
void gmp_ota_client_clear_cancel(void);

#ifdef __cplusplus
}
#endif
