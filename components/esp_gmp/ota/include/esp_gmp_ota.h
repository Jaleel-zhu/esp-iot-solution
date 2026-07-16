/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "esp_err.h"
#include "esp_gmp.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /** OTA partition label; NULL uses esp_ota_get_next_update_partition(). */
    const char *ota_partition_label;
    /**
     * Suggested OTA_UPLOAD_DATA.data bytes per chunk in start response.
     * 0 = auto from esp_gmp_max_payload_effective(link) - 8.
     */
    uint16_t chunk_hint;
    /** Delay before esp_restart() after successful apply (ms). */
    uint32_t restart_delay_ms;
} esp_gmp_ota_config_t;

/** Initialize OTA handler. @p cfg may be NULL for Kconfig defaults. */
esp_err_t esp_gmp_ota_init(const esp_gmp_ota_config_t *cfg);

void esp_gmp_ota_deinit(void);

/**
 * Dispatch GRP_OTA packets from esp_gmp_on_packet.
 * Handles OTA_UPLOAD_CONTROL / DATA / QUERY and sends GMP responses.
 */
/** Return true when rx->frame_buf ownership is taken (OTA DATA zero-copy path). */
bool esp_gmp_ota_on_packet(const esp_gmp_rx_t *pkt);

/** Call on link disconnect or esp_gmp_link_unregister. */
void esp_gmp_ota_on_link_down(esp_gmp_link_t link);

#ifdef __cplusplus
}
#endif
