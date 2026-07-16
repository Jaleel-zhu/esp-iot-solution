/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "esp_err.h"
#include "esp_gmp_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize global GMP state. Call once before other APIs. */
void esp_gmp_init(void);

/**
 * Release internal resources after all links have been unregistered and all
 * transport TX completions have been delivered.
 *
 * This is not a runtime link teardown API. If any link is still active, the
 * call is ignored to avoid freeing mutexes/buffers still owned by transports.
 */
void esp_gmp_deinit(void);

/**
 * Optional maintenance hook for TX queue retry or statistics.
 * Transport timeouts are handled by the bound transport.
 */
void esp_gmp_poll(void);

/**
 * Register a link bound to a transport.
 * The transport ops must outlive the registration.
 */
esp_err_t esp_gmp_link_register(esp_gmp_link_t link, const esp_gmp_transport_t *ops, void *transport_ctx);

/**
 * Unregister a link and drop queued (not yet in-flight) TX buffers.
 * In-flight frames are released later via esp_gmp_transport_tx_done or a failed
 * send path; the link slot may remain until those completions arrive.
 */
void esp_gmp_link_unregister(esp_gmp_link_t link);

/** Register application handler for complete reassembled GMP messages. */
void esp_gmp_on_packet_register(esp_gmp_on_packet_fn cb, void *user_ctx);

/**
 * Inject one complete GMP frame from a transport.
 * Return true if on_packet took ownership of @p data; otherwise caller must free it.
 */
bool esp_gmp_input(esp_gmp_link_t link, const uint8_t *data, size_t len);

/**
 * Notify GMP that an asynchronous transport has finished using a TX frame.
 * A transport that returns ESP_OK from send() takes temporary ownership of the
 * frame pointer and must eventually call this once it no longer uses the bytes.
 * Call from the transport completion path after send() has returned.
 */
void esp_gmp_transport_tx_done(esp_gmp_link_t link, const uint8_t *data);

/**
 * Send one complete GMP message via the bound transport.
 * Copies @p payload into an owned frame buffer.
 */
esp_err_t esp_gmp_send(esp_gmp_link_t link, const esp_gmp_tx_params_t *params, const uint8_t *payload, size_t payload_len);

/** Effective max GMP payload for the link (policy cap ∩ transport limit). */
size_t esp_gmp_max_payload_effective(esp_gmp_link_t link);

#ifdef __cplusplus
}
#endif
