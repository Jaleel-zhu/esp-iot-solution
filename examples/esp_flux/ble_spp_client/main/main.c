/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "ble_manager.h"
#include "gatt_discovery.h"
#include "flux_gatt_session.h"

static const char *TAG = "spp client";

/* 16 Bit SPP Service UUID */
#define BLE_SVC_SPP_UUID16      0xABF0
/* 16 Bit SPP Service Characteristic UUID */
#define BLE_SVC_SPP_CHR_UUID16  0xABF1

#define LL_PACKET_TIME      2120 // 2120 microseconds (2120 * 1.25ms = 2650ms)
#define LL_PACKET_LENGTH    251  // 251 bytes (251 * 8 = 2008 bits)  (251 * 1.25ms = 313.75ms)
#define OPTIMAL_MTU_SIZE    498  // Optimal MTU size for LL_PACKET_LENGTH 251 bytes

#define MAX_CONNECTIONS CONFIG_BLE_MANAGER_MAX_CONNECTIONS

#define SPP_TEST_TRANSFER_SIZE  (482 + 491 * 8) // 482 + 491 * 8 = 482 + 3928 = 4411

static ble_manager_t *g_ble_manager = NULL;
static gatt_session_t *g_session_instances[MAX_CONNECTIONS] = {0};
static bool g_conn_params_ready[MAX_CONNECTIONS] = {0};
static SemaphoreHandle_t g_sessions_mutex = NULL;

static inline void sessions_lock(void)
{
    if (g_sessions_mutex) {
        xSemaphoreTake(g_sessions_mutex, portMAX_DELAY);
    }
}

static inline void sessions_unlock(void)
{
    if (g_sessions_mutex) {
        xSemaphoreGive(g_sessions_mutex);
    }
}

static int spp_find_session_idx_by_conn(uint16_t conn_handle)
{
    sessions_lock();
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (g_session_instances[i] && g_session_instances[i]->conn_handle == conn_handle) {
            sessions_unlock();
            return i;
        }
    }
    sessions_unlock();
    return -1;
}

static void spp_fill_test_buffer(uint8_t *buf, size_t size)
{
    const char *base_text = "The quick brown fox jumps over the lazy dog. This is a test message for BLE SPP communication. ";
    const size_t base_len = strlen(base_text);
    size_t written = 0;

    while (written < size) {
        size_t to_copy = (size - written < base_len) ? (size - written) : base_len;
        memcpy(buf + written, base_text, to_copy);
        written += to_copy;
    }
}

static void gatt_session_complete_cb(gatt_session_t *session, uint8_t stream_id, esp_err_t status, const uint8_t *data, uint32_t size)
{
    if (status == ESP_OK) {
        ESP_LOGI(TAG, "Session conn_handle %d stream %d send completed successfully (%" PRIu32 " bytes)", session->conn_handle, stream_id, size);
    } else {
        ESP_LOGE(TAG, "Session conn_handle %d stream %d send failed: %d", session->conn_handle, stream_id, status);
    }

    // The library passes back the original source buffer (malloced by this app before
    // gatt_session_send()) so the correct one is freed. The library has already
    // released its internal send-slot state. stream_id identifies which transfer this was.
    if (data) {
        free((void *)data);
    }
}

static void gatt_data_received_cb(gatt_session_t *session, uint8_t stream_id, const uint8_t *data, uint32_t size)
{
    ESP_LOGW(TAG, "Session conn_handle: %d stream %d received data, data size: %" PRIu32 " bytes (%.2f KB)", session->conn_handle, stream_id, size, size / 1024.0f);

    // Note: Free the data pointer after use.
    free((void *)data);
}

static void gatt_progress_cb(gatt_session_t *session, uint32_t transferred, uint32_t total)
{
    // Prevent division by zero
    if (total == 0) {
        ESP_LOGW(TAG, "Session conn_handle %d progress callback: total is 0, transferred=%" PRIu32, session->conn_handle, transferred);
        return;
    }
    uint8_t progress = (transferred * 100) / total;
    ESP_LOGI(TAG, "Session conn_handle: %d progress: %u%% (%" PRIu32 "/%" PRIu32 " bytes)", session->conn_handle, progress, transferred, total);
}

static esp_err_t gatt_enable_notification(gatt_discovery_t *discovery)
{
    gatt_descriptor_t *dsc = gatt_discovery_find_descriptor_by_uuid(discovery, BLE_UUID16_DECLARE(BLE_SVC_SPP_UUID16),
                                                                    BLE_UUID16_DECLARE(BLE_SVC_SPP_CHR_UUID16), BLE_UUID16_DECLARE(BLE_GATT_DSC_CLT_CFG_UUID16));
    if (!dsc) {
        ESP_LOGE(TAG, "CCCD not found, cannot enable notifications for conn_handle=%d", discovery->conn_handle);
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t value[2] = {1, 0};
    int rc = ble_gattc_write_flat(discovery->conn_handle, dsc->dsc.handle, value, sizeof(value), NULL, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error: Failed to subscribe to characteristic; rc=%d\n", rc);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static void gatt_discovery_complete_cb(gatt_discovery_t *discovery, int status, void *arg)
{
    if (status == ESP_OK) {
        ESP_LOGI(TAG, "Service discovery completed for connection %d", discovery->conn_handle);

        gatt_characteristic_t *chr = gatt_discovery_find_characteristic_by_uuid(discovery, BLE_UUID16_DECLARE(BLE_SVC_SPP_UUID16),
                                                                                BLE_UUID16_DECLARE(BLE_SVC_SPP_CHR_UUID16));
        if (!chr) {
            ESP_LOGW(TAG, "SPP characteristic not found after discovery, conn_handle=%d", discovery->conn_handle);
        }
        if (chr) {
            // Notification enable
            esp_err_t notify_err = gatt_enable_notification(discovery);
            if (notify_err != ESP_OK) {
                ESP_LOGW(TAG, "Skip session creation due to notification setup failure, conn_handle=%d err=%d",
                         discovery->conn_handle, notify_err);
                gatt_discovery_destroy(discovery);
                return;
            }

            ble_session_callbacks_t callbacks = {
                .session_complete_cb = gatt_session_complete_cb,
                .data_received_cb = gatt_data_received_cb,
                .progress_cb = gatt_progress_cb,
                .error_cb = NULL
            };
            // Use MASTER role as client
            uint16_t mtu = BLE_ATT_MTU_DFLT; // Default MTU
            ble_manager_get_mtu_by_conn(g_ble_manager, discovery->conn_handle, &mtu);
            if (mtu < BLE_ATT_MTU_DFLT) {
                mtu = BLE_ATT_MTU_DFLT;
            }
            gatt_session_t *session = gatt_session_create(discovery->conn_handle, &callbacks, chr->chr.val_handle,
                                                          chr->chr.val_handle, mtu, GATT_SESSION_ROLE_MASTER);
            if (session) {
                int session_idx = -1;
                sessions_lock();
                for (int i = 0; i < MAX_CONNECTIONS; i++) {
                    if (!g_session_instances[i]) {
                        g_session_instances[i] = session;
                        session_idx = i;
                        break;
                    }
                }
                sessions_unlock();

                if (session_idx >= 0) {
                    // conn_update may have fired before async discovery finished.
                    struct ble_gap_conn_desc desc;
                    if (ble_gap_conn_find(discovery->conn_handle, &desc) == 0) {
                        g_conn_params_ready[session_idx] = true;
                        ESP_LOGW(TAG, "GATT session ready for conn_handle=%d (interval=%.2fms)",
                                 session->conn_handle, desc.conn_itvl * 1.25f);
                    } else {
                        g_conn_params_ready[session_idx] = false;
                        ESP_LOGW(TAG, "GATT session ready for conn_handle=%d, waiting for connection parameter update",
                                 session->conn_handle);
                    }
                }

                // if (ble_manager_get_total_connections(g_ble_manager) < 2) {
                //     // Restart scan, connect to next device
                //     ble_manager_start_scan(g_ble_manager, 100000);
                // }
            }
        }
    } else {
        ESP_LOGE(TAG, "Service discovery failed for connection %d: %d", discovery->conn_handle, status);
    }

    // Free discovery
    gatt_discovery_destroy(discovery);
}

static void gatt_service_found_cb(gatt_discovery_t *discovery, uint16_t conn_handle, const struct ble_gatt_svc *service, void *arg)
{
    // char uuid_str[37];
    // ESP_LOGI(TAG, "Service found: %s, handle: (0x%04X-0x%04X)", ble_uuid_to_str(&service->uuid.u, uuid_str), service->start_handle, service->end_handle);
}

static void gatt_characteristic_found_cb(gatt_discovery_t *discovery, uint16_t conn_handle, const struct ble_gatt_chr *characteristic, void *arg)
{
    // char uuid_str[37];
    // ESP_LOGI(TAG, "Characteristic found: %s, def_handle: 0x%04X, val_handle: 0x%04X",
    //     ble_uuid_to_str(&characteristic->uuid.u, uuid_str), characteristic->def_handle, characteristic->val_handle);
}

static void gatt_descriptor_found_cb(gatt_discovery_t *discovery, uint16_t conn_handle, const struct ble_gatt_dsc *descriptor, void *arg)
{
    // char uuid_str[37];
    // ESP_LOGI(TAG, "Descriptor found: %s, handle: (0x%04X)", ble_uuid_to_str(&descriptor->uuid.u, uuid_str), descriptor->handle);
}

static void ble_manager_scan_cb(ble_manager_t *manager, uint8_t type, const struct ble_gap_disc_desc *result, void *arg)
{
    switch (type) {
    case BLE_GAP_EVENT_DISC_COMPLETE:
        /* code */
        break;

    case BLE_GAP_EVENT_DISC: {
        // ESP_LOG_BUFFER_HEX_LEVEL(TAG, result->data, result->length_data, ESP_LOG_INFO);

        /* Try to connect to the device */
        struct ble_hs_adv_fields fields;
        int rc = ble_hs_adv_parse_fields(&fields, result->data, result->length_data);
        if (rc != 0) {
            ESP_LOGE(TAG, "Failed to parse advertisement data: %d", rc);
            break;
        }

        for (uint8_t i = 0; i < fields.num_uuids16; i++) {
            if (ble_uuid_u16(&fields.uuids16[i].u) == BLE_SVC_SPP_UUID16) {
                // for max throughput:
                // min_interval: 6 + (ble_manager_get_total_connections(g_ble_manager)) * 5
                // max_interval: 6 + (ble_manager_get_total_connections(g_ble_manager)) * 5
                // should change here if 0x0212(BLE_ERR_INV_HCI_CMD_PARMS) error happens
                // ble_manager_connect(g_ble_manager, &result->addr, 6 + (ble_manager_get_total_connections(g_ble_manager)) * 5,
                //                     6 + (ble_manager_get_total_connections(g_ble_manager)) * 5, 0, 300);
                ble_manager_connect(g_ble_manager, &result->addr, 24, 24, 0, 300);
            }
        }
        break;
    }

    default:
        break;
    }
}

static void ble_manager_connect_cb(ble_manager_t *manager, uint8_t type, ble_connection_t *conn, int status, void *arg)
{
    switch (type) {
    case BLE_EVENT_TYPE_CONNECT_COMPLETE:
    case BLE_EVENT_TYPE_RECONNECT_COMPLETE: {
        if (type == BLE_EVENT_TYPE_RECONNECT_COMPLETE) {
            ESP_LOGI(TAG, "Reconnected successfully to device %s", addr_str(conn->peer_addr.val));
        } else {
            ESP_LOGI(TAG, "Connected successfully to device %s (first time)", addr_str(conn->peer_addr.val));

            // On first connection, auto-reconnect can be enabled
            // Example: reconnect every 5 seconds, maximum 3 reconnections (0 means unlimited reconnection)
            esp_err_t ret = ble_manager_enable_auto_reconnect(g_ble_manager, conn->conn_handle, 5000, 3);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Auto reconnect enabled for device %s", addr_str(conn->peer_addr.val));
            }
        }

        // Start service discovery
        ble_discovery_callbacks_t callbacks = {
            .discovery_complete_cb = gatt_discovery_complete_cb,
            .service_found_cb = gatt_service_found_cb,
            .characteristic_found_cb = gatt_characteristic_found_cb,
            .descriptor_found_cb = gatt_descriptor_found_cb,
            .arg = NULL
        };
        gatt_discovery_t *discovery = gatt_discovery_start(conn->conn_handle, &callbacks);
        if (discovery) {
            gatt_discovery_discover_service_by_uuid(discovery, BLE_UUID16_DECLARE(BLE_SVC_SPP_UUID16));
        } else {
            ESP_LOGE(TAG, "Failed to start service discovery");
        }
        break;
    }
    default:
        break;
    }
}

static void ble_manager_conn_update_cb(ble_manager_t *manager, ble_connection_t *conn, void *arg)
{
    int idx = spp_find_session_idx_by_conn(conn->conn_handle);
    if (idx < 0) {
        return;
    }

    g_conn_params_ready[idx] = true;
    ESP_LOGW(TAG, "Connection parameters ready for conn_handle=%d (interval=%.2fms)",
             conn->conn_handle, conn->conn_interval * 1.25f);
}

static void ble_manager_disconnect_cb(ble_manager_t *manager, ble_connection_t *conn, int status, void *arg)
{
    ESP_LOGI(TAG, "Disconnected from device %s (conn_handle=%d)", addr_str(conn->peer_addr.val), conn->conn_handle);

    // Clean up session instance
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        gatt_session_t *session_to_destroy = NULL;
        sessions_lock();
        if (g_session_instances[i] && g_session_instances[i]->conn_handle == conn->conn_handle) {
            session_to_destroy = g_session_instances[i];
            g_session_instances[i] = NULL;
            g_conn_params_ready[i] = false;
        }
        sessions_unlock();
        if (session_to_destroy) {
            gatt_session_destroy(session_to_destroy);
            ESP_LOGI(TAG, "GATT session destroyed for conn_handle=%d", conn->conn_handle);
        }
    }
}

static void nimble_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host Task Started");

    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

static void nimble_host_on_reset(int reason)
{
    ESP_LOGI(TAG, "BLE Host Resetting state; reason=%d\n", reason);
}

static void nimble_host_on_sync(void)
{
    ESP_LOGI(TAG, "BLE Host Synced");
    int rc;

    /* Make sure we have proper identity address set (public preferred) */
    rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);
}

void ble_store_config_init(void);
static void nimble_host_config(void)
{
    ESP_LOGI(TAG, "BLE Host config");

    /* Configure the nimble host. */
    ble_hs_cfg.reset_cb = nimble_host_on_reset;
    ble_hs_cfg.sync_cb = nimble_host_on_sync;
    ble_hs_cfg.gatts_register_cb = NULL;
    ble_hs_cfg.store_status_cb = NULL;

    /* Configure the nimble store.*/
    ble_store_config_init();
}

void app_main(void)
{
    esp_err_t ret;

    ESP_LOGI(TAG, "Starting BLE Component Test Example");
    esp_log_level_set("ble_manager", ESP_LOG_WARN);
    esp_log_level_set("gatt_discovery", ESP_LOG_WARN);
    esp_log_level_set("gatt_session", ESP_LOG_WARN);
    esp_log_level_set("esp_flux", ESP_LOG_WARN);
    esp_log_level_set("spp server", ESP_LOG_WARN);
    esp_log_level_set("spp client", ESP_LOG_WARN);
    esp_log_level_set("NimBLE", ESP_LOG_WARN);

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize nimble: %d", ret);
        return;
    }

    nimble_host_config();

    ble_svc_gap_device_name_set("ble-spp-client");

    nimble_port_freertos_init(nimble_host_task);

    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGW(TAG, "Free Heap: %" PRIu32 " bytes", esp_get_free_heap_size());

    ble_manager_callbacks_t manager_callbacks = {
        .ble_adv_cb = NULL,
        .ble_scan_cb = ble_manager_scan_cb,
        .ble_connect_cb = ble_manager_connect_cb,
        .ble_disconnect_cb = ble_manager_disconnect_cb,
        .ble_conn_update_cb = ble_manager_conn_update_cb,
        .arg = NULL
    };
    g_ble_manager = ble_manager_init(&manager_callbacks);
    if (!g_ble_manager) {
        ESP_LOGE(TAG, "Failed to initialize BLE manager");
        return;
    }
    g_sessions_mutex = xSemaphoreCreateMutex();
    if (!g_sessions_mutex) {
        ESP_LOGE(TAG, "Failed to create sessions mutex");
        return;
    }

    /* Start scanning for 10 seconds */
    ble_manager_start_scan(g_ble_manager, 100000);

    uint32_t heap_log_counter = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));

        for (int i = 0; i < MAX_CONNECTIONS; i++) {
            sessions_lock();
            gatt_session_t *session = g_session_instances[i];
            bool ready = g_conn_params_ready[i];
            if (!session || !ready || !gatt_session_send_idle(session)) {
                sessions_unlock();
                continue;
            }

            uint8_t *send_buffer = malloc(SPP_TEST_TRANSFER_SIZE);
            if (!send_buffer) {
                sessions_unlock();
                ESP_LOGE(TAG, "Failed to allocate send buffer");
                continue;
            }
            spp_fill_test_buffer(send_buffer, SPP_TEST_TRANSFER_SIZE);
            uint16_t conn_handle = session->conn_handle;
            ret = gatt_session_send(session, send_buffer, SPP_TEST_TRANSFER_SIZE, 0xFF, 80);
            sessions_unlock();
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Started send to conn_handle=%d (%d bytes)", conn_handle, SPP_TEST_TRANSFER_SIZE);
            } else {
                ESP_LOGW(TAG, "Failed to send to conn_handle=%d: %d", conn_handle, ret);
                free(send_buffer);
            }
        }

        if (++heap_log_counter >= 50) {
            heap_log_counter = 0;
            ESP_LOGW(TAG, "Free Heap: %" PRIu32 " bytes, active connections: %d/%d",
                     esp_get_free_heap_size(), ble_manager_get_active_connections(g_ble_manager), MAX_CONNECTIONS);
        }
    }
}
