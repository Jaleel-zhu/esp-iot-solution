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
#include "esp_random.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "ble_manager.h"
#include "gatt_discovery.h"
#include "flux_gatt_session.h"

static const char *TAG = "spp server";

/* 16 Bit SPP Service UUID */
#define BLE_SVC_SPP_UUID16      0xABF0
/* 16 Bit SPP Service Characteristic UUID */
#define BLE_SVC_SPP_CHR_UUID16  0xABF1

#define MAX_CONNECTIONS CONFIG_BLE_MANAGER_MAX_CONNECTIONS

#define SPP_TEST_TRANSFER_SIZE  (482 + 491 * 8) // 482 + 491 * 8 = 482 + 3928 = 4411

static uint16_t ble_spp_svc_gatt_read_val_handle = 0xFFFF;

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
        ESP_LOGW(TAG, "Session conn_handle %d stream %d completed successfully (%" PRIu32 " bytes)", session->conn_handle, stream_id, size);
    } else {
        ESP_LOGE(TAG, "Session conn_handle %d stream %d failed: %d", session->conn_handle, stream_id, status);
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
    ESP_LOGW(TAG, "Session %d stream %d received data, Data Size: %" PRIu32 " bytes (%.2f KB)", session->conn_handle, stream_id, size, size / 1024.0f);

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
    ESP_LOGI(TAG, "Session conn_handle %d progress: %u%% (%" PRIu32 "/%" PRIu32 " bytes)", session->conn_handle, progress, transferred, total);
}

static void ble_manager_adv_cb(ble_manager_t *manager, uint8_t type, void *arg)
{
    switch (type) {
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "Advertising complete");
        break;
    default:
        break;
    }
}

static void ble_manager_connect_cb(ble_manager_t *manager, uint8_t type, ble_connection_t *conn, int status, void *arg)
{
    switch (type) {
    case BLE_EVENT_TYPE_CONNECT_COMPLETE:
    case BLE_EVENT_TYPE_RECONNECT_COMPLETE:
        if (type == BLE_EVENT_TYPE_RECONNECT_COMPLETE) {
            ESP_LOGI(TAG, "Reconnected successfully to device %s (conn_handle=%d)",
                     addr_str(conn->peer_addr.val), conn->conn_handle);
        } else {
            ESP_LOGI(TAG, "Connected successfully to device %s (conn_handle=%d, first time)",
                     addr_str(conn->peer_addr.val), conn->conn_handle);

            // On first connection, auto-reconnect can be enabled
            // Example: reconnect every 2 seconds, unlimited reconnection (0 means unlimited)
            esp_err_t ret = ble_manager_enable_auto_reconnect(g_ble_manager, conn->conn_handle, 2000, 0);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Auto reconnect enabled for device %s", addr_str(conn->peer_addr.val));
            }
        }

        // As BLE slave (peripheral), service discovery is not needed
        // Directly use known characteristic value handle to create session
        if (ble_spp_svc_gatt_read_val_handle == 0xFFFF) {
            ESP_LOGW(TAG, "Characteristic handle not initialized yet, waiting for GATT service registration");
            return;
        }

        // Get MTU size (if negotiated)
        uint16_t mtu = BLE_ATT_MTU_DFLT; // Default MTU
        esp_err_t ret = ble_manager_get_mtu_by_conn(g_ble_manager, conn->conn_handle, &mtu);
        if (ret != ESP_OK || mtu < BLE_ATT_MTU_DFLT) {
            mtu = BLE_ATT_MTU_DFLT; // Use default MTU if not negotiated yet
        }

        // Set callback functions
        ble_session_callbacks_t callbacks = {
            .session_complete_cb = gatt_session_complete_cb,
            .data_received_cb = gatt_data_received_cb,
            .progress_cb = gatt_progress_cb,
            .error_cb = NULL
        };
        // Create GATT session (read and write use same handle, as BLE slave)
        gatt_session_t *session = gatt_session_create(conn->conn_handle, &callbacks,
                                                      ble_spp_svc_gatt_read_val_handle, ble_spp_svc_gatt_read_val_handle, mtu, GATT_SESSION_ROLE_SLAVE);
        if (session) {
            // Save session instance
            sessions_lock();
            for (int i = 0; i < MAX_CONNECTIONS; i++) {
                if (!g_session_instances[i]) {
                    g_session_instances[i] = session;
                    ESP_LOGI(TAG, "GATT session created for conn_handle=%d, val_handle=0x%04X, mtu=%d",
                             conn->conn_handle, ble_spp_svc_gatt_read_val_handle, mtu);
                    break;
                }
            }
            sessions_unlock();
        } else {
            ESP_LOGE(TAG, "Failed to create GATT session for conn_handle=%d", conn->conn_handle);
        }
        break;
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

/**
 * Enables advertising with the following parameters:
 *     o General discoverable mode.
 *     o Undirected connectable mode.
 */
static void
ble_spp_server_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    const char *name;
    int rc;
    esp_err_t ret;

    /**
     *  Set the advertisement data included in our advertisements:
     *     o Flags (indicates advertisement type and other general info).
     *     o Advertising tx power.
     *     o Device name.
     *     o 16-bit service UUIDs (alert notifications).
     */

    memset(&fields, 0, sizeof fields);

    /* Advertise two flags:
     *     o Discoverability in forthcoming advertisement (general)
     *     o BLE-only (BR/EDR unsupported).
     */
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    /* Indicate that the TX power level field should be included; have the
     * stack fill this value automatically.  This is done by assigning the
     * special value BLE_HS_ADV_TX_PWR_LVL_AUTO.
     */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    fields.uuids16 = (ble_uuid16_t[]) {
        BLE_UUID16_INIT(BLE_SVC_SPP_UUID16)
    };
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    /* Set advertisement data (application layer sets adv data) */
    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "error setting advertisement data; rc=%d", rc);
        return;
    }

    /* Configure advertisement parameters */
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    /* Start advertising using ble_manager API */
    ret = ble_manager_start_advertising(g_ble_manager, &adv_params, 0); // 0 = forever
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start advertising: %d", ret);
        return;
    }
}

/* Define new custom service */
static const struct ble_gatt_svc_def new_ble_svc_gatt_defs[] = {
    {
        /*** Service: SPP */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(BLE_SVC_SPP_UUID16),
        .characteristics = (struct ble_gatt_chr_def[])
        { {
                /* Support SPP service */
                .uuid = BLE_UUID16_DECLARE(BLE_SVC_SPP_CHR_UUID16),
                .access_cb = gatt_session_gatt_handler,
                .val_handle = &ble_spp_svc_gatt_read_val_handle,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
            }, {
                0, /* No more characteristics */
            }
        },
    },
    {
        0, /* No more services. */
    },
};

int gatt_svr_init(void)
{
    int rc = 0;
    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(new_ble_svc_gatt_defs);

    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(new_ble_svc_gatt_defs);
    if (rc != 0) {
        return rc;
    }

    return 0;
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

    /* Check if characteristic value handle is set */
    if (ble_spp_svc_gatt_read_val_handle != 0xFFFF) {
        ESP_LOGI(TAG, "GATT characteristic handle initialized: 0x%04X", ble_spp_svc_gatt_read_val_handle);
    } else {
        ESP_LOGW(TAG, "GATT characteristic handle not yet initialized (0x%04X)", ble_spp_svc_gatt_read_val_handle);
    }
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
    int rc = 0;
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

    ble_svc_gap_device_name_set("ble-spp-server");

    /* Initialize GATT server */
    rc = gatt_svr_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to initialize GATT server: %d", rc);
        return;
    }

    nimble_port_freertos_init(nimble_host_task);

    vTaskDelay(pdMS_TO_TICKS(3000));
    ESP_LOGW(TAG, "Free Heap: %" PRIu32 " bytes", esp_get_free_heap_size());

    ble_manager_callbacks_t manager_callbacks = {
        .ble_adv_cb = ble_manager_adv_cb,
        .ble_scan_cb = NULL,
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

    /* Start advertising (after BLE host sync) */
    ble_spp_server_advertise();

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
