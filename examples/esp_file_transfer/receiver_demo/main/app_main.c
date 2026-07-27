/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <assert.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "host/ble_gap.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "ble_manager.h"
#include "esp_file_transfer.h"
#include "file_transfer_example_common.h"
#include "flux_gatt_session.h"

static const char *TAG = "ft_receiver_demo";

static ble_manager_t *s_manager;
static gatt_session_t *s_session;
static gatt_session_t *s_teardown_pending;
static portMUX_TYPE s_session_lock = portMUX_INITIALIZER_UNLOCKED;
static uint16_t s_rx_handle;
static uint16_t s_tx_handle;

static const ble_uuid128_t s_service_uuid =
    FILE_TRANSFER_EXAMPLE_SERVICE_UUID;
static const ble_uuid128_t s_rx_uuid = FILE_TRANSFER_EXAMPLE_RX_UUID;
static const ble_uuid128_t s_tx_uuid = FILE_TRANSFER_EXAMPLE_TX_UUID;

static void teardown_session(gatt_session_t *session)
{
    if (!session) {
        return;
    }
    file_transfer_example_link_down(session);
    esp_err_t err = gatt_session_schedule_destroy(session);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to schedule session destroy: %s",
                 esp_err_to_name(err));
    }
}

static void teardown_task(void *arg)
{
    teardown_session((gatt_session_t *)arg);
    vTaskDelete(NULL);
}

static void process_pending_teardown(void)
{
    gatt_session_t *session;
    portENTER_CRITICAL(&s_session_lock);
    session = s_teardown_pending;
    s_teardown_pending = NULL;
    portEXIT_CRITICAL(&s_session_lock);
    teardown_session(session);
}

static bool set_session_if_empty(gatt_session_t *session)
{
    bool stored = false;
    portENTER_CRITICAL(&s_session_lock);
    if (!s_session) {
        s_session = session;
        stored = true;
    }
    portEXIT_CRITICAL(&s_session_lock);
    return stored;
}

static void start_advertising(void)
{
    struct ble_hs_adv_fields fields = { 0 };
    struct ble_hs_adv_fields response = { 0 };
    struct ble_gap_adv_params params = { 0 };
    const char *name = ble_svc_gap_device_name();

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set advertising fields: rc=%d", rc);
        return;
    }

    response.uuids128 = (ble_uuid128_t *)&s_service_uuid;
    response.num_uuids128 = 1;
    response.uuids128_is_complete = 1;
    rc = ble_gap_adv_rsp_set_fields(&response);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set scan response: rc=%d", rc);
        return;
    }

    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    esp_err_t err = ble_manager_start_advertising(s_manager, &params, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start advertising: %s",
                 esp_err_to_name(err));
    }
}

static void on_connect(ble_manager_t *manager, uint8_t type,
                       ble_connection_t *connection, int status, void *arg)
{
    (void)manager;
    (void)status;
    (void)arg;

    if (type != BLE_EVENT_TYPE_CONNECT_COMPLETE &&
            type != BLE_EVENT_TYPE_RECONNECT_COMPLETE) {
        return;
    }
    if (!s_rx_handle || !s_tx_handle) {
        ESP_LOGE(TAG, "Data channel handles are not ready");
        ble_gap_terminate(connection->conn_handle,
                          BLE_ERR_REM_USER_CONN_TERM);
        return;
    }

    uint16_t mtu = BLE_ATT_MTU_DFLT;
    esp_err_t mtu_err = ble_manager_get_mtu_by_conn(
                            s_manager, connection->conn_handle, &mtu);
    if (mtu_err != ESP_OK || mtu < BLE_ATT_MTU_DFLT) {
        mtu = BLE_ATT_MTU_DFLT;
    }
    gatt_session_t *session = gatt_session_create(
                                  connection->conn_handle, NULL,
                                  s_tx_handle, s_rx_handle, mtu,
                                  GATT_SESSION_ROLE_SLAVE);
    if (!session) {
        ESP_LOGE(TAG, "Failed to create link session");
        ble_gap_terminate(connection->conn_handle,
                          BLE_ERR_REM_USER_CONN_TERM);
        return;
    }

    ble_session_callbacks_t callbacks;
    esp_err_t err = file_transfer_example_get_session_callbacks(
                        session, &callbacks);
    if (err == ESP_OK) {
        session->callbacks = callbacks;
        err = file_transfer_example_link_up(session);
    }
    if (err != ESP_OK || !set_session_if_empty(session)) {
        ESP_LOGE(TAG, "Failed to activate file transfer link: %s",
                 esp_err_to_name(err != ESP_OK ? err : ESP_FT_ERR_BUSY));
        file_transfer_example_link_down(session);
        gatt_session_schedule_destroy(session);
        ble_gap_terminate(connection->conn_handle,
                          BLE_ERR_REM_USER_CONN_TERM);
        return;
    }

    ESP_LOGI(TAG, "Receiver link ready, conn=%u, mtu=%u",
             connection->conn_handle, mtu);
}

static void on_disconnect(ble_manager_t *manager,
                          ble_connection_t *connection,
                          int status, void *arg)
{
    (void)manager;
    (void)status;
    (void)arg;

    gatt_session_t *session = NULL;
    portENTER_CRITICAL(&s_session_lock);
    if (s_session &&
            s_session->conn_handle == connection->conn_handle) {
        session = s_session;
        s_session = NULL;
    }
    portEXIT_CRITICAL(&s_session_lock);

    if (session &&
            xTaskCreate(teardown_task, "ft_link_down", 4096,
                        session, 4, NULL) != pdPASS) {
        portENTER_CRITICAL(&s_session_lock);
        s_teardown_pending = session;
        portEXIT_CRITICAL(&s_session_lock);
    }
    ESP_LOGI(TAG, "Disconnected, conn=%u", connection->conn_handle);
    start_advertising();
}

static const struct ble_gatt_svc_def s_services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[])
        {
            {
                .uuid = &s_rx_uuid.u,
                .access_cb = gatt_session_gatt_handler,
                .val_handle = &s_rx_handle,
                .flags = BLE_GATT_CHR_F_WRITE |
                         BLE_GATT_CHR_F_WRITE_NO_RSP,
            }, {
                .uuid = &s_tx_uuid.u,
                .access_cb = gatt_session_gatt_handler,
                .val_handle = &s_tx_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 },
        },
    },
    { 0 },
};

static int gatt_server_init(void)
{
    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(s_services);
    if (rc == 0) {
        rc = ble_gatts_add_svcs(s_services);
    }
    return rc;
}

static void host_task(void *arg)
{
    (void)arg;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void ble_store_config_init(void);

static void host_on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);
}

static void configure_host(void)
{
    ble_hs_cfg.reset_cb = NULL;
    ble_hs_cfg.sync_cb = host_on_sync;
    ble_hs_cfg.store_status_cb = NULL;
    ble_store_config_init();
}

static void init_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
            err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

void app_main(void)
{
    init_nvs();
    ESP_ERROR_CHECK(file_transfer_example_init());
    ESP_ERROR_CHECK(nimble_port_init());
    configure_host();
    ESP_ERROR_CHECK(ble_svc_gap_device_name_set(
                        FILE_TRANSFER_EXAMPLE_DEVICE_NAME));

    int rc = gatt_server_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "GATT server initialization failed: rc=%d", rc);
        return;
    }
    nimble_port_freertos_init(host_task);

    const ble_manager_callbacks_t callbacks = {
        .ble_adv_cb = NULL,
        .ble_scan_cb = NULL,
        .ble_connect_cb = on_connect,
        .ble_disconnect_cb = on_disconnect,
        .ble_conn_update_cb = NULL,
        .arg = NULL,
    };
    s_manager = ble_manager_init(&callbacks);
    if (!s_manager) {
        ESP_LOGE(TAG, "Link manager initialization failed");
        return;
    }

    while (!ble_hs_synced()) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    start_advertising();
    ESP_LOGI(TAG, "Receiver demo ready");

    while (true) {
        file_transfer_example_poll();
        process_pending_teardown();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
