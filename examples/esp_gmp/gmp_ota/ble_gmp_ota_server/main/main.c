/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <inttypes.h>
#include <stdlib.h>

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "ble_manager.h"
#include "flux_gatt_session.h"
#include "esp_gmp.h"
#include "esp_gmp_flux.h"
#include "esp_gmp_ota.h"
#include "gmp_ble_uuids.h"
#include "gmp_session_teardown.h"
#include "sdkconfig.h"

static const char *TAG = "gmp_ota_srv";

#define MAX_CONNECTIONS CONFIG_BLE_MANAGER_MAX_CONNECTIONS

static ble_manager_t *g_ble_manager;
static gatt_session_t *g_sessions[MAX_CONNECTIONS];
static gatt_session_t *g_teardown_pending[MAX_CONNECTIONS];
static portMUX_TYPE g_teardown_lock = portMUX_INITIALIZER_UNLOCKED;
static uint16_t g_chr_rx_handle;
static uint16_t g_chr_tx_handle;

static const ble_uuid128_t gmp_svc_uuid = GMP_BLE_UUID_SVC;
static const ble_uuid128_t gmp_chr_rx_uuid = GMP_BLE_UUID_CHR_RX;
static const ble_uuid128_t gmp_chr_tx_uuid = GMP_BLE_UUID_CHR_TX;

static int find_session_idx(uint16_t conn_handle)
{
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (g_sessions[i] && g_sessions[i]->conn_handle == conn_handle) {
            return i;
        }
    }
    return -1;
}

static void build_os_cap_report(uint8_t *cap, esp_gmp_link_t link)
{
    size_t max_payload = esp_gmp_max_payload_effective(link);
    if (max_payload > 0xFFFF) {
        max_payload = 0xFFFF;
    }

    cap[0] = 0x02;
    cap[1] = 0x02;
    cap[2] = 0x01;
    cap[3] = 0x02;
    cap[4] = 0x04;
    cap[5] = 0x00;
    cap[6] = (uint8_t)(max_payload >> 24);
    cap[7] = (uint8_t)(max_payload >> 16);
    cap[8] = (uint8_t)(max_payload >> 8);
    cap[9] = (uint8_t)max_payload;
    cap[10] = 0x01;
    cap[11] = 0x00;
}

static bool on_gmp_packet(void *ctx, const esp_gmp_rx_t *pkt)
{
    (void)ctx;

    if (pkt->group_id == ESP_GMP_GRP_OTA) {
        return esp_gmp_ota_on_packet(pkt);
    }

    if (pkt->op == ESP_GMP_OP_READ_REQ && pkt->group_id == ESP_GMP_GRP_OS &&
            pkt->command_id == ESP_GMP_OS_CAP_QUERY) {
        uint8_t cap[12];
        build_os_cap_report(cap, pkt->link);

        esp_gmp_tx_params_t tx = {
            .ver = ESP_GMP_VER,
            .op = ESP_GMP_OP_READ_RSP,
            .group_id = pkt->group_id,
            .sequence = pkt->sequence,
            .command_id = pkt->command_id,
            .flags = 0,
            .status = ESP_GMP_STATUS_OK,
        };
        esp_gmp_send(pkt->link, &tx, cap, sizeof(cap));
        return false;
    }

    if (pkt->op == ESP_GMP_OP_WRITE_REQ || pkt->op == ESP_GMP_OP_READ_REQ) {
        esp_gmp_tx_params_t tx = {
            .ver = ESP_GMP_VER,
            .op = (pkt->op == ESP_GMP_OP_WRITE_REQ) ? ESP_GMP_OP_WRITE_RSP : ESP_GMP_OP_READ_RSP,
            .group_id = pkt->group_id,
            .sequence = pkt->sequence,
            .command_id = pkt->command_id,
            .flags = 0,
            .status = ESP_GMP_STATUS_UNKNOWN_COMMAND,
        };
        esp_gmp_send(pkt->link, &tx, NULL, 0);
    }
    return false;
}

static esp_err_t setup_gmp_callbacks(gatt_session_t *session)
{
    ble_session_callbacks_t callbacks = { 0 };
    esp_err_t err = esp_gmp_flux_get_gatt_callbacks(session, &callbacks, NULL);
    if (err != ESP_OK) {
        return err;
    }
    session->callbacks = callbacks;
    return ESP_OK;
}

static esp_err_t setup_gmp_session(gatt_session_t *session)
{
    return esp_gmp_flux_link_register(session, session->flux_session);
}

static void teardown_gmp_session(gatt_session_t *session)
{
    esp_gmp_ota_on_link_down(session);
    esp_gmp_flux_link_unregister(session);
}

static void teardown_session(gatt_session_t *session)
{
    if (!session) {
        return;
    }

    if (gmp_session_teardown(session, teardown_gmp_session) != ESP_OK) {
        ESP_LOGE(TAG, "failed to schedule session destroy conn=%d", session->conn_handle);
    }
}

static bool teardown_pending_push(gatt_session_t *session)
{
    bool stored = false;

    portENTER_CRITICAL(&g_teardown_lock);
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (!g_teardown_pending[i]) {
            g_teardown_pending[i] = session;
            stored = true;
            break;
        }
    }
    portEXIT_CRITICAL(&g_teardown_lock);
    return stored;
}

static gatt_session_t *teardown_pending_pop(void)
{
    gatt_session_t *session = NULL;

    portENTER_CRITICAL(&g_teardown_lock);
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (g_teardown_pending[i]) {
            session = g_teardown_pending[i];
            g_teardown_pending[i] = NULL;
            break;
        }
    }
    portEXIT_CRITICAL(&g_teardown_lock);
    return session;
}

static void teardown_session_task(void *arg)
{
    teardown_session((gatt_session_t *)arg);
    vTaskDelete(NULL);
}

static void process_pending_teardowns(void)
{
    gatt_session_t *session = NULL;

    while ((session = teardown_pending_pop()) != NULL) {
        teardown_session(session);
    }
}

static void ble_manager_connect_cb(ble_manager_t *manager, uint8_t type, ble_connection_t *conn, int status, void *arg)
{
    (void)manager;
    (void)status;
    (void)arg;

    if (type != BLE_EVENT_TYPE_CONNECT_COMPLETE && type != BLE_EVENT_TYPE_RECONNECT_COMPLETE) {
        return;
    }

    if (g_chr_rx_handle == 0 || g_chr_tx_handle == 0) {
        ESP_LOGW(TAG, "GATT handles not ready");
        return;
    }

    uint16_t mtu = BLE_ATT_MTU_DFLT;
    ble_manager_get_mtu_by_conn(g_ble_manager, conn->conn_handle, &mtu);

    gatt_session_t *session = gatt_session_create(conn->conn_handle, NULL,
                                                  g_chr_tx_handle, g_chr_rx_handle,
                                                  mtu, GATT_SESSION_ROLE_SLAVE);
    if (!session) {
        ESP_LOGE(TAG, "gatt_session_create failed");
        return;
    }

    if (setup_gmp_callbacks(session) != ESP_OK) {
        ESP_LOGE(TAG, "setup GMP callbacks failed");
        gatt_session_destroy(session);
        ble_gap_terminate(conn->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        return;
    }

    bool stored = false;
    for (int i = 0; i < MAX_CONNECTIONS; i++) {
        if (!g_sessions[i]) {
            g_sessions[i] = session;
            esp_err_t err = setup_gmp_session(session);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "GMP Flux registration failed: %s", esp_err_to_name(err));
                g_sessions[i] = NULL;
                gatt_session_destroy(session);
                ble_gap_terminate(conn->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
                return;
            }
            stored = true;
            ESP_LOGI(TAG, "GMP session ready conn=%d mtu=%u", conn->conn_handle, (unsigned)mtu);
            break;
        }
    }
    if (!stored) {
        ESP_LOGW(TAG, "max connections (%d) reached, rejecting conn=%d",
                 MAX_CONNECTIONS, conn->conn_handle);
        gatt_session_destroy(session);
        ble_gap_terminate(conn->conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
}

static void ble_manager_disconnect_cb(ble_manager_t *manager, ble_connection_t *conn, int status, void *arg)
{
    (void)manager;
    (void)status;
    (void)arg;

    int idx = find_session_idx(conn->conn_handle);
    if (idx < 0) {
        return;
    }

    gatt_session_t *session = g_sessions[idx];
    g_sessions[idx] = NULL;
    if (xTaskCreate(teardown_session_task, "gmp_srv_teardown", 4096, session, 4, NULL) != pdPASS &&
            !teardown_pending_push(session)) {
        ESP_LOGE(TAG, "failed to schedule teardown for conn=%d", conn->conn_handle);
    }
    ESP_LOGI(TAG, "Disconnected conn=%d", conn->conn_handle);
}

static void start_advertising(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields adv_fields;
    struct ble_hs_adv_fields rsp_fields;
    const char *name = ble_svc_gap_device_name();
    int rc;

    memset(&adv_fields, 0, sizeof(adv_fields));
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv_fields.tx_pwr_lvl_is_present = 1;
    adv_fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    adv_fields.name = (uint8_t *)name;
    adv_fields.name_len = strlen(name);
    adv_fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&adv_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_set_fields rc=%d", rc);
        return;
    }

    memset(&rsp_fields, 0, sizeof(rsp_fields));
    rsp_fields.uuids128 = (ble_uuid128_t *)&gmp_svc_uuid;
    rsp_fields.num_uuids128 = 1;
    rsp_fields.uuids128_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_rsp_set_fields rc=%d", rc);
        return;
    }

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    esp_err_t ret = ble_manager_start_advertising(g_ble_manager, &adv_params, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "start_advertising failed: %s", esp_err_to_name(ret));
    }
}

static const struct ble_gatt_svc_def gmp_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gmp_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[])
        {
            {
                .uuid = &gmp_chr_rx_uuid.u,
                .access_cb = gatt_session_gatt_handler,
                .val_handle = &g_chr_rx_handle,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            }, {
                .uuid = &gmp_chr_tx_uuid.u,
                .access_cb = gatt_session_gatt_handler,
                .val_handle = &g_chr_tx_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 },
        },
    },
    { 0 },
};

static int gatt_svr_init(void)
{
    int rc;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gmp_gatt_svcs);
    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(gmp_gatt_svcs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}

static void nimble_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void ble_store_config_init(void);

static void nimble_host_on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    assert(rc == 0);
}

static void nimble_host_config(void)
{
    ble_hs_cfg.reset_cb = NULL;
    ble_hs_cfg.sync_cb = nimble_host_on_sync;
    ble_hs_cfg.store_status_cb = NULL;
    ble_store_config_init();
}

void app_main(void)
{
    esp_err_t ret;

    esp_log_level_set("NimBLE", ESP_LOG_WARN);
    esp_log_level_set("ble_manager", ESP_LOG_WARN);
    esp_log_level_set("gatt_session", ESP_LOG_WARN);
    esp_log_level_set("esp_flux", ESP_LOG_WARN);

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_gmp_init();
    esp_gmp_on_packet_register(on_gmp_packet, NULL);
    ESP_ERROR_CHECK(esp_gmp_ota_init(NULL));

    ESP_ERROR_CHECK(nimble_port_init());
    nimble_host_config();

    ble_svc_gap_device_name_set(GMP_BLE_DEVICE_NAME);

    int rc = gatt_svr_init();
    if (rc != 0) {
        ESP_LOGE(TAG, "gatt_svr_init rc=%d", rc);
        return;
    }

    nimble_port_freertos_init(nimble_host_task);

    ble_manager_callbacks_t mgr_cbs = {
        .ble_adv_cb = NULL,
        .ble_scan_cb = NULL,
        .ble_connect_cb = ble_manager_connect_cb,
        .ble_disconnect_cb = ble_manager_disconnect_cb,
        .ble_conn_update_cb = NULL,
        .arg = NULL,
    };
    g_ble_manager = ble_manager_init(&mgr_cbs);
    if (!g_ble_manager) {
        ESP_LOGE(TAG, "ble_manager_init failed");
        return;
    }

    while (!ble_hs_synced()) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    start_advertising();

    ESP_LOGI(TAG, "GMP OTA server ready (%s)", GMP_BLE_DEVICE_NAME);

    while (1) {
        esp_gmp_poll();
        process_pending_teardowns();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
