/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "gmp_ota_client.h"
#include "esp_gmp.h"
#include "esp_gmp_ota_proto.h"
#include "esp_gmp_sha256.h"
#include "esp_log.h"
#include <inttypes.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/portmacro.h"
#include <string.h>

static const char *TAG = "gmp_ota_client";

/** Max in-flight OTA_UPLOAD_DATA GMP messages (matches Flux / esp_gmp TX concurrency). */
#ifndef GMP_OTA_CLIENT_PIPE_DEPTH
#define GMP_OTA_CLIENT_PIPE_DEPTH 2
#endif

#define GMP_RSP_TIMEOUT_MS 40000
#define GMP_DATA_BUSY_MAX_RETRIES 5
#define GMP_CTRL_BUSY_MAX_RETRIES 5

typedef struct {
    uint16_t seq;
    esp_gmp_link_t link;
    esp_gmp_tx_params_t tx;
    uint8_t *payload;
    size_t payload_len;
    uint8_t status;
    uint8_t retries;
    bool active;
    bool done;
} gmp_pipe_slot_t;

static SemaphoreHandle_t s_rsp_sem;
static SemaphoreHandle_t s_pipe_sem;

static bool s_ctrl_wait_active;
static uint16_t s_ctrl_wait_seq;
static uint8_t s_rsp_status;
static uint8_t s_rsp_payload[64];
static size_t s_rsp_payload_len;

static gmp_pipe_slot_t s_pipe[GMP_OTA_CLIENT_PIPE_DEPTH];
static portMUX_TYPE s_ctrl_lock = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_pipe_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_upload_cancel;

static uint16_t rd_be16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] << 8 | p[1]);
}

static uint32_t rd_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

static void wr_be16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}

static void wr_be32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

static void pipe_reset(void)
{
    portENTER_CRITICAL(&s_pipe_lock);
    for (int i = 0; i < GMP_OTA_CLIENT_PIPE_DEPTH; i++) {
        if (s_pipe[i].payload) {
            free(s_pipe[i].payload);
        }
    }
    memset(s_pipe, 0, sizeof(s_pipe));
    portEXIT_CRITICAL(&s_pipe_lock);
    if (s_pipe_sem) {
        while (xSemaphoreTake(s_pipe_sem, 0) == pdTRUE) {
        }
    }
}

static int pipe_in_flight(void)
{
    int count = 0;

    portENTER_CRITICAL(&s_pipe_lock);
    for (int i = 0; i < GMP_OTA_CLIENT_PIPE_DEPTH; i++) {
        if (s_pipe[i].active && !s_pipe[i].done) {
            count++;
        }
    }
    portEXIT_CRITICAL(&s_pipe_lock);
    return count;
}

static esp_err_t pipe_track(esp_gmp_link_t link, const esp_gmp_tx_params_t *tx, const uint8_t *pl, size_t pl_len)
{
    uint8_t *copy = malloc(pl_len);
    if (!copy) {
        return ESP_ERR_NO_MEM;
    }
    memcpy(copy, pl, pl_len);

    portENTER_CRITICAL(&s_pipe_lock);
    for (int i = 0; i < GMP_OTA_CLIENT_PIPE_DEPTH; i++) {
        if (!s_pipe[i].active) {
            s_pipe[i].seq = tx->sequence;
            s_pipe[i].link = link;
            s_pipe[i].tx = *tx;
            s_pipe[i].payload = copy;
            s_pipe[i].payload_len = pl_len;
            s_pipe[i].status = ESP_GMP_STATUS_OK;
            s_pipe[i].active = true;
            s_pipe[i].done = false;
            portEXIT_CRITICAL(&s_pipe_lock);
            return ESP_OK;
        }
    }
    portEXIT_CRITICAL(&s_pipe_lock);
    free(copy);
    return ESP_ERR_NO_MEM;
}

static void pipe_clear_slot_locked(int i)
{
    if (s_pipe[i].payload) {
        free(s_pipe[i].payload);
    }
    memset(&s_pipe[i], 0, sizeof(s_pipe[i]));
}

static void pipe_on_response(uint16_t seq, uint8_t status)
{
    bool signaled = false;

    portENTER_CRITICAL(&s_pipe_lock);
    for (int i = 0; i < GMP_OTA_CLIENT_PIPE_DEPTH; i++) {
        if (s_pipe[i].active && !s_pipe[i].done && s_pipe[i].seq == seq) {
            s_pipe[i].status = status;
            s_pipe[i].done = true;
            signaled = true;
            break;
        }
    }
    portEXIT_CRITICAL(&s_pipe_lock);

    if (signaled && s_pipe_sem) {
        xSemaphoreGive(s_pipe_sem);
    }
}

static esp_err_t pipe_wait_one(uint8_t *out_status)
{
    while (true) {
        if (s_upload_cancel) {
            return ESP_ERR_INVALID_STATE;
        }
        if (xSemaphoreTake(s_pipe_sem, pdMS_TO_TICKS(GMP_RSP_TIMEOUT_MS)) != pdTRUE) {
            ESP_LOGE(TAG, "DATA pipeline response timeout (in_flight=%d)", pipe_in_flight());
            return ESP_ERR_TIMEOUT;
        }

        portENTER_CRITICAL(&s_pipe_lock);
        for (int i = 0; i < GMP_OTA_CLIENT_PIPE_DEPTH; i++) {
            if (s_pipe[i].active && s_pipe[i].done) {
                uint8_t status = s_pipe[i].status;
                if (status == ESP_GMP_STATUS_BUSY &&
                        s_pipe[i].retries < GMP_DATA_BUSY_MAX_RETRIES) {
                    esp_gmp_tx_params_t tx = s_pipe[i].tx;
                    esp_gmp_link_t link = s_pipe[i].link;
                    uint8_t *payload = s_pipe[i].payload;
                    size_t payload_len = s_pipe[i].payload_len;
                    uint8_t retries = s_pipe[i].retries + 1;
                    s_pipe[i].done = false;
                    s_pipe[i].retries = retries;
                    portEXIT_CRITICAL(&s_pipe_lock);
                    vTaskDelay(pdMS_TO_TICKS(20U * retries));
                    esp_err_t err = esp_gmp_send(link, &tx, payload, payload_len);
                    if (err != ESP_OK) {
                        return err;
                    }
                    continue;
                }
                pipe_clear_slot_locked(i);
                portEXIT_CRITICAL(&s_pipe_lock);
                if (out_status) {
                    *out_status = status;
                }
                if (status != ESP_GMP_STATUS_OK) {
                    ESP_LOGE(TAG, "pipelined DATA status=%02x", status);
                    return ESP_FAIL;
                }
                return ESP_OK;
            }
        }
        portEXIT_CRITICAL(&s_pipe_lock);
    }
}

static bool gmp_client_on_packet(void *ctx, const esp_gmp_rx_t *p)
{
    (void)ctx;
    bool ctrl_signaled = false;

    if (p->op != ESP_GMP_OP_WRITE_RSP && p->op != ESP_GMP_OP_READ_RSP) {
        return false;
    }

    portENTER_CRITICAL(&s_ctrl_lock);
    if (s_ctrl_wait_active && p->sequence == s_ctrl_wait_seq) {
        s_rsp_status = p->status;
        s_rsp_payload_len = p->payload_len < sizeof(s_rsp_payload) ? p->payload_len : sizeof(s_rsp_payload);
        if (s_rsp_payload_len > 0) {
            memcpy(s_rsp_payload, p->payload, s_rsp_payload_len);
        }
        s_ctrl_wait_active = false;
        ctrl_signaled = true;
    }
    portEXIT_CRITICAL(&s_ctrl_lock);

    if (ctrl_signaled) {
        xSemaphoreGive(s_rsp_sem);
        return false;
    }

    pipe_on_response(p->sequence, p->status);
    return false;
}

static void ctrl_wait_begin(uint16_t seq)
{
    while (xSemaphoreTake(s_rsp_sem, 0) == pdTRUE) {
    }

    portENTER_CRITICAL(&s_ctrl_lock);
    s_ctrl_wait_active = true;
    s_ctrl_wait_seq = seq;
    s_rsp_status = ESP_GMP_STATUS_OK;
    s_rsp_payload_len = 0;
    portEXIT_CRITICAL(&s_ctrl_lock);
}

static void ctrl_wait_cancel(void)
{
    portENTER_CRITICAL(&s_ctrl_lock);
    s_ctrl_wait_active = false;
    portEXIT_CRITICAL(&s_ctrl_lock);
}

static esp_err_t wait_ctrl_rsp(uint16_t seq, uint32_t timeout_ms)
{
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    while (xTaskGetTickCount() < deadline) {
        if (s_upload_cancel) {
            ctrl_wait_cancel();
            return ESP_ERR_INVALID_STATE;
        }
        TickType_t remain = deadline - xTaskGetTickCount();
        if (xSemaphoreTake(s_rsp_sem, remain) == pdTRUE) {
            if (s_rsp_status != ESP_GMP_STATUS_OK) {
                ESP_LOGE(TAG, "GMP status=%02x seq=%u", s_rsp_status, (unsigned)seq);
                return ESP_FAIL;
            }
            return ESP_OK;
        }
    }
    ctrl_wait_cancel();
    ESP_LOGE(TAG, "control response timeout seq=%u", (unsigned)seq);
    return ESP_ERR_TIMEOUT;
}

static esp_err_t gmp_send_and_wait(esp_gmp_link_t link, esp_gmp_tx_params_t *tx, const uint8_t *pl, size_t pl_len)
{
    for (uint8_t retries = 0; retries <= GMP_CTRL_BUSY_MAX_RETRIES; retries++) {
        ctrl_wait_begin(tx->sequence);
        esp_err_t err = esp_gmp_send(link, tx, pl, pl_len);
        if (err != ESP_OK) {
            ctrl_wait_cancel();
            return err;
        }

        err = wait_ctrl_rsp(tx->sequence, GMP_RSP_TIMEOUT_MS);
        if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
            return err;
        }
        if (s_rsp_status != ESP_GMP_STATUS_BUSY || retries == GMP_CTRL_BUSY_MAX_RETRIES) {
            return err;
        }

        vTaskDelay(pdMS_TO_TICKS(20U * (retries + 1U)));
    }

    return ESP_FAIL;
}

static esp_err_t gmp_send_data_pipelined(esp_gmp_link_t link, esp_gmp_tx_params_t *tx, const uint8_t *pl, size_t pl_len)
{
    esp_err_t err = pipe_track(link, tx, pl, pl_len);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_gmp_send(link, tx, pl, pl_len);
    if (err != ESP_OK) {
        portENTER_CRITICAL(&s_pipe_lock);
        for (int i = 0; i < GMP_OTA_CLIENT_PIPE_DEPTH; i++) {
            if (s_pipe[i].active && s_pipe[i].seq == tx->sequence) {
                pipe_clear_slot_locked(i);
                break;
            }
        }
        portEXIT_CRITICAL(&s_pipe_lock);
        return err;
    }

    esp_gmp_poll();
    return ESP_OK;
}

static void send_abort_best_effort(esp_gmp_link_t link, uint16_t *seq, uint8_t session_id)
{
    uint8_t abort_pl[ESP_GMP_OTA_CTRL_ABORT_ERASE_LEN];

    abort_pl[0] = ESP_GMP_OTA_CTRL_ACTION_ABORT;
    abort_pl[1] = session_id;
    wr_be16(&abort_pl[2], 0);
    wr_be32(&abort_pl[4], 0);

    (*seq)++;
    esp_gmp_tx_params_t tx = {
        .ver = ESP_GMP_VER,
        .op = ESP_GMP_OP_WRITE_REQ,
        .group_id = ESP_GMP_GRP_OTA,
        .sequence = *seq,
        .command_id = ESP_GMP_OTA_UPLOAD_CONTROL,
        .flags = 0,
        .status = 0,
    };
    (void)esp_gmp_send(link, &tx, abort_pl, sizeof(abort_pl));
    esp_gmp_poll();
}

static esp_err_t upload_fail_cleanup(esp_gmp_link_t link, uint16_t *seq, uint8_t session_id,
                                     esp_gmp_sha256_ctx_t *sha, bool sha_started, uint8_t *data_pl,
                                     esp_err_t err)
{
    send_abort_best_effort(link, seq, session_id);
    if (sha_started && sha) {
        esp_gmp_sha256_abort(sha);
    }
    if (data_pl) {
        free(data_pl);
    }
    pipe_reset();
    return err;
}

void gmp_ota_client_install_handler(void)
{
    if (!s_rsp_sem) {
        s_rsp_sem = xSemaphoreCreateBinary();
    }
    if (!s_pipe_sem) {
        s_pipe_sem = xSemaphoreCreateCounting(GMP_OTA_CLIENT_PIPE_DEPTH, 0);
    }
    pipe_reset();
    s_upload_cancel = false;
    esp_gmp_on_packet_register(gmp_client_on_packet, NULL);
}

void gmp_ota_client_request_cancel(void)
{
    s_upload_cancel = true;
}

void gmp_ota_client_clear_cancel(void)
{
    s_upload_cancel = false;
}

esp_err_t gmp_ota_client_query_caps(esp_gmp_link_t link, gmp_ota_client_caps_t *out)
{
    if (!link || !out) {
        return ESP_ERR_INVALID_ARG;
    }

    static uint16_t seq;
    seq++;

    esp_gmp_tx_params_t tx = {
        .ver = ESP_GMP_VER,
        .op = ESP_GMP_OP_READ_REQ,
        .group_id = ESP_GMP_GRP_OS,
        .sequence = seq,
        .command_id = ESP_GMP_OS_CAP_QUERY,
        .flags = 0,
        .status = 0,
    };

    esp_err_t err = gmp_send_and_wait(link, &tx, NULL, 0);
    if (err != ESP_OK) {
        return err;
    }

    if (s_rsp_payload_len < 12) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    out->link = link;
    out->max_gmp_payload = rd_be32(&s_rsp_payload[6]);
    out->chunk_size = esp_gmp_ota_chunk_for_flash(out->max_gmp_payload);
    return ESP_OK;
}

static esp_err_t mem_read_fn(void *ctx, size_t offset, uint8_t *buf, size_t len, size_t *out_len)
{
    const uint8_t *image = ctx;

    memcpy(buf, image + offset, len);
    *out_len = len;
    return ESP_OK;
}

esp_err_t gmp_ota_client_upload_stream(esp_gmp_link_t link, size_t image_len, gmp_ota_client_read_fn_t read_fn, void *read_ctx)
{
    if (!link || !read_fn || image_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    pipe_reset();
    gmp_ota_client_clear_cancel();

    gmp_ota_client_caps_t caps;
    esp_err_t err = gmp_ota_client_query_caps(link, &caps);
    if (err != ESP_OK) {
        return err;
    }

    static uint16_t seq;
    uint16_t chunk = caps.chunk_size;
    if (chunk == 0) {
        chunk = ESP_GMP_OTA_FLASH_WRITE_ALIGN;
    }

    seq++;
    uint8_t start_pl[ESP_GMP_OTA_CTRL_START_LEN];
    start_pl[0] = ESP_GMP_OTA_CTRL_ACTION_START;
    start_pl[1] = 0;
    wr_be16(&start_pl[2], 0);
    wr_be32(&start_pl[4], (uint32_t)image_len);

    esp_gmp_tx_params_t tx = {
        .ver = ESP_GMP_VER,
        .op = ESP_GMP_OP_WRITE_REQ,
        .group_id = ESP_GMP_GRP_OTA,
        .sequence = seq,
        .command_id = ESP_GMP_OTA_UPLOAD_CONTROL,
        .flags = 0,
        .status = 0,
    };

    err = gmp_send_and_wait(link, &tx, start_pl, sizeof(start_pl));
    if (err != ESP_OK) {
        return err;
    }

    if (s_rsp_payload_len < ESP_GMP_OTA_CTRL_RSP_LEN) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    uint8_t session_id = s_rsp_payload[0];
    uint16_t hint = rd_be16(&s_rsp_payload[1]);
    if (hint > 0) {
        hint = esp_gmp_ota_align_chunk(hint);
        if (hint < chunk) {
            chunk = hint;
        }
    }
    chunk = esp_gmp_ota_align_chunk(chunk);

    uint8_t *data_pl = malloc(ESP_GMP_OTA_DATA_HDR_LEN + chunk);
    if (!data_pl) {
        return upload_fail_cleanup(link, &seq, session_id, NULL, false, NULL, ESP_ERR_NO_MEM);
    }

    esp_gmp_sha256_ctx_t sha;
    err = esp_gmp_sha256_begin(&sha);
    if (err != ESP_OK) {
        return upload_fail_cleanup(link, &seq, session_id, NULL, false, data_pl, err);
    }

    size_t send_offset = 0;
    size_t chunks_acked = 0;
    int in_flight = 0;

    while (send_offset < image_len || in_flight > 0) {
        if (s_upload_cancel) {
            return upload_fail_cleanup(link, &seq, session_id, &sha, true, data_pl, ESP_ERR_INVALID_STATE);
        }
        while (send_offset < image_len && in_flight < GMP_OTA_CLIENT_PIPE_DEPTH) {
            size_t data_len = image_len - send_offset;
            if (data_len > chunk) {
                data_len = chunk;
            }

            size_t got = 0;
            err = read_fn(read_ctx, send_offset, &data_pl[ESP_GMP_OTA_DATA_HDR_LEN], data_len, &got);
            if (err != ESP_OK || got != data_len) {
                ESP_LOGE(TAG, "read failed at offset=%zu want=%zu got=%zu",
                         send_offset, data_len, got);
                return upload_fail_cleanup(link, &seq, session_id, &sha, true, data_pl,
                                           err != ESP_OK ? err : ESP_FAIL);
            }

            err = esp_gmp_sha256_update(&sha, &data_pl[ESP_GMP_OTA_DATA_HDR_LEN], data_len);
            if (err != ESP_OK) {
                return upload_fail_cleanup(link, &seq, session_id, &sha, true, data_pl, err);
            }

            data_pl[0] = session_id;
            data_pl[1] = (send_offset + data_len >= image_len) ? ESP_GMP_OTA_DATA_FLAG_LAST_CHUNK : 0;
            wr_be16(&data_pl[2], (uint16_t)data_len);
            wr_be32(&data_pl[4], (uint32_t)send_offset);

            seq++;
            tx.sequence = seq;
            tx.command_id = ESP_GMP_OTA_UPLOAD_DATA;
            err = gmp_send_data_pipelined(link, &tx, data_pl, ESP_GMP_OTA_DATA_HDR_LEN + data_len);
            if (err != ESP_OK) {
                return upload_fail_cleanup(link, &seq, session_id, &sha, true, data_pl, err);
            }

            send_offset += data_len;
            in_flight++;
        }

        err = pipe_wait_one(NULL);
        if (err != ESP_OK) {
            return upload_fail_cleanup(link, &seq, session_id, &sha, true, data_pl, err);
        }

        in_flight--;
        chunks_acked++;
        if ((chunks_acked % 16) == 0 || (send_offset == image_len && in_flight == 0)) {
            ESP_LOGI(TAG, "upload pipelined %zu/%zu chunks (in_flight=%d)",
                     chunks_acked, (image_len + chunk - 1) / chunk, in_flight);
        }

        esp_gmp_poll();
    }

    free(data_pl);
    pipe_reset();

    uint8_t digest[32];
    err = esp_gmp_sha256_finish(&sha, digest);
    if (err != ESP_OK) {
        return upload_fail_cleanup(link, &seq, session_id, &sha, true, NULL, err);
    }

    uint8_t finish_pl[ESP_GMP_OTA_CTRL_FINISH_LEN];
    finish_pl[0] = ESP_GMP_OTA_CTRL_ACTION_FINISH;
    finish_pl[1] = session_id;
    wr_be16(&finish_pl[2], ESP_GMP_OTA_CTRL_FLAG_SHA256);
    wr_be32(&finish_pl[4], (uint32_t)image_len);
    memcpy(&finish_pl[8], digest, 32);

    seq++;
    tx.sequence = seq;
    tx.command_id = ESP_GMP_OTA_UPLOAD_CONTROL;
    err = gmp_send_and_wait(link, &tx, finish_pl, sizeof(finish_pl));
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "OTA finish OK — device should reboot");
        return ESP_OK;
    }
    return upload_fail_cleanup(link, &seq, session_id, NULL, false, NULL, err);
}

esp_err_t gmp_ota_client_upload(esp_gmp_link_t link, const uint8_t *image, size_t image_len)
{
    if (!image) {
        return ESP_ERR_INVALID_ARG;
    }

    return gmp_ota_client_upload_stream(link, image_len, mem_read_fn, (void *)image);
}
