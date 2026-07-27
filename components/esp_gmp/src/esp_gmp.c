/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_gmp.h"
#include "esp_gmp_frame.h"
#include "esp_gmp_types.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "esp_gmp";

uint16_t esp_gmp_crc16_ccitt_false(const uint8_t *data, size_t len);

typedef struct {
    uint8_t *frame;
    size_t frame_len;
} tx_item_t;

typedef struct {
    esp_gmp_link_t key;
    const esp_gmp_transport_t *ops;
    void *transport_ctx;
    SemaphoreHandle_t mutex;
    bool closing;
    tx_item_t txq[CONFIG_ESP_GMP_TX_QUEUE_DEPTH];
    size_t txq_head;
    size_t txq_count;
    uint8_t *in_flight[CONFIG_ESP_GMP_TX_QUEUE_DEPTH];
} link_entry_t;

typedef struct {
    esp_gmp_link_t link;
    const esp_gmp_transport_t *ops;
    void *transport_ctx;
    uint8_t *frame;
    size_t frame_len;
} tx_send_op_t;

static link_entry_t s_links[CONFIG_ESP_GMP_MAX_LINKS];
static SemaphoreHandle_t s_links_mutex;
static esp_gmp_on_packet_fn s_cb;
static void *s_cb_ctx;
static bool s_inited;

static bool is_req_op(uint8_t op)
{
    return op == ESP_GMP_OP_READ_REQ || op == ESP_GMP_OP_WRITE_REQ;
}

static bool is_rsp_op(uint8_t op)
{
    return op == ESP_GMP_OP_READ_RSP || op == ESP_GMP_OP_WRITE_RSP;
}

static uint8_t rsp_op(uint8_t req_op)
{
    if (req_op == ESP_GMP_OP_READ_REQ) {
        return ESP_GMP_OP_READ_RSP;
    }
    if (req_op == ESP_GMP_OP_WRITE_REQ) {
        return ESP_GMP_OP_WRITE_RSP;
    }
    return ESP_GMP_OP_READ_RSP;
}

static link_entry_t *link_find(esp_gmp_link_t k)
{
    if (k == NULL) {
        return NULL;
    }
    for (int i = 0; i < CONFIG_ESP_GMP_MAX_LINKS; i++) {
        if (s_links[i].key == k) {
            return &s_links[i];
        }
    }
    return NULL;
}

static link_entry_t *link_alloc(esp_gmp_link_t k)
{
    for (int i = 0; i < CONFIG_ESP_GMP_MAX_LINKS; i++) {
        if (s_links[i].key == NULL) {
            SemaphoreHandle_t existing_mutex = s_links[i].mutex;
            memset(&s_links[i], 0, sizeof(s_links[i]));
            s_links[i].key = k;
            s_links[i].mutex = existing_mutex ? existing_mutex : xSemaphoreCreateMutex();
            if (!s_links[i].mutex) {
                s_links[i].key = NULL;
                return NULL;
            }
            return &s_links[i];
        }
    }
    return NULL;
}

static void tx_item_clear(tx_item_t *item)
{
    if (item->frame) {
        free(item->frame);
        item->frame = NULL;
    }
    item->frame_len = 0;
}

static void link_drain_tx_queue(link_entry_t *L);

/* Caller must hold L->mutex. */
static size_t max_payload_effective_locked(const link_entry_t *L)
{
    if (!L || !L->ops || !L->ops->max_payload) {
        return 0;
    }

    size_t cap = (size_t)CONFIG_ESP_GMP_MAX_PAYLOAD;
    size_t by_transport = L->ops->max_payload(L->transport_ctx);
    if (by_transport > 0 && by_transport < cap) {
        cap = by_transport;
    }
    return cap;
}

size_t esp_gmp_max_payload_effective(esp_gmp_link_t link)
{
    link_entry_t *L = link_find(link);
    if (!L || L->key != link || !L->mutex) {
        return 0;
    }
    if (xSemaphoreTake(L->mutex, portMAX_DELAY) != pdTRUE) {
        return 0;
    }

    size_t cap = 0;
    if (L->key == link) {
        cap = max_payload_effective_locked(L);
    }
    xSemaphoreGive(L->mutex);
    return cap;
}

static bool in_flight_track(link_entry_t *L, uint8_t *frame)
{
    for (int i = 0; i < CONFIG_ESP_GMP_TX_QUEUE_DEPTH; i++) {
        if (L->in_flight[i] == NULL) {
            L->in_flight[i] = frame;
            return true;
        }
    }
    return false;
}

static bool in_flight_forget(link_entry_t *L, const uint8_t *frame, bool free_frame)
{
    if (!frame) {
        return false;
    }
    for (int i = 0; i < CONFIG_ESP_GMP_TX_QUEUE_DEPTH; i++) {
        if (L->in_flight[i] == frame) {
            if (free_frame) {
                free(L->in_flight[i]);
            }
            L->in_flight[i] = NULL;
            return true;
        }
    }
    return false;
}

static void in_flight_release(link_entry_t *L, const uint8_t *frame)
{
    (void)in_flight_forget(L, frame, true);
}

static bool in_flight_any(const link_entry_t *L)
{
    for (int i = 0; i < CONFIG_ESP_GMP_TX_QUEUE_DEPTH; i++) {
        if (L->in_flight[i]) {
            return true;
        }
    }
    return false;
}

static bool link_any_active(void)
{
    for (int i = 0; i < CONFIG_ESP_GMP_MAX_LINKS; i++) {
        if (s_links[i].key != NULL) {
            return true;
        }
    }
    return false;
}

static void link_clear_if_idle_locked(link_entry_t *L)
{
    if (!L->closing || in_flight_any(L)) {
        return;
    }
    L->key = NULL;
    L->ops = NULL;
    L->transport_ctx = NULL;
    L->closing = false;
}

static bool link_can_send(const link_entry_t *L)
{
    return !L->ops->can_send || L->ops->can_send(L->transport_ctx);
}

static esp_err_t link_queue_frame_locked(link_entry_t *L, uint8_t *frame, size_t frame_len)
{
    if (L->txq_count >= CONFIG_ESP_GMP_TX_QUEUE_DEPTH) {
        free(frame);
        return ESP_ERR_INVALID_STATE;
    }

    size_t tail = (L->txq_head + L->txq_count) % CONFIG_ESP_GMP_TX_QUEUE_DEPTH;
    L->txq[tail].frame = frame;
    L->txq[tail].frame_len = frame_len;
    L->txq_count++;
    return ESP_OK;
}

static bool link_fill_send_op_locked(link_entry_t *L, uint8_t *frame, size_t frame_len, tx_send_op_t *op)
{
    if (!in_flight_track(L, frame)) {
        return false;
    }

    op->link = L->key;
    op->ops = L->ops;
    op->transport_ctx = L->transport_ctx;
    op->frame = frame;
    op->frame_len = frame_len;
    return true;
}

static esp_err_t link_tx_submit_locked(link_entry_t *L, uint8_t *frame, size_t frame_len, tx_send_op_t *op)
{
    if (!L->ops || !L->ops->send || L->closing || !frame || frame_len == 0) {
        free(frame);
        return ESP_ERR_INVALID_ARG;
    }

    if (!link_can_send(L)) {
        return link_queue_frame_locked(L, frame, frame_len);
    }

    if (link_fill_send_op_locked(L, frame, frame_len, op)) {
        return ESP_OK;
    }
    return link_queue_frame_locked(L, frame, frame_len);
}

static void link_drop_tx_head(link_entry_t *L)
{
    tx_item_t *item = &L->txq[L->txq_head];
    item->frame = NULL;
    item->frame_len = 0;
    L->txq_head = (L->txq_head + 1) % CONFIG_ESP_GMP_TX_QUEUE_DEPTH;
    L->txq_count--;
}

static void link_finish_failed_send(esp_gmp_link_t link, uint8_t *frame, size_t frame_len, esp_err_t err)
{
    link_entry_t *L = link_find(link);
    /* Link gone or mutex unavailable: unregister/tx_done owns or owned the frame. */
    if (!L || !L->mutex || xSemaphoreTake(L->mutex, portMAX_DELAY) != pdTRUE) {
        return;
    }

    (void)in_flight_forget(L, frame, false);
    if (err == ESP_ERR_INVALID_STATE && L->key == link && L->ops && !L->closing) {
        (void)link_queue_frame_locked(L, frame, frame_len);
    } else {
        free(frame);
        link_clear_if_idle_locked(L);
    }
    xSemaphoreGive(L->mutex);
}

static void tx_send_op_run(const tx_send_op_t *op)
{
    if (!op->ops || !op->ops->send || !op->frame || op->frame_len == 0) {
        return;
    }
    esp_err_t err = op->ops->send(op->transport_ctx, op->frame, op->frame_len);
    if (err != ESP_OK) {
        link_finish_failed_send(op->link, op->frame, op->frame_len, err);
    }
}

static void link_drain_tx_queue(link_entry_t *L)
{
    esp_gmp_link_t link = L->key;

    while (L->txq_count > 0) {
        if (!L->ops || !L->ops->send || L->closing || !link_can_send(L)) {
            break;
        }

        tx_item_t *item = &L->txq[L->txq_head];
        tx_send_op_t op = { 0 };
        if (!link_fill_send_op_locked(L, item->frame, item->frame_len, &op)) {
            break;
        }
        link_drop_tx_head(L);
        xSemaphoreGive(L->mutex);
        tx_send_op_run(&op);
        if (xSemaphoreTake(L->mutex, portMAX_DELAY) != pdTRUE) {
            return;
        }
        if (L->key != link || L->closing) {
            break;
        }
    }
}

static esp_err_t auto_rsp_build(const esp_gmp_parsed_t *ref, uint8_t status, uint8_t **out_frame, size_t *out_len)
{
    size_t frame_len = 10;
    uint8_t *frame = malloc(frame_len);
    if (!frame) {
        return ESP_ERR_NO_MEM;
    }

    size_t built = esp_gmp_frame_build(frame, frame_len, ESP_GMP_VER, rsp_op(ref->op),
                                       ref->group_id, ref->seq_host, ref->command_id, 0, status,
                                       NULL, 0);
    if (built == 0) {
        free(frame);
        return ESP_ERR_INVALID_SIZE;
    }

    *out_frame = frame;
    *out_len = built;
    return ESP_OK;
}

static esp_gmp_status_t validate_request(link_entry_t *L, const uint8_t *data, size_t len, const esp_gmp_parsed_t *f)
{
    if (f->ver != ESP_GMP_VER) {
        return ESP_GMP_STATUS_NOT_SUPPORTED;
    }
    if (!is_req_op(f->op)) {
        return ESP_GMP_STATUS_NOT_SUPPORTED;
    }
    if (f->reserved0 != 0) {
        return ESP_GMP_STATUS_NOT_SUPPORTED;
    }
    if ((f->flags & ESP_GMP_FLAG_FRAGMENTED) != 0) {
        return ESP_GMP_STATUS_NOT_SUPPORTED;
    }
    if ((f->flags & ESP_GMP_FLAG_RFU_MASK) != 0) {
        return ESP_GMP_STATUS_NOT_SUPPORTED;
    }
    if (f->status_rsv != 0) {
        return ESP_GMP_STATUS_NOT_SUPPORTED;
    }
    if (len != 10 + (size_t)f->packet_length_host) {
        return ESP_GMP_STATUS_BAD_LENGTH;
    }
    if (f->payload_len != (size_t)f->packet_length_host) {
        return ESP_GMP_STATUS_BAD_LENGTH;
    }

    size_t max_pl = max_payload_effective_locked(L);
    if (f->packet_length_host > max_pl) {
        return ESP_GMP_STATUS_BAD_LENGTH;
    }

    if ((f->flags & ESP_GMP_FLAG_CRC16_TAIL) != 0) {
        if (f->payload_len < 2) {
            return ESP_GMP_STATUS_BAD_LENGTH;
        }
        size_t body_len = f->payload_len - 2;
        uint16_t expect = esp_gmp_crc16_ccitt_false(f->payload, body_len);
        uint16_t got = (uint16_t)((uint16_t)f->payload[body_len] << 8 | f->payload[body_len + 1]);
        if (expect != got) {
            return ESP_GMP_STATUS_CRC_ERROR;
        }
    }

    return ESP_GMP_STATUS_OK;
}

void esp_gmp_init(void)
{
    if (s_inited) {
        return;
    }
    memset(s_links, 0, sizeof(s_links));
    s_links_mutex = xSemaphoreCreateMutex();
    if (!s_links_mutex) {
        return;
    }
    s_cb = NULL;
    s_cb_ctx = NULL;
    s_inited = true;
}

void esp_gmp_deinit(void)
{
    if (!s_inited) {
        return;
    }
    if (link_any_active()) {
        return;
    }
    for (int i = 0; i < CONFIG_ESP_GMP_MAX_LINKS; i++) {
        if (s_links[i].mutex) {
            vSemaphoreDelete(s_links[i].mutex);
            s_links[i].mutex = NULL;
        }
    }
    if (s_links_mutex) {
        vSemaphoreDelete(s_links_mutex);
        s_links_mutex = NULL;
    }
    s_inited = false;
}

void esp_gmp_poll(void)
{
    for (int i = 0; i < CONFIG_ESP_GMP_MAX_LINKS; i++) {
        link_entry_t *L = &s_links[i];
        if (!L->key || !L->mutex) {
            continue;
        }
        if (xSemaphoreTake(L->mutex, 0) == pdTRUE) {
            link_drain_tx_queue(L);
            xSemaphoreGive(L->mutex);
        }
    }
}

esp_err_t esp_gmp_link_register(esp_gmp_link_t link, const esp_gmp_transport_t *ops, void *transport_ctx)
{
    if (!link || !ops || !ops->send || !ops->max_payload) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_links_mutex || xSemaphoreTake(s_links_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_INVALID_STATE;
    }

    link_entry_t *existing = link_find(link);
    if (existing) {
        xSemaphoreGive(s_links_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    link_entry_t *L = link_alloc(link);
    if (!L) {
        xSemaphoreGive(s_links_mutex);
        return ESP_ERR_NO_MEM;
    }
    L->ops = ops;
    L->transport_ctx = transport_ctx;
    xSemaphoreGive(s_links_mutex);
    return ESP_OK;
}

void esp_gmp_link_unregister(esp_gmp_link_t link)
{
    if (!s_links_mutex || xSemaphoreTake(s_links_mutex, portMAX_DELAY) != pdTRUE) {
        return;
    }

    link_entry_t *L = link_find(link);
    if (!L) {
        xSemaphoreGive(s_links_mutex);
        return;
    }

    if (L->mutex) {
        xSemaphoreTake(L->mutex, portMAX_DELAY);
    }

    for (size_t i = 0; i < CONFIG_ESP_GMP_TX_QUEUE_DEPTH; i++) {
        tx_item_clear(&L->txq[i]);
    }
    /* Leave in_flight buffers for tx_done / failed-send; do not free under send(). */
    L->txq_head = 0;
    L->txq_count = 0;
    L->ops = NULL;
    L->transport_ctx = NULL;
    L->closing = true;
    link_clear_if_idle_locked(L);

    if (L->mutex) {
        xSemaphoreGive(L->mutex);
    }
    xSemaphoreGive(s_links_mutex);
}

void esp_gmp_on_packet_register(esp_gmp_on_packet_fn cb, void *user_ctx)
{
    if (s_links_mutex && xSemaphoreTake(s_links_mutex, portMAX_DELAY) == pdTRUE) {
        s_cb = cb;
        s_cb_ctx = user_ctx;
        xSemaphoreGive(s_links_mutex);
        return;
    }
    s_cb = cb;
    s_cb_ctx = user_ctx;
}

bool esp_gmp_input(esp_gmp_link_t link, const uint8_t *data, size_t len)
{
    esp_gmp_on_packet_fn cb = NULL;
    void *cb_ctx = NULL;
    link_entry_t *L = link_find(link);
    if (!L || L->key != link || !data) {
        return false;
    }

    esp_gmp_parsed_t f;
    if (esp_gmp_frame_parse(data, len, &f) != 0) {
        return false;
    }

    bool request = is_req_op(f.op);
    bool response = is_rsp_op(f.op);
    if (!request && !response) {
        return false;
    }

    if (xSemaphoreTake(L->mutex, portMAX_DELAY) != pdTRUE) {
        return false;
    }
    if (L->key != link || !L->ops) {
        xSemaphoreGive(L->mutex);
        return false;
    }

    tx_send_op_t auto_rsp_op = { 0 };
    if (request) {
        esp_gmp_status_t st = validate_request(L, data, len, &f);
        if (st != ESP_GMP_STATUS_OK) {
            uint8_t *frame = NULL;
            size_t frame_len = 0;
            if (auto_rsp_build(&f, (uint8_t)st, &frame, &frame_len) == ESP_OK) {
                esp_err_t tx_err = link_tx_submit_locked(L, frame, frame_len, &auto_rsp_op);
                if (tx_err != ESP_OK) {
                    ESP_LOGW(TAG, "drop auto response status=%u err=%s",
                             (unsigned)st, esp_err_to_name(tx_err));
                }
            }
            xSemaphoreGive(L->mutex);
            tx_send_op_run(&auto_rsp_op);
            return false;
        }
    } else {
        if (len < 10 || len != 10 + (size_t)f.packet_length_host) {
            xSemaphoreGive(L->mutex);
            return false;
        }
    }

    xSemaphoreGive(L->mutex);

    if (s_links_mutex && xSemaphoreTake(s_links_mutex, portMAX_DELAY) == pdTRUE) {
        cb = s_cb;
        cb_ctx = s_cb_ctx;
        xSemaphoreGive(s_links_mutex);
    }
    if (!cb) {
        if (request) {
            uint8_t *frame = NULL;
            size_t frame_len = 0;
            if (auto_rsp_build(&f, ESP_GMP_STATUS_UNKNOWN_COMMAND, &frame, &frame_len) == ESP_OK) {
                tx_send_op_t op = { 0 };
                if (xSemaphoreTake(L->mutex, portMAX_DELAY) == pdTRUE) {
                    if (L->key == link && L->ops && !L->closing) {
                        esp_err_t tx_err = link_tx_submit_locked(L, frame, frame_len, &op);
                        if (tx_err != ESP_OK) {
                            ESP_LOGW(TAG, "drop unknown-command response err=%s",
                                     esp_err_to_name(tx_err));
                        }
                    } else {
                        free(frame);
                    }
                    xSemaphoreGive(L->mutex);
                } else {
                    free(frame);
                }
                tx_send_op_run(&op);
            }
        }
        return false;
    }

    esp_gmp_rx_t rx = {
        .link = link,
        .ver = f.ver,
        .op = f.op,
        .group_id = f.group_id,
        .sequence = f.seq_host,
        .command_id = f.command_id,
        .flags = f.flags,
        .status = f.status_rsv,
        .payload = f.payload,
        .payload_len = f.payload_len,
        .frame_buf = (uint8_t *)data,
        .frame_len = len,
    };
    return cb(cb_ctx, &rx);
}

void esp_gmp_transport_tx_done(esp_gmp_link_t link, const uint8_t *data)
{
    link_entry_t *L = link_find(link);
    if (!L || L->key != link || !L->mutex) {
        return;
    }

    if (xSemaphoreTake(L->mutex, portMAX_DELAY) != pdTRUE) {
        return;
    }
    if (L->key != link) {
        xSemaphoreGive(L->mutex);
        return;
    }

    in_flight_release(L, data);
    if (L->closing || !L->ops) {
        link_clear_if_idle_locked(L);
    } else {
        link_drain_tx_queue(L);
    }
    xSemaphoreGive(L->mutex);
}

esp_err_t esp_gmp_send(esp_gmp_link_t link, const esp_gmp_tx_params_t *params, const uint8_t *payload, size_t payload_len)
{
    if (!params) {
        return ESP_ERR_INVALID_ARG;
    }

    link_entry_t *L = link_find(link);
    if (!L || L->key != link || !L->ops) {
        return ESP_ERR_INVALID_STATE;
    }

    size_t max_pl = esp_gmp_max_payload_effective(link);
    if (payload_len > max_pl) {
        return ESP_ERR_INVALID_SIZE;
    }

    size_t frame_len = 10 + payload_len;
    uint8_t *frame = malloc(frame_len);
    if (!frame) {
        return ESP_ERR_NO_MEM;
    }

    size_t built = esp_gmp_frame_build(frame, frame_len, params->ver, params->op, params->group_id,
                                       params->sequence, params->command_id, params->flags,
                                       params->status, payload, payload_len);
    if (built == 0) {
        free(frame);
        return ESP_ERR_INVALID_SIZE;
    }

    if (xSemaphoreTake(L->mutex, portMAX_DELAY) != pdTRUE) {
        free(frame);
        return ESP_ERR_INVALID_STATE;
    }
    if (L->key != link || !L->ops) {
        xSemaphoreGive(L->mutex);
        free(frame);
        return ESP_ERR_INVALID_STATE;
    }

    tx_send_op_t op = { 0 };
    esp_err_t err = link_tx_submit_locked(L, frame, built, &op);
    xSemaphoreGive(L->mutex);
    tx_send_op_run(&op);
    return err;
}
