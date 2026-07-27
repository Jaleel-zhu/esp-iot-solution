/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "esp_timer.h"
#include "esp_file_transfer.h"
#include "esp_file_transfer_protocol.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ESP_FT_DEFAULT_MAX_FILE_SIZE    (16u * 1024u * 1024u)
#define ESP_FT_DEFAULT_MAX_BLOCK_SIZE   (16u * 1024u)
#define ESP_FT_EVENT_QUEUE_LEN          8u
#define ESP_FT_URGENT_QUEUE_LEN         2u
#define ESP_FT_WORKER_STACK_SIZE        6144u
#define ESP_FT_WORKER_PRIORITY          5u
#define ESP_FT_META_RSP_TIMEOUT_US      (10ULL * 1000ULL * 1000ULL)
#define ESP_FT_BLOCK_RSP_TIMEOUT_US     (10ULL * 1000ULL * 1000ULL)
#define ESP_FT_DATA_RX_TIMEOUT_US       (10ULL * 1000ULL * 1000ULL)
#define ESP_FT_FINAL_TIMEOUT_US         (20ULL * 1000ULL * 1000ULL)

typedef enum {
    TRANSFER_STATE_PREPARING = 0,
    TRANSFER_STATE_WAIT_META_RSP,
    TRANSFER_STATE_WAIT_DATA_BLOCK,
    TRANSFER_STATE_SENDING_DATA,
    TRANSFER_STATE_WAIT_FINAL_CONFIRM,
    TRANSFER_STATE_FINALIZING,
    TRANSFER_STATE_ERROR,
} transfer_state_t;

typedef enum {
    FT_TIMER_NONE = 0,
    FT_TIMER_META_RSP,
    FT_TIMER_BLOCK_RSP,
    FT_TIMER_DATA_RX,
    FT_TIMER_FINAL,
} ft_timer_kind_t;

typedef enum {
    FT_WORK_SEND_CMD = 0,
    FT_WORK_ABORT_CMD,
    FT_WORK_GMP_PACKET,
    FT_WORK_TIMEOUT,
    FT_WORK_DEINIT,
} ft_worker_event_type_t;

typedef struct ft_context ft_context_t;
typedef struct ft_instance ft_instance_t;
typedef struct ft_hash_context ft_hash_context_t;

typedef struct {
    ft_timer_kind_t kind;
    uint32_t generation;
    uint32_t transfer_id;
} ft_timeout_tuple_t;

typedef struct {
    esp_gmp_link_t link;
    uint8_t ver;
    uint8_t op;
    uint8_t group_id;
    uint16_t sequence;
    uint8_t command_id;
    uint8_t flags;
    uint8_t status;
    uint8_t *payload;
    size_t payload_len;
} ft_gmp_packet_t;

typedef struct {
    SemaphoreHandle_t done;
    esp_err_t *result;
} ft_command_reply_t;

typedef struct {
    ft_worker_event_type_t type;
    union {
        struct {
            char *src_path;
            char *remote_name;
            ft_command_reply_t reply;
        } send;
        struct {
            ft_command_reply_t reply;
        } abort;
        ft_gmp_packet_t packet;
        ft_timeout_tuple_t timeout;
        struct {
            ft_command_reply_t reply;
        } deinit;
    } data;
} ft_worker_event_t;

struct ft_context {
    esp_file_transfer_role_t role;
    transfer_state_t state;
    esp_gmp_link_t link;
    uint32_t transfer_id;
    uint32_t current_block;
    uint32_t total_blocks;
    uint32_t block_size;
    uint64_t file_size;
    uint64_t bytes_transferred;
    char file_name[ESP_FT_FILE_NAME_MAX_LEN + 1];
    char saved_name[ESP_FT_FILE_NAME_MAX_LEN + 1];
    char *source_path;
    char *temp_path;
    char *target_path;
    FILE *file;
    ft_hash_context_t *hash;
    uint8_t expected_sha256[32];
    uint8_t computed_sha256[32];
    uint8_t *block_buffer;
    size_t block_buffer_size;
    bool metadata_sent;
    bool pending_valid;
    uint16_t pending_sequence;
    uint8_t pending_command;
    bool early_final_valid;
    ft_final_confirm_t early_final;
    uint8_t last_reported_percent;
    bool progress_reported;
    bool terminal_emitted;
    bool target_committed;
    uint16_t reason_code;
    esp_err_t detail;
};

struct ft_instance {
    bool initialized;
    atomic_bool accepting_events;
    atomic_bool user_abort_pending;
    atomic_bool user_abort_completed;
    atomic_bool link_down_pending;
    atomic_bool transport_error_pending;
    atomic_bool timeout_pending;
    atomic_int transport_error;
    atomic_uint hook_ref_count;
    esp_gmp_link_t gmp_link;
    char *recv_dir;
    size_t max_file_size;
    size_t configured_block_size;
    size_t block_size;
    size_t effective_payload;
    esp_file_transfer_event_cb_t event_cb;
    void *event_ctx;
    TaskHandle_t worker_task;
    QueueHandle_t event_queue;
    QueueHandle_t urgent_queue;
    SemaphoreHandle_t snapshot_mutex;
    SemaphoreHandle_t abort_mutex;
    ft_context_t *active_ctx;
    uint16_t next_sequence;
    uint32_t next_transfer_id;
    esp_timer_handle_t timer;
    ft_timer_kind_t timer_kind;
    uint32_t timer_generation;
    uint32_t timer_transfer_id;
    ft_timeout_tuple_t timeout_tuple;
    ft_timeout_tuple_t timeout_pending_tuple;
    portMUX_TYPE timeout_lock;
    esp_file_transfer_status_t snapshot;
};

ft_instance_t *ft_instance_get(void);
uint16_t ft_next_sequence(ft_instance_t *instance);
uint32_t ft_next_transfer_id(ft_instance_t *instance);
esp_err_t ft_refresh_capabilities(ft_instance_t *instance);
bool ft_async_enter(ft_instance_t *instance);
void ft_async_exit(ft_instance_t *instance);
esp_err_t ft_timer_arm(ft_instance_t *instance, ft_timer_kind_t kind, uint64_t timeout_us);
void ft_timer_disarm(ft_instance_t *instance);
bool ft_process_pending_termination(ft_instance_t *instance);
void ft_snapshot_update(ft_instance_t *instance);
void ft_emit_event(ft_instance_t *instance, esp_file_transfer_event_id_t event_id);
void ft_emit_rejected_event(ft_instance_t *instance, const char *file_name,
                            uint64_t file_size, uint16_t reason, esp_err_t detail);
void ft_finish_transfer(ft_instance_t *instance, esp_file_transfer_event_id_t terminal_event,
                        uint16_t reason, esp_err_t detail);
void ft_worker_wake(ft_instance_t *instance);

esp_err_t ft_gmp_send(ft_instance_t *instance, uint8_t op, uint16_t sequence,
                      uint8_t command, uint8_t status, const uint8_t *payload,
                      size_t payload_len);
esp_err_t ft_gmp_send_request(ft_instance_t *instance, ft_context_t *ctx, uint8_t command,
                              const uint8_t *payload, size_t payload_len, bool expect_response);
esp_err_t ft_gmp_send_response(ft_instance_t *instance, const ft_gmp_packet_t *request,
                               uint8_t gmp_status, const uint8_t *payload, size_t payload_len);
void ft_gmp_send_abort(ft_instance_t *instance, ft_context_t *ctx, uint16_t reason);
bool ft_gmp_pending_matches(const ft_context_t *ctx, const ft_gmp_packet_t *packet,
                            uint8_t command);

esp_err_t ft_data_send_request(ft_instance_t *instance, ft_context_t *ctx,
                               uint32_t block_index, size_t data_len);
esp_err_t ft_data_send_response(ft_instance_t *instance, const ft_gmp_packet_t *request,
                                uint32_t transfer_id, uint32_t block_index,
                                uint8_t status, uint16_t reason);

esp_err_t ft_fs_prepare(const char *recv_dir);
void ft_fs_cleanup_parts(const char *recv_dir);
bool ft_fs_name_valid(const char *name);
const char *ft_fs_basename(const char *path);
esp_err_t ft_fs_source_info(const char *path, uint64_t *size);
esp_err_t ft_fs_open_source(const char *path, FILE **file);
esp_err_t ft_fs_create_temp(const char *recv_dir, uint32_t transfer_id, const char *file_name,
                            char **temp_path, FILE **file);
esp_err_t ft_fs_get_free_bytes(const char *recv_dir, uint64_t *free_bytes);
esp_err_t ft_fs_commit_temp(const char *recv_dir, const char *temp_path, const char *file_name,
                            char **target_path, char saved_name[ESP_FT_FILE_NAME_MAX_LEN + 1]);
void ft_fs_remove(const char *path);

esp_err_t ft_hash_create(ft_hash_context_t **context);
esp_err_t ft_hash_update(ft_hash_context_t *context, const void *data, size_t len);
esp_err_t ft_hash_finish(ft_hash_context_t *context, uint8_t digest[32]);
void ft_hash_destroy(ft_hash_context_t *context);

esp_err_t ft_sender_start(ft_instance_t *instance, const char *src_path, const char *remote_name);
void ft_sender_prepare(ft_instance_t *instance);
void ft_sender_handle_packet(ft_instance_t *instance, const ft_gmp_packet_t *packet);
void ft_sender_handle_timeout(ft_instance_t *instance, ft_timer_kind_t kind);

void ft_receiver_handle_meta(ft_instance_t *instance, const ft_gmp_packet_t *packet);
void ft_receiver_handle_data(ft_instance_t *instance, const ft_gmp_packet_t *packet);
void ft_receiver_handle_timeout(ft_instance_t *instance, ft_timer_kind_t kind);

void ft_handle_peer_abort(ft_instance_t *instance, const ft_gmp_packet_t *packet);
void ft_abort_active(ft_instance_t *instance, uint16_t reason, esp_err_t detail,
                     bool notify_peer, bool cancelled);

#ifdef __cplusplus
}
#endif
