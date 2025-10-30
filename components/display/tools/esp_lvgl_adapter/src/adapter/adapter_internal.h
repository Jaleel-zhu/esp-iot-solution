/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file adapter_internal.h
 * @brief Internal header for LVGL adapter implementation
 *
 * This file contains internal structures and functions that should not be
 * exposed in the public API.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_lv_adapter.h"

#if CONFIG_ESP_LVGL_ADAPTER_ENABLE_FS
#include "esp_lv_fs.h"
#endif

#if CONFIG_ESP_LVGL_ADAPTER_ENABLE_DECODER
#include "esp_lv_decoder.h"
#endif

/* Forward declarations */
struct esp_lv_adapter_display_bridge;

/* Maximum number of frame buffers supported */
#define ESP_LV_ADAPTER_MAX_FRAME_BUFFERS  3

/*****************************************************************************
 *                         Internal Data Structures                          *
 *****************************************************************************/

/**
 * @brief Display runtime configuration
 *
 * Extended configuration that includes runtime allocated resources
 */
typedef struct {
    esp_lv_adapter_display_config_t base;         /*!< Base configuration */
    size_t draw_buf_pixels;                 /*!< Number of pixels in draw buffer */
    void *draw_buf_primary;                 /*!< Primary draw buffer pointer */
    void *draw_buf_secondary;               /*!< Secondary draw buffer pointer (can be NULL) */
    uint8_t frame_buffer_count;             /*!< Number of frame buffers */
    void *frame_buffers[ESP_LV_ADAPTER_MAX_FRAME_BUFFERS]; /*!< Frame buffer pointers */
    size_t frame_buffer_size;               /*!< Size of each frame buffer in bytes */
    bool dummy_draw_enabled;                /*!< Dummy draw mode flag */
    esp_lv_adapter_dummy_draw_callbacks_t dummy_draw_cbs; /*!< Dummy draw callback collection */
    void *dummy_draw_user_ctx;              /*!< User context for dummy draw callbacks */
    lv_display_t *lv_disp;                   /*!< Associated LVGL display handle */
} esp_lv_adapter_display_runtime_config_t;

/**
 * @brief Display node in the linked list
 *
 * Each registered display is stored as a node in a linked list
 */
typedef struct esp_lv_adapter_display_node {
    esp_lv_adapter_display_runtime_config_t cfg;  /*!< Display runtime configuration */
    lv_display_t *lv_disp;                  /*!< LVGL display object */
    struct esp_lv_adapter_display_bridge *bridge; /*!< Display bridge for hardware interface */
    uint8_t prev_flush_status;              /*!< Previous flush status (thread-safe per display) */
#if LVGL_VERSION_MAJOR < 9
    lv_disp_draw_buf_t draw_buf;            /*!< LVGL v8 draw buffer */
    lv_disp_drv_t disp_drv;                 /*!< LVGL v8 display driver */
#endif
#if CONFIG_ESP_LVGL_ADAPTER_ENABLE_FPS_STATS
    /* FPS statistics */
    struct {
        uint32_t frame_count;               /*!< Frame count in current time window */
        int64_t window_start_time;          /*!< Time window start time (microseconds) */
        uint32_t current_fps;               /*!< Current FPS (cached value, integer to avoid FPU in ISR) */
        bool enabled;                       /*!< FPS statistics enabled flag */
    } fps_stats;
#endif
    struct esp_lv_adapter_display_node *next;     /*!< Next node in the linked list */
} esp_lv_adapter_display_node_t;

#if CONFIG_ESP_LVGL_ADAPTER_ENABLE_FS
/**
 * @brief File system node in the linked list
 *
 * Each mounted file system is stored as a node in a linked list
 */
typedef struct esp_lv_adapter_fs_node {
    esp_lv_fs_handle_t handle;              /*!< File system handle */
    struct esp_lv_adapter_fs_node *next;          /*!< Next node in the linked list */
} esp_lv_adapter_fs_node_t;
#endif

#if CONFIG_ESP_LVGL_ADAPTER_ENABLE_FREETYPE
/**
 * @brief FreeType font node in the linked list
 *
 * Each initialized FreeType font is stored as a node in a linked list
 */
typedef struct esp_lv_adapter_ft_font_node {
#if LVGL_VERSION_MAJOR >= 9
    lv_font_t *font;                        /*!< LVGL v9: Font pointer */
    lv_font_info_t font_info;               /*!< LVGL v9: Font info structure */
#else
    lv_ft_info_t ft_info;                   /*!< LVGL v8: FreeType info structure */
#endif
    char *name_copy;                        /*!< Copy of font name */
    bool initialized;                       /*!< Initialization flag */
    struct esp_lv_adapter_ft_font_node *next;     /*!< Next node in the linked list */
} esp_lv_adapter_ft_font_node_t;
#endif

/**
 * @brief Input device types enumeration
 */
typedef enum {
    ESP_LV_ADAPTER_INPUT_TYPE_TOUCH,      /*!< Touch screen input */
#if CONFIG_ESP_LVGL_ADAPTER_ENABLE_BUTTON
    ESP_LV_ADAPTER_INPUT_TYPE_BUTTON,     /*!< Navigation buttons input */
#endif
#if CONFIG_ESP_LVGL_ADAPTER_ENABLE_KNOB
    ESP_LV_ADAPTER_INPUT_TYPE_ENCODER,    /*!< Rotary encoder/knob input */
#endif
} esp_lv_adapter_input_type_t;

/**
 * @brief Input device node in the linked list
 *
 * Each registered input device is stored as a node in a linked list
 * for automatic cleanup on deinit.
 */
typedef struct esp_lv_adapter_input_node {
    lv_indev_t *indev;                      /*!< LVGL input device object */
    esp_lv_adapter_input_type_t type;             /*!< Input device type */
    void *user_ctx;                         /*!< User context pointer */
    struct esp_lv_adapter_input_node *next;       /*!< Next node in the linked list */
} esp_lv_adapter_input_node_t;

/**
 * @brief LVGL adapter context structure
 *
 * Global context that maintains the state of the LVGL adapter
 */
typedef struct {
    bool inited;                            /*!< Initialization flag */
    volatile bool task_exit_requested;      /*!< Flag to request LVGL task exit */
    SemaphoreHandle_t lvgl_mutex;           /*!< Recursive mutex for LVGL library calls */
    SemaphoreHandle_t dummy_draw_mutex;     /*!< Recursive mutex for dummy draw operations */
    TaskHandle_t task;                      /*!< LVGL task handle */
    void *tick_timer;                       /*!< LVGL tick timer handle (esp_timer_handle_t) */
    esp_lv_adapter_config_t config;       /*!< Adapter configuration */
    esp_lv_adapter_display_node_t *display_list;  /*!< Linked list of registered displays */
    esp_lv_adapter_input_node_t *input_list;      /*!< Linked list of registered input devices */
#if CONFIG_ESP_LVGL_ADAPTER_ENABLE_DECODER
    esp_lv_decoder_handle_t decoder_handle; /*!< Image decoder handle */
#endif
#if CONFIG_ESP_LVGL_ADAPTER_ENABLE_FS
    esp_lv_adapter_fs_node_t *fs_list;            /*!< Linked list of mounted file systems */
#endif
#if CONFIG_ESP_LVGL_ADAPTER_ENABLE_FREETYPE
    esp_lv_adapter_ft_font_node_t *font_list;     /*!< Linked list of FreeType fonts */
#endif
} esp_lv_adapter_context_t;

/*****************************************************************************
 *                         Internal Functions                                *
 *****************************************************************************/

/**
 * @brief Get the global context of the LVGL adapter
 *
 * @return Pointer to the global adapter context
 */
esp_lv_adapter_context_t *esp_lv_adapter_get_context(void);

/**
 * @brief Register an input device in the global list
 *
 * This function adds an input device to the adapter's tracking list
 * for automatic cleanup on deinit.
 *
 * @param indev LVGL input device handle
 * @param type Input device type
 * @param user_ctx User context pointer (for cleanup)
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_NO_MEM: Memory allocation failed
 */
esp_err_t esp_lv_adapter_register_input_device(lv_indev_t *indev,
                                               esp_lv_adapter_input_type_t type,
                                               void *user_ctx);

/**
 * @brief Unregister an input device from the global list
 *
 * This function removes an input device from the adapter's tracking list.
 * It does NOT delete the LVGL input device or free the user context.
 *
 * @param indev LVGL input device handle
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_NOT_FOUND: Input device not found in list
 */
esp_err_t esp_lv_adapter_unregister_input_device(lv_indev_t *indev);
