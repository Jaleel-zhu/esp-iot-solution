/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Display Manager - Manages LVGL display lifecycle and configuration
 */

/*********************
 *      INCLUDES
 *********************/
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_cache.h"
#include "esp_private/esp_cache_private.h"
#include "soc/soc_caps.h"
#if SOC_LCDCAM_RGB_LCD_SUPPORTED
#include "esp_lcd_panel_rgb.h"
#endif
#if SOC_MIPI_DSI_SUPPORTED
#include "esp_lcd_mipi_dsi.h"
#endif
#include "lvgl.h"
#if LVGL_VERSION_MAJOR >= 9
#include "lvgl_private.h"
#endif
#include "adapter_internal.h"
#include "display_bridge.h"
#include "lvgl_port_alignment.h"
#include "lvgl_port_ppa.h"
#include "display_manager.h"

/*********************
 *      TYPEDEFS
 *********************/

/**
 * @brief Internal render mode for LVGL display
 */
typedef enum {
    ESP_LV_ADAPTER_DISPLAY_RENDER_MODE_PARTIAL,    /*!< Partial screen refresh */
    ESP_LV_ADAPTER_DISPLAY_RENDER_MODE_FULL,       /*!< Full screen refresh */
    ESP_LV_ADAPTER_DISPLAY_RENDER_MODE_DIRECT,     /*!< Direct mode (no internal buffer management) */
} esp_lv_adapter_display_render_mode_t;

/**********************
 *  STATIC VARIABLES
 **********************/
static const char *TAG = "esp_lvgl:disp";

/**********************
 *  FORWARD DECLARATIONS
 **********************/

/* Configuration validation and selection */
static bool display_manager_validate_tearing_mode(const esp_lv_adapter_display_config_t *cfg);
static esp_lv_adapter_display_render_mode_t display_manager_pick_render_mode(const esp_lv_adapter_display_runtime_config_t *cfg);

/* Display node lifecycle */
static void display_manager_destroy_node(esp_lv_adapter_display_node_t *node);

#if CONFIG_ESP_LVGL_ADAPTER_ENABLE_FPS_STATS
/* FPS statistics */
static void display_manager_fps_frame_done(esp_lv_adapter_display_node_t *node);
#endif

#if LVGL_VERSION_MAJOR >= 9
void display_manager_flush_ready(lv_display_t *disp)
{
#if CONFIG_ESP_LVGL_ADAPTER_ENABLE_FPS_STATS
    esp_lv_adapter_display_node_t *node = lv_display_get_user_data(disp);
    if (node && lv_display_flush_is_last(disp)) {
        display_manager_fps_frame_done(node);
    }
#endif
    lv_display_flush_ready(disp);
}
#else
void display_manager_flush_ready(lv_disp_drv_t *drv)
{
#if CONFIG_ESP_LVGL_ADAPTER_ENABLE_FPS_STATS
    esp_lv_adapter_display_node_t *node = (esp_lv_adapter_display_node_t *)drv->user_data;
    if (node && lv_disp_flush_is_last(drv)) {
        display_manager_fps_frame_done(node);
    }
#endif
    lv_disp_flush_ready(drv);
}
#endif

/* Buffer management */
static bool display_manager_prepare_buffers(esp_lv_adapter_display_node_t *node,
                                            esp_lv_adapter_display_render_mode_t mode,
                                            size_t color_size);
static void *display_manager_alloc_draw_buffer(size_t size, bool use_psram);
static size_t display_manager_ppa_alignment(void);

/* Frame buffer helpers */
static bool display_manager_fetch_panel_frame_buffers(esp_lv_adapter_display_node_t *node,
                                                      uint8_t required,
                                                      size_t color_size);
static uint8_t display_manager_required_buffer_count(const esp_lv_adapter_display_runtime_config_t *cfg,
                                                     esp_lv_adapter_display_render_mode_t mode);
static size_t display_manager_default_buffer_pixels(const esp_lv_adapter_display_runtime_config_t *cfg,
                                                    esp_lv_adapter_display_render_mode_t mode);
static uint16_t display_manager_effective_buffer_height(const esp_lv_adapter_display_runtime_config_t *cfg);

/* Internal node management */
static bool display_manager_init_node(esp_lv_adapter_display_node_t *node);
static esp_lv_adapter_display_node_t *display_manager_find_node(lv_display_t *disp);

/* LVGL callbacks */
#if LVGL_VERSION_MAJOR >= 9
static void display_manager_flush_cb_v9(lv_display_t *disp,
                                        const lv_area_t *area,
                                        uint8_t *color_map);
#else
static void display_manager_flush_cb_v8(lv_disp_drv_t *drv,
                                        const lv_area_t *area,
                                        lv_color_t *color_map);
#endif

/**********************
 *   PUBLIC API
 **********************/

/**
 * @brief Register a new display with the adapter
 */
lv_display_t *display_manager_register(const esp_lv_adapter_display_config_t *cfg)
{
    if (!cfg) {
        return NULL;
    }

    if (!display_manager_validate_tearing_mode(cfg)) {
        return NULL;
    }

    esp_lv_adapter_display_node_t *node = calloc(1, sizeof(*node));
    if (!node) {
        ESP_LOGE(TAG, "no memory for display node");
        return NULL;
    }

    node->cfg.base = *cfg;

    if (!display_manager_init_node(node)) {
        free(node);
        return NULL;
    }

    esp_lv_adapter_context_t *ctx = esp_lv_adapter_get_context();
    if (!ctx || !ctx->inited) {
        ESP_LOGE(TAG, "Adapter not initialized");
        display_manager_destroy_node(node);
        return NULL;
    }

    node->next = ctx->display_list;
    ctx->display_list = node;

    return node->lv_disp;
}

/**
 * @brief Enable or disable dummy draw mode for a display
 */
esp_err_t display_manager_set_dummy_draw(lv_display_t *disp, bool enable)
{
    ESP_RETURN_ON_FALSE(disp, ESP_ERR_INVALID_ARG, TAG, "Display handle cannot be NULL");

    esp_lv_adapter_display_node_t *node = display_manager_find_node(disp);
    ESP_RETURN_ON_FALSE(node, ESP_ERR_INVALID_ARG, TAG, "Display not registered");

    if (node->cfg.dummy_draw_enabled == enable) {
        return ESP_OK;
    }

    node->cfg.dummy_draw_enabled = enable;
    if (node->bridge && node->bridge->set_dummy_draw) {
        node->bridge->set_dummy_draw(node->bridge, enable);
    }

    if (enable) {
        if (node->cfg.dummy_draw_cbs.on_enable) {
            node->cfg.dummy_draw_cbs.on_enable(node->lv_disp, node->cfg.dummy_draw_user_ctx);
        }
    } else {
        if (node->cfg.dummy_draw_cbs.on_disable) {
            node->cfg.dummy_draw_cbs.on_disable(node->lv_disp, node->cfg.dummy_draw_user_ctx);
        }
    }

    return ESP_OK;
}

esp_err_t display_manager_set_dummy_draw_callbacks(lv_display_t *disp,
                                                   const esp_lv_adapter_dummy_draw_callbacks_t *cbs,
                                                   void *user_ctx)
{
    ESP_RETURN_ON_FALSE(disp, ESP_ERR_INVALID_ARG, TAG, "Display handle cannot be NULL");

    esp_lv_adapter_display_node_t *node = display_manager_find_node(disp);
    ESP_RETURN_ON_FALSE(node, ESP_ERR_INVALID_ARG, TAG, "Display not registered");

    if (cbs) {
        node->cfg.dummy_draw_cbs = *cbs;
        node->cfg.dummy_draw_user_ctx = user_ctx;
    } else {
        memset(&node->cfg.dummy_draw_cbs, 0, sizeof(node->cfg.dummy_draw_cbs));
        node->cfg.dummy_draw_user_ctx = NULL;
    }

    if (node->bridge && node->bridge->set_dummy_draw_callbacks) {
        node->bridge->set_dummy_draw_callbacks(node->bridge, cbs, user_ctx);
    }

    return ESP_OK;
}

esp_err_t display_manager_dummy_draw_blit(lv_display_t *disp,
                                          int x_start,
                                          int y_start,
                                          int x_end,
                                          int y_end,
                                          const void *frame_buffer,
                                          bool wait)
{
    ESP_RETURN_ON_FALSE(disp, ESP_ERR_INVALID_ARG, TAG, "Display handle cannot be NULL");
    ESP_RETURN_ON_FALSE(frame_buffer, ESP_ERR_INVALID_ARG, TAG, "Frame buffer cannot be NULL");
    ESP_RETURN_ON_FALSE(x_start < x_end && y_start < y_end, ESP_ERR_INVALID_ARG, TAG,
                        "Invalid coordinates: start must be less than end");

    esp_lv_adapter_display_node_t *node = display_manager_find_node(disp);
    ESP_RETURN_ON_FALSE(node, ESP_ERR_INVALID_ARG, TAG, "Display not registered");
    ESP_RETURN_ON_FALSE(node->cfg.dummy_draw_enabled, ESP_ERR_INVALID_STATE, TAG, "Dummy draw not enabled");
    ESP_RETURN_ON_FALSE(node->bridge && node->bridge->dummy_draw_blit,
                        ESP_ERR_NOT_SUPPORTED, TAG, "Bridge does not support dummy draw blit");

    return node->bridge->dummy_draw_blit(node->bridge, x_start, y_start, x_end, y_end, frame_buffer, wait);
}

void display_manager_request_full_refresh(lv_display_t *disp)
{
    if (!disp) {
        return;
    }

#if LVGL_VERSION_MAJOR >= 9
    lv_obj_t *scr = lv_display_get_screen_active(disp);
    if (scr) {
        lv_obj_invalidate(scr);
    }
    lv_refr_now(disp);
#else
    lv_disp_t *disp_v8 = (lv_disp_t *)disp;
    lv_obj_t *scr = lv_disp_get_scr_act(disp_v8);
    if (scr) {
        lv_obj_invalidate(scr);
    }
    lv_refr_now(disp_v8);
#endif
}

esp_err_t display_manager_get_dummy_draw_state(lv_display_t *disp, bool *enabled)
{
    ESP_RETURN_ON_FALSE(disp && enabled, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    esp_lv_adapter_display_node_t *node = display_manager_find_node(disp);
    ESP_RETURN_ON_FALSE(node, ESP_ERR_INVALID_ARG, TAG, "Display not registered");

    *enabled = node->cfg.dummy_draw_enabled;
    return ESP_OK;
}

#if CONFIG_ESP_LVGL_ADAPTER_ENABLE_FPS_STATS
/**
 * @brief Get display node by LVGL display handle
 */
esp_lv_adapter_display_node_t *display_manager_get_node(lv_display_t *disp)
{
    esp_lv_adapter_context_t *ctx = esp_lv_adapter_get_context();
    if (!ctx) {
        return NULL;
    }

    /* If disp is NULL, return the first registered display */
    if (!disp) {
        return ctx->display_list;
    }

    /* Otherwise, find the matching display */
    return display_manager_find_node(disp);
}
#endif

/**
 * @brief Check if a pointer is a panel frame buffer
 *
 * Panel frame buffers are obtained from RGB or MIPI DSI hardware and
 * should not be freed by the display manager.
 *
 * @param node Display node
 * @param ptr Pointer to check
 * @return true if ptr points to a panel frame buffer, false otherwise
 */
static bool display_manager_is_panel_frame_buffer(const esp_lv_adapter_display_node_t *node,
                                                  const void *ptr)
{
    if (!ptr || !node) {
        return false;
    }

    /* Check if ptr matches any of the panel frame buffers */
    for (uint8_t i = 0; i < node->cfg.frame_buffer_count; i++) {
        if (node->cfg.frame_buffers[i] == ptr) {
            return true;
        }
    }

    return false;
}

/**
 * @brief Free draw buffers if they were allocated by display manager
 *
 * Only frees buffers that were allocated via heap_caps_malloc or
 * heap_caps_aligned_alloc. Buffers obtained from panel frame buffers
 * (RGB/MIPI DSI) are not freed.
 *
 * @param node Display node
 */
static void display_manager_free_draw_buffers(esp_lv_adapter_display_node_t *node)
{
    if (!node) {
        return;
    }

    esp_lv_adapter_display_runtime_config_t *cfg = &node->cfg;

    /* Free primary buffer if it's not from panel frame buffer */
    if (cfg->draw_buf_primary &&
            !display_manager_is_panel_frame_buffer(node, cfg->draw_buf_primary)) {
        free(cfg->draw_buf_primary);
        cfg->draw_buf_primary = NULL;
    }

    /* Free secondary buffer if it's not from panel frame buffer */
    if (cfg->draw_buf_secondary &&
            !display_manager_is_panel_frame_buffer(node, cfg->draw_buf_secondary)) {
        free(cfg->draw_buf_secondary);
        cfg->draw_buf_secondary = NULL;
    }
}

/**
 * @brief Destroy a single display node and free resources
 *
 * @param node Display node to destroy
 */
static void display_manager_destroy_node(esp_lv_adapter_display_node_t *node)
{
    if (!node) {
        return;
    }

    /* Destroy the bridge (hardware interface) */
    if (node->bridge && node->bridge->destroy) {
        node->bridge->destroy(node->bridge);
        node->bridge = NULL;
    }

    /* Delete the LVGL display object */
#if LVGL_VERSION_MAJOR >= 9
    if (node->lv_disp) {
        lv_display_set_user_data(node->lv_disp, NULL);
        lv_display_delete(node->lv_disp);
        node->lv_disp = NULL;
    }
#else
    if (node->lv_disp) {
        lv_disp_remove(node->lv_disp);
        node->lv_disp = NULL;
    }
#endif

    /* Free draw buffers if they were allocated by us */
    display_manager_free_draw_buffers(node);

    /* Free the node itself */
    free(node);
}

/**
 * @brief Unregister and destroy a single display
 */
esp_err_t display_manager_unregister(lv_display_t *disp)
{
    ESP_RETURN_ON_FALSE(disp, ESP_ERR_INVALID_ARG, TAG, "Display handle cannot be NULL");

    esp_lv_adapter_context_t *ctx = esp_lv_adapter_get_context();
    ESP_RETURN_ON_FALSE(ctx, ESP_ERR_INVALID_STATE, TAG, "Adapter context not available");
    ESP_RETURN_ON_FALSE(ctx->inited, ESP_ERR_INVALID_STATE, TAG, "Adapter not initialized");

    /* Find the node in the linked list */
    esp_lv_adapter_display_node_t **cursor = &ctx->display_list;
    esp_lv_adapter_display_node_t *node = NULL;

    while (*cursor) {
        if ((*cursor)->lv_disp == disp) {
            /* Found the node, remove it from the list */
            node = *cursor;
            *cursor = node->next;
            break;
        }
        cursor = &(*cursor)->next;
    }

    ESP_RETURN_ON_FALSE(node, ESP_ERR_NOT_FOUND, TAG, "Display not found in registered list");

    /* Destroy the node and free all resources */
    display_manager_destroy_node(node);

    ESP_LOGI(TAG, "Display unregistered successfully");
    return ESP_OK;
}

/**
 * @brief Clear and destroy all registered displays
 */
void display_manager_clear(void)
{
    esp_lv_adapter_context_t *ctx = esp_lv_adapter_get_context();
    if (!ctx) {
        return;
    }

    esp_lv_adapter_display_node_t *node = ctx->display_list;
    ctx->display_list = NULL;

    while (node) {
        esp_lv_adapter_display_node_t *next = node->next;
        display_manager_destroy_node(node);
        node = next;
    }
}

/**********************
 *   LVGL CALLBACKS
 **********************/

#if LVGL_VERSION_MAJOR >= 9
/**
 * @brief LVGL v9 flush callback
 */
static void display_manager_flush_cb_v9(lv_display_t *disp,
                                        const lv_area_t *area,
                                        uint8_t *color_map)
{
    esp_lv_adapter_display_node_t *node = lv_display_get_user_data(disp);
    if (!node) {
        display_manager_flush_ready(disp);
        return;
    }

    esp_lv_adapter_display_bridge_t *bridge = node->bridge;
    if (!bridge || !bridge->flush) {
        display_manager_flush_ready(disp);
        return;
    }

    bridge->flush(bridge, disp, area, color_map);
}
#else
/**
 * @brief LVGL v8 flush callback
 */
static void display_manager_flush_cb_v8(lv_disp_drv_t *drv,
                                        const lv_area_t *area,
                                        lv_color_t *color_map)
{
    esp_lv_adapter_display_node_t *node = (esp_lv_adapter_display_node_t *)drv->user_data;
    if (!node) {
        display_manager_flush_ready(drv);
        return;
    }

    esp_lv_adapter_display_bridge_t *bridge = node->bridge;
    if (!bridge || !bridge->flush) {
        display_manager_flush_ready(drv);
        return;
    }

    bridge->flush(bridge, drv, area, (uint8_t *)color_map);
}
#endif

/**********************
 *   INTERNAL NODE MANAGEMENT
 **********************/

/**
 * @brief Initialize a display node with LVGL and bridge setup
 */
static bool display_manager_init_node(esp_lv_adapter_display_node_t *node)
{
    esp_lv_adapter_display_runtime_config_t *cfg = &node->cfg;
    const esp_lv_adapter_display_config_t *pub = &cfg->base;
    esp_lv_adapter_display_render_mode_t render_mode = display_manager_pick_render_mode(cfg);

#if CONFIG_ESP_LVGL_ADAPTER_ENABLE_FPS_STATS
    /* Initialize FPS statistics */
    node->fps_stats.frame_count = 0;
    node->fps_stats.window_start_time = esp_timer_get_time();
    node->fps_stats.current_fps = 0;  /* Use integer to avoid FPU in ISR */
    node->fps_stats.enabled = false;  /* Disabled by default */
#endif

    if (pub->tear_avoid_mode == ESP_LV_ADAPTER_TEAR_AVOID_MODE_NONE && pub->profile.rotation != ESP_LV_ADAPTER_ROTATE_0 && pub->profile.interface != ESP_LV_ADAPTER_PANEL_IF_OTHER) {
        ESP_LOGE(TAG, "rotation not supported under TEAR_AVOID_MODE_NONE");
        return false;
    }

#if LVGL_VERSION_MAJOR >= 9
    lv_display_t *disp = lv_display_create(
                             (pub->profile.rotation == ESP_LV_ADAPTER_ROTATE_90 ||
                              pub->profile.rotation == ESP_LV_ADAPTER_ROTATE_270) ? pub->profile.ver_res : pub->profile.hor_res,
                             (pub->profile.rotation == ESP_LV_ADAPTER_ROTATE_90 ||
                              pub->profile.rotation == ESP_LV_ADAPTER_ROTATE_270) ? pub->profile.hor_res : pub->profile.ver_res);
    if (!disp) {
        return false;
    }

    size_t color_size = lv_color_format_get_size(lv_display_get_color_format(disp));
    if (!display_manager_prepare_buffers(node, render_mode, color_size)) {
        lv_display_delete(disp);
        node->lv_disp = NULL;
        return false;
    }

    size_t buf_bytes = cfg->draw_buf_pixels * color_size;

    lv_display_set_buffers(disp,
                           cfg->draw_buf_primary,
                           cfg->draw_buf_secondary,
                           buf_bytes,
                           render_mode == ESP_LV_ADAPTER_DISPLAY_RENDER_MODE_FULL ? LV_DISPLAY_RENDER_MODE_FULL :
                           render_mode == ESP_LV_ADAPTER_DISPLAY_RENDER_MODE_DIRECT ? LV_DISPLAY_RENDER_MODE_DIRECT :
                           LV_DISPLAY_RENDER_MODE_PARTIAL);

    node->lv_disp = disp;
    cfg->lv_disp = disp;

    /* Create display bridge based on LVGL version */
    esp_lv_adapter_display_bridge_t *bridge = esp_lv_adapter_display_bridge_v9_create(cfg);

    if (!bridge) {
        lv_display_delete(disp);
        node->lv_disp = NULL;
        return false;
    }

    node->bridge = bridge;
    if (node->bridge && node->bridge->set_dummy_draw) {
        node->bridge->set_dummy_draw(node->bridge, node->cfg.dummy_draw_enabled);
    }
    lv_display_set_user_data(disp, node);
    lv_display_set_flush_cb(disp, display_manager_flush_cb_v9);

#if CONFIG_SOC_PPA_SUPPORTED
    if (pub->profile.enable_ppa_accel) {
        /* PPA acceleration assumes draw unit count is forced to 1 via configuration. */
        lvgl_port_ppa_v9_init(disp);
#if defined(LV_DRAW_SW_DRAW_UNIT_CNT) && (LV_DRAW_SW_DRAW_UNIT_CNT > 1)
        ESP_LOGW(TAG, "PPA acceleration requires LV_DRAW_SW_DRAW_UNIT_CNT == 1 (current=%d)",
                 LV_DRAW_SW_DRAW_UNIT_CNT);
#endif
    }
#endif
#else /* LVGL v8 */
    lv_coord_t hor_res = (pub->profile.rotation == ESP_LV_ADAPTER_ROTATE_90 ||
                          pub->profile.rotation == ESP_LV_ADAPTER_ROTATE_270) ? pub->profile.ver_res : pub->profile.hor_res;
    lv_coord_t ver_res = (pub->profile.rotation == ESP_LV_ADAPTER_ROTATE_90 ||
                          pub->profile.rotation == ESP_LV_ADAPTER_ROTATE_270) ? pub->profile.hor_res : pub->profile.ver_res;

    size_t color_size = sizeof(lv_color_t);
    if (!display_manager_prepare_buffers(node, render_mode, color_size)) {
        return false;
    }

    lv_disp_draw_buf_init(&node->draw_buf,
                          (lv_color_t *)cfg->draw_buf_primary,
                          (lv_color_t *)cfg->draw_buf_secondary,
                          cfg->draw_buf_pixels);

    lv_disp_drv_init(&node->disp_drv);
    node->disp_drv.hor_res = hor_res;
    node->disp_drv.ver_res = ver_res;
    node->disp_drv.flush_cb = display_manager_flush_cb_v8;
    node->disp_drv.draw_buf = &node->draw_buf;
    node->disp_drv.user_data = node;

    if (render_mode == ESP_LV_ADAPTER_DISPLAY_RENDER_MODE_FULL) {
        node->disp_drv.full_refresh = 1;
    } else if (render_mode == ESP_LV_ADAPTER_DISPLAY_RENDER_MODE_DIRECT) {
        node->disp_drv.direct_mode = 1;
    }

    lv_disp_t *disp = lv_disp_drv_register(&node->disp_drv);
    if (!disp) {
        return false;
    }

    node->lv_disp = disp;
    cfg->lv_disp = disp;
    if (node->bridge && node->bridge->set_dummy_draw) {
        node->bridge->set_dummy_draw(node->bridge, node->cfg.dummy_draw_enabled);
    }

#if CONFIG_SOC_PPA_SUPPORTED
    if (pub->profile.enable_ppa_accel) {
        /* PPA acceleration assumes draw unit count is forced to 1 via configuration. */
        lvgl_port_ppa_v8_init(&node->disp_drv);
#if defined(LV_DRAW_SW_DRAW_UNIT_CNT) && (LV_DRAW_SW_DRAW_UNIT_CNT > 1)
        ESP_LOGW(TAG, "PPA acceleration requires LV_DRAW_SW_DRAW_UNIT_CNT == 1 (current=%d)",
                 LV_DRAW_SW_DRAW_UNIT_CNT);
#endif
    }
#endif

    /* Create display bridge for LVGL v8 */
    esp_lv_adapter_display_bridge_t *bridge = esp_lv_adapter_display_bridge_v8_create(cfg);
    if (!bridge) {
        lv_disp_remove(disp);
        node->lv_disp = NULL;
        return false;
    }

    node->bridge = bridge;
#endif

    return true;
}

/**
 * @brief Find display node by LVGL display handle
 */
static esp_lv_adapter_display_node_t *display_manager_find_node(lv_display_t *disp)
{
    esp_lv_adapter_context_t *ctx = esp_lv_adapter_get_context();
    esp_lv_adapter_display_node_t *node = ctx->display_list;
    while (node) {
        if (node->lv_disp == disp) {
            return node;
        }
        node = node->next;
    }
    return NULL;
}

/**********************
 *   BUFFER MANAGEMENT
 **********************/

/**
 * @brief Get PPA alignment requirement
 */
static size_t display_manager_ppa_alignment(void)
{
    /* Default PPA alignment requirement */
#define DEFAULT_PPA_ALIGNMENT  (128)  /* 128 bytes for cache line optimization */

    static size_t s_align = 0;
    if (s_align) {
        return s_align;
    }

#if CONFIG_SOC_PPA_SUPPORTED
    size_t align = 0;
    if (esp_cache_get_alignment(MALLOC_CAP_SPIRAM, &align) != ESP_OK || align == 0) {
        align = DEFAULT_PPA_ALIGNMENT;
    }
#else
    size_t align = DEFAULT_PPA_ALIGNMENT;
#endif

    s_align = align;
    return s_align;
}

/**
 * @brief Allocate a draw buffer with optional PPA alignment
 */
static void *display_manager_alloc_draw_buffer(size_t size, bool use_psram)
{
    const uint32_t caps = use_psram ? MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT : MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    size_t align = display_manager_ppa_alignment();

    if (align > 0) {
        void *buf = heap_caps_aligned_alloc(align, size, caps);
        if (buf) {
            return buf;
        }
    }

    return heap_caps_malloc(size, caps);
}

/**
 * @brief Prepare all required buffers for display operation
 */
static bool display_manager_use_panel_buffers(esp_lv_adapter_display_node_t *node,
                                              size_t full_pixels,
                                              uint8_t primary_idx,
                                              int secondary_idx)
{
    esp_lv_adapter_display_runtime_config_t *cfg = &node->cfg;

    if (cfg->frame_buffer_count <= primary_idx ||
            !cfg->frame_buffers[primary_idx]) {
        return false;
    }

    if (secondary_idx >= 0) {
        if (cfg->frame_buffer_count <= (uint8_t)secondary_idx ||
                !cfg->frame_buffers[secondary_idx]) {
            return false;
        }
    }

    cfg->draw_buf_pixels = full_pixels;

    if (cfg->base.profile.rotation != ESP_LV_ADAPTER_ROTATE_0) {
        if (cfg->frame_buffer_count <= 2 || !cfg->frame_buffers[2]) {
            return false;
        }
        cfg->draw_buf_primary = cfg->frame_buffers[2];
        cfg->draw_buf_secondary = NULL;
        return true;
    }

    cfg->draw_buf_primary = cfg->frame_buffers[primary_idx];
    cfg->draw_buf_secondary = (secondary_idx >= 0) ? cfg->frame_buffers[secondary_idx] : NULL;
    return true;
}

static bool display_manager_prepare_buffers(esp_lv_adapter_display_node_t *node,
                                            esp_lv_adapter_display_render_mode_t mode,
                                            size_t color_size)
{
    esp_lv_adapter_display_runtime_config_t *cfg = &node->cfg;
    esp_lv_adapter_display_config_t *pub = &cfg->base;
    esp_lv_adapter_display_profile_t *profile = &pub->profile;

    if (!profile->hor_res || !profile->ver_res) {
        ESP_LOGE(TAG, "invalid resolution %ux%u", profile->hor_res, profile->ver_res);
        return false;
    }

    uint8_t required_frames = display_manager_required_frame_buffer_count(pub->tear_avoid_mode, pub->profile.rotation);
    /* For SPI/I2C interfaces with TEAR_AVOID_MODE_NONE, we don't need panel frame buffers */
    if (pub->profile.interface == ESP_LV_ADAPTER_PANEL_IF_OTHER && pub->tear_avoid_mode == ESP_LV_ADAPTER_TEAR_AVOID_MODE_NONE) {
        required_frames = 0;
    }
    bool have_panel_fb = false;
    if (required_frames) {
        have_panel_fb = display_manager_fetch_panel_frame_buffers(node, required_frames, color_size);
    }

    size_t full_pixels = (size_t)profile->hor_res * profile->ver_res;

    /* Try to use panel frame buffers for tearing modes */
    switch (pub->tear_avoid_mode) {
    case ESP_LV_ADAPTER_TEAR_AVOID_MODE_DOUBLE_DIRECT:
        if (have_panel_fb && display_manager_use_panel_buffers(node, full_pixels, 0, 1)) {
            return true;
        }
        ESP_LOGW(TAG, "double direct mode falling back to allocated buffers");
        break;
    case ESP_LV_ADAPTER_TEAR_AVOID_MODE_DOUBLE_FULL:
        if (have_panel_fb && display_manager_use_panel_buffers(node, full_pixels, 0, 1)) {
            return true;
        }
        ESP_LOGW(TAG, "double full mode falling back to allocated buffers");
        break;
    case ESP_LV_ADAPTER_TEAR_AVOID_MODE_TRIPLE_FULL:
        if (have_panel_fb && display_manager_use_panel_buffers(node, full_pixels, 1, 2)) {
            return true;
        }
        ESP_LOGW(TAG, "triple full mode falling back to allocated buffers");
        break;
    case ESP_LV_ADAPTER_TEAR_AVOID_MODE_TRIPLE_PARTIAL:
        if (!have_panel_fb || node->cfg.frame_buffer_count < 3) {
            ESP_LOGW(TAG, "triple partial mode without panel frame buffers, behaviour degraded");
        }
        cfg->draw_buf_pixels = (size_t)profile->hor_res * profile->buffer_height;
        void *buf = display_manager_alloc_draw_buffer(cfg->draw_buf_pixels * color_size, false);
        if (!buf) {
            ESP_LOGE(TAG, "alloc primary buffer %zu bytes failed", cfg->draw_buf_pixels * color_size);
            return false;
        }
        cfg->draw_buf_primary = buf;
        cfg->draw_buf_secondary = NULL;

        return true;
    default:
        break;
    }

    /* Allocate buffers manually */
    uint8_t buffer_count = display_manager_required_buffer_count(cfg, mode);

    if (cfg->base.tear_avoid_mode == ESP_LV_ADAPTER_TEAR_AVOID_MODE_NONE && profile->require_double_buffer) {
        if (buffer_count < 2) {
            buffer_count = 2;
        }
    }

    if (!cfg->draw_buf_pixels) {
        cfg->draw_buf_pixels = display_manager_default_buffer_pixels(cfg, mode);
    }

    size_t buf_bytes = cfg->draw_buf_pixels * color_size;
    if (!buf_bytes) {
        ESP_LOGE(TAG, "draw buffer size invalid (pixels=%zu color_size=%zu)", cfg->draw_buf_pixels, color_size);
        return false;
    }

    bool need_secondary = buffer_count >= 2;

    bool use_psram = profile->use_psram;

    if (!cfg->draw_buf_primary) {
        void *buf = display_manager_alloc_draw_buffer(buf_bytes, use_psram);
        if (!buf) {
            ESP_LOGE(TAG, "alloc primary buffer %zu bytes failed", buf_bytes);
            return false;
        }
        cfg->draw_buf_primary = buf;
    }

    if (need_secondary && !cfg->draw_buf_secondary) {
        void *buf = display_manager_alloc_draw_buffer(buf_bytes, use_psram);
        if (!buf) {
            ESP_LOGE(TAG, "alloc secondary buffer %zu bytes failed", buf_bytes);
            cfg->draw_buf_secondary = NULL;
            return false;
        }
        cfg->draw_buf_secondary = buf;
    } else {
        if (!need_secondary && cfg->draw_buf_secondary) {
            ESP_LOGW(TAG, "secondary buffer provided but not required by mode");
        }
        cfg->draw_buf_secondary = NULL;
    }

    return true;
}

/**********************
 *   FRAME BUFFER HELPERS
 **********************/

/**
 * @brief Fetch frame buffers from LCD panel driver
 */
static bool display_manager_fetch_panel_frame_buffers(esp_lv_adapter_display_node_t *node,
                                                      uint8_t required,
                                                      size_t color_size)
{
    if (!node || !node->cfg.base.panel || !required) {
        return false;
    }

    if (node->cfg.frame_buffer_count >= required && node->cfg.frame_buffers[0]) {
        return true;
    }

    void *fb0 = NULL;
    void *fb1 = NULL;
    void *fb2 = NULL;
    esp_err_t err = ESP_ERR_NOT_SUPPORTED;

    switch (node->cfg.base.profile.interface) {
    case ESP_LV_ADAPTER_PANEL_IF_RGB:
#if SOC_LCDCAM_RGB_LCD_SUPPORTED
        if (required == 1) {
            err = esp_lcd_rgb_panel_get_frame_buffer(node->cfg.base.panel, 1, &fb0);
        } else if (required == 2) {
            err = esp_lcd_rgb_panel_get_frame_buffer(node->cfg.base.panel, 2, &fb0, &fb1);
        } else if (required >= 3) {
            err = esp_lcd_rgb_panel_get_frame_buffer(node->cfg.base.panel, 3, &fb0, &fb1, &fb2);
        }
#endif
        break;
    case ESP_LV_ADAPTER_PANEL_IF_MIPI_DSI:
#if SOC_MIPI_DSI_SUPPORTED
        if (required == 1) {
            err = esp_lcd_dpi_panel_get_frame_buffer(node->cfg.base.panel, 1, &fb0);
        } else if (required == 2) {
            err = esp_lcd_dpi_panel_get_frame_buffer(node->cfg.base.panel, 2, &fb0, &fb1);
        } else if (required >= 3) {
            err = esp_lcd_dpi_panel_get_frame_buffer(node->cfg.base.panel, 3, &fb0, &fb1, &fb2);
        }
#endif
        break;
    case ESP_LV_ADAPTER_PANEL_IF_OTHER:
    default:
        break;
    }

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "panel frame buffer request failed (err=%d, required=%u)", err, required);
        return false;
    }

    node->cfg.frame_buffers[0] = fb0;
    node->cfg.frame_buffers[1] = fb1;
    node->cfg.frame_buffers[2] = fb2;
    node->cfg.frame_buffer_count = required;
    node->cfg.frame_buffer_size = (size_t)node->cfg.base.profile.hor_res * node->cfg.base.profile.ver_res * color_size;
    return true;
}

/**
 * @brief Calculate number of draw buffers required
 */
static uint8_t display_manager_required_buffer_count(const esp_lv_adapter_display_runtime_config_t *cfg,
                                                     esp_lv_adapter_display_render_mode_t mode)
{
    switch (cfg->base.tear_avoid_mode) {
    case ESP_LV_ADAPTER_TEAR_AVOID_MODE_TRIPLE_FULL:
        return 2;
    case ESP_LV_ADAPTER_TEAR_AVOID_MODE_TRIPLE_PARTIAL:
        return 1;
    case ESP_LV_ADAPTER_TEAR_AVOID_MODE_DOUBLE_FULL:
    case ESP_LV_ADAPTER_TEAR_AVOID_MODE_DOUBLE_DIRECT:
        return 2;
    default:
        break;
    }

    if (mode == ESP_LV_ADAPTER_DISPLAY_RENDER_MODE_FULL || mode == ESP_LV_ADAPTER_DISPLAY_RENDER_MODE_DIRECT) {
        return 2;
    }
    return 1;
}

/**
 * @brief Calculate number of panel frame buffers required
 *
 * This function is also exposed as a public API (esp_lv_adapter_get_required_frame_buffer_count)
 * for users to query the required buffer count before hardware initialization.
 *
 */
uint8_t display_manager_required_frame_buffer_count(esp_lv_adapter_tear_avoid_mode_t tear_avoid_mode,
                                                    esp_lv_adapter_rotation_t rotation)
{
    /* Rotation 90° or 270° always requires 3 buffers for rotation processing */
    if (rotation == ESP_LV_ADAPTER_ROTATE_90 || rotation == ESP_LV_ADAPTER_ROTATE_270) {
        return 3;
    }

    /* Determine buffer count based on tearing mode */
    switch (tear_avoid_mode) {
    case ESP_LV_ADAPTER_TEAR_AVOID_MODE_TRIPLE_FULL:
    case ESP_LV_ADAPTER_TEAR_AVOID_MODE_TRIPLE_PARTIAL:
        return 3;
    case ESP_LV_ADAPTER_TEAR_AVOID_MODE_DOUBLE_FULL:
    case ESP_LV_ADAPTER_TEAR_AVOID_MODE_DOUBLE_DIRECT:
        return 2;
    case ESP_LV_ADAPTER_TEAR_AVOID_MODE_NONE:
        /* Return 1 for RGB/MIPI DSI hardware minimum requirement */
        return 1;
    default:
        return 1;
    }
}

/**
 * @brief Calculate default buffer size in pixels
 */
static size_t display_manager_default_buffer_pixels(const esp_lv_adapter_display_runtime_config_t *cfg,
                                                    esp_lv_adapter_display_render_mode_t mode)
{
    const esp_lv_adapter_display_profile_t *profile = &cfg->base.profile;

    size_t full_pixels = (size_t)profile->hor_res * profile->ver_res;
    if (mode == ESP_LV_ADAPTER_DISPLAY_RENDER_MODE_FULL) {
        return full_pixels;
    }

    uint16_t effective_height = display_manager_effective_buffer_height(cfg);

    size_t line_pixels = (size_t)profile->hor_res * effective_height;
    if (!line_pixels) {
        line_pixels = full_pixels ? full_pixels : 1;
    }
    return line_pixels;
}

/**
 * @brief Calculate effective buffer height for partial mode
 */
static uint16_t display_manager_effective_buffer_height(const esp_lv_adapter_display_runtime_config_t *cfg)
{
    /* Default buffer height divisor: 10% of screen height for partial updates */
#define DEFAULT_BUFFER_HEIGHT_DIVISOR  (10)  /* Use 1/10 of vertical resolution */
#define MINIMUM_BUFFER_HEIGHT          (1)   /* Minimum buffer height in pixels */

    const esp_lv_adapter_display_profile_t *profile = &cfg->base.profile;

    uint16_t height = profile->buffer_height;
    if (!height || height > profile->ver_res) {
        /* Default to 10% of screen height for reasonable memory/performance balance */
        height = profile->ver_res / DEFAULT_BUFFER_HEIGHT_DIVISOR;
        if (!height) {
            height = profile->ver_res ? profile->ver_res : MINIMUM_BUFFER_HEIGHT;
        }
    }
    return height;
}

/**********************
 *   VALIDATION & CONFIGURATION
 **********************/

/**
 * @brief Validate tearing mode is compatible with panel interface
 */
static bool display_manager_validate_tearing_mode(const esp_lv_adapter_display_config_t *cfg)
{
    if (!cfg) {
        return false;
    }

    esp_lv_adapter_tear_avoid_mode_t mode = cfg->tear_avoid_mode;

    switch (cfg->profile.interface) {
    case ESP_LV_ADAPTER_PANEL_IF_RGB:
    case ESP_LV_ADAPTER_PANEL_IF_MIPI_DSI:
        if (mode == ESP_LV_ADAPTER_TEAR_AVOID_MODE_NONE ||
                mode == ESP_LV_ADAPTER_TEAR_AVOID_MODE_DOUBLE_FULL ||
                mode == ESP_LV_ADAPTER_TEAR_AVOID_MODE_TRIPLE_FULL ||
                mode == ESP_LV_ADAPTER_TEAR_AVOID_MODE_DOUBLE_DIRECT ||
                mode == ESP_LV_ADAPTER_TEAR_AVOID_MODE_TRIPLE_PARTIAL) {
            return true;
        }
        ESP_LOGE(TAG, "tear mode %d unsupported on panel interface %d", mode, cfg->profile.interface);
        return false;
    case ESP_LV_ADAPTER_PANEL_IF_OTHER:
    default:
        if (mode == ESP_LV_ADAPTER_TEAR_AVOID_MODE_NONE) {
            return true;
        }
        ESP_LOGE(TAG, "tear mode %d unsupported on panel interface %d", mode, cfg->profile.interface);
        return false;
    }
}

/**
 * @brief Select appropriate render mode based on tearing mode
 */
static esp_lv_adapter_display_render_mode_t display_manager_pick_render_mode(const esp_lv_adapter_display_runtime_config_t *cfg)
{
    switch (cfg->base.tear_avoid_mode) {
    case ESP_LV_ADAPTER_TEAR_AVOID_MODE_DOUBLE_FULL:
    case ESP_LV_ADAPTER_TEAR_AVOID_MODE_TRIPLE_FULL:
        return ESP_LV_ADAPTER_DISPLAY_RENDER_MODE_FULL;
    case ESP_LV_ADAPTER_TEAR_AVOID_MODE_DOUBLE_DIRECT:
        return ESP_LV_ADAPTER_DISPLAY_RENDER_MODE_DIRECT;
    default:
        return ESP_LV_ADAPTER_DISPLAY_RENDER_MODE_PARTIAL;
    }
}

/**********************
 *   FPS STATISTICS
 **********************/

#if CONFIG_ESP_LVGL_ADAPTER_ENABLE_FPS_STATS
/**
 * @brief Update FPS statistics when a frame is fully rendered
 *
 * Called exactly once per LVGL frame (after the last flush completes).
 * Implements a 1-second sliding window with minimal overhead.
 *
 * @param node Display node containing FPS statistics
 */
static void display_manager_fps_frame_done(esp_lv_adapter_display_node_t *node)
{
    /* FPS calculation time window */
#define FPS_UPDATE_INTERVAL_US  (1000000)  /* 1 second in microseconds */

    if (!node || !node->fps_stats.enabled) {
        return;
    }

    int64_t now = esp_timer_get_time();  /* Get current time in microseconds */

    /* Increment frame counter */
    node->fps_stats.frame_count++;

    /* Calculate FPS every 1 second */
    int64_t elapsed = now - node->fps_stats.window_start_time;
    if (elapsed >= FPS_UPDATE_INTERVAL_US) {
        /* Calculate FPS using integer arithmetic to avoid FPU in ISR
         * Formula: (frame_count * FPS_UPDATE_INTERVAL_US) / elapsed */
        node->fps_stats.current_fps = (uint32_t)((int64_t)node->fps_stats.frame_count * FPS_UPDATE_INTERVAL_US / elapsed);

        /* Reset for next window */
        node->fps_stats.frame_count = 0;
        node->fps_stats.window_start_time = now;
    }
}
#endif
