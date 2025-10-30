# ChangeLog

## v0.1.0-beta.1 (2025-10-27)

* Support for LVGL v8.x and v9.x with unified API
* Unified display registration for MIPI DSI, RGB, QSPI, SPI, I2C, I80 interfaces
* Multiple buffering modes and screen rotation support (0°/90°/180°/270°)
* Tearing effect avoidance for RGB and MIPI DSI displays
* PPA hardware acceleration support for ESP32-P4
* Thread-safe LVGL access with lock/unlock APIs
* Input device support: touch screen, button navigation, rotary encoder
* Optional filesystem bridge integration (esp_lv_fs)
* Optional image decoder support (esp_lv_decoder)
* Optional FreeType font rendering support
* FPS statistics and Dummy Draw mode
* API naming aligned with esp_lv_decoder and esp_lv_fs for consistency
