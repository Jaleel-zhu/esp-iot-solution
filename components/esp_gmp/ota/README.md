# ESP-GMP OTA (direct write to OTA partition)

**English** | [中文](README_CN.md)

Device-side GMP OTA handler: on `OTA_UPLOAD_DATA`, firmware is written to the next OTA partition with `esp_ota_write()`. There is **no SPIFFS staging** in this submodule.

Flow:

1. `OTA_UPLOAD_CONTROL(start)` → `esp_ota_begin()` (includes erase of the target OTA partition)
2. `OTA_UPLOAD_DATA` → accept/enqueue then respond; a background worker runs `esp_ota_write()` and incremental SHA256
3. `OTA_UPLOAD_CONTROL(finish + sha256)` → drain the queue → verify → `esp_ota_end()` → set boot partition → restart

`image_size` is the full image size. Every `start` always transfers from image offset 0; **resume / partial upload is not supported**. `OTA_UPLOAD_CONTROL(abort)` / `erase` only reset the server session; they do **not** erase flash by themselves (physical erase happens at start / `esp_ota_begin`).

`WRITE_RSP OK` for `OTA_UPLOAD_DATA` means the chunk was accepted into the current session queue, not that flash has been persisted. The final result of the whole image is reported by the `finish` response.

SHA256 is for transfer integrity only; it does not prove firmware provenance. `esp_gmp_ota_on_packet()` also does **not** decide whether the caller is authorized to perform OTA. Production products must add signed app / secure boot, link authentication, and/or application-level authorization.

Protocol layout: [`SPEC.md`](../SPEC.md) §12.

## Integration

```c
#include "esp_gmp.h"
#include "esp_gmp_ota.h"

static bool on_gmp_packet(void *ctx, const esp_gmp_rx_t *pkt)
{
    (void)ctx;
    if (pkt->group_id == ESP_GMP_GRP_OTA) {
        return esp_gmp_ota_on_packet(pkt);
    }
    /* OS_CAP_QUERY, … */
    return false;
}

void app_init(void)
{
    esp_gmp_init();
    ESP_ERROR_CHECK(esp_gmp_ota_init(NULL));
    esp_gmp_on_packet_register(on_gmp_packet, NULL);
}
```

On link disconnect, call `esp_gmp_ota_on_link_down(link)`.

## Kconfig

| Option | Description |
|--------|-------------|
| `ESP_GMP_OTA_CHUNK_HINT` | Suggested chunk size in the start response; `0` = auto from the link (default `0`) |
| `ESP_GMP_OTA_RESTART_DELAY_MS` | Delay before reboot after a successful apply (default `500` ms) |
| `ESP_GMP_OTA_APPLY_TASK_STACK` | Apply-task stack size in bytes (default `4096`) |
| `ESP_GMP_OTA_APPLY_TASK_PRIO` | Apply-task priority (default `5`) |

## Layout

```text
ota/
├── include/
│   ├── esp_gmp_ota_proto.h
│   ├── esp_gmp_ota.h
│   └── esp_gmp_sha256.h
└── src/
    ├── esp_gmp_ota_proto.c
    ├── esp_gmp_ota.c
    └── esp_gmp_sha256.c
```

Firmware file management (SPIFFS, etc.) belongs to the application. See `examples/esp_gmp/gmp_ota/ble_gmp_ota_client`.
