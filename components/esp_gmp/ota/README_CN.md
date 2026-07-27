# ESP-GMP OTA（直写 OTA 分区）

[English](README.md) | **中文**

GMP OTA 设备侧处理器：收到 `OTA_UPLOAD_DATA` 后通过 `esp_ota_write()` 直接写入下一个 OTA 分区，**不经 SPIFFS 暂存**。

流程：

1. `OTA_UPLOAD_CONTROL(start)` → `esp_ota_begin()`（内含目标 OTA 分区擦除）
2. `OTA_UPLOAD_DATA` → 接收/入队后响应；后台 worker 执行 `esp_ota_write()` + 增量 SHA256
3. `OTA_UPLOAD_CONTROL(finish + sha256)` → drain 队列 → 校验 → `esp_ota_end()` → 切换启动分区 → 重启

`image_size` 为完整镜像总大小；每次 `start` 固定从镜像偏移 0 开始传输，**不支持断点续传**。`OTA_UPLOAD_CONTROL(abort)` / `erase` 仅重置服务端会话状态，**不**单独擦除 flash（物理擦除由 start / `esp_ota_begin` 完成）。

`OTA_UPLOAD_DATA` 的 `WRITE_RSP OK` 只表示该 chunk 已被当前会话接受并进入处理队列，不表示 flash 已经持久化。整包最终结果以 `finish` 响应为准。

SHA256 只用于传输完整性校验，不证明固件来源可信；`esp_gmp_ota_on_packet()` 也不判断调用方是否有 OTA 权限。生产产品需要自行接入 signed app、secure boot、链路鉴权或业务授权策略。

协议布局见 [`SPEC.md`](../SPEC.md) §12。

## 集成

```c
#include "esp_gmp.h"
#include "esp_gmp_ota.h"

static bool on_gmp_packet(void *ctx, const esp_gmp_rx_t *pkt)
{
    (void)ctx;
    if (pkt->group_id == ESP_GMP_GRP_OTA) {
        return esp_gmp_ota_on_packet(pkt);
    }
    /* OS_CAP_QUERY 等 */
    return false;
}

void app_init(void)
{
    esp_gmp_init();
    ESP_ERROR_CHECK(esp_gmp_ota_init(NULL));
    esp_gmp_on_packet_register(on_gmp_packet, NULL);
}
```

断链时调用 `esp_gmp_ota_on_link_down(link)`。

## Kconfig

| 选项 | 说明 |
|------|------|
| `ESP_GMP_OTA_CHUNK_HINT` | start 响应中的建议块大小；`0`=按 link 自动计算（默认 `0`） |
| `ESP_GMP_OTA_RESTART_DELAY_MS` | 刷写成功后延迟重启（默认 `500` ms） |
| `ESP_GMP_OTA_APPLY_TASK_STACK` | apply 任务栈大小（字节，默认 `4096`） |
| `ESP_GMP_OTA_APPLY_TASK_PRIO` | apply 任务优先级（默认 `5`） |

## 目录

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

固件文件管理（SPIFFS 等）属于应用层；示例见 `examples/esp_gmp/gmp_ota/ble_gmp_ota_client`。
