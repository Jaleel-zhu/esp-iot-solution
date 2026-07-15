# ChangeLog

## v0.0.2 - 2026-07-15

### Bug Fixes:

* Notify adapters of in-flight zero-copy sends dropped during `flux_session_destroy` via `session_complete_cb` (`ESP_ERR_INVALID_STATE`), so caller-owned source buffers can be released.
* Add `gatt_session_schedule_destroy()` to post GATT session teardown onto the NimBLE host event queue, avoiding reentrancy when destroy is requested outside the host task.

## v0.0.1 - 2026-06-12

### Enhancements:

* Initial release of the ESP Flux transport and GATT session component.
