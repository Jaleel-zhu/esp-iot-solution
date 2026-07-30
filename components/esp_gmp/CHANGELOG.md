# ChangeLog

## v0.0.2 - 2026-07-28

### Bug Fixes:

* Embed `psa_hash_operation_t` in `esp_gmp_sha256_ctx_t` instead of a fixed opaque buffer, fixing `_Static_assert` failures when `psa_hash_operation_t` grows with SHA-512 (e.g. ESP32-C3 defaults).
* Fix `components/esp_gmp/test` build discovery via `EXTRA_COMPONENT_DIRS` for `esp_gmp` / `esp_flux`, and enable NimBLE in `sdkconfig.defaults`.

### Enhancements:

* Add GitLab CI compile jobs for BLE GMP OTA client/server examples and the component unit test app (BLE-only targets).
* Add `idf_component.yml` for Component Manager metadata and dependencies.

## v0.0.1 - 2026-07-16

### Enhancements:

* Initial release of the ESP-GMP management protocol core (framing, CRC, multi-link TX).
* Add Flux/GATT adapter (`esp_gmp_flux_*`) for BLE stacks using `esp_flux`.
* Add optional OTA submodule with incremental SHA256 transfer integrity verify.
* Add BLE GMP OTA client/server examples and component unit tests.
