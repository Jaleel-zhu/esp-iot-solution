idf_component_register(SRCS "esp_flux_transport.c" "flux_gatt_session.c"
                        INCLUDE_DIRS include
                        REQUIRES freertos esp_timer bt)
