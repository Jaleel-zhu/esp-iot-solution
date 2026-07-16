idf_component_register(
    SRCS
        "gmp_ota_client.c"
        "gmp_session_teardown.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_gmp esp_flux freertos
)
