idf_component_register(
    SRCS
        "p1p2_thread.c"
        "p1p2_ota.c"
        "p1p2_config_store.c"
    INCLUDE_DIRS "include"
    REQUIRES
        nvs_flash
        esp_timer
        app_update
)

# Note: When OpenThread is available, add: openthread esp_openthread
