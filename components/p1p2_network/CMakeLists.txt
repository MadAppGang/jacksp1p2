set(NET_REQUIRES
    nvs_flash
    esp_timer
    app_update
)

# When esp-matter SDK is available, add OpenThread dependencies
if(DEFINED ENV{ESP_MATTER_PATH})
    list(APPEND NET_REQUIRES openthread esp_openthread)
endif()

idf_component_register(
    SRCS
        "p1p2_thread.c"
        "p1p2_ota.c"
        "p1p2_config_store.c"
    INCLUDE_DIRS "include"
    REQUIRES ${NET_REQUIRES}
)

# Define compile flag so C code knows the SDK is available
if(DEFINED ENV{ESP_MATTER_PATH})
    target_compile_definitions(${COMPONENT_LIB} PUBLIC P1P2_MATTER_SDK_AVAILABLE=1)
endif()
