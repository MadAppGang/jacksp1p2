idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        p1p2_bus
        p1p2_protocol
        p1p2_matter
        p1p2_network
        p1p2_cli
        driver
        esp_timer
        nvs_flash
)
