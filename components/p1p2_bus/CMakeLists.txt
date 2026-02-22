idf_component_register(
    SRCS
        "p1p2_mcpwm_rx.c"
        "p1p2_mcpwm_tx.c"
        "p1p2_bus.c"
        "p1p2_adc.c"
    INCLUDE_DIRS "include"
    REQUIRES
        driver
        esp_timer
        esp_adc
        hal
)
