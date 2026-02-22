idf_component_register(
    SRCS
        "p1p2_matter_device.c"
        "p1p2_matter_thermostat.c"
        "p1p2_matter_fan.c"
        "p1p2_matter_sensors.c"
        "p1p2_matter_custom.c"
    INCLUDE_DIRS "include"
    REQUIRES
        p1p2_protocol
        p1p2_bus
)

# Note: When esp-matter SDK is available, add these REQUIRES:
#   chip esp_matter esp_matter_console
# For now, Matter integration compiles as stubs that can be
# linked against the real SDK when available.
