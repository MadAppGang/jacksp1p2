set(MATTER_SRCS
    "p1p2_matter_device.c"
    "p1p2_matter_thermostat.c"
    "p1p2_matter_fan.c"
    "p1p2_matter_sensors.c"
    "p1p2_matter_custom.c"
)

set(MATTER_REQUIRES
    p1p2_protocol
    p1p2_bus
)

# When esp-matter SDK is available, add the C++ bridge and SDK dependencies
if(DEFINED ENV{ESP_MATTER_PATH})
    list(APPEND MATTER_SRCS "p1p2_matter_bridge.cpp")
    list(APPEND MATTER_REQUIRES esp_matter chip openthread)
endif()

idf_component_register(
    SRCS ${MATTER_SRCS}
    INCLUDE_DIRS "include"
    REQUIRES ${MATTER_REQUIRES}
)

# Define compile flag so C code knows the bridge is available
if(DEFINED ENV{ESP_MATTER_PATH})
    target_compile_definitions(${COMPONENT_LIB} PUBLIC P1P2_MATTER_SDK_AVAILABLE=1)
endif()
