cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    "components/p1p2_bus"
    "components/p1p2_protocol"
    "components/p1p2_matter"
    "components/p1p2_network"
    "components/p1p2_cli"
)

# If using esp-matter SDK, include it
if(DEFINED ENV{ESP_MATTER_PATH})
    set(ESP_MATTER_PATH $ENV{ESP_MATTER_PATH})
    set(MATTER_SDK_PATH ${ESP_MATTER_PATH}/connectedhomeip/connectedhomeip)

    # esp-matter components and chip component
    list(APPEND EXTRA_COMPONENT_DIRS
        "${ESP_MATTER_PATH}/components"
        "${MATTER_SDK_PATH}/config/esp32/components"
    )
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(p1p2mqtt_esp32c6)

# Matter-specific compile options (match esp-matter examples)
if(DEFINED ENV{ESP_MATTER_PATH})
    idf_build_set_property(CXX_COMPILE_OPTIONS "-std=gnu++17;-Os;-DCHIP_HAVE_CONFIG_H;-Wno-overloaded-virtual" APPEND)
    idf_build_set_property(C_COMPILE_OPTIONS "-Os" APPEND)
    idf_build_set_property(COMPILE_OPTIONS "-Wno-format-nonliteral;-Wno-format-security" APPEND)
endif()
