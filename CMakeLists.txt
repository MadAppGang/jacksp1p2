cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    "components/p1p2_bus"
    "components/p1p2_protocol"
    "components/p1p2_matter"
    "components/p1p2_network"
    "components/p1p2_cli"
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# If using esp-matter SDK, include it
if(DEFINED ENV{ESP_MATTER_PATH})
    set(MATTER_SDK_PATH $ENV{ESP_MATTER_PATH})
    list(APPEND EXTRA_COMPONENT_DIRS
        "${MATTER_SDK_PATH}/components"
        "${MATTER_SDK_PATH}/device_hal/device"
        "${MATTER_SDK_PATH}/examples/common"
    )
    # connectedhomeip path
    if(DEFINED ENV{MATTER_SDK_PATH})
        set(CHIP_ROOT $ENV{MATTER_SDK_PATH})
    else()
        set(CHIP_ROOT "${MATTER_SDK_PATH}/connectedhomeip/connectedhomeip")
    endif()
endif()

project(p1p2mqtt_esp32c6)
