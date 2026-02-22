#!/bin/bash
# P1P2MQTT ESP32-C6 build helper
# Usage: source build.sh       (sets up environment)
#        ./build.sh build       (builds firmware)
#        ./build.sh flash       (flashes to device)
#        ./build.sh monitor     (serial monitor)
#        ./build.sh menuconfig  (Kconfig menu)

export IDF_PATH="$HOME/esp/esp-idf"
export IDF_PYTHON_ENV_PATH="$HOME/.espressif/python_env/idf5.4_py3.14_env"

# Source ESP-IDF environment
source "$IDF_PATH/export.sh" 2>/dev/null

cd "$(dirname "$0")"

if [ -n "$1" ]; then
    idf.py "$@"
fi
