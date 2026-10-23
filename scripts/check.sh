#!/bin/sh
# One-command quality gate: static analysis (gcc -fanalyzer + strict warnings) then host tests.
set -eu
cd "$(dirname "$0")/.."
INC="-IDrivers/Common -IDrivers/MS5611 -IDrivers/BMP581 -IDrivers/RSC -IDrivers/CAN -IServices/AirData -IServices/Filters -IServices/SensorManager -IApplication"
WARN="-Wall -Wextra -Wpedantic -Wconversion -Wshadow -Wcast-qual -Wundef -Wdouble-promotion \
      -Wformat=2 -Wstrict-prototypes -Wmissing-prototypes -Wswitch-enum -Werror"
for f in Drivers/MS5611/ms5611.c Drivers/BMP581/bmp581.c Drivers/RSC/rsc.c Drivers/CAN/dronecan.c \
         Services/AirData/airdata.c Services/Filters/filters.c Services/SensorManager/sensor_manager.c Application/app.c; do
    echo "[analyze] $f"
    gcc -std=c11 -fsyntax-only -fanalyzer $WARN $INC "$f"
done
echo "[analyze] Platform/stm32h7 (syntax only, against the HOST STUB, not the real HAL)"
for f in Platform/stm32h7/adb_platform.c Platform/stm32h7/app_main.c; do
    gcc -std=c11 -fsyntax-only -Wall -Wextra -Wconversion -Wshadow -Werror -DADB_HOST_SYNTAX_CHECK \
        -IPlatform/stub -IPlatform/stm32h7 $INC "$f"
done
echo "[test] host unit tests (ASan + UBSan)"
make -C Tests clean test
echo "ALL CHECKS PASSED"
