# Host-side tests. Needs only gcc (or clang) + make.   Usage: make -C Tests test
CC      ?= gcc
CFLAGS  := -std=c11 -g -O1 -Wall -Wextra -Wshadow -Wconversion -Wstrict-prototypes -Werror \
           -fsanitize=address,undefined -fno-sanitize-recover=undefined \
           -I../Drivers/Common -I../Drivers/MS5611 -I../Drivers/BMP581 -I../Services/AirData -I../Drivers/RSC -I../Services/Filters -I../Services/SensorManager -I../Drivers/CAN -I../Application
all: test_ms5611 test_bmp581 test_airdata test_rsc test_filters test_sensor_manager test_dronecan test_app
test_ms5611: test_ms5611.c ../Drivers/MS5611/ms5611.c ; $(CC) $(CFLAGS) -o $@ $^
test_bmp581: test_bmp581.c ../Drivers/BMP581/bmp581.c ; $(CC) $(CFLAGS) -o $@ $^
test_airdata: test_airdata.c ../Services/AirData/airdata.c ; $(CC) $(CFLAGS) -o $@ $^ -lm
test_rsc: test_rsc.c ../Drivers/RSC/rsc.c ; $(CC) $(CFLAGS) -o $@ $^ -lm
test_filters: test_filters.c ../Services/Filters/filters.c ; $(CC) $(CFLAGS) -o $@ $^ -lm
test_sensor_manager: test_sensor_manager.c ../Services/SensorManager/sensor_manager.c ../Services/Filters/filters.c ; $(CC) $(CFLAGS) -o $@ $^ -lm
test_dronecan: test_dronecan.c ../Drivers/CAN/dronecan.c ; $(CC) $(CFLAGS) -o $@ $^ -lm
APP_SRC := ../Application/app.c ../Drivers/MS5611/ms5611.c ../Drivers/BMP581/bmp581.c ../Drivers/RSC/rsc.c ../Drivers/CAN/dronecan.c \
          ../Services/AirData/airdata.c ../Services/Filters/filters.c ../Services/SensorManager/sensor_manager.c
test_app: test_app.c $(APP_SRC) ; $(CC) $(CFLAGS) -o $@ $^ -lm
test: all ; ./test_ms5611 && ./test_bmp581 && ./test_airdata && ./test_rsc && ./test_filters && ./test_sensor_manager && ./test_dronecan && ./test_app
clean: ; rm -f test_ms5611 test_bmp581 test_airdata test_rsc test_filters test_sensor_manager test_dronecan test_app test_filters
