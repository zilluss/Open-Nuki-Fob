# compilation flags for gdb

CFLAGS += -O3 -g3
CFLAGS += -DNRF51822
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DS130
CFLAGS += -DNRF_SD_BLE_API_VERSION=2
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DSWI_DISABLE0
CFLAGS += -DSHA2_USE_INTTYPES_H
CFLAGS += -DBYTE_ORDER=LITTLE_ENDIAN
CFLAGS += -Iinclude
ASMFLAGS += -O3 -g3 
ASMFLAGS += -DNRF51822
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DS130
ASMFLAGS += -DNRF_SD_BLE_API_VERSION=2
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DSWI_DISABLE0
ASMFLAGS += -D__HEAP_SIZE=0

LDSCRIPT = s130_v2_nrf51822_QFAA.ld

INC_PATHS += -I$(SDK_PATH)/components/softdevice/s130/headers
INC_PATHS += -I./src
INC_PATHS += -I./external

VPATH=src:external

# object files
OBJS =  nrf_log_frontend.o nrf_ble_gatt.o crc16.o nrf_queue.o nrf_drv_rng.o
OBJS+= nrf_log_backend_serial.o nrf_drv_uart.o
OBJS += app_button.o app_error.o app_error_weak.o app_fifo.o app_timer.o app_util_platform.o
OBJS += hardfault_implementation.o nrf_assert.o
OBJS += nrf_drv_clock.o nrf_drv_common.o nrf_drv_gpiote.o  ble_flash.o
OBJS += tweetnacl.o sha2.o hmac_sha256.o
OBJS += utils.o bt_comm.o nuki.o nuki_unlock.o nuki_pairing.o main.o 
OBJS += ble_db_discovery.o ble_advdata.o ble_conn_params.o ble_srv_common.o
OBJS += system_nrf51.o softdevice_handler.o

# include common make file
include Makefile.common
