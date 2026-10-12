# Host-side self-tests. No hardware, no HAL: the real Devices sources are
# compiled against a mock BSP, or are pure logic with no BSP at all.
#
#   make -C tests run

CC      ?= gcc
CFLAGS  ?= -std=c11 -Wall -Wextra -Werror -O2
INCLUDES = -I../Devices -I../Platform

NVRAM_SRCS  = test_nvram.c ../Devices/drv_nvram.c ../Devices/drv_eeprom.c
HM_KEY_SRCS = test_hm_key.c ../Devices/drv_hm_key_decode.c
BINS = test_nvram test_hm_key

.PHONY: all run clean

all: $(BINS)

test_nvram: $(NVRAM_SRCS)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(NVRAM_SRCS)

test_hm_key: $(HM_KEY_SRCS)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(HM_KEY_SRCS)

run: $(BINS)
	./test_nvram
	./test_hm_key

clean:
	rm -f $(BINS)