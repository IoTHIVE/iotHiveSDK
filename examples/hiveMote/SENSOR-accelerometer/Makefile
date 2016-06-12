all: accelerometer

CONTIKI=../../..

### TARGET DECLARATION
TARGET = hiveMote
CPU_FAMILY = cc26xx
# CPU_FAMILY = cc13xx

### Use ADXL345 as accelerometer
WITH_ADXL345=1

# TODO: Move to a platform Makefile
ifeq ($(WITH_ADXL345),1)
	CFLAGS += -DHAS_ADXL345=1
endif

include $(CONTIKI)/Makefile.include
