all:coap-client-demo

CONTIKI = ../../..

### TARGET DECLARATION
TARGET = hiveMote
CPU_FAMILY=cc26xx
# CPU_FAMILY=cc13xx

### STACK DECLARATION
APPS += er-coap
APPS += rest-engine

CONTIKI_WITH_IPV6 = 1

### Use ADXL345 as accelerometer
WITH_ADXL345=1

### Use LM35 as temperature sensor
# WITH_LM35=1

# TODO: Move to a platform Makefile
ifeq ($(WITH_ADXL345),1)
	CFLAGS += -DADXL345_SPI_INTERFACE=1
	CFLAGS += -DHAS_ADXL345=1
endif

ifeq ($(WITH_LM35),1)
	CFLAGS += -DHAS_LM35=1
endif

CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"

include $(CONTIKI)/Makefile.include
