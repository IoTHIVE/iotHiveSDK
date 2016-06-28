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

### Accelerometer Used
### Options for hiveMote:
### > NO SENSOR - leave blank
### > ADXL345 (SPI or I2C)
### > MPU6050 (I2C only)
ACCELEROMETER_USED =  ADXL345
ACCELEROMETER_INTERFACE =  SPI

### Temperature Sensor Used
### Options for hiveMote:
### > NO SENSOR - leave blank
### > LM35 (ANALOG)
TEMP_SENSOR_USED =  LM35
TEMP_SENSOR_INTERFACE = ANALOG

### Light Sensor Used
### Options for hiveMote:
### > NO SENSOR - leave blank
### > LDR (ANALOG)
LIGHT_SENSOR_USED =  LDR
LIGHT_SENSOR_INTERFACE = ANALOG

### Use Button to send an event
# WITH_BUTTON = 1

ifneq ($(ACCELEROMETER_USED),)
	WITH_$(ACCELEROMETER_USED)=1
	CFLAGS += -D$(ACCELEROMETER_USED)_$(ACCELEROMETER_INTERFACE)_INTERFACE=1
	CFLAGS += -DHAS_$(ACCELEROMETER_USED)=1
endif

ifneq ($(TEMP_SENSOR_USED),)
	WITH_$(TEMP_SENSOR_USED)=1
	CFLAGS += -D$(TEMP_SENSOR_USED)_$(TEMP_SENSOR_INTERFACE)_INTERFACE=1
	CFLAGS += -DHAS_$(TEMP_SENSOR_USED)=1
endif

ifneq ($(LIGHT_SENSOR_USED),)
	WITH_$(LIGHT_SENSOR_USED)=1
	CFLAGS += -D$(LIGHT_SENSOR_USED)_$(LIGHT_SENSOR_INTERFACE)_INTERFACE=1
	CFLAGS += -DHAS_$(LIGHT_SENSOR_USED)=1
endif

endif

ifeq ($(WITH_BUTTON),1)
	CFLAGS += -DHAS_BUTTON=1
endif

CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"

include $(CONTIKI)/Makefile.include
