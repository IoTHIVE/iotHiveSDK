all: mqtt-client

CONTIKI = ../../..

### TARGET DECLARATION
TARGET = hiveMote
CPU_FAMILY = cc26xx
# CPU_FAMILY = cc13xx

CONTIKI_WITH_IPV6 = 1

APPS += mqtt

### Arduino Shield being used
SHIELD = SEEED_RELAY_BOARD_V2

PROJECT_CONF_H=\"project-conf.h\"

include $(CONTIKI)/Makefile.include