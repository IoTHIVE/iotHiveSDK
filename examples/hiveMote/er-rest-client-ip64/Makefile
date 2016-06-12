all:er-example-client

CONTIKI = ../../..

### TARGET DECLARATION
TARGET = hiveMote
# CPU_FAMILY=cc26xx
CPU_FAMILY=cc13xx

### STACK DECLARATION
APPS += er-coap
APPS += rest-engine

CONTIKI_WITH_IPV6 = 1

CFLAGS += -DPROJECT_CONF_H=\"project-conf.h\"

include $(CONTIKI)/Makefile.include
