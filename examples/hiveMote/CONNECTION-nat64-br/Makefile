
CONTIKI_PROJECT = border-router
all: $(CONTIKI_PROJECT) 

### Specify where CONTIKI resides
CONTIKI = ../../..

### TARGET DECLARATION
TARGET = hiveMote

# CPU_FAMILY=cc26xx
CPU_FAMILY=cc13xx

### STACK INCLUDES
CONTIKI_WITH_IP64=1
WITH_ENC28J60=1

ifeq ($(WITH_ENC28J60),1)
	CFLAGS += -DHAS_ENC28J60=1
	APPS += enc28j60
endif

CONTIKI_WITH_IPV6 = 1

### Include project specific configuration file
DEFINES += PROJECT_CONF_H=\"project-conf.h\"

### Include Contiki's global makefile
include $(CONTIKI)/Makefile.include
