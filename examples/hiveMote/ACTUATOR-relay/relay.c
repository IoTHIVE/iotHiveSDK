/*
 *	Blinky Application
 */

#include "contiki.h"
#include "dev/leds.h"
#include "button-sensor.h"
#include "ti-lib.h"
/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINT6ADDR(addr) PRINTF("[%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x]", ((uint8_t *)addr)[0], ((uint8_t *)addr)[1], ((uint8_t *)addr)[2], ((uint8_t *)addr)[3], ((uint8_t *)addr)[4], ((uint8_t *)addr)[5], ((uint8_t *)addr)[6], ((uint8_t *)addr)[7], ((uint8_t *)addr)[8], ((uint8_t *)addr)[9], ((uint8_t *)addr)[10], ((uint8_t *)addr)[11], ((uint8_t *)addr)[12], ((uint8_t *)addr)[13], ((uint8_t *)addr)[14], ((uint8_t *)addr)[15])
#define PRINTLLADDR(lladdr) PRINTF("[%02x:%02x:%02x:%02x:%02x:%02x]", (lladdr)->addr[0], (lladdr)->addr[1], (lladdr)->addr[2], (lladdr)->addr[3], (lladdr)->addr[4], (lladdr)->addr[5])
#else
#define PRINTF(...)
#define PRINT6ADDR(addr)
#define PRINTLLADDR(addr)
#endif
/*---------------------------------------------------------------------------*/
PROCESS(relay_process, "Relay process");
AUTOSTART_PROCESSES(&relay_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(relay_process, ev, data)
{
    static uint8_t toggle_flag = 0x00;
	
    PROCESS_BEGIN();
	
    PRINTF("Relay Process started...\r\n");

    SENSORS_ACTIVATE(button_sensor);
    ti_lib_ioc_pin_type_gpio_output(RELAY_1_IOID);
    ti_lib_ioc_pin_type_gpio_output(RELAY_2_IOID);
    ti_lib_ioc_pin_type_gpio_output(RELAY_3_IOID);
    ti_lib_ioc_pin_type_gpio_output(RELAY_4_IOID);


    // ti_lib_gpio_pin_write(RELAY_1_IOID, toggle_flag);

    while(1) {
    	PROCESS_YIELD();

        if(ev == sensors_event && data == &button_sensor) {
            
            if(toggle_flag)
                toggle_flag = 0x00;
            else
                toggle_flag = 0x01;

            // toggle_flag = (~toggle_flag) & 0x01;
            ti_lib_gpio_pin_write((1 << RELAY_1_IOID), toggle_flag);
            ti_lib_gpio_pin_write((1 << RELAY_2_IOID), toggle_flag);
            ti_lib_gpio_pin_write((1 << RELAY_3_IOID), toggle_flag);
            ti_lib_gpio_pin_write((1 << RELAY_4_IOID), toggle_flag);
    	}
    }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
