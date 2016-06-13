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
PROCESS(motor_process, "Motor process");
AUTOSTART_PROCESSES(&motor_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(motor_process, ev, data)
{
    static uint8_t toggle_spin = 0x00;
	
    PROCESS_BEGIN();
	
    PRINTF("Motor Process started...\r\n");

    SENSORS_ACTIVATE(button_sensor);

    ti_lib_ioc_pin_type_gpio_output(CHANNEL_0_ENABLE);
    ti_lib_ioc_pin_type_gpio_output(CHANNEL_0_OUT1);
    ti_lib_ioc_pin_type_gpio_output(CHANNEL_0_OUT2);
    ti_lib_ioc_pin_type_gpio_output(CHANNEL_1_ENABLE);
    ti_lib_ioc_pin_type_gpio_output(CHANNEL_1_OUT1);
    ti_lib_ioc_pin_type_gpio_output(CHANNEL_1_OUT2);

    ti_lib_gpio_pin_write((1 << CHANNEL_0_ENABLE), 1); // Enabling Channel 0
    ti_lib_gpio_pin_write((1 << CHANNEL_1_ENABLE), 1); // Enabling Channel 1

    while(1) {
    	PROCESS_YIELD();

        if(ev == sensors_event && data == &button_sensor) {
            toggle_spin = (~toggle_spin) & 0x01;
            ti_lib_gpio_pin_write((1 << CHANNEL_0_OUT1), toggle_spin & 0x01);
            ti_lib_gpio_pin_write((1 << CHANNEL_0_OUT2), ~toggle_spin & 0x01);

            ti_lib_gpio_pin_write((1 << CHANNEL_1_OUT1), toggle_spin & 0x01);
            ti_lib_gpio_pin_write((1 << CHANNEL_1_OUT2), ~toggle_spin & 0x01);
    	}
    }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
