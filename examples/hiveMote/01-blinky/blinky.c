/*
 *	Blinky Application
 */

#include "contiki.h"
#include "dev/leds.h"
#ifdef BLINK_USING_BUTTON
#include "button-sensor.h"
#endif
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
PROCESS(blinky_process, "Blinky process");
AUTOSTART_PROCESSES(&blinky_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(blinky_process, ev, data)
{
#ifndef BLINK_USING_BUTTON
	static struct etimer et;
#endif
	PROCESS_BEGIN();
	PRINTF("Blinky Process started...\r\n");
#ifdef BLINK_USING_BUTTON
    SENSORS_ACTIVATE(button_sensor);
#else
    etimer_set(&et, CLOCK_SECOND);
#endif

	PROCESS_PAUSE();
    while(1) {
    	PROCESS_YIELD();
#ifdef BLINK_USING_BUTTON
        if(ev == sensors_event && data == &button_sensor) {
            PRINTF("Button Pressed!!!\r\n");
#else
    	if(etimer_expired(&et)) {
            etimer_reset(&et);
#endif
    		leds_toggle(LEDS_ALL);
    	}
    }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
