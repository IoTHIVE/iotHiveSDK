/*
 *	Description: Light Sensor Application
 *  Sensor Used: Light Dependent Resistor (Analog)
 *
 */

#include "contiki.h"
#include "dev/leds.h"
#include "ti-lib.h"
#include "adc-sensor.h"
/*---------------------------------------------------------------------------*/
#define ADC_FIXED_REF_VOLATGE   4300000 /* 4.3 V == 4300000 uV */
/*---------------------------------------------------------------------------*/
#define DEBUG 1
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
PROCESS(light_sense_process, "Light Sensor process");
AUTOSTART_PROCESSES(&light_sense_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(light_sense_process, ev, data)
{
	static struct etimer et;
    uint8_t intVal = 0, fracVal = 0;

	PROCESS_BEGIN();

    SENSORS_ACTIVATE(adc_sensor);
    adc_sensor.configure(ADC_SENSOR_SET_CHANNEL, ADC_COMPB_IN_AUXIO7);

	PRINTF("Light Sensor Process started...\r\n");

    etimer_set(&et, CLOCK_SECOND);

    while(1) {
    	PROCESS_YIELD();
    	if(etimer_expired(&et)) {
            PRINTF("Light Reading: %d\r\n", adc_sensor.value(ADC_SENSOR_VALUE));
            etimer_reset(&et);
    	}
    }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
