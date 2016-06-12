/*
 *	Blinky Application
 */

#include "contiki.h"
#include "dev/leds.h"
#include "ti-lib.h"
#include "driverlib/aon_batmon.h"
#include "aon_batmon.h"
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
PROCESS(temp_sense_process, "Temperature Sensor process");
AUTOSTART_PROCESSES(&temp_sense_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(temp_sense_process, ev, data)
{
	static struct etimer et;

	PROCESS_BEGIN();

    ti_lib_aon_batmon_enable();

	PRINTF("Temperature Sensor Process started...\r\n");

    etimer_set(&et, CLOCK_SECOND);

	PROCESS_PAUSE();
    while(1) {
    	PROCESS_YIELD();
    	if(etimer_expired(&et)) {
            etimer_reset(&et);
            // PRINTF("Temperature RAW Val: %3x\r\n", AONBatMonTemperatureGetRAW());
    		PRINTF("Temperature in Deg C: %d\r\n", ti_lib_aon_batmon_temperature_get_deg_c());
    	}
    }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
