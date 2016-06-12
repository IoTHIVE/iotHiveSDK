/*
 *	Blinky Application
 */
#ifndef _ACCELEROMETER_SENSOR_APP_C_
#define _ACCELEROMETER_SENSOR_APP_C_

#include "contiki.h"
#include "adxl345.h"
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
PROCESS(accm_sense_process, "Accelerometer Sensor process");
AUTOSTART_PROCESSES(&accm_sense_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(accm_sense_process, ev, data)
{
	static struct etimer et;
    int16_t x, y, z;

	PROCESS_BEGIN();

    accm_init();

	PRINTF("Accelerometer Sensor Process started...\r\n");

    etimer_set(&et, CLOCK_SECOND);

	PROCESS_PAUSE();
    while(1) {

    	PROCESS_YIELD();
    	if(etimer_expired(&et)) {
            x = accm_read_axis(X_AXIS);
            y = accm_read_axis(Y_AXIS);
            z = accm_read_axis(Z_AXIS);
            PRINTF("x: %d y: %d z: %d\n\r", x, y, z);
            etimer_reset(&et);
    	}
    }
    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
#endif /* _ACCELEROMETER_SENSOR_APP_C_ */