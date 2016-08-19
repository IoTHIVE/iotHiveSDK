/*
 * Copyright (c) 2013, Institute for Pervasive Computing, ETH Zurich
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 */

/**
 * \file
 *      Erbium (Er) CoAP client example.
 * \author
 *      Matthias Kovatsch <kovatsch@inf.ethz.ch>
 */
/*---------------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "contiki.h"
#include "contiki-net.h"
#include "er-coap-engine.h"
#include "ip64-addr.h"
#include "dev/leds.h"

#if HAS_BUTTON
#include "button-sensor.h"
#endif
#ifdef  HAS_ADXL345
#include "adxl345.h"
#endif

#if (HAS_LM35 || HAS_LDR)
#include "ti-lib.h"
#include "adc-sensor.h"
#define   ADC_FIXED_REF_VOLATGE   4300000 /* 4.3 V == 4300000 uV */
#endif
#ifdef HAS_MPU6050
#include "mpu-6050-sensor.h"
#endif
/*---------------------------------------------------------------------------*/
#define DEBUG 1
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#define PRINT6ADDR(addr) PRINTF("[%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x]", ((uint8_t *)addr)[0], ((uint8_t *)addr)[1], ((uint8_t *)addr)[2], ((uint8_t *)addr)[3], ((uint8_t *)addr)[4], ((uint8_t *)addr)[5], ((uint8_t *)addr)[6], ((uint8_t *)addr)[7], ((uint8_t *)addr)[8], ((uint8_t *)addr)[9], ((uint8_t *)addr)[10], ((uint8_t *)addr)[11], ((uint8_t *)addr)[12], ((uint8_t *)addr)[13], ((uint8_t *)addr)[14], ((uint8_t *)addr)[15])
#define PRINTLLADDR(lladdr) PRINTF("[%02x:%02x:%02x:%02x:%02x:%02x]", (lladdr)->addr[0], (lladdr)->addr[1], (lladdr)->addr[2], (lladdr)->addr[3], (lladdr)->addr[4], (lladdr)->addr[5])
#else
#define PRINTF(...)
#define PRINT6ADDR(addr)
#define PRINTLLADDR(addr)
#endif
/*---------------------------------------------------------------------------*/
/* IoTHive Cloud (188.166.219.178) */
#define SERVER_NODE(ipaddr)   uip_ipaddr(ipaddr, 188, 166, 219, 178)
#define REMOTE_PORT           UIP_HTONS(COAP_DEFAULT_PORT)
/*---------------------------------------------------------------------------*/
PROCESS(er_example_client, "Erbium Example Client");
AUTOSTART_PROCESSES(&er_example_client);
/*---------------------------------------------------------------------------*/
uip_ip4addr_t server_ip4addr;
uip_ip6addr_t server_ip6addr;
uint8_t flag = 0;
#if HAS_BUTTON
uint8_t toggle = 0;
#endif
/*---------------------------------------------------------------------------*/
#define NUMBER_OF_URLS 5

/* /token/deviceId/<SENS/ACT>/<sensorId/actuatorId> */
char *service_urls[NUMBER_OF_URLS] = {  ".well-known/core", 
                                        "/c4daee07/200000/SENS/0", /* Accelerometer */
                                        "/c4daee07/200001/SENS/1", /* Temperature */
                                        "/508cff8c/000000/SENS/3", /* Button */
                                        "/508cff8c/000000/SENS/4", /* LDR */
                                      };
/*---------------------------------------------------------------------------*/
/* This function is will be passed to COAP_BLOCKING_REQUEST() to handle responses. */
void client_chunk_handler(void *response){

  const uint8_t *chunk;

  int len = coap_get_payload(response, &chunk);
  PRINTF("|%s\n\r", (char *)chunk);
  // PRINTF("|%.*s", len, (char *)chunk);
}
/*---------------------------------------------------------------------------*/
static void
route_callback(int event, uip_ipaddr_t *route, uip_ipaddr_t *ipaddr, int numroutes)
{
  if(event == UIP_DS6_NOTIFICATION_DEFRT_ADD) 
  {
    leds_on(LEDS_GREEN);
    if(flag == 0)
    {
      process_poll(&er_example_client);
      flag=1;
    }
  }
  else if(event == UIP_DS6_NOTIFICATION_DEFRT_RM)
  {
    leds_off(LEDS_GREEN);
  }
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(er_example_client, ev, data)
{

  static struct etimer et;
  static coap_packet_t request[1];
	static struct uip_ds6_notification n;
  PROCESS_BEGIN();
  uip_ds6_notification_add(&n, route_callback);

  SERVER_NODE(&server_ip4addr);
  ip64_addr_4to6(&server_ip4addr, &server_ip6addr);

#if HAS_BUTTON
  SENSORS_ACTIVATE(button_sensor);
#endif
  coap_init_engine();

#if  (HAS_ADXL345 || HAS_MPU6050)
  accm_init();
#endif

#if (HAS_LM35 || HAS_LDR)
  SENSORS_ACTIVATE(adc_sensor);
  adc_sensor.configure(ADC_SENSOR_SET_CHANNEL, ADC_COMPB_IN_AUXIO7);
#endif

  PRINTF("CoAP Sensor and Actuator Demo Started.\r\n");

  while(1) {
    PROCESS_YIELD();

    if(ev == PROCESS_EVENT_POLL)
    {
    	etimer_set(&et, 2 * CLOCK_SECOND);
    }
    else if(etimer_expired(&et))
    {
#ifdef  HAS_ADXL345
    	PRINTF("Sending Accelerometer Data...\r\n");
      leds_on(LEDS_RED);
      coap_init_message(request, COAP_TYPE_CON, COAP_POST, 0);
      coap_set_header_uri_path(request, service_urls[1]);
      coap_set_header_content_format(request, TEXT_PLAIN);

      char msg1[15];
      memset(msg1, 0, 15);
      int16_t x, y, z;
      x = accm_read_axis(X_AXIS);
      y = accm_read_axis(Y_AXIS);
      z = accm_read_axis(Z_AXIS);
      PRINTF("x:%d,y:%d,z:%d\n\r", x, y, z);

      snprintf(msg1, 15, "%d,%d,%d:", x, y, z);
      coap_set_payload(request, (uint8_t *)msg1, sizeof(msg1) - 1);
      COAP_BLOCKING_REQUEST(&server_ip6addr, REMOTE_PORT, request, client_chunk_handler);
      leds_off(LEDS_RED);
      PRINTF("Done!!!\n\r");
#endif

#ifdef  HAS_MPU6050
      PRINTF("Sending Accelerometer Data...\r\n");
      leds_on(LEDS_RED);
      coap_init_message(request, COAP_TYPE_CON, COAP_POST, 0);
      coap_set_header_uri_path(request, service_urls[1]);
      coap_set_header_content_format(request, TEXT_PLAIN);

      char msg1[15];
      memset(msg1, 0, 15);
      int16_t x, y, z;
      x = accm_read_axis(X_AXIS);
      y = accm_read_axis(Y_AXIS);
      z = accm_read_axis(Z_AXIS);
      PRINTF("x:%d,y:%d,z:%d\n\r", x, y, z);

      snprintf(msg1, 15, "%d,%d,%d:", x, y, z);
      coap_set_payload(request, (uint8_t *)msg1, sizeof(msg1) - 1);
      COAP_BLOCKING_REQUEST(&server_ip6addr, REMOTE_PORT, request, client_chunk_handler);
      leds_off(LEDS_RED);
      PRINTF("Done!!!\n\r");
#endif

#ifdef  HAS_LM35
      PRINTF("Sending Temperature Data...\r\n");
      leds_on(LEDS_RED);
      coap_init_message(request, COAP_TYPE_CON, COAP_POST, 0);
      coap_set_header_uri_path(request, service_urls[2]);
      coap_set_header_content_format(request, TEXT_PLAIN);

      char msg2[5];
      memset(msg2, 0, 5);
      uint8_t intVal = 0, fracVal = 0;
      intVal = ti_lib_aux_adc_value_to_microvolts(ADC_FIXED_REF_VOLATGE, adc_sensor.value(ADC_SENSOR_VALUE))/10000;
      fracVal = adc_sensor.value(ADC_SENSOR_VALUE) - intVal * 10000;
      PRINTF("Temperature: %2d.%2d\r\n", intVal, fracVal);
      snprintf(msg2, 5, "%2d.%2d", intVal, fracVal);
      coap_set_payload(request, (uint8_t *)msg2, sizeof(msg2) - 1);
      COAP_BLOCKING_REQUEST(&server_ip6addr, REMOTE_PORT, request, client_chunk_handler);
      leds_off(LEDS_RED);
      PRINTF("Done!!!\n\r");
#endif

#ifdef  HAS_LDR
      PRINTF("Sending LDR Data...\r\n");
      leds_on(LEDS_RED);
      coap_init_message(request, COAP_TYPE_CON, COAP_POST, 0);
      coap_set_header_uri_path(request, service_urls[4]);
      coap_set_header_content_format(request, TEXT_PLAIN);
      char dataLDR[4];
      snprintf(dataLDR, 3, "%3d",
        ti_lib_aux_adc_value_to_microvolts(ADC_FIXED_REF_VOLATGE, adc_sensor.value(ADC_SENSOR_VALUE)));
      PRINTF("LDR: %s\r\n", dataLDR);
      coap_set_payload(request, (uint8_t *)dataLDR, sizeof(dataLDR) - 1);
      COAP_BLOCKING_REQUEST(&server_ip6addr, REMOTE_PORT, request, client_chunk_handler);
      leds_off(LEDS_RED);
      PRINTF("Done!!!\n\r");
#endif
      etimer_reset(&et);
    }
#ifdef  HAS_BUTTON
    else if(ev == sensors_event && data == &button_sensor) {
      PRINTF("Sending Button Toggle...\r\n");
      leds_on(LEDS_RED);
      coap_init_message(request, COAP_TYPE_CON, COAP_POST, 0);
      coap_set_header_uri_path(request, service_urls[3]);
      coap_set_header_content_format(request, TEXT_PLAIN);
      char msg3[4];
      snprintf(msg3, 4, "1:%d", toggle);
      coap_set_payload(request, (uint8_t *)msg3, sizeof(msg3) - 1);
      COAP_BLOCKING_REQUEST(&server_ip6addr, REMOTE_PORT, request, client_chunk_handler);
      toggle = ~toggle & 0x01;
      leds_off(LEDS_RED);
      PRINTF("Done!!!\n\r");
    }
#endif
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/