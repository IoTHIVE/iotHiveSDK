/*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \addtogroup cc26xx-web-demo
 * @{
 *
 * \file
 *   MQTT/IBM cloud service client for the CC26XX web demo.
 */
/*---------------------------------------------------------------------------*/
#include "contiki-conf.h"
#include "rpl/rpl-private.h"
#include "mqtt.h"
#include "net/rpl/rpl.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-icmp6.h"
#include "sys/etimer.h"
#include "sys/ctimer.h"
#include "lib/sensors.h"
#include "button-sensor.h"
#include "clock.h"
#include "dev/leds.h"
#include "mqtt-client.h"
#include "ti-lib.h"

#include <string.h>
#include <strings.h>
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
/*
 * IBM server: messaging.quickstart.internetofthings.ibmcloud.com
 * (184.172.124.189) mapped in an NAT64 (prefix 64:ff9b::/96) IPv6 address
 * Note: If not able to connect; lookup the IP address again as it may change.
 *
 * If the node has a broker IP setting saved on flash, this value here will
 * get ignored
 */
// static char *broker_ip = "0064:ff9b:0000:0000:0000:0000:b8ac:7cbd";
// static char *broker_ip = "0000:0000:0000:0000:0000:ffff:c0a8:106"; /* 192.168.1.6 */
// static char *broker_ip = "0000:0000:0000:0000:0000:ffff:341d:c1d8"; /* broker.hivemq.com (52.29.193.216) */
static char *broker_ip = "0000:0000:0000:0000:0000:ffff:bca6:dbb2"; /* iothive cloud (188.166.219.178) */

/*---------------------------------------------------------------------------*/
/*
 * A timeout used when waiting for something to happen (e.g. to connect or to
 * disconnect)
 */
#define STATE_MACHINE_PERIODIC     (CLOCK_SECOND >> 1)
/*---------------------------------------------------------------------------*/
/* Provide visible feedback via LEDS during various states */
/* When connecting to broker */
#define CONNECTING_LED_DURATION    (CLOCK_SECOND >> 3)

/* Each time we try to publish */
#define PUBLISH_LED_ON_DURATION    (CLOCK_SECOND)
/*---------------------------------------------------------------------------*/
/* Connections and reconnections */
#define RETRY_FOREVER              0xFF
#define RECONNECT_INTERVAL         (CLOCK_SECOND * 2)

/*
 * Number of times to try reconnecting to the broker.
 * Can be a limited number (e.g. 3, 10 etc) or can be set to RETRY_FOREVER
 */
#define RECONNECT_ATTEMPTS         5
#define CONNECTION_STABLE_TIME     (CLOCK_SECOND * 5)
#define NEW_CONFIG_WAIT_INTERVAL   (CLOCK_SECOND * 20)
static struct timer connection_life;
static uint8_t connect_attempt;
/*---------------------------------------------------------------------------*/
/* Various states */
static uint8_t state;
#define MQTT_CLIENT_STATE_INIT            0
#define MQTT_CLIENT_STATE_REGISTERED      1
#define MQTT_CLIENT_STATE_CONNECTING      2
#define MQTT_CLIENT_STATE_CONNECTED       3
#define MQTT_CLIENT_STATE_PUBLISHING      4
#define MQTT_CLIENT_STATE_DISCONNECTED    5
#define MQTT_CLIENT_STATE_NEWCONFIG       6
#define MQTT_CLIENT_STATE_CONFIG_ERROR 0xFE
#define MQTT_CLIENT_STATE_ERROR        0xFF
/*---------------------------------------------------------------------------*/
/* Maximum TCP segment size for outgoing segments of our socket */
#define MQTT_CLIENT_MAX_SEGMENT_SIZE    32
#define MQTT_CONNECTED_LED              LEDS_RED
/*---------------------------------------------------------------------------*/
/*
 * Buffers for Client ID and Topic.
 * Make sure they are large enough to hold the entire respective string
 *
 * d:quickstart:status:EUI64 is 32 bytes long
 * iot-2/evt/status/fmt/json is 25 bytes
 * We also need space for the null termination
 */
#define BUFFER_SIZE 64
static char client_id[] = "123456789";
static char pub_topic[] = "/hello/1";
static char sub_topic[] = "/508cff8c/000000/ACT/3";
/*---------------------------------------------------------------------------*/
/*
 * The main MQTT buffers.
 * We will need to increase if we start publishing more data.
 */
#define APP_BUFFER_SIZE 512
static struct mqtt_connection conn;
static char app_buffer[] = "BHAUMIK IS KING!";
/*---------------------------------------------------------------------------*/
// #define QUICKSTART "quickstart"
/*---------------------------------------------------------------------------*/
static struct mqtt_message *msg_ptr = 0;
// static struct etimer publish_periodic_timer;
static struct ctimer subsWait;
static struct ctimer connWait;
static char *buf_ptr;
static uint16_t seq_nr_value = 0;
/*---------------------------------------------------------------------------*/
static uip_ip6addr_t def_route;
static uint8_t flag = 0;
/*---------------------------------------------------------------------------*/
/* Parent RSSI functionality */
// extern int def_rt_rssi;
/*---------------------------------------------------------------------------*/
// const static cc26xx_web_demo_sensor_reading_t *reading;
/*---------------------------------------------------------------------------*/
mqtt_client_config_t *conf;
/*---------------------------------------------------------------------------*/
PROCESS(mqtt_client_process, "CC26XX MQTT Client");
AUTOSTART_PROCESSES(&mqtt_client_process);
/*---------------------------------------------------------------------------*/
static void
route_callback(int event, uip_ipaddr_t *route, uip_ipaddr_t *ipaddr, int numroutes)
{
  if(event == UIP_DS6_NOTIFICATION_DEFRT_ADD) 
  {
      if(flag == 0)
      {
        process_poll(&mqtt_client_process);
        flag=1;
      }
  }
}
/*---------------------------------------------------------------------------*/
static void
connect_to_broker() {
  /* Connect to MQTT server */
  mqtt_connect(&conn, broker_ip, 1883, CLOCK_SECOND * 30);
}
/*---------------------------------------------------------------------------*/
static void
pub_handler(const char *topic, uint16_t topic_len, const uint8_t *chunk, uint16_t chunk_len)
{
  uint16_t i;

  PRINTF("PUB HANDLER: topic='%s' (len=%u), chunk_len=%u, chunk=", topic, topic_len,
      chunk_len);

  for(i=0; i<chunk_len; i++)
  {
    PRINTF("%d",*(chunk+i));
  }
  PRINTF("\n\r");

  if(*chunk == '1')
  {
    if(*(chunk+2) == '1')
      ti_lib_gpio_pin_write((1 << RELAY_3_IOID), 1);
    else if(*(chunk+2) == '0')
      ti_lib_gpio_pin_write((1 << RELAY_3_IOID), 0);
  }
}
/*---------------------------------------------------------------------------*/
static void
subscribe(void)
{
  /* Publish MQTT topic in IBM quickstart format */
  mqtt_status_t status;
  status = mqtt_subscribe(&conn, 1234, sub_topic, MQTT_QOS_LEVEL_0);
  PRINTF("SUBSCRIBE: Requested\n\r");
  if(status == MQTT_STATUS_OUT_QUEUE_FULL) {
    PRINTF("SUBSCRIBE: Tried to subscribe but command queue was full!\n\r");
  }
}
/*---------------------------------------------------------------------------*/
static void
mqtt_event(struct mqtt_connection *m, mqtt_event_t event, void *data)
{
  switch(event) {
  case MQTT_EVENT_CONNECTED: {
    PRINTF("MQTT EVENT: Application has a MQTT connection\n\r");
    leds_on(MQTT_CONNECTED_LED);
    ctimer_set(&subsWait, CLOCK_SECOND * 2, subscribe, NULL);
    // timer_set(&connection_life, CONNECTION_STABLE_TIME);
    state = MQTT_CLIENT_STATE_CONNECTED;
    break;
  }
  case MQTT_EVENT_DISCONNECTED: {
    PRINTF("MQTT EVENT: MQTT Disconnect. Reason %u\n\r", *((mqtt_event_t *)data));
    leds_off(MQTT_CONNECTED_LED);
    // state = MQTT_CLIENT_STATE_DISCONNECTED;
    break;
  }
  case MQTT_EVENT_PUBLISH: {
    msg_ptr = data;

    /* Implement first_flag in publish message? */
    if(msg_ptr->first_chunk) {
      msg_ptr->first_chunk = 0;
      PRINTF("MQTT EVENT: Application received a publish on topic '%s'. Payload "
          "size is %i bytes. Content:\n\n\r",
          msg_ptr->topic, msg_ptr->payload_length);
    }

    pub_handler(msg_ptr->topic, strlen(msg_ptr->topic), msg_ptr->payload_chunk,
                msg_ptr->payload_length);
    break;
  }
  case MQTT_EVENT_SUBACK: {
    PRINTF("MQTT EVENT: Application is subscribed to topic successfully\n\r");
    break;
  }
  case MQTT_EVENT_UNSUBACK: {
    PRINTF("MQTT EVENT: Application is unsubscribed to topic successfully\n\r");
    break;
  }
  case MQTT_EVENT_PUBACK: {
    PRINTF("MQTT EVENT: Publishing complete.\n\r");
    break;
  }
  default:
    PRINTF("MQTT EVENT: Application got a unhandled MQTT event: %i\n\r", event);
    break;
  }
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(mqtt_client_process, ev, data)
{
  static struct uip_ds6_notification n;
  PROCESS_BEGIN();
  uip_ds6_notification_add(&n, route_callback);

  printf("PROCESS: MQTT Client Process Started\n\r");

  state = MQTT_CLIENT_STATE_INIT;
  SENSORS_ACTIVATE(button_sensor);
  ti_lib_ioc_pin_type_gpio_output(RELAY_3_IOID);

  while(1) {

    PROCESS_YIELD();

    if(ev == sensors_event && data == &button_sensor) 
    {
      if(state == MQTT_CLIENT_STATE_CONNECTED)
      {
        // PRINTF("PROCESS EVENT: Subcribing...\n\r");
        if(mqtt_ready(&conn) && conn.out_buffer_sent) 
        {
          mqtt_publish(&conn, NULL, pub_topic, (uint8_t *)app_buffer, strlen(app_buffer), MQTT_QOS_LEVEL_0, MQTT_RETAIN_OFF);
          PRINTF("PROCESS: Publish!\n\r");
        }
      }
    }
    else if(ev == PROCESS_EVENT_POLL) 
    {
      mqtt_register(&conn, &mqtt_client_process, client_id, mqtt_event, MQTT_CLIENT_MAX_SEGMENT_SIZE);
      // mqtt_set_username_password(&conn, "test", "test");
      conn.auto_reconnect = 1;
      PRINTF("Registered. Connect attempt %u\n\r", connect_attempt);
      ctimer_set(&connWait, CLOCK_SECOND * 1, connect_to_broker, NULL);
    }
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 */
