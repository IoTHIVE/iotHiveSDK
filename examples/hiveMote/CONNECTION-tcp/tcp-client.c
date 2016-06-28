/*
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
 *
 */

#include "contiki.h"
#include "lib/random.h"
#include "sys/ctimer.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/ip/uip-udp-packet.h"
#include "net/ipv6/sicslowpan.h"
#include "sys/ctimer.h"
#ifdef WITH_COMPOWER
#include "powertrace.h"
#endif
#include <stdio.h>
#include <string.h>

#include "dev/leds.h"

// #define CLIENT_PORT 8765
#define SERVER_PORT 8080

#define DEBUG DEBUG_PRINT
#include "net/ip/uip-debug.h"

#ifndef PERIOD
#define PERIOD 1
#endif

#define SEND_INTERVAL		(PERIOD * CLOCK_SECOND)
#define MAX_PAYLOAD_LEN		30

static struct uip_conn *client_conn;
static uip_ipaddr_t server_ipaddr;
uint8_t flag = 0;
/*---------------------------------------------------------------------------*/
PROCESS(tcp_client_process, "TCP client process");
AUTOSTART_PROCESSES(&tcp_client_process);
/*---------------------------------------------------------------------------*/
static void
tcpip_handler(void)
{
  char *str;

  if(uip_newdata()) {
    str = uip_appdata;
    str[uip_datalen()] = '\0';
    printf("DATA recv '%s'\n", str);
    PRINTF("with RSSI: %d", sicslowpan_get_last_rssi());
  }
}
/*---------------------------------------------------------------------------*/
static void
send_packet()
{
  static int seq_id;
  char buf[MAX_PAYLOAD_LEN];

  leds_toggle(LEDS_RED);

  seq_id++;
  PRINTF("DATA send to %d 'Hello %d'\n",
         server_ipaddr.u8[sizeof(server_ipaddr.u8) - 1], seq_id);
  sprintf(buf, "Hello %d from the client\n\r", seq_id);
  uip_send(buf, strlen(buf));
  // uip_udp_packet_sendto(client_conn, buf, strlen(buf),
                        // &server_ipaddr, UIP_HTONS(SERVER_PORT));
}
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  PRINTF("Client IPv6 addresses: ");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      PRINTF("\n\r");
      /* hack to make address "final" */
      if (state == ADDR_TENTATIVE) {
	uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
set_global_address(void)
{
  uip_ipaddr_t ipaddr;

  uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

  uip_ip6addr(&server_ipaddr, 0xaaaa, 0, 0, 0, 0x0212, 0x4b00, 0x07b4, 0xe300);
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
      process_poll(&tcp_client_process);
      flag=1;
    }
  }
  else if(event == UIP_DS6_NOTIFICATION_DEFRT_RM)
  {
    leds_off(LEDS_GREEN);
  }
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(tcp_client_process, ev, data)
{
  static struct etimer periodic;
#if WITH_COMPOWER
  static int print = 0;
#endif

  static struct uip_ds6_notification n;
  PROCESS_BEGIN();
  uip_ds6_notification_add(&n, route_callback);

  PROCESS_PAUSE();

  set_global_address();
  
  PRINTF("TCP client process started\n\r");

  print_local_addresses();

  PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_POLL);

  /* new connection with remote host */
  client_conn = tcp_connect(&server_ipaddr, UIP_HTONS(SERVER_PORT), NULL);
  // tcp_attach();
  if(client_conn == NULL) {
    PRINTF("No TCP connection available, exiting the process!\n\r");
    PROCESS_EXIT();
  }
  printf("Connecting...\n\r");
  PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event);

  if(uip_aborted() || uip_timedout() || uip_closed()) {
    printf("Could not establish connection\n");
  } else if(uip_connected()) {
    printf("Connected\n\r");
  }

 //  PRINTF("Created a connection with the server ");
 //  PRINT6ADDR(&client_conn->ripaddr);
 //  PRINTF(" local/remote port %u/%u\n",
	// UIP_HTONS(client_conn->lport), UIP_HTONS(client_conn->rport));

#if WITH_COMPOWER
  powertrace_sniff(POWERTRACE_ON);
#endif

  etimer_set(&periodic, SEND_INTERVAL);
  while(1) {
    PROCESS_YIELD();
    if(ev == tcpip_event) {
      if((uip_closed() || uip_aborted() || uip_timedout()))
      {
        printf("Connection closed.\n\r");
      }
      else
      {
        tcpip_handler();
      }
    }
    
    if(etimer_expired(&periodic)) {
      etimer_reset(&periodic);
      if(!uip_connected())
        send_packet();

#if WITH_COMPOWER
      if (print == 0) {
	powertrace_print("#P");
      }
      if (++print == 3) {
	print = 0;
      }
#endif

    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
