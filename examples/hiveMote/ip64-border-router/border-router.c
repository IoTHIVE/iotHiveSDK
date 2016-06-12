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
/**
 * \file
 *         border-router
 * \author
 *         Niclas Finne <nfi@sics.se>
 *         Joakim Eriksson <joakime@sics.se>
 *         Nicolas Tsiftes <nvt@sics.se>
 */

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/ip/uip.h"
#include "net/ipv6/uip-ds6.h"
#include "net/rpl/rpl.h"

#include "ip64.h"
/* Statistics only */
#include "ip64-addrmap.h"

#include "net/netstack.h"
#include "dev/button-sensor.h"
#include "dev/slip.h"
#include "dev/leds.h"

#include "stats.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"
/*---------------------------------------------------------------------------*/
// #define PUSH_MSG_PORT           61616
// #define PUSH_MSG_BUFSIZE        400
/*---------------------------------------------------------------------------
#define ADD(...)                                                        \
  do                                                                    \
  {                                                                     \
    push_msg_len += snprintf((char*)&push_msg_buf[push_msg_len],        \
                             PUSH_MSG_BUFSIZE - push_msg_len, __VA_ARGS__); \
  } while(0)
---------------------------------------------------------------------------*/
// static uint8_t push_msg_buf[PUSH_MSG_BUFSIZE];
// static int push_msg_len;
static struct udp_socket udp_socket;
// static uint32_t pkt_no;
static uip_ipaddr_t ripaddr;
/*---------------------------------------------------------------------------*/
/* Various configuration parameters */
/* The interval at which we toggle the "operating" LED (in seconds)*/
#define BLINK_INTERVAL                  2
/* The interval we use to dump statistics. Set to 0 to turn this feature off */
#define STAT_INTERVAL                   0//10
/*---------------------------------------------------------------------------*/
/* static uip_ipaddr_t prefix; */
/* static uint8_t ip4addr_set; */

/* The Contiki prefix */
static const uint8_t contiki_pfx[8] = {0xfd,0x4d,0x42,0x67,0x5f,0x8c,0,0};

#if STAT_INTERVAL
static int stat_prescaler;
#endif
/*---------------------------------------------------------------------------*/
PROCESS(border_router_process, "Border router process");
AUTOSTART_PROCESSES(&border_router_process);
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{

  int i;
  uint8_t state;

  uip_ipaddr_t addr;

  addr.u8[0] = 0;
  addr.u8[1] = 0;
  addr.u8[2] = 0;
  addr.u8[3] = 0;

  addr.u8[4] = 0;
  addr.u8[5] = 0;
  addr.u8[6] = 0;
  addr.u8[7] = 0;

  addr.u8[8] = 0;
  addr.u8[9] = 0;
  addr.u8[10] = 0xff;
  addr.u8[11] = 0xff;

  addr.u8[12] = 192;
  addr.u8[13] = 168;
  addr.u8[14] = 1;
  addr.u8[15] = 1;

  memcpy((void*)&ripaddr,
         (const void*)&addr,
         sizeof(uip_ipaddr_t));
    
  printf("IPv6 addresses:\n");
  
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      printf(" ");
      uip_debug_ipaddr_print(&uip_ds6_if.addr_list[i].ipaddr);
      printf("\n");
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
create_rpl_dag(void)
{

  rpl_dag_t *dag;
  uip_ipaddr_t ipaddr;
  uip_ipaddr_t prefix;

  memset(&prefix, 0, 16);
  memcpy(&prefix, contiki_pfx, 8);
  memcpy(&ipaddr, contiki_pfx, 16);

  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

  dag = rpl_set_root(RPL_DEFAULT_INSTANCE, &ipaddr);
  if(dag != NULL) {
    rpl_set_prefix(dag, &prefix, 64);
    PRINTF("Created a new RPL dag using prefix:\r\n ");
    uip_debug_ipaddr_print(&prefix);
    PRINTF("\n");
  } else {
    PRINTF("Failed to create a new RPL dag!\r\n");
  }
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(border_router_process, ev, data)
{

  static struct etimer et;

  PROCESS_BEGIN();
  PRINTF("IP64 BORDER ROUTER PROCESS STARTED...\r\n");
  /* 
   * While waiting for the IPv4 address to be assigned via DHCP, the future
   * border router can join an existing DAG as a parent or child, 
   * or acquire a default router that will later take precedence over the 
   * fallback interface.
   * Prevent that by turning the radio off until we are initialized as a DAG 
   * root.
   */

  NETSTACK_MAC.off(0);

  /* 
   * Initialize the IP64 module so we'll start translating packets.
   * If IP64_CONF_DHCP is set, we will first try to optain an IPv4 
   * address via DHCP.
   */

  ip64_init();

  #if !IP64_CONF_DHCP
  #error You have to set IP64_CONF_DHCP in order to obtain an IPv4 address! 
  #endif

  leds_off(LEDS_ALL);

  /* Wait for IPv4 address to be assigned via DHCP */
  PRINTF("Requesting IPv4 address via DHCPv4\r\n");
  for(;;) {

    if (ip64_hostaddr_is_configured()) {

      const uip_ip4addr_t *hostaddr = ip64_get_hostaddr();
      const uip_ip4addr_t *netmask = ip64_get_netmask();
      const uip_ip4addr_t *draddr = ip64_get_draddr();

      leds_off(LEDS_ALL);

      PRINTF("assigned via DHCP:\n");
      PRINTF(" IPv4 address  : %d.%d.%d.%d\n",
           hostaddr->u8[0], hostaddr->u8[1],
           hostaddr->u8[2], hostaddr->u8[3]);
      PRINTF(" netmask       : %d.%d.%d.%d\n",
           netmask->u8[0], netmask->u8[1],
           netmask->u8[2], netmask->u8[3]);
      PRINTF(" default router: %d.%d.%d.%d\n",
           draddr->u8[0], draddr->u8[1],
           draddr->u8[2], draddr->u8[3]);
      break;
    }

    PRINTF(".");
    leds_toggle(LEDS_ALL);

    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
  }

  /* Now turn the radio on, but disable radio duty cycling */
  NETSTACK_MAC.off(1);

  create_rpl_dag();

  print_local_addresses();

  PRINTF("RPL border router up and running\n");

  udp_socket_register(&udp_socket, NULL, NULL);

  while(1) {
    PROCESS_YIELD();

  } /* while() */

  PROCESS_END();

}
/*---------------------------------------------------------------------------*/
