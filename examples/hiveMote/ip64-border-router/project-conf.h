/*
 * Copyright (c) 2015, Weptech elektronik GmbH Germany
 * http://www.weptech.de
 *
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

#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_
/*---------------------------------------------------------------------------*/
// #undef	IEEE802154_CONF_PANID
// #define IEEE802154_CONF_PANID         0xABCD

// #undef	RF_CORE_CONF_CHANNEL
// #define RF_CORE_CONF_CHANNEL          25
/*---------------------------------------------------------------------------*/
/**
 * \name Set up processor speed
 *
 * Run at maximum speed (32 MHz) unless defined elsewhere 
 * @{
 */
// #ifndef SYS_CTRL_CONF_SYS_DIV 
// #define SYS_CTRL_CONF_SYS_DIV           SYS_CTRL_CLOCK_CTRL_SYS_DIV_32MHZ
// #endif /* SYS_CTRL_CONF_SYS_DIV */

// #ifndef SYS_CTRL_CONF_IO_DIV 
// #define SYS_CTRL_CONF_IO_DIV            SYS_CTRL_CLOCK_CTRL_IO_DIV_32MHZ
// #endif /* SYS_CTRL_CONF_IO_DIV */
// /** @} */
// /*---------------------------------------------------------------------------*/
// /*  Let's use the external 32kHz watch crystal */
// #define SYS_CTRL_CONF_OSC32K_USE_XTAL   1
/*---------------------------------------------------------------------------*/
#define UIP_CONF_RECEIVE_WINDOW         128
#define WEBSERVER_CONF_CFS_CONNS        2
#define UIP_CONF_BUFFER_SIZE            900
#define UIP_CONF_TCP_MSS                128
/*---------------------------------------------------------------------------*/
/*
 * Set size of routing table and ND cache. Fortunately this platform has some
 * RAM to spend...
 */
#ifndef UIP_CONF_MAX_ROUTES 
#define UIP_CONF_MAX_ROUTES             32
#endif
#ifndef NBR_TABLE_CONF_MAX_NEIGHBORS  
#define NBR_TABLE_CONF_MAX_NEIGHBORS    32
#endif
/*---------------------------------------------------------------------------*/
/* Let's assume we need an IP64 table entry for each of our children */
#ifndef IP64_ADDRMAP_CONF_ENTRIES  
#define IP64_ADDRMAP_CONF_ENTRIES       32
#endif
/*---------------------------------------------------------------------------*/
/* Radio configuration */
//#define PLATFORM_CONF_USE_CC1200        1
/*---------------------------------------------------------------------------*/
/* Configure the UIP fallback interface: we are using ethernet */
#define UIP_FALLBACK_INTERFACE          ip64_eth_interface
/* 
 * We explicitly turn of SLIP. This is a workaround for the construction 
 * used in the platform's Makefile (SLIP_ARCH_CONF_ENABLED is set as soon
 * as UIP_FALLBACK is defined)
 */ 
#define SLIP_ARCH_CONF_ENABLED          0
/*---------------------------------------------------------------------------*/
/* Make sure DHCP is enabled for our IPv4 interface */
#define IP64_CONF_DHCP                  1
/*---------------------------------------------------------------------------*/
/* Debug output via UART */
#define DBG_CONF_USB                    0
/*---------------------------------------------------------------------------*/
/*
 * Enable RF specific statistics. 
 * Disable CONTIKI_WITH_RIME in the project's Makefile to save memory if
 * not used.
 */
// #define RIMESTATS_CONF_ENABLED          1
/* 
 * Enable platform specific statistics. This causes the border router to 
 * collect additional information.
 */
// #define PLATFORM_CONF_STATS             1
/*---------------------------------------------------------------------------*/

#endif /* PROJECT_CONF_H_ */

