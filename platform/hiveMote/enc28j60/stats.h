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

/**
 * \addtogroup cc2538-cc1200-eth
 *
 * @{
 *
 * \defgroup cc2538-cc1200-eth-statistics Statistic modul
 *
 * @{
 *
 * \file Header file for the statistics module
 */

#ifndef STATS_H
#define STATS_H

#include "contiki.h"

/*---------------------------------------------------------------------------*/
#ifdef PLATFORM_CONF_STATS
#define PLATFORM_STATS          PLATFORM_CONF_STATS
#else
#define PLATFORM_STATS          0
#endif
/*---------------------------------------------------------------------------*/
typedef struct {

  /** The number of packets received via ethernet interface */
  unsigned long eth_rx_pkts;
  /** The number of bytes received via ethernet interface */
  unsigned long eth_rx_bytes;
  /** The number of packets send via ethernet interface */
  unsigned long eth_tx_pkts;
  /** The number of bytes send via ethernet interface */
  unsigned long eth_tx_bytes;

} stats_t;
/*---------------------------------------------------------------------------*/
#if PLATFORM_STATS
extern stats_t stats;
#define STATS_INC(x)            stats.x++
#define STATS_ADD(x,y)          (stats.x+=y)
#define STATS_GET(x)            stats.x
#else /* PLATFORM_STATS */
#define STATS_INC(x)
#define STATS_ADD(x,y)
#define STATS_GET(x)
#endif /* PLATFORM_STATS */
/*---------------------------------------------------------------------------*/

#endif /* STATS_H */

/**
 * @}
 * @}
 */
