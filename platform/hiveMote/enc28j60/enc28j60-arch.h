/*
 * Copyright (c) 2015, Weptech elektronik GmbH Germany 
 * http://www.weptech.de
 * Author: Ulf Knoblich (ulf.knoblich@weptech.de)
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
 *
 * This file is part of the Contiki operating system.
 */

#ifndef ENC28J60_ARCH_H
#define ENC28J60_ARCH_H

#include <stdint.h>
#include <string.h>
/*---------------------------------------------------------------------------*/
/**
 * Initialize SPI module & Pins. 
 *
 * The function has to accomplish the following tasks:
 * - Enable SPI and configure SPI (CPOL = 0, CPHA = 0)
 * - Configure MISO, MOSI, SCLK accordingly
 * - Configure INT_N (input)
 * - Configure RESET_N (output high)
 * - Configure CS_N (output high)
 */
void 
enc28j60_arch_spi_init(void);
/*---------------------------------------------------------------------------*/
/**
 * Select ENC28J60 (pull down CS_N pin).
 */
void
enc28j60_arch_spi_select(void);
/*---------------------------------------------------------------------------*/
/**
 * De-select ENC28J60 (release CSn pin).
 */
void
enc28j60_arch_spi_deselect(void);
/*---------------------------------------------------------------------------*/
/**
 * Configure port IRQ for INT_N (falling edge)
 */
void 
enc28j60_arch_setup_irq(void);
/*---------------------------------------------------------------------------*/
/**
 * Reset interrupt flag and enable INT_N port IRQ.
 */
void 
enc28j60_arch_enable_irq(void);
/*---------------------------------------------------------------------------*/
/**
 * Disable INT_N port IRQ.
 */
void 
enc28j60_arch_disable_irq(void);
/*---------------------------------------------------------------------------*/
/**
 * Write a single byte via SPI, return response
 */
int
enc28j60_arch_spi_rw_byte(uint8_t c);
/*---------------------------------------------------------------------------*/
/**
 * Write a sequence of bytes while reading back the response
 */
int enc28j60_arch_spi_rw(uint8_t *read_buf, 
						 const uint8_t *write_buf, 
						 uint16_t len);
/*---------------------------------------------------------------------------*/
/**
 * The interrupt handler exported from the enc28j60 driver
 *
 * To be called by the hardware interrupt handler(s), 
 * which are defined as part of the enc28j60-arch interface.
 */
int enc28j60_interrupt_handler(void);
/*---------------------------------------------------------------------------*/
#endif /* #ifndef ENC28J60_ARCH_H */

