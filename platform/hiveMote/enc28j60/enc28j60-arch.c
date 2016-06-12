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
 *
 * This file is part of the Contiki operating system.
 */

#include "enc28j60-arch.h"
#include "board.h"
#include "board-spi.h"
#include "ti-lib.h"
#include "gpio-interrupt.h"
/*---------------------------------------------------------------------------*/
/* Taking CC2538 Approach. TODO: Put it in a better form. */
#define ENC28J60_SPI_TXBUF              SPI_TXBUF(ENC28J60_SPI_INSTANCE)
#define ENC28J60_SPI_RXBUF              SPI_RXBUF(ENC28J60_SPI_INSTANCE)
#define ENC28J60_SPI_WAITFORTxREADY()   SPI_WAITFORTxREADY(ENC28J60_SPI_INSTANCE)
#define ENC28J60_SPI_WAITFOREORx()      SPI_WAITFOREORx(ENC28J60_SPI_INSTANCE)
#define ENC28J60_SPI_FLUSH()            SPI_FLUSH(ENC28J60_SPI_INSTANCE)
/*---------------------------------------------------------------------------*/
static void
int_n_callback(uint8_t ioid)
{
  enc28j60_interrupt_handler();
}
/*---------------------------------------------------------------------------*/
void
enc28j60_arch_spi_init(void)
{
  /* SPI CS Config */
  ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_ENC28J60_CS);

  /* Deselect CS */
  enc28j60_arch_spi_deselect();
  
  /* INT_N input */
  ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_ENC28J60_INT);
  
  /* Set up SPI */
  board_spi_open(ENC28J60_SPI_INSTANCE, ENC28J60_BIT_RATE);
}
/*---------------------------------------------------------------------------*/
void 
enc28j60_arch_setup_irq(void)
{
  ti_lib_gpio_event_clear(BOARD_ENC28J60_INT);
  ti_lib_ioc_port_configure_set(BOARD_IOID_ENC28J60_INT, IOC_PORT_GPIO, BOARD_IOID_ENC28J60_INT_CFG);
  ti_lib_gpio_dir_mode_set(BOARD_ENC28J60_INT, GPIO_DIR_MODE_IN);
  gpio_interrupt_register_handler(BOARD_IOID_ENC28J60_INT, int_n_callback);
}
/*---------------------------------------------------------------------------*/
void 
enc28j60_arch_enable_irq(void)
{
  /* Reset interrupt trigger */
  ti_lib_gpio_event_clear(BOARD_ENC28J60_INT);
  /* Enable interrupt on the INT_N pin */
  ti_lib_ioc_int_enable(BOARD_IOID_ENC28J60_INT);
}
/*---------------------------------------------------------------------------*/
void 
enc28j60_arch_disable_irq(void)
{
  /* Disable interrupt on the INT_N pin */
  ti_lib_ioc_int_disable(BOARD_IOID_ENC28J60_INT);
}
/*---------------------------------------------------------------------------*/
uint8_t
enc28j60_arch_spi_write(uint8_t c)
{
  ENC28J60_SPI_WAITFORTxREADY();
  ENC28J60_SPI_TXBUF = c;
  ENC28J60_SPI_WAITFOREORx();
  return ENC28J60_SPI_RXBUF;
}
/*---------------------------------------------------------------------------*/
uint8_t
enc28j60_arch_spi_read(void)
{
  ENC28J60_SPI_WAITFORTxREADY();
  ENC28J60_SPI_TXBUF = 0;


  ENC28J60_SPI_WAITFOREORx();
  return ENC28J60_SPI_RXBUF;
}
/*---------------------------------------------------------------------------*/
int
enc28j60_arch_spi_rw_byte(uint8_t c)
{
  ENC28J60_SPI_WAITFORTxREADY();
  ENC28J60_SPI_TXBUF = c;


  ENC28J60_SPI_WAITFOREORx();
  return ENC28J60_SPI_RXBUF;
}
/*---------------------------------------------------------------------------*/
int
enc28j60_arch_spi_rw(uint8_t *read_buf,
                     const uint8_t *write_buf,
                     uint16_t len)
{
  uint16_t i;

  if(read_buf == NULL && write_buf == NULL) {
    /* Allow also dummy read
     in order to flush receive buffer in case of invalid
     packets */
    for(i = 0; i < len; i++) {
      ENC28J60_SPI_WAITFORTxREADY();
      ENC28J60_SPI_TXBUF = 0;
      ENC28J60_SPI_WAITFOREORx();
      ENC28J60_SPI_RXBUF;
    }
  } else if(read_buf == NULL) {
    for(i = 0; i < len; i++) {
      ENC28J60_SPI_WAITFORTxREADY();
      ENC28J60_SPI_TXBUF = write_buf[i];
      ENC28J60_SPI_WAITFOREORx();
      ENC28J60_SPI_RXBUF;
    }
  } else if(write_buf == NULL) {
    for(i = 0; i < len; i++) {
      ENC28J60_SPI_WAITFORTxREADY();
      ENC28J60_SPI_TXBUF = 0;
      ENC28J60_SPI_WAITFOREORx();
      read_buf[i] = ENC28J60_SPI_RXBUF;
    }
  } else {
    for(i = 0; i < len; i++) {
      ENC28J60_SPI_WAITFORTxREADY();
      ENC28J60_SPI_TXBUF = write_buf[i];
      ENC28J60_SPI_WAITFOREORx();
      read_buf[i] = ENC28J60_SPI_RXBUF;
    }
  }

  return 0;
}
/*---------------------------------------------------------------------------*/
void
enc28j60_arch_spi_select(void)
{
  ti_lib_gpio_pin_write(BOARD_ENC28J60_CS, 0);
}
/*---------------------------------------------------------------------------*/
void
enc28j60_arch_spi_deselect(void)
{
  ti_lib_gpio_pin_write(BOARD_ENC28J60_CS, 1);
}
/*---------------------------------------------------------------------------*/
