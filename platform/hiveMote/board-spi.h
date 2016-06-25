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
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup sensortag-cc26xx-peripherals
 * @{
 *
 * \defgroup sensortag-cc26xx-spi SensorTag 2.0 SPI functions
 * @{
 *
 * \file
 * Header file for the Sensortag-CC26xx SPI Driver
 */
/*---------------------------------------------------------------------------*/
#ifndef BOARD_SPI_H_
#define BOARD_SPI_H_
/*---------------------------------------------------------------------------*/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "ti-lib.h"
#include "board.h"

#include "ssi.h"
#include "hw_types.h"

#include "sys/cc.h"
/*---------------------------------------------------------------------------*/
#define SSI_INSTANCE_COUNT 2
/*---------------------------------------------------------------------------*/
typedef struct {
  uint32_t base;
  uint32_t prcm_peripheral;
  uint32_t clk_ioid;
  uint32_t miso_ioid;
  uint32_t mosi_ioid;
} spi_regs_t;
/*---------------------------------------------------------------------------*/
static const spi_regs_t spi_regs[SSI_INSTANCE_COUNT] = {
  {
    .base = SSI0_BASE,
    .prcm_peripheral = PRCM_PERIPH_SSI0,
    .clk_ioid = BOARD_IOID_SPI_0_SCK,
    .miso_ioid = BOARD_IOID_SPI_0_MISO,
    .mosi_ioid = BOARD_IOID_SPI_0_MOSI
  }, {
	.base = SSI1_BASE,
    .prcm_peripheral = PRCM_PERIPH_SSI1,
    .clk_ioid = BOARD_IOID_SPI_1_SCK,
    .miso_ioid = BOARD_IOID_SPI_1_MISO,
    .mosi_ioid = BOARD_IOID_SPI_1_MOSI
  }
};
/*---------------------------------------------------------------------------*/
/* TODO:  Remove and use the below functions. This 
          will require change of ENC28J60 Arch Driver.        
*/
#define SPI_TXBUF(spi)              HWREG(CC_CONCAT3(SSI, spi, _BASE) + SSI_O_DR)
#define SPI_RXBUF(spi)              HWREG(CC_CONCAT3(SSI, spi, _BASE) + SSI_O_DR)
#define SPI_WAITFORTxREADY(spi)     do { \
                                      while(!(ti_lib_ssi_status(CC_CONCAT3(SSI, spi, _BASE)) & SSI_TX_EMPTY)) ; \
                                    } while(0)
#define SPI_WAITFOREORx(spi)        do { \
                                      while(!(ti_lib_ssi_status(CC_CONCAT3(SSI, spi, _BASE)) & SSI_RX_NOT_EMPTY)) ; \
                                    } while(0)
#define SPI_FLUSH(spi)              do { \
                                      board_spi_flush(spi) ; \
                                    } while(0)
/*---------------------------------------------------------------------------*/
/**
 * \brief Initialize the SPI interface
 * \param bit_rate The bit rate to use
 * \param clk_pin The IOID for the clock pin. This can be IOID_0 etc
 * \return none
 *
 * This function will make sure the peripheral is powered, clocked and
 * initialised. A chain of calls to board_spi_read(), board_spi_write() and
 * board_spi_flush() must be preceded by a call to this function. It is
 * recommended to call board_spi_close() after such chain of calls.
 */
void board_spi_open(uint8_t spi, uint32_t bit_rate, uint32_t spi_mode);
/*---------------------------------------------------------------------------*/
/**
 * \brief Close the SPI interface
 * \return True when successful.
 *
 * This function will stop clocks to the SSI module and will set MISO, MOSI
 * and CLK to a low leakage state. It is recommended to call this function
 * after a chain of calls to board_spi_read() and board_spi_write()
 */
void board_spi_close(uint8_t spi);
/*---------------------------------------------------------------------------*/
/**
 * \brief Clear data from the SPI interface
 * \return none
 */
void board_spi_flush(uint8_t spi);
/*---------------------------------------------------------------------------*/
/**
 * \brief Read from an SPI device
 * \param buf The buffer to store data
 * \param length The number of bytes to read
 * \return True when successful.
 *
 * Calls to this function must be preceded by a call to board_spi_open(). It is
 * recommended to call board_spi_close() at the end of an operation.
 */
bool board_spi_read(uint8_t spi, uint8_t *buf, size_t length);
/*---------------------------------------------------------------------------*/
/**
 * \brief Write to an SPI device
 * \param buf The buffer with the data to write
 * \param length The number of bytes to write
 * \return True when successful.
 *
 * Calls to this function must be preceded by a call to board_spi_open(). It is
 * recommended to call board_spi_close() at the end of an operation.
 */
bool board_spi_write(uint8_t spi, const uint8_t *buf, size_t length);
/*---------------------------------------------------------------------------*/
#endif /* BOARD_SPI_H_ */
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
