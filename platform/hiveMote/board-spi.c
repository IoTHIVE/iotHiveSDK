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
 * \addtogroup sensortag-cc26xx-spi
 * @{
 *
 * \file
 * Board-specific SPI driver for the Sensortag-CC26xx
 */
/*---------------------------------------------------------------------------*/
#include "board-spi.h"
/*---------------------------------------------------------------------------*/
static bool
accessible(void)
{
  /* First, check the PD */
  if(ti_lib_prcm_power_domain_status(PRCM_DOMAIN_SERIAL)
     != PRCM_DOMAIN_POWER_ON) {
    return false;
  }

  /* Then check the 'run mode' clock gate */
  if(!(HWREG(PRCM_BASE + PRCM_O_SSICLKGR) & PRCM_SSICLKGR_CLK_EN_SSI0)) {
    return false;
  }

  return true;
}
/*---------------------------------------------------------------------------*/
bool
board_spi_write(uint8_t spi, const uint8_t *buf, size_t len)
{
  if(accessible() == false) {
    return false;
  }

  const spi_regs_t *regs = &spi_regs[spi];

  while(len > 0) {
    uint32_t ul;

    ti_lib_ssi_data_put(regs->base, *buf);
    ti_lib_rom_ssi_data_get(regs->base, &ul);
    len--;
    buf++;
  }

  return true;
}
/*---------------------------------------------------------------------------*/
bool
board_spi_read(uint8_t spi, uint8_t *buf, size_t len)
{
  if(accessible() == false) {
    return false;
  }

  const spi_regs_t *regs = &spi_regs[spi];

  while(len > 0) {
    uint32_t ul;

    if(!ti_lib_rom_ssi_data_put_non_blocking(regs->base, 0)) {
      /* Error */
      return false;
    }
    ti_lib_rom_ssi_data_get(regs->base, &ul);
    *buf = (uint8_t)ul;
    len--;
    buf++;
  }
  return true;
}
/*---------------------------------------------------------------------------*/
void
board_spi_flush(uint8_t spi)
{
  if(accessible() == false) {
    return;
  }

  const spi_regs_t *regs = &spi_regs[spi];

  uint32_t ul;
  while(ti_lib_rom_ssi_data_get_non_blocking(regs->base, &ul));
}
/*---------------------------------------------------------------------------*/
void
board_spi_open(uint8_t spi, uint32_t bit_rate)
{
  uint32_t buf;

  const spi_regs_t *regs = &spi_regs[spi];

  /* First, make sure the SERIAL PD is on */
  ti_lib_prcm_power_domain_on(PRCM_DOMAIN_SERIAL);
  while((ti_lib_prcm_power_domain_status(PRCM_DOMAIN_SERIAL)
        != PRCM_DOMAIN_POWER_ON));

  /* Enable clock in active mode */
  ti_lib_rom_prcm_peripheral_run_enable(regs->prcm_peripheral);
  ti_lib_prcm_load_set();
  while(!ti_lib_prcm_load_get());

  /* SPI configuration */
  ti_lib_ssi_int_disable(regs->base, SSI_RXOR | SSI_RXFF | SSI_RXTO | SSI_TXFF);
  ti_lib_ssi_int_clear(regs->base, SSI_RXOR | SSI_RXTO);
  ti_lib_rom_ssi_config_set_exp_clk(regs->base, ti_lib_sys_ctrl_clock_get(),
                                    SSI_FRF_MOTO_MODE_0,
                                    SSI_MODE_MASTER, bit_rate, 8);
  ti_lib_rom_ioc_pin_type_ssi_master(regs->base, regs->miso_ioid, regs->mosi_ioid, IOID_UNUSED, regs->clk_ioid);
  ti_lib_ssi_enable(regs->base);

  /* Get rid of residual data from SSI port */
  while(ti_lib_ssi_data_get_non_blocking(regs->base, &buf));
}
/*---------------------------------------------------------------------------*/
void
board_spi_close(uint8_t spi)
{
  const spi_regs_t *regs = &spi_regs[spi];

  /* Power down SSI0 */
  ti_lib_rom_prcm_peripheral_run_disable(regs->prcm_peripheral);
  ti_lib_prcm_load_set();
  while(!ti_lib_prcm_load_get());

  /* Restore pins to a low-consumption state */
  ti_lib_ioc_pin_type_gpio_input(regs->miso_ioid);
  ti_lib_ioc_io_port_pull_set(regs->miso_ioid, IOC_IOPULL_DOWN);

  ti_lib_ioc_pin_type_gpio_input(regs->mosi_ioid);
  ti_lib_ioc_io_port_pull_set(regs->mosi_ioid, IOC_IOPULL_DOWN);

  ti_lib_ioc_pin_type_gpio_input(regs->clk_ioid);
  ti_lib_ioc_io_port_pull_set(regs->clk_ioid, IOC_IOPULL_DOWN);
}
/*---------------------------------------------------------------------------*/
/** @} */
