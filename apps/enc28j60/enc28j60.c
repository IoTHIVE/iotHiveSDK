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

#include "contiki.h"
#include "enc28j60.h"
#include "enc28j60-arch.h"
#include "enc28j60-const.h"
#include "eui48.h"
#include "stats.h"

#if CONTIKI_WITH_IP64
/* In interrupt mode (USE_INT_N), we pass data directly to the IP64 module */
#include "ip64.h"
#endif

#include <stdio.h>
#include <string.h>

/*---------------------------------------------------------------------------*/
/* Various implementation specific defines */
/*---------------------------------------------------------------------------*/
/** The debug level to use */
#define DEBUG_LEVEL                     3
/** 
 * Set this parameter to 1 if full-duplex mode should be used.
 * \attention Driver is not tested in half-duplex mode! 
 */
#define FULL_DUPLEX                     1
/** Wether to use interrupts or not */
#define USE_INT_N                       1
/** 
 * If set, auto increment is used for the buffer pointers. Else, we set
 * them by hand allowing to read only parts of the received frame (e.g.
 *  no need to read padding byte, crc etc.) 
 */
#define AUTO_INCREMENT                  0
/** 
 * If set, we wait for TX to be completed \e before we send a new frame. Else
 * we wait \e after TX was started 
 */
#define TX_WAIT_BEFORE                  1
/** Wether to use loopback mode or not */
#define ENABLE_LOOPBACK_MODE            0
/** We automaticall pad frames to 60 bytes. Both settings belong together.. */
#define PADDING_MIN_SIZE                60
#define PADDING_CFG                     ENC_MACON3_PADCFG_1
/*---------------------------------------------------------------------------*/
/* Main driver configurations settings. Don't touch! */
/*---------------------------------------------------------------------------*/
/* B7 Silicon Errata #3:
   Use the lower segment of the buffer memory for
   the receive buffer, starting at address 0000h. For
   example, use the range (0000h to n) for the
   receive buffer, and ((n + 1) – 8191) for the transmit
   buffer */
/** Start of receive buffer */
#define RX_BUF_START                    0x0000
/** End of receive buffer (4k) */
#define RX_BUF_END                      0x0fff
/** Start of transmit buffer (should point to an even address) */
#define TX_BUF_START                    0x1200
/** Maximum 802.3 frame lenght */
#define MAX_8023_FRAME_LENGTH           1518
/** TX packet timeout. 10ms should be sufficient */
#define TX_PKT_TIMEOUT                  (RTIMER_SECOND/100)
/** Oscillator Start-up timeout */
#define PWR_ON_TIMEOUT_US               120
/** Oscillator Start-up timeout */
#define OST_TIMEOUT                     (RTIMER_SECOND/100)
/*---------------------------------------------------------------------------*/
/* Debug settings */
#define DEBUG 0
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
#define BUSYWAIT_UNTIL(cond, max_time)                                  \
  do {                                                                  \
    rtimer_clock_t t0;                                                  \
    t0 = RTIMER_NOW();                                                  \
    while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time))) {} \
  } while(0)
/*---------------------------------------------------------------------------*/
PROCESS(enc28j60_process, "Enc28j60 watchdog");
/*---------------------------------------------------------------------------*/
#define ETH_FLAGS_INITIALIZED           0x01
#define ETH_FLAGS_ON                    0x02
#define ETH_FLAGS_PROMISCUOUS_MODE      0x04
/*---------------------------------------------------------------------------*/
/** Header added by ENC28j60 for each packet received */
typedef struct {
  uint16_t next_pkt_ptr;
  uint16_t byte_count;
  uint16_t status;
} rx_header_t;
/*---------------------------------------------------------------------------*/
/* Static variables */
/*---------------------------------------------------------------------------*/
/** The state of the driver */
static uint8_t eth_flags;
/** The number or packets received */
static uint32_t num_rx_pkts;
/** The number of packets sent */
static uint32_t num_tx_pkts;
/** The currently selected bank in order to avoid duplicated bank switch */
static uint8_t selected_bank;
/** The EUI48 passed from upper layer or read from EEPROM. We need to store
    this parameter due to the watchdog-reset which might call configure()
    at arbitrary times */
static uint8_t eui48[EUI48_SIZE];
/*---------------------------------------------------------------------------*/
/* Static function prototypes */
/*---------------------------------------------------------------------------*/
static uint8_t
read_control_reg(uint8_t addr);
static void
read_buf_mem(uint8_t *data, uint16_t data_len);
static void
write_control_reg(uint8_t bank_addr, uint8_t data);
static void
set_bit_field(uint8_t addr, uint8_t data);
static void
clear_bit_field(uint8_t addr, uint8_t data);
static void
reset(void);
static void
select_bank(uint8_t bank);
static uint16_t
read_phy_reg(uint8_t addr);
static void
write_phy_reg(uint8_t addr, uint16_t data);
static void
dump_rx_status(uint8_t pkt_count, const rx_header_t *rx_header);
static void
dump_rx_data(const uint8_t *data, uint16_t data_len);
static void
dump_tx_data(const uint8_t *data, uint16_t data_len);
static void
configure(const uint8_t *mac_addr);
#if USE_INT_N
static void
pollhandler(void);
#endif
/*---------------------------------------------------------------------------*/
/* Static functions */
/*---------------------------------------------------------------------------*/
/*
 * The Read Control Register (RCR) command allows the host controller to 
 * read any of the ETH, MAC and MII registers in any order. 
 */
static uint8_t
read_control_reg(uint8_t bank_addr)
{

  uint8_t ret;
  uint8_t bank = ((bank_addr >> 5) & 0x03);
  uint8_t addr = (bank_addr & ENC_ADDR_MASK);

  if (addr < (ENC_EIE & ENC_ADDR_MASK)) {
    /* Bank select only for addresses < 0x1B */
    select_bank(bank);
  }

  enc28j60_arch_spi_select();
  enc28j60_arch_spi_rw_byte(ENC_CMD_RCR | addr);
  if (bank_addr & ENC_MAC_PFX) {
    /* Read dummy byte for MAC and MII registers */
    enc28j60_arch_spi_rw_byte(0);
  }
  ret = enc28j60_arch_spi_rw_byte(0);
  enc28j60_arch_spi_deselect();

  return ret;
} /* read_control_reg(), uk 10.04.2015 */
/*---------------------------------------------------------------------------*/
/* 
 * The Read Buffer Memory (RBM) command allows the
 * host controller to read bytes from the integrated 8-Kbyte
 * transmit and receive buffer memory. 
 */
static void
read_buf_mem(uint8_t *data, uint16_t data_len)
{

  enc28j60_arch_spi_select();
  enc28j60_arch_spi_rw_byte(ENC_CMD_RBM);
  enc28j60_arch_spi_rw(data, NULL, data_len);
  enc28j60_arch_spi_deselect();

} /* read_buf_mem(), uk 10.04.2015 */
/*---------------------------------------------------------------------------*/
/* 
 * The Write Control Register (WCR) command allows
 * the host controller to write to any of the ETH, MAC and
 * MII Control registers in any order. 
 */
static void
write_control_reg(uint8_t bank_addr, uint8_t data)
{

  uint8_t bank = ((bank_addr >> 5) & 0x03);
  uint8_t addr = (bank_addr & ENC_ADDR_MASK);

  if (addr < (ENC_EIE & ENC_ADDR_MASK)) {
    /* Bank select only for addresses < 0x1B */
    select_bank(bank);
  }

  enc28j60_arch_spi_select();
  enc28j60_arch_spi_rw_byte(ENC_CMD_WCR | addr);
  enc28j60_arch_spi_rw_byte(data);
  enc28j60_arch_spi_deselect();

} /* read_control_reg(), uk 10.04.2015 */
/*---------------------------------------------------------------------------*/
/*
 * The Write Buffer Memory (WBM) command allows the
 * host controller to write bytes to the integrated 8-Kbyte
 * transmit and receive buffer memory. 
 */
static void
write_buf_mem(const uint8_t *data, uint16_t data_len)
{

  enc28j60_arch_spi_select();
  enc28j60_arch_spi_rw_byte(ENC_CMD_WBM);
  enc28j60_arch_spi_rw(NULL, data, data_len);
  enc28j60_arch_spi_deselect();

} /* write_buf_mem(), uk 10.04.2015 */
/*---------------------------------------------------------------------------*/
/** The Bit Field Set (BFS) command is used to set up to
    8 bits in any of the ETH Control registers.
    Not allowed for MAC registers! */
static void
set_bit_field(uint8_t bank_addr, uint8_t data)
{

  if (!(bank_addr & ENC_MAC_PFX)) {

    uint8_t bank = ((bank_addr >> 5) & 0x03);
    uint8_t addr = (bank_addr & ENC_ADDR_MASK);

    if (addr < (ENC_EIE & ENC_ADDR_MASK)) {
      /* Bank select only for addresses < 0x1B */
      select_bank(bank);
    }

    enc28j60_arch_spi_select();
    enc28j60_arch_spi_rw_byte(ENC_CMD_BFS | addr);
    enc28j60_arch_spi_rw_byte(data);
    enc28j60_arch_spi_deselect();

  } else {
    /* Optional: set bits for MAC registers "by hand" (read/set/write) */
  }

} /* set_bit_field(), uk 13.04.2015 */
/*---------------------------------------------------------------------------*/
/** 
 * The Bit Field Clear (BFC) command is used to clear up
 * to 8 bits in any of the ETH Control registers.
 * Not allowed for MAC registers! 
 */
static void
clear_bit_field(uint8_t bank_addr, uint8_t data)
{

  if (!(bank_addr & ENC_MAC_PFX)) {

    uint8_t bank = ((bank_addr >> 5) & 0x03);
    uint8_t addr = (bank_addr & ENC_ADDR_MASK);

    if (addr < (ENC_EIE & ENC_ADDR_MASK)) {
      /* Bank select only for addresses < 0x1B */
      select_bank(bank);
    }

    enc28j60_arch_spi_select();
    enc28j60_arch_spi_rw_byte(ENC_CMD_BFC | addr);
    enc28j60_arch_spi_rw_byte(data);
    enc28j60_arch_spi_deselect();

  } else {
    /* Optional: clear bits for MAC registers "by hand" (read/clear/write) */
  }

} /* clear_bit_field(), uk 13.04.2015 */
/*---------------------------------------------------------------------------*/
/** 
 * The System Reset Command (SRC) allows the host
 * controller to issue a System Soft Reset command. 
 */
static void
reset(void)
{

  enc28j60_arch_spi_select();
  enc28j60_arch_spi_rw_byte(ENC_CMD_SRC);
  enc28j60_arch_spi_deselect();
  selected_bank = 0;

  /* 
   * B7 Silicon Errata #1:
   * After issuing the Reset command, wait for at least
   * 1 ms in firmware for the device to be ready. 
   */
  clock_delay_usec(2000);

} /* reset(), uk 14.04.2015 */
/*---------------------------------------------------------------------------*/
/** Select one out of the 3 available banks. */
static void
select_bank(uint8_t bank)
{

  if (bank != selected_bank) {

    uint8_t reg;

    /* Read ECON1 from arbitrary bank */
    enc28j60_arch_spi_select();
    enc28j60_arch_spi_rw_byte(ENC_CMD_RCR | ENC_ECON1);
    reg = enc28j60_arch_spi_rw_byte(0);
    enc28j60_arch_spi_deselect();

    // TODO bit-field operationen verwenden

    reg &= ~0x03;
    reg |= (bank & 0x03);

    /* Write ECON1 to arbitrary bank */
    enc28j60_arch_spi_select();
    enc28j60_arch_spi_rw_byte(ENC_CMD_WCR | ENC_ECON1);
    enc28j60_arch_spi_rw_byte(reg);
    enc28j60_arch_spi_deselect();

    selected_bank = bank;

  }

} /* select_bank(), uk 14.04.2015 */
/*---------------------------------------------------------------------------*/
/** Read 16 bits from a PHY register */
static uint16_t
read_phy_reg(uint8_t addr)
{

  uint16_t phy_reg;

  /* 1. Write the address of the PHY register to read
     from into the MIREGADR register.
     2. Set the MICMD.MIIRD bit. The read operation
     begins and the MISTAT.BUSY bit is set.
     3. Wait 10.24 µS. Poll the MISTAT.BUSY bit to be
     certain that the operation is complete. While
     busy, the host controller should not start any
     MIISCAN operations or write to the MIWRH
     register.
     When the MAC has obtained the register
     contents, the BUSY bit will clear itself.
     4. Clear the MICMD.MIIRD bit.
     5. Read the desired data from the MIRDL and
     MIRDH registers. The order that these bytes are
     accessed is unimportant  */

  write_control_reg(ENC_MIREGADR, addr);
  write_control_reg(ENC_MICMD, ENC_MICMD_MIIRD);
  while(read_control_reg(ENC_MISTAT) & ENC_MISTAT_BUSY) {
    clock_delay(15);
  }
  write_control_reg(ENC_MICMD, 0x00);
  phy_reg = read_control_reg(ENC_MIRDH);
  phy_reg <<= 8;
  phy_reg |= read_control_reg(ENC_MIRDL);

  return phy_reg;

} /* read_phy_reg(), uk 13.04.2015 */
/*---------------------------------------------------------------------------*/
/** Write 16 bits to a PHY register */
static void
write_phy_reg(uint8_t addr, uint16_t data)
{

  /* 1. Write the address of the PHY register to write to
     into the MIREGADR register.
     2. Write the lower 8 bits of data to write into the
     MIWRL register.
     3. Write the upper 8 bits of data to write into the
     MIWRH register. Writing to this register auto-
     matically begins the MIIM transaction, so it must
     be written to after MIWRL. The MISTAT.BUSY
     bit becomes set. */

  write_control_reg(ENC_MIREGADR, addr);
  write_control_reg(ENC_MIWRL, (data & 0xFF));
  write_control_reg(ENC_MIWRH, (data >> 8));
  while(read_control_reg(ENC_MISTAT) & ENC_MISTAT_BUSY) {
    clock_delay(15);
  }

} /* write_phy_reg(), uk 13.04.2015 */
/*---------------------------------------------------------------------------*/
/** RX debug information */
static void
dump_rx_status(uint8_t pkt_count, const rx_header_t *rx_header)
{
  PRINTF("ETH: RX status\n");
  PRINTF("  rx_pkts : %6d\n",  num_rx_pkts);
  PRINTF("  pkt_cnt : %6d\n", pkt_count);
  PRINTF("  nxt_pkt : 0x%04x\n", rx_header->next_pkt_ptr);
  PRINTF("  byte_cnt: %6d\n", rx_header->byte_count);
  PRINTF("  status  : 0x%04x\n", rx_header->status);
}
/*---------------------------------------------------------------------------*/
/** RX debug information */
static void
dump_rx_data(const uint8_t *data, uint16_t data_len)
{
  PRINTF("ETH: RX data\n");
  PRINTF("  head    : %02x %02x %02x %02x %02x %02x\n",
       data[0],
       data[1],
       data[2],
       data[3],
       data[4],
       data[5]);
  PRINTF("  tail    : %02x %02x %02x %02x %02x %02x\n",
       data[data_len - 6],
       data[data_len - 5],
       data[data_len - 4],
       data[data_len - 3],
       data[data_len - 2],
       data[data_len - 1]);
}
/*---------------------------------------------------------------------------*/
/** TX debug information */
static void
dump_tx_data(const uint8_t *data, uint16_t data_len)
{
  PRINTF("ETH: TX data\n");
  PRINTF("  head    : %02x %02x %02x %02x %02x %02x\n",
       data[0],
       data[1],
       data[2],
       data[3],
       data[4],
       data[5]);
  PRINTF("  tail    : %02x %02x %02x %02x %02x %02x\n",
       data[data_len - 6],
       data[data_len - 5],
       data[data_len - 4],
       data[data_len - 3],
       data[data_len - 2],
       data[data_len - 1]);
}
/*---------------------------------------------------------------------------*/
/** Configure the ECN28j60 for desired operation */
static void
configure(const uint8_t *mac_addr)
{

  uint8_t reg;
  uint16_t phy_reg;

  PRINTF("ETH: Configure\n");

  if (!(eth_flags & ETH_FLAGS_INITIALIZED)) {

    /* Set up SPI and pins */
    enc28j60_arch_spi_init();

    /* Turn on voltage supply and wait until stabilized */
    // GPIO_SET_PIN(GPIO_PORT_TO_BASE(ENC28J60_PWR_ON_PORT_NUM), GPIO_PIN_MASK(ENC28J60_PWR_ON_PIN_NUM)); //Bhaumik
    clock_delay_usec(PWR_ON_TIMEOUT_US);

#if USE_INT_N
    enc28j60_arch_setup_irq();
#endif
    
    /* If the initialization procedure is being executed immedi-
       ately following a Power-on Reset, the ESTAT.CLKRDY
       bit should be polled to make certain that enough time
       has elapsed before proceeding to modify the MAC and
       PHY registers. */
    BUSYWAIT_UNTIL((read_control_reg(ENC_ESTAT) &
                    ENC_ESTAT_CLKRDY),
                   OST_TIMEOUT);
    PRINTF("****************************************2\r\n");
    if (!read_control_reg(ENC_ESTAT) & ENC_ESTAT_CLKRDY) {
      PRINTF("ETH: Clock does not respond or SPI failure!\n");
      return;
    }

    /* Check silicon revision / SPI communication */
    reg = read_control_reg(ENC_EREVID);
    if (reg != 0x06) {
      PRINTF("ETH: Wrong silicon revision (0x%02x)!\n",
            reg);
      return;
    }
  } else {

#if USE_INT_N
    /* Avoid spurious interrupts while configuring the chip */
    enc28j60_arch_disable_irq();
#endif

  }

  /* Perform a software reset */
  reset();

  /***************************************************************************
   * Initialize MAC
   **************************************************************************/

  /* 1. Set the MARXEN bit in MACON1 to enable the
     MAC to receive frames. If using full duplex, most
     applications should also set TXPAUS and
     RXPAUS to allow IEEE defined flow control to
     function. */

#if FULL_DUPLEX
  write_control_reg(ENC_MACON1,
                    (ENC_MACON1_MARXEN +
                     ENC_MACON1_TXPAUS +
                     ENC_MACON1_RXPAUS));
#else
  write_control_reg(ENC_MACON1,
                    ENC_MACON1_MARXEN);
#endif

  /*  2. Configure the PADCFG, TXCRCEN and
      FULDPX bits of MACON3. Most applications
      should enable automatic padding to at least
      60 bytes and always append a valid CRC. For
      convenience, many applications may wish to set
      the FRMLNEN bit as well to enable frame length
      status reporting. The FULDPX bit should be set
      if the application will be connected to a
      full-duplex configured remote node; otherwise, it
      should be left clear. */

#if FULL_DUPLEX
  write_control_reg(ENC_MACON3,
                    (PADDING_CFG +
                     ENC_MACON3_TXCRCEN +
                     ENC26J60_MACON3_FRMLNEN +
                     ENC_MACON3_FULDPX));
#else
  write_control_reg(ENC_MACON3,
                    (PADDING_CFG +
                     ENC_MACON3_TXCRCEN +
                     ENC26J60_MACON3_FRMLNEN));
#endif  /* #if FULL_DUPLEX */

  /* 3. Configure the bits in MACON4. For confor-
     mance to the IEEE 802.3 standard, set the
     DEFER bit. */

#if !FULL_DUPLEX
  write_control_reg(ENC_MACON4,
                    ENC_MACON4_DEFER);
#endif

  /* 4. Program the MAMXFL registers with the maxi-
     mum frame length to be permitted to be received
     or transmitted. Normal network nodes are
     designed to handle packets that are 1518 bytes
     or less. */

  write_control_reg(ENC_MAMXFLL, MAX_8023_FRAME_LENGTH & 0xFF);
  write_control_reg(ENC_MAMXFLH, MAX_8023_FRAME_LENGTH >> 8);

  /* 5. Configure the Back-to-Back Inter-Packet Gap
     register, MABBIPG. Most applications will pro-
     gram this register with 15h when Full-Duplex
     mode is used and 12h when Half-Duplex mode
     is used. */

#if FULL_DUPLEX
  write_control_reg(ENC_MABBIPG, 0x15);
#else
  write_control_reg(ENC_MABBIPG, 0x12);
#endif

  /*  6. Configure the Non-Back-to-Back Inter-Packet
      Gap register low byte, MAIPGL. Most applications
      will program this register with 12h. */

  write_control_reg(ENC_MAIPGL, 0x12);

  /* 7. If half duplex is used, the Non-Back-to-Back
     Inter-Packet Gap register high byte, MAIPGH,
     should be programmed. Most applications will
     program this register to 0Ch. */

#if !FULL_DUPLEX
  write_control_reg(ENC_MAIPGH, 0x0C);
#endif

  /* 8. If Half-Duplex mode is used, program the
     Retransmission and Collision Window registers,
     MACLCON1 and MACLCON2. Most applications
     will not need to change the default Reset values.
     If the network is spread over exceptionally long
     cables, the default value of MACLCON2 may
     need to be increased. */

#if !FULL_DUPLEX
#endif

  /* 9. Program the local MAC address into the
     MAADR1:MAADR6 registers. */

  write_control_reg(ENC_MAADR6, mac_addr[5]);
  write_control_reg(ENC_MAADR5, mac_addr[4]);
  write_control_reg(ENC_MAADR4, mac_addr[3]);
  write_control_reg(ENC_MAADR3, mac_addr[2]);
  write_control_reg(ENC_MAADR2, mac_addr[1]);
  write_control_reg(ENC_MAADR1, mac_addr[0]);

  PRINTF("ETH: Assigned MAC address %02x:%02x:%02x:%02x:%02x:%02x\n",
       mac_addr[0],
       mac_addr[1],
       mac_addr[2],
       mac_addr[3],
       mac_addr[4],
       mac_addr[5]);

  /***************************************************************************
   * Initialize PHY
   **************************************************************************/

  /* Depending on the application, bits in three of the PHY module’s
     registers may also require configuration.  The PHCON1.PDPXMD bit
     partially controls the device’s half/full-duplex
     configuration. Normally, this bit is initialized correctly by the
     external circuitry (see Section 2.6 “LED Configuration). If the
     external circuitry is not present or incorrect, however, the host
     controller must program the bit properly. Alternatively, for an
     externally configurable system, the PDPXMD bit may be read and the
     FULDPX bit be programmed to match.

     For proper duplex operation, the PHCON1.PDPXMD bit must also match
     the value of the MACON3.FULDPX bit.

     If using half duplex, the host controller may wish to set the
     PHCON2.HDLDIS bit to prevent automatic loopback of the data which
     is transmitted.  The PHY register, PHLCON, controls the outputs of
     LEDA and LEDB. If an application requires a LED configuration
     other than the default, PHLCON must be altered to match the new
     requirements. The settings for LED operation are discussed in
     Section 2.6 “LED Configuration. The PHLCON register is shown in
     Register 2-2 (page 9). */

  /* Read PHCON1 in order to check if full-duplex operation is enabled
     (configured via LEDB polarity) */

  phy_reg = read_phy_reg(ENC_PHCON1);
#if FULL_DUPLEX
  if (!(phy_reg & ENC_PHCON1_PDPXMD)) {
    PRINTF("ETH: Wrong PHY duplex configuration!\n");
    phy_reg |= ENC_PHCON1_PDPXMD;
    write_phy_reg(ENC_PHCON1, phy_reg);
  }
#else
  if (phy_reg & ENC_PHCON1_PDPXMD) {
    PRINTF("ETH: Wrong PHY duplex configuration!\n");
    phy_reg &= ~ENC_PHCON1_PDPXMD;
    write_phy_reg(ENC_PHCON1, phy_reg);
  }
  phy_reg = read_phy_reg(ENC_PHCON2);
  phy_reg |= ENC_PHCON2_HDLDIS;
  write_phy_reg(ENC_PHCON2, phy_reg);
#endif

  /***************************************************************************
   * Configure LEDs
   **************************************************************************/

  /* Display link status and receive activity (always stretched) */
  enc28j60_configure_leda(0x0C);
  /*  Display transmit activity (stretchable) */
  enc28j60_configure_ledb(0x01);

  /***************************************************************************
   * Enable reception
   **************************************************************************/

  /* Before receiving any packets, the receive buffer must
     be initialized by programming the ERXST and ERXND
     Pointers. All memory between and including the
     ERXST and ERXND addresses will be dedicated to the
     receive hardware. It is recommended that the ERXST
     Pointer be programmed with an even address. */

  write_control_reg(ENC_ERXSTL, RX_BUF_START & 0xFF);
  write_control_reg(ENC_ERXSTH, RX_BUF_START >> 8);
  write_control_reg(ENC_ERXNDL, RX_BUF_END & 0xFF);
  write_control_reg(ENC_ERXNDH, RX_BUF_END >> 8);

  /* For tracking purposes, the ERXRDPT registers should
     additionally be programmed with the same value. To
     program ERXRDPT, the host controller must write to
     ERXRDPTL first, followed by ERXRDPTH. */

  write_control_reg(ENC_ERXRDPTL, RX_BUF_START & 0xFF);
  write_control_reg(ENC_ERXRDPTH, RX_BUF_START >> 8);

  /* First frame is placed at RF_BUX_START, adjust the read
     pointer accordingly */

  write_control_reg(ENC_ERDPTL, RX_BUF_START & 0xff);
  write_control_reg(ENC_ERDPTH, RX_BUF_START >> 8);

#if AUTO_INCREMENT
  /* Turn on autoincrement for buffer access */
  set_bit_field(ENC_ECON2, ENC_ECON2_AUTOINC);
#endif

  if (!(eth_flags & ETH_FLAGS_PROMISCUOUS_MODE)) {
    /* Configure receive filter. Allow unicast, multicast +
       broadcast packets to pass, check CRC */
    write_control_reg(ENC_ERXFCON,
                      (ENC_ERXFCON_UCEN +
                       ENC_ERXFCON_CRCEN +
                       ENC_ERXFCON_MCEN +
                       ENC_ERXFCON_BCEN));
  } else {
    /* Enter promiscuous mode */
    write_control_reg(ENC_ERXFCON, 0);
  }

  /* Assuming that the receive buffer has been initialized,
     the MAC has been properly configured and the receive
     filters have been configured to receive Ethernet
     packets, the host controller should:
     1. If an interrupt is desired whenever a packet is
     received, set EIE.PKTIE and EIE.INTIE.
     2. If an interrupt is desired whenever a packet is
     dropped due to insufficient buffer space, clear
     EIR.RXERIF and set both EIE.RXERIE and
     EIE.INTIE
     3. Enable reception by setting ECON1.RXEN. */

#if USE_INT_N
  enc28j60_arch_enable_irq();
  /* Enable RX interrupt + global INT */
  write_control_reg(ENC_EIE, (ENC_EIE_INTIE + ENC_EIE_PKTIE));
#endif

  /* Turn on reception */
  set_bit_field(ENC_ECON1, ENC_ECON1_RXEN);

} /* configure(), uk 28.04.2015 */
/*---------------------------------------------------------------------------*/
#if USE_INT_N
static
void pollhandler(void)
{

  /* When an enabled interrupt occurs, the interrupt pin will
     remain low until all flags which are causing the interrupt
     are cleared or masked off (enable bit is cleared) by the
     host controller. If more than one interrupt source is
     enabled, the host controller must poll each flag in the
     EIR register to determine the source(s) of the interrupt.
     It is recommended that the Bit Field Clear (BFC) SPI
     command be used to reset the flag bits in the EIR reg-
     ister rather than the normal Write Control Register
     (WCR) command.

     ...

     After an interrupt occurs, the host controller should
     clear the global enable bit for the interrupt pin before
     servicing the interrupt. Clearing the enable bit will
     cause the interrupt pin to return to the non-asserted
     state (high). Doing so will prevent the host controller
     from missing a falling edge should another interrupt
     occur while the immediate interrupt is being serviced.
     After the interrupt has been serviced, the global enable
     bit may be restored. If an interrupt event occurred while
     the previous interrupt was being processed, the act of
     resetting the global enable bit will cause a new falling
     edge on the interrupt pin to occur. */

  uint16_t len;
#if ENABLE_LOOPBACK_MODE
  static uint8_t buf[UIP_BUFSIZE];
#endif

  /* Clear global INT enable bit */
  clear_bit_field(ENC_EIE, ENC_EIE_INTIE);
  /* Reset Packet Pending Interrupt Flag */
  clear_bit_field(ENC_EIR, ENC_EIR_PKTIF);

  len = 0;
  do {

#if CONTIKI_WITH_IP64
    /* Pass the data to the IP64 layer */
    len = enc28j60_read(ip64_packet_buffer, ip64_packet_buffer_maxlen);
    if(len > 0) {
      IP64_INPUT(ip64_packet_buffer, len);
    }
#endif

#if ENABLE_LOOPBACK_MODE
    /* Read packet and send it back */
    len = enc28j60_read(buf, UIP_BUFSIZE);
    if (len != 0) {
      enc28j60_send((const uint8_t*)buf, len);
    }
#endif

  } while(len != 0);

  /* Re-enable global INT enable bit */
  set_bit_field(ENC_EIE, ENC_EIE_INTIE);

}
#endif
/*---------------------------------------------------------------------------*/
void
enc28j60_init(const uint8_t *eth_eui48)
{
  if(eth_flags & ETH_FLAGS_INITIALIZED) {
    return;
  }
  /* Start watchdog process */
  process_start(&enc28j60_process, NULL);

  configure(eth_eui48);

  /* Save EUI48, we need it in case of a watchdog reset */
  memcpy((void*)&eui48, (const void*)&eth_eui48, sizeof(eui48));

  eth_flags |= (ETH_FLAGS_INITIALIZED + ETH_FLAGS_ON);

}
/*---------------------------------------------------------------------------*/
int
enc28j60_send(const uint8_t *data, uint16_t data_len)
{

  uint8_t control_byte;
  /* uint8_t padding; */

  if(!(eth_flags & ETH_FLAGS_INITIALIZED)) {
    return -1;
  }

#if TX_WAIT_BEFORE

  BUSYWAIT_UNTIL(!(read_control_reg(ENC_ECON1) &
                   ENC_ECON1_TXRTS),
                 TX_PKT_TIMEOUT);

  if(read_control_reg(ENC_ESTAT) & ENC_ESTAT_TXABRT) {
    PRINTF("ETH: TX error occured!\n");
  }

#endif

  /*  1. Appropriately program the ETXST pointer to point to an unused
      location in memory. It will point to the per packet control
      byte. It is recommended that an even address be used for ETXST. */

  write_control_reg(ENC_EWRPTL, (TX_BUF_START & 0xFF));
  write_control_reg(ENC_EWRPTH, (TX_BUF_START >> 8));
  write_control_reg(ENC_ETXSTL, (TX_BUF_START & 0xFF));
  write_control_reg(ENC_ETXSTH, (TX_BUF_START >> 8));

  /* Write the transmission control register as the first byte of the
     output packet. We write 0x00 to indicate that the default
     configuration (the values in MACON3) will be used. */

  control_byte = 0x00;

  /* 2. Use the WBM SPI command to write the per packet control byte,
     the destination address, the source MAC address, the
     type/length and the data payload. */

  write_buf_mem(&control_byte, 1);
  write_buf_mem(data, data_len);

  /* 3. Appropriately program the ETXND pointer. It should point to the
     last byte in the data payload. */

  write_control_reg(ENC_ETXNDL, (TX_BUF_START + data_len) & 0xFF);
  write_control_reg(ENC_ETXNDH, (TX_BUF_START + data_len) >> 8);

#if !FULL_DUPLEX
  /* B7 Silicon Errata # 10 */
  if(read_control_reg(ENC_ECON1) & ENC_EIR_TXERIF) {
    set_bit_field(ENC_ECON1, ENC_ECON1_TXRST);
    clear_bit_field(ENC_ECON1, ENC_ECON1_TXRST);
    clear_bit_field(ENC_EIR, ENC_EIR_TXERIF);
  }
#endif

  /* 4. Clear EIR.TXIF, set EIE.TXIE and set EIE.INTIE to enable an
     interrupt when done (if desired). */

  /* 5. Start the transmission process by setting ECON1.TXRTS. */

  set_bit_field(ENC_ECON1, ENC_ECON1_TXRTS);

  /* We use automatic padding. Adjust data_len accordingly, this way
     we always return the correct number of bytes sent */
  if (data_len < PADDING_MIN_SIZE) {
    data_len = PADDING_MIN_SIZE;
  }

#if !TX_WAIT_BEFORE

  /* When the packet is finished transmitting or is aborted
     due to an error/cancellation, the ECON1.TXRTS bit will
     be cleared, a seven-byte transmit status vector will be
     written to the location pointed to by ETXND + 1, the
     EIR.TXIF will be set and an interrupt will be generated
     (if enabled). */

  BUSYWAIT_UNTIL(!(read_control_reg(ENC_ECON1) &
                   ENC_ECON1_TXRTS),
                 TX_PKT_TIMEOUT);

  /* To check if the packet was successfully
     transmitted, the ESTAT.TXABRT bit should be read. If
     it was set, the host controller may interrogate the
     ESTAT.LATECOL bit in addition to the various fields in
     the transmit status vector to determine the cause. */

  if(read_control_reg(ENC_ESTAT) & ENC_ESTAT_TXABRT) {
    PRINTF("ETH: tx err: %d: %02x:%02x:%02x:%02x:%02x:%02x\n", data_len,
         0xff&data[0], 0xff&data[1], 0xff&data[2],
         0xff&data[3], 0xff&data[4], 0xff&data[5]);
  } else {
#if DEBUG_LEVEL > 4
    dump_tx_data(data, data_len);
#endif
  }

#else
#if DEBUG_LEVEL > 4
  dump_tx_data(data, data_len);
#endif
#endif

  STATS_INC(eth_tx_pkts);
  STATS_ADD(eth_tx_bytes, data_len);

  num_tx_pkts++;

  PRINTF("ETH: num_tx_pkts %d\n", num_tx_pkts);

  return data_len;

} /* enc28j60_send(), uk 30.04.2015 */
/*---------------------------------------------------------------------------*/
int
enc28j60_read(uint8_t *buf, uint16_t buf_size)
{

  int err = 0;
  uint8_t pkt_count;
  rx_header_t rx_header;

  /* B7 Silicon Errata # 4:
     The Receive Packet Pending Interrupt Flag
     (EIR.PKTIF) does not reliably/accurately report
     the status of pending packets.
     ...
     If polling to see if a packet is pending, check the
     value in EPKTCNT. */

  pkt_count = read_control_reg(ENC_EPKTCNT);
  if(pkt_count == 0) {
    return 0;
  }

  /* Read header */
  read_buf_mem((uint8_t*)&rx_header, sizeof(rx_header));

#if !AUTO_INCREMENT
#if 0
  /* Subtract CRC / padding byte */
  if((rx_header.byte_count % 2) != 0) {
    rx_header.byte_count -= 5;
  } else {
    rx_header.byte_count -= 4;
  }
#else
  /* Subtract CRC */
  rx_header.byte_count -= 4;
#endif
#endif

  /* Increase rx packet counter */
  num_rx_pkts++;

#if 0
  /* printf("rx_pkts : %6d\n",  num_rx_pkts); */
  /* printf("byte_cnt: %6d\n", rx_header.byte_count); */
  dump_rx_status(pkt_count, (const rx_header_t*)&rx_header);
#endif

  if(rx_header.byte_count <= buf_size) {
    /* Pass packet to next upper layer */
    read_buf_mem(buf, rx_header.byte_count);
  } else {
    /* Packet too big. Discard it...*/
    err = 1;
#if AUTO_INCREMENT
    read_buf_mem(NULL, rx_header.byte_count);
#endif
  }

#if AUTO_INCREMENT
  if((rx_header.byte_count % 2) != 0) {
    /* If the last byte in the packet ends on an odd value
       address, the hardware will automatically add a padding
       byte when advancing the Hardware Write Pointer. As
       such, all packets will start on an even boundary.*/

    /* Read this additional byte to avoid FIFO corruption */
    read_buf_mem(NULL, 1);
  }
#endif

  /* After the host controller has processed a packet (or part
     of the packet) and wishes to free the buffer space used
     by the processed data, the host controller must advance
     the Receive Buffer Read Pointer, ERXRDPT. The
     ENC28J60 will always write up to, but not including, the
     memory pointed to by the Receive Buffer Read Pointer.
     If the ENC28J60 ever attempts to overwrite the Receive
     Buffer Read Pointer location, the packet in progress will
     be aborted, the EIR.RXERIF will be set and an interrupt
     will be generated (if enabled). In this manner, the
     hardware will never overwrite unprocessed packets.
     Normally, the ERXRDPT will be advanced to the value
     pointed to by the next Packet Pointer which precedes
     the receive status vector for the current packet. Follow-
     ing such a procedure will not require any pointer
     calculations to account for wrapping at the end of the
     circular receive buffer. */

#if !AUTO_INCREMENT
  /* Set the read pointer to the head of the next frame */
  write_control_reg(ENC_ERDPTL, rx_header.next_pkt_ptr & 0xFF);
  write_control_reg(ENC_ERDPTH, rx_header.next_pkt_ptr >> 8);
#endif

  /* B7 Silicon Errata #11 */
  if(rx_header.next_pkt_ptr == RX_BUF_START) {
    rx_header.next_pkt_ptr = RX_BUF_END;
  } else {
    rx_header.next_pkt_ptr = rx_header.next_pkt_ptr - 1;
  }

  /* Order matters */
  write_control_reg(ENC_ERXRDPTL, rx_header.next_pkt_ptr & 0xFF);
  write_control_reg(ENC_ERXRDPTH, rx_header.next_pkt_ptr >> 8);

  /* In addition to advancing the Receive Buffer Read
     Pointer, after each packet is fully processed, the host
     controller must write a ‘1’ to the ECON2.PKTDEC bit. */
  set_bit_field(ENC_ECON2, ENC_ECON2_PKTDEC);

  if(err) {
    PRINTF("ETH: rx err: flushed %d\n", rx_header.byte_count);
    return 0;
  }

#if DEBUG_LEVEL > 4
  dump_rx_data(buffer, rx_header.byte_count);
#endif

  STATS_INC(eth_rx_pkts);
  STATS_ADD(eth_rx_bytes, rx_header.byte_count); 

  return rx_header.byte_count;

} /* enc28j60_read(), uk 15.04.2015 */
/*---------------------------------------------------------------------------*/
void
enc28j60_on(void)
{

  PRINTF("ETH: On\n");

  if (!(eth_flags & ETH_FLAGS_ON)) {

    /* 1. Wake-up by clearing ECON2.PWRSV.
       2. Wait at least 300 µs for the PHY to stabilize. To
       accomplish the delay, the host controller may
       poll ESTAT.CLKRDY and wait for it to become
       set.
       3. Restore receive capability by setting
       ECON1.RXEN. */

    clear_bit_field(ENC_ECON2, ENC_ECON2_PWRSV);
    BUSYWAIT_UNTIL((read_control_reg(ENC_ESTAT) &
                    ENC_ESTAT_CLKRDY),
                   OST_TIMEOUT);
    set_bit_field(ENC_ECON1, ENC_ECON1_RXEN);

    eth_flags |= ETH_FLAGS_ON;

  }

} /* enc28j60_on(), uk 14.04.2015 */
/*---------------------------------------------------------------------------*/
void
enc28j60_off(void)
{

  PRINTF("ETH: Off\n");

  if (eth_flags & ETH_FLAGS_ON) {

    /* 1. Turn off packet reception by clearing
       ECON1.RXEN.
       2. Wait for any in-progress packets to finish being
       received by polling ESTAT.RXBUSY. This bit
       should be clear before proceeding.
       3. Wait for any current transmissions to end by
       confirming ECON1.TXRTS is clear.
       4. Set ECON2.VRPS (if not already set).
       5. Enter Sleep by setting ECON2.PWRSV. All
       MAC, MII and PHY registers become
       inaccessible as a result. Setting PWRSV also
       clears ESTAT.CLKRDY automatically. */

    clear_bit_field(ENC_ECON1, ENC_ECON1_RXEN);
    BUSYWAIT_UNTIL(!(read_control_reg(ENC_ESTAT) &
                     ENC_ESTAT_RXBUSY),
                   RTIMER_SECOND/100);
    BUSYWAIT_UNTIL(!(read_control_reg(ENC_ECON1) &
                     ENC_ECON1_TXRTS),
                   RTIMER_SECOND/100);
    set_bit_field(ENC_ECON2, ENC_ECON2_VRPS);
    set_bit_field(ENC_ECON2, ENC_ECON2_PWRSV);

    eth_flags &= ~ETH_FLAGS_ON;

  }

} /* enc28j60_off(), uk 14.04.2015 */
/*---------------------------------------------------------------------------*/
void
enc28j60_configure_leda(uint8_t cfg)
{

  uint16_t phy_reg;

  phy_reg = read_phy_reg(ENC_PHLCON);
  phy_reg &= ~0x0F00;
  phy_reg |= ((cfg & 0x0F) << 8);
  write_phy_reg(ENC_PHLCON, phy_reg);

} /* enc28j60_configure_leda(), uk 14.04.2015 */
/*---------------------------------------------------------------------------*/
void
enc28j60_configure_ledb(uint8_t cfg)
{

  uint16_t phy_reg;

  phy_reg = read_phy_reg(ENC_PHLCON);
  phy_reg &= ~0x00F0;
  phy_reg |= ((cfg & 0x0F) << 4);
  write_phy_reg(ENC_PHLCON, phy_reg);

} /* enc28j60_configure_ledb(), uk 14.04.2015 */
/*---------------------------------------------------------------------------*/
void
enc28j60_configure_stretching(uint8_t cfg)
{

  uint16_t phy_reg;

  phy_reg = read_phy_reg(ENC_PHLCON);
  phy_reg &= ~0x000F;
  phy_reg |= ((cfg & 0x07) << 1);
  write_phy_reg(ENC_PHLCON, phy_reg);

} /* enc28j60_configure_stretching(), uk 14.04.2015 */
/*---------------------------------------------------------------------------*/
int
enc28j60_interrupt_handler(void)
{

  PRINTF("ETH: IR handler\n");
#if USE_INT_N
  process_poll(&enc28j60_process);
#endif
  return 0;

}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(enc28j60_process, ev, data)
{

#if USE_ETH_WATCHDOG
  
  static struct etimer et;
  
#if USE_INT_N
  PROCESS_POLLHANDLER(pollhandler());
#endif
  
  PROCESS_BEGIN();
  
  while(1) {
#define RESET_PERIOD (30*CLOCK_SECOND)
    etimer_set(&et, RESET_PERIOD);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    
    PRINTF("ETH: test received_packet %d > sent_packets %d\n", 
         num_rx_pkts, 
         num_tx_pkts);
    if(num_rx_pkts <= num_tx_pkts) {
      PRINTF("ETH: resetting chip\n");
      configure(eui48);
    }
    num_rx_pkts = 0;
    num_tx_pkts = 0;
  }

  PROCESS_END();

#else /* USE_ETH_WATCHDOG */
  
#if USE_INT_N
  PROCESS_POLLHANDLER(pollhandler());
#endif
  
  PROCESS_BEGIN();

  PRINTF("ENC28J60 PROCESS STARTED...\r\n");

  PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_EXIT);

  PROCESS_END();

#endif /* USE_ETH_WATCHDOG */

}
/*---------------------------------------------------------------------------*/
