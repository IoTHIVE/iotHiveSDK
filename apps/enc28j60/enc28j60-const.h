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
 * \addtogroup cc2538-cc1200-eth-enc
 *
 * @{
 *
 * \file Header file containing enc28j60 device specific constants
 */

#ifndef ENC_CONST_H
#define ENC_CONST_H

/** ENC28J60 uses 5 bit addresses */
#define ENC_ADDR_MASK           0x1F
/*---------------------------------------------------------------------------*/
/* Prefixes used to identify the bank a register is located in and wheter
   it is a MAC register or not */
/*---------------------------------------------------------------------------*/
/** The prefix used to indentify a MAC register (dummy read!) */
#define ENC_MAC_PFX             0x80
/** The prefix used to identify a register located in bank 1 */
#define ENC_BANK1_PFX           (1 << 5)
/** The prefix used to identify a register located in bank 1 */
#define ENC_BANK2_PFX           (2 << 5)
/** The prefix used to identify a register located in bank 1 */
#define ENC_BANK3_PFX           (3 << 5)
/*---------------------------------------------------------------------------*/
/* SPI instruction set */
/*---------------------------------------------------------------------------*/
/** Read control register, to be XOR'ed with argument */
#define ENC_CMD_RCR             0x00
/** Read buffer memory */
#define ENC_CMD_RBM             0x3A
/** Write control register, to be XOR'ed with argument */
#define ENC_CMD_WCR             0x40
/** Write buffer memory */
#define ENC_CMD_WBM             0x7A
/** Bit field set, to be XOR'ed with argument */
#define ENC_CMD_BFS             0x80
/** Bit field clear, to be XOR'ed with argument */
#define ENC_CMD_BFC             0xA0
/** Soft reset */
#define ENC_CMD_SRC             0xFF
/*---------------------------------------------------------------------------*/
/* Registers available in all banks */
/*---------------------------------------------------------------------------*/
/** Ethernet interrupt enable register */
#define ENC_EIE                 0x1B
/** Global INT Interrupt Enable bit */
#define ENC_EIE_INTIE           (1 << 7)
/** Receive Packet Pending Interrupt Enable bit */
#define ENC_EIE_PKTIE           (1 << 6)
/** DMA Interrupt Enable bit */
#define ENC_EIE_DMAIE           (1 << 5)
/** Link Status Change Interrupt Enable bit */
#define ENC_EIE_LINKIE          (1 << 4)
/** Transmit Enable bit */
#define ENC_EIE_TXIE            (1 << 3)
/** Transmit Error Interrupt Enable bit */
#define ENC_EIE_TXERIE          (1 << 1)
/** Receive Error Interrupt Enable bit */
#define ENC_EIE_RXERIE          (1 << 0)

/** Ethernet interrupt flag register */
#define ENC_EIR                 0x1C
/** Receive Packet Pending Interrupt Flag bit */
#define ENC_EIR_PKTIF           (1 << 6)
/** DMA Interrupt Flag bit */
#define ENC_EIR_DMAIF           (1 << 5)
/** Link Change Interrupt Flag bit */
#define ENC_EIR_LINKIF          (1 << 4)
/** Transmit Interrupt Flag bit */
#define ENC_EIR_TXIF            (1 << 3)
/* Transmit Error Interrupt Flag bit */
#define ENC_EIR_TXERIF          (1 << 1)
/** Receive Error Interrupt Flag bit */
#define ENC_EIR_RXERIF          (1 << 0)

/** Ethernet status register */
#define ENC_ESTAT               0x1D
#define ENC_ESTAT_INT           (1 << 7)
#define ENC_ESTAT_BUFER         (1 << 6)
#define ENC_ESTAT_LATECOL       (1 << 4)
#define ENC_ESTAT_RXBUSY        (1 << 2)
#define ENC_ESTAT_TXABRT        (1 << 1)
#define ENC_ESTAT_CLKRDY        (1 << 0)

#define ENC_ECON2               0x1E
/** Automatic Buffer Pointer Increment Enable bit */
#define ENC_ECON2_AUTOINC       (1 << 7)
/** Packet Decrement bit */
#define ENC_ECON2_PKTDEC        (1 << 6)
/** Power Save Enable bit */
#define ENC_ECON2_PWRSV         (1 << 5)
/** Voltage Regulator Power Save Enable bit */
#define ENC_ECON2_VRPS          (1 << 3)

#define ENC_ECON1               0x1F
/** Transmit Logic Reset bit */
#define ENC_ECON1_TXRST         (1 << 7)
/** Transmit Request to Send bit */
#define ENC_ECON1_TXRTS         (1 << 3)
/** Receive Enable bit */
#define ENC_ECON1_RXEN          (1 << 2)

/*---------------------------------------------------------------------------*/
/* Control registers in bank 0 */
/*---------------------------------------------------------------------------*/
#define ENC_ERDPTL              0x00
#define ENC_ERDPTH              0x01
#define ENC_EWRPTL              0x02
#define ENC_EWRPTH              0x03
#define ENC_ETXSTL              0x04
#define ENC_ETXSTH              0x05
#define ENC_ETXNDL              0x06
#define ENC_ETXNDH              0x07
#define ENC_ERXSTL              0x08
#define ENC_ERXSTH              0x09
#define ENC_ERXNDL              0x0A
#define ENC_ERXNDH              0x0B
#define ENC_ERXRDPTL            0x0C
#define ENC_ERXRDPTH            0x0D
#define ENC_ERXWRPTL            0x0E
#define ENC_ERXWRPTH            0x0F
/*---------------------------------------------------------------------------*/
/* Registers in bank 1. Prefixed with 0x20 */
/*---------------------------------------------------------------------------*/
#define ENC_ERXFCON             (ENC_BANK1_PFX | 0x18)
/** Unicast Filter Enable bit */
#define ENC_ERXFCON_UCEN        (1 << 7)
/** AND/OR Filter Select bit */
#define ENC_ERXFCON_ANDOR       (1 << 6)
/** Post-Filter CRC Check Enable bit */
#define ENC_ERXFCON_CRCEN       (1 << 5)
/** Pattern Match Filter Enable bit */
#define ENC_ERXFCON_PMEN        (1 << 4)
/** Magic PacketTM Filter Enable bit */
#define ENC_ERXFCON_MPEN        (1 << 3)
/** Hash Table Filter Enable bit */
#define ENC_ERXFCON_HTEN        (1 << 2)
/** Multicast Filter Enable bit */
#define ENC_ERXFCON_MCEN        (1 << 1)
/** Broadcast Filter Enable bit */
#define ENC_ERXFCON_BCEN        (1 << 0)

#define ENC_EPKTCNT             (ENC_BANK1_PFX | 0x19)
/*---------------------------------------------------------------------------*/
/* Registers in bank 2. Prefixed with 0x40 */
/*---------------------------------------------------------------------------*/
#define ENC_MACON1              (ENC_MAC_PFX | ENC_BANK2_PFX | 0x00)
/** Pause Control Frame Transmission Enable bit */
#define ENC_MACON1_TXPAUS       (1 << 3)
/** Pause Control Frame Reception Enable bit */
#define ENC_MACON1_RXPAUS       (1 << 2)
/** MAC Receive Enable bit */
#define ENC_MACON1_MARXEN       (1 << 0)

#define ENC_MACON3              (ENC_MAC_PFX | ENC_BANK2_PFX | 0x02)
/** All short frames will be zero-padded to 64 bytes
  and a valid CRC will then be appended */
#define ENC_MACON3_PADCFG_7     (7 << 5)
/** All short frames will be zero-padded to 60 bytes
  and a valid CRC will then be appended */
#define ENC_MACON3_PADCFG_1     (1 << 5)
/** No automatic padding of short frames */
#define ENC_MACON3_PADCFG_0     (0 << 5)

/** Transmit CRC Enable bit */
#define ENC_MACON3_TXCRCEN      (1 << 4)
/** Frame Length Checking Enable bit */
#define ENC26J60_MACON3_FRMLNEN (1 << 1)
/** MAC Full-Duplex Enable bit */
#define ENC_MACON3_FULDPX       (1 << 0)

#define ENC_MACON4              (ENC_MAC_PFX | ENC_BANK2_PFX | 0x03)
/** Defer Transmission Enable bit (applies to half duplex only) */
#define ENC_MACON4_DEFER        (1 << 6)
/** No Backoff During Backpressure Enable bit (applies to half duplex only) */
#define ENC_MACON4_BPEN         (1 << 5)
/** No Backoff Enable bit (applies to half duplex only) */
#define ENC_MACON4_NOBKOFF      (1 << 4)

#define ENC_MABBIPG             (ENC_MAC_PFX | ENC_BANK2_PFX | 0x04)
#define ENC_MAIPGL              (ENC_MAC_PFX | ENC_BANK2_PFX | 0x06)
#define ENC_MAIPGH              (ENC_MAC_PFX | ENC_BANK2_PFX | 0x07)
#define ENC_MAMXFLL             (ENC_MAC_PFX | ENC_BANK2_PFX | 0x0A)
#define ENC_MAMXFLH             (ENC_MAC_PFX | ENC_BANK2_PFX | 0x0B)

#define ENC_MICMD               (ENC_MAC_PFX | ENC_BANK2_PFX | 0x12)
/** MII Scan Enable bit */
#define ENC_MICMD_MIISCAN       (1 << 1)
/** MII Read Enable bit */
#define ENC_MICMD_MIIRD         (1 << 0)

#define ENC_MIREGADR            (ENC_MAC_PFX | ENC_BANK2_PFX | 0x14)
#define ENC_MIWRL               (ENC_MAC_PFX | ENC_BANK2_PFX | 0x16)
#define ENC_MIWRH               (ENC_MAC_PFX | ENC_BANK2_PFX | 0x17)
#define ENC_MIRDL               (ENC_MAC_PFX | ENC_BANK2_PFX | 0x18)
#define ENC_MIRDH               (ENC_MAC_PFX | ENC_BANK2_PFX | 0x19)
/*---------------------------------------------------------------------------*/
/* Registers in bank 3. Prefixed with 0x60 */
/*---------------------------------------------------------------------------*/
#define ENC_MAADR5              (ENC_MAC_PFX | ENC_BANK3_PFX | 0x00)
#define ENC_MAADR6              (ENC_MAC_PFX | ENC_BANK3_PFX | 0x01)
#define ENC_MAADR3              (ENC_MAC_PFX | ENC_BANK3_PFX | 0x02)
#define ENC_MAADR4              (ENC_MAC_PFX | ENC_BANK3_PFX | 0x03)
#define ENC_MAADR1              (ENC_MAC_PFX | ENC_BANK3_PFX | 0x04)
#define ENC_MAADR2              (ENC_MAC_PFX | ENC_BANK3_PFX | 0x05)

#define ENC_MISTAT              (ENC_MAC_PFX | ENC_BANK3_PFX | 0x0A)
/** MII Management Scan Operation bit */
#define ENC_MISTAT_SCAN         (1 << 1)
/** MII Management Busy bit */
#define ENC_MISTAT_BUSY         (1 << 0)

#define ENC_EREVID              (ENC_BANK3_PFX | 0x12)
/*---------------------------------------------------------------------------*/
/* PHY registers */
/*---------------------------------------------------------------------------*/
#define ENC_PHCON1              0x00
/** PHY Software Reset bit */
#define ENC_PHCON1_PRST         (1 << 15)
/** PHY Loopback bit */
#define ENC_PHCON1_PLOOPBK      (1 << 14)
/** PHY Power-Down bit */
#define ENC_PHCON1_PPWRSV       (1 << 11)
/**PHY Duplex Mode bit */
#define ENC_PHCON1_PDPXMD       (1 << 8)

#define ENC_PHSTAT1             0x01
#define ENC_PHID1               0x02
#define ENC_PHID2               0x03

#define ENC_PHCON2              0x10
/** PHY Half-Duplex Loopback Disable bit */
#define ENC_PHCON2_HDLDIS       (1 << 8)

#define ENC_PHSTAT2             0x11
#define ENC_PHIE                0x12
#define ENC_PHIR                0x13
#define ENC_PHLCON              0x14
/*---------------------------------------------------------------------------*/

#endif  /* #ifndef ENC_CONST_H */

/**
 * @}
 */
