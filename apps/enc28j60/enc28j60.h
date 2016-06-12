/*
 * Copyright (c) 2012-2013, Thingsquare, http://www.thingsquare.com/.
 *
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
 * \defgroup cc2538-cc1200-eth-enc The platform's ethernet interface
 *
 * A 10BASE-T ethernet interface is realized by means of a Microchip
 * enc28j60 ethernet controller. This controller is connected to the
 * cc2538 using a dedicated SPI interface.
 *
 * @{
 *
 * \file Header file for the platform's ethernet interface
 */

#ifndef ENC28J60_H
#define ENC28J60_H

/*---------------------------------------------------------------------------*/
/**
 * \brief Initialize enc28j60 ethernet controller
 *
 * \param eth_eui48 The EUI-48 MAC address to be used by the controller
 * \note In fact the EUI-48 is read from the I2C-EEPROM. If this fails, 
 * the parameter passed is used as a fallback solution.
 */ 
void 
enc28j60_init(const uint8_t *eth_eui48);
/*---------------------------------------------------------------------------*/
/**
 * \brief Send a raw IP packet
 *
 * Places the data in the controllers TX FIFO and blocks until transmission
 * has completed.
 *
 * \param data The data to be send
 * \param data_len The number of bytes to send
 * \return The number of bytes send in case of success
 */ 
int 
enc28j60_send(const uint8_t *data, uint16_t data_len);
/*---------------------------------------------------------------------------*/
/**
 * \brief Read a raw IP packet
 *
 * Reads a raw IP packet from the controller's RX FIFO (if available). 
 *
 * \param buf The buffer to copy the IP packet
 * \param buf_size The size of the buffer
 * \return 0 in case of an empty RX FIFO, else the size of the IP packet read
 */
int 
enc28j60_read(uint8_t *buf, uint16_t buf_size);
/*---------------------------------------------------------------------------*/
/**
 * \brief Configure the controllers LEDA 
 *
 * \param cfg The LEDs configuration according to the following enumeration
 *  LEDx Configuration bits:
 * - 1111 = Reserved
 * - 1110 = Display duplex status and collision activity (always stretched)
 * - 1101 = Display link status and transmit/receive activity (always stretched)
 * - 1100 = Display link status and receive activity (always stretched)
 * - 1011 = Blink slow
 * - 1010 = Blink fast
 * - 1001 = Off
 * - 1000 = On
 * - 0111 = Display transmit and receive activity (stretchable)
 * - 0110 = Reserved
 * - 0101 = Display duplex status
 * - 0100 = Display link status
 * - 0011 = Display collision activity (stretchable)
 * - 0010 = Display receive activity (stretchable)
 * - 0001 = Display transmit activity (stretchable)
 * - 0000 = Reserved
 *
 * Default values:
 * - LEDA 0100 = Display link status
 * - LEDB 0010 = Display receive activity
 */
/*---------------------------------------------------------------------------*/
void
enc28j60_configure_leda(uint8_t cfg);
/*---------------------------------------------------------------------------*/
/**
 * \brief Configure the controllers LEDB
 *
 * \param cfg see \ref enc28j60_configure_leda()
 */
void
enc28j60_configure_ledb(uint8_t cfg);
/*---------------------------------------------------------------------------*/
/**
 * \brief Configure the stretching behavior of the controller's LEDs
 *
 * \param cfg The stretching behavior according to the following enumeration:
 *
 * Stretching configuration bit <1:2>
 * - 11 = Reserved
 * - 10 = Stretch LED events by T LSTRCH
 * - 01 = Stretch LED events by T MSTRCH
 * - 00 = Stretch LED events by T NSTRC
 *
 * Stretching configuration bit <0>
 * - 1 = Stretchable LED events will cause lengthened LED pulses
 * - 0 = Stretchable LED events will only be displayed while they are occurring
 */
void
enc28j60_configure_stretching(uint8_t cfg);
/*---------------------------------------------------------------------------*/

#endif /* ENC28J60_H */

/**
 * @}
 * @}
 */
