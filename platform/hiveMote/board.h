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
/** \addtogroup hiveMotes
 * @{
 *
 * \defgroup Peripherals for the HiveMotes
 *
 * This file provides connectivity information on LEDs, Buttons, UART and
 * other peripherals
 *
 * This file can be used as the basis to configure other boards using the
 * CC13xx/CC26xx code as their basis.
 *
 * This file is not meant to be modified by the user.
 * @{
 *
 * \file
 * Header file with definitions related to the I/O connections on the TI
 * SmartRF06 Evaluation Board with a CC26xxEM
 *
 * \note   Do not include this file directly. It gets included by contiki-conf
 *         after all relevant directives have been set.
 */
/*---------------------------------------------------------------------------*/
#ifndef BOARD_H_
#define BOARD_H_
/*---------------------------------------------------------------------------*/
#include "ioc.h"
/*---------------------------------------------------------------------------*/
/**
 * \name LED configurations
 *
 * Those values are not meant to be modified by the user
 * @{
 */
/* Some files include leds.h before us, so we need to get rid of defaults in
 * leds.h before we provide correct definitions */
#undef LEDS_GREEN
#undef LEDS_RED
#undef LEDS_CONF_ALL

/* TODO: Change LED color name as used in the board. */
#define LEDS_RED       			1 /**< LED1 (Red)    */
#define LEDS_GREEN    			2 /**< LED2 (Yellow) */

#define LEDS_CONF_ALL 			3

/* Notify various examples that we have LEDs */
#define PLATFORM_HAS_LEDS        1
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name LED IOID mappings
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_LED_1          IOID_6
#define BOARD_IOID_LED_2          IOID_7
#define BOARD_LED_1               (1 << BOARD_IOID_LED_1)
#define BOARD_LED_2               (1 << BOARD_IOID_LED_2)
#define BOARD_LED_ALL             (BOARD_LED_1 | BOARD_LED_2)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name UART IOID mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_UART_RX        IOID_2
#define BOARD_IOID_UART_TX        IOID_3
#define BOARD_IOID_UART_CTS       IOID_UNUSED
#define BOARD_IOID_UART_RTS       IOID_UNUSED
#define BOARD_UART_RX             (1 << BOARD_IOID_UART_RX)
#define BOARD_UART_TX             (1 << BOARD_IOID_UART_TX)
#define BOARD_UART_CTS            (1 << BOARD_IOID_UART_CTS)
#define BOARD_UART_RTS            (1 << BOARD_IOID_UART_RTS)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name Button IOID mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_USER_BUTTON		IOID_11
#define BOARD_USER_BUTTON          	(1 << BOARD_IOID_USER_BUTTON)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name SPI IOID mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_SPI_0_SCK        IOID_10
#define BOARD_IOID_SPI_0_MOSI       IOID_9
#define BOARD_IOID_SPI_0_MISO       IOID_8
#define BOARD_SPI_0_SCK             (1 << BOARD_IOID_SPI_0_SCK)
#define BOARD_SPI_0_MOSI            (1 << BOARD_IOID_SPI_0_MOSI)
#define BOARD_SPI_0_MISO            (1 << BOARD_IOID_SPI_0_MISO)

#define BOARD_IOID_SPI_1_SCK        IOID_18 /* MOD_DIO_13 */
#define BOARD_IOID_SPI_1_MISO       IOID_19 /* MOD_DIO_12 */
#define BOARD_IOID_SPI_1_MOSI       IOID_20 /* MOD_DIO_11 */
#define BOARD_SPI_1_SCK             (1 << BOARD_IOID_SPI_1_SCK)
#define BOARD_SPI_1_MOSI            (1 << BOARD_IOID_SPI_1_MOSI)
#define BOARD_SPI_1_MISO            (1 << BOARD_IOID_SPI_1_MISO)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \brief I2C IOID mappings
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_SDA            	IOID_29 /**< Interface 0 SDA: All sensors bar MPU */
#define BOARD_IOID_SCL            	IOID_30 /**< Interface 0 SCL: All sensors bar MPU */
// #define BOARD_IOID_SDA_HP         IOID_8 /**< Interface 1 SDA: MPU */
// #define BOARD_IOID_SCL_HP         IOID_9 /**< Interface 1 SCL: MPU */
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \brief MPU IOID mappings
 *
 * Those values are not meant to be modified by the user
 * @{
 */
// #define BOARD_IOID_MPU_INT        IOID_7
// #define BOARD_IOID_MPU_POWER      IOID_12
// #define BOARD_MPU_INT             (1 << BOARD_IOID_MPU_INT)
// #define BOARD_MPU_POWER           (1 << BOARD_IOID_MPU_POWER)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name SD Card IOID mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define SDCARD_SPI_INSTANCE 		0

#define BOARD_IOID_SDCARD_CS      	IOID_12
#define BOARD_SDCARD_CS           	(1 << BOARD_IOID_SDCARD_CS)
#define BOARD_IOID_SDCARD_CD      	IOID_13
#define BOARD_SDCARD_CD           	(1 << BOARD_IOID_SDCARD_CD)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name ENC28J60 IOID mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */

#define ENC28J60_SPI_INSTANCE 		1

#define BOARD_IOID_ENC28J60_INT   	IOID_15 /* MOD_DIO_2 */
#define BOARD_IOID_ENC28J60_CS    	IOID_21 /* MOD_DIO_10 */
#define BOARD_ENC28J60_INT        	(1 << BOARD_IOID_ENC28J60_INT)
#define BOARD_ENC28J60_CS         	(1 << BOARD_IOID_ENC28J60_CS)

#define ENC28J60_BIT_RATE			8000000  /* 8 MHz */

#define BOARD_IOID_ENC28J60_INT_CFG	(IOC_CURRENT_2MA  | IOC_STRENGTH_AUTO | \
	                                 IOC_IOPULL_UP    | IOC_SLEW_DISABLE  | \
	                                 IOC_HYST_DISABLE | IOC_FALLING_EDGE  | \
	                                 IOC_INT_ENABLE   | IOC_IOMODE_NORMAL | \
	                                 IOC_NO_WAKE_UP   | IOC_INPUT_ENABLE)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name Device string used on startup
 * @{
 */
#define BOARD_STRING "IoT HiveMote"
/** @} */
/*---------------------------------------------------------------------------*/
#endif /* BOARD_H_ */
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
