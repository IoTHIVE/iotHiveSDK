/*
 * Copyright (c) 2010, Swedish Institute of Computer Science.
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         Device drivers for adxl345 accelerometer in HiveMote.
 * \author
 *         Marcus Lundén, SICS <mlunden@sics.se>
 *         Enric M. Calvo, Zolertia <ecalvo@zolertia.com>
 */
#include "contiki.h"
#include "mpu-6050-sensor.h"
#include "board-i2c.h"
/*---------------------------------------------------------------------------*/
#define DEBUG 1
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
/* MPU6050 Interface Selection */
#define SENSOR_I2C_ADDRESS  0x68
#define SENSOR_SELECT()     board_i2c_select(BOARD_I2C_INTERFACE_0, SENSOR_I2C_ADDRESS)
#define SENSOR_DESELECT()   board_i2c_deselect()
/* -------------------------------------------------------------------------- */
/* Callback pointers when interrupt occurs */
// void (*accm_int1_cb)(uint8_t reg);
// void (*accm_int2_cb)(uint8_t reg);

// process_event_t int1_event, int2_event;

/* Bitmasks for the interrupts */
// static uint16_t int1_mask = 0, int2_mask = 0;

/* Keep track of when the interrupt was last seen in order to reduce the amount
  of interrupts. Kind of like button debouncing. This can't be per int-pin, as
  there can be several very different int per pin (eg tap && freefall). */
// XXX Not used now, only one global timer.
//static volatile clock_time_t ints_lasttime[] = {0, 0, 0, 0, 0, 0, 0, 0};

/* Bitmasks and bit flag variable for keeping track of adxl345 status. */
enum MPU6050_STATUSTYPES {
    /* must be a bit and not more, not using 0x00. */
    INITED = 0x01,
    RUNNING = 0x02,
    STOPPED = 0x04,
    LOW_POWER = 0x08,
    AAA = 0x10,   // available to extend this...
    BBB = 0x20,   // available to extend this...
    CCC = 0x40,   // available to extend this...
    DDD = 0x80,   // available to extend this...
};
// static enum MPU6050_STATUSTYPES _MPU6050_STATUS = 0x00;

/* Default values for adxl345 at startup. This will be sent to the adxl345 in a
    stream at init to set it up in a default state */
// static uint8_t adxl345_default_settings[] = {
//   /* Note, as the two first two bulks are to be written in a stream, they contain
//     the register address as first byte in that section. */
//   /* 0--14 are in one stream, start at MPU6050_THRESH_TAP */
//   MPU6050_THRESH_TAP,         // XXX NB Register address, not register value!!
//   MPU6050_THRESH_TAP_DEFAULT,
//   MPU6050_OFSX_DEFAULT,
//   MPU6050_OFSY_DEFAULT,
//   MPU6050_OFSZ_DEFAULT,
//   MPU6050_DUR_DEFAULT,
//   MPU6050_LATENT_DEFAULT,
//   MPU6050_WINDOW_DEFAULT,
//   MPU6050_THRESH_ACT_DEFAULT,
//   MPU6050_THRESH_INACT_DEFAULT,
//   MPU6050_TIME_INACT_DEFAULT,
//   MPU6050_ACT_INACT_CTL_DEFAULT,
//   MPU6050_THRESH_FF_DEFAULT,
//   MPU6050_TIME_FF_DEFAULT,
//   MPU6050_TAP_AXES_DEFAULT,

//   /* 15--19 start at MPU6050_BW_RATE */
//   MPU6050_BW_RATE,    // XXX NB Register address, not register value!!
//   MPU6050_BW_RATE_DEFAULT,
//   MPU6050_POWER_CTL_DEFAULT,
//   MPU6050_INT_ENABLE_DEFAULT,
//   MPU6050_INT_MAP_DEFAULT,

//   /* These two: 20, 21 write separately */
//   MPU6050_DATA_FORMAT_DEFAULT,
//   MPU6050_FIFO_CTL_DEFAULT
// };
/*---------------------------------------------------------------------------*/
// PROCESS(accmeter_process, "Accelerometer process");
/*---------------------------------------------------------------------------*/
/* Write to a register.
    args:
      reg       register to write to
      val       value to write
*/
// uint8_t acc_range = ACC_RANGE_2G;

float
acc_convert(int16_t raw_data)
{
  float v = 0;

  // switch(acc_range) {
  // case ACC_RANGE_2G:
    /* Calculate acceleration, unit G, range -2, +2 */
    v = (raw_data * 1) / (32768 / 2);
  //   break;
  // case ACC_RANGE_4G:
  //   /* Calculate acceleration, unit G, range -4, +4 */
  //   v = (raw_data * 1.0) / (32768 / 4);
  //   break;
  // case ACC_RANGE_8G:
  //    Calculate acceleration, unit G, range -8, +8 
  //   v = (raw_data * 1.0) / (32768 / 8);
  //   break;
  // case ACC_RANGE_16G:
  //   /* Calculate acceleration, unit G, range -16, +16 */
  //   v = (raw_data * 1.0) / (32768 / 16);
  //   break;
  // default:
  //   v = 0;
  //   break;
  // }
  return v;
}

void
accm_write_reg(uint8_t reg, uint8_t val) {

  uint8_t buffer[2] = {reg,val};
  SENSOR_SELECT();

    // sensor_common_write_reg(reg, &val, 1);
    board_i2c_write(buffer, 2);
  SENSOR_DESELECT();
  PRINTF("WRITE_REG 0x%02X @ reg 0x%02X\n\r", val, reg);
}
/*---------------------------------------------------------------------------*/
/* Write several registers from a stream.
    args:
      len       number of bytes to read
      data      pointer to where the data is read from

  First byte in stream must be the register address to begin writing to.
  The data is then written from second byte and increasing. */

void
accm_write_stream(uint8_t len, uint8_t *data) {

  SENSOR_SELECT();
  board_i2c_write(data, len);
  SENSOR_DESELECT();
  PRINTF("WRITE_STR %u B to 0x%02X\n\r", len, data[0]);
}
/*---------------------------------------------------------------------------*/
/* Read one register.
    args:
      reg       what register to read
    returns the value of the read register
*/

uint8_t
accm_read_reg(uint8_t reg) {

  uint8_t retVal = 0;
  PRINTF("READ_REG 0x%02X\n\r", reg);
  SENSOR_SELECT();
  // sensor_common_read_reg(reg, &retVal, 1);
  board_i2c_write_read(&reg, 1, &retVal, 1);
  SENSOR_DESELECT();
  return retVal;
}
/*---------------------------------------------------------------------------*/
/* Read several registers in a stream.
    args:
      reg       what register to start reading from
      len       number of bytes to read
      whereto   pointer to where the data is saved
*/

void
accm_read_stream(uint8_t reg, uint8_t len, uint8_t *whereto) {

  SENSOR_SELECT();
  board_i2c_write_read(&reg, 1, whereto, len);
  SENSOR_DESELECT();
}
/*---------------------------------------------------------------------------*/
/* Read an axis of the accelerometer (x, y or z). Return value is a signed 10 bit int.
  The resolution of the acceleration measurement can be increased up to 13 bit, but
  will change the data format of this read out. Refer to the data sheet if so is
  wanted/needed. */

int16_t
accm_read_axis(uint8_t axis){
  int16_t rd = 0;
  uint8_t tmp[2];
  // if(axis > Z_AXIS){
  //   return 0;
  // }
  // accm_read_stream(ACCEL_XOUT_H + axis, 2, &tmp[0]);
  accm_read_stream(ACCEL_XOUT_H + axis, 2, tmp);
  rd = (int16_t)(tmp[0] | (tmp[1]<<8));
  return rd;
}
/*---------------------------------------------------------------------------*/
/* Sets the g-range, ie the range the accelerometer measures (ie 2g means -2 to +2 g
    on every axis). Possible values:
        MPU6050_RANGE_2G
        MPU6050_RANGE_4G
        MPU6050_RANGE_8G
        MPU6050_RANGE_16G
    Example:
        accm_set_grange(MPU6050_RANGE_4G);
    */

void
accm_set_grange(uint8_t grange){

  // if(grange > MPU6050_RANGE_16G) {
  //   // invalid g-range.
  //   PRINTF("ADXL grange invalid: %u\n", grange);
  //   return;
  // }
  uint8_t tempreg = 0;

  /* preserve the previous contents of the register */
  // tempreg = (accm_read_reg(MPU6050_DATA_FORMAT) & 0xFC);  // zero out the last two bits (grange)
  // tempreg |= grange;                                      // set new range
  // accm_write_reg(MPU6050_DATA_FORMAT, tempreg);
}

/*---------------------------------------------------------------------------*/
/* Init the accelerometer: ports, pins, registers, interrupts (none enabled), I2C,
    default threshold values etc. */

void
accm_init(void) {

    PRINTF("Initializing MPU6050 1\n\r");
    
    // accm_int1_cb = NULL;
    // accm_int2_cb = NULL;
    // int1_event = process_alloc_event();
    // int2_event = process_alloc_event();

    /* Set up ports and pins for interrups. */
    // MPU6050_DIR  &=~ (MPU6050_INT1_PIN | MPU6050_INT2_PIN);
    // MPU6050_SEL  &=~ (MPU6050_INT1_PIN | MPU6050_INT2_PIN);
    // MPU6050_SEL2 &=~ (MPU6050_INT1_PIN | MPU6050_INT2_PIN);
    uint8_t data[] = {0x6B, 0};
    SENSOR_SELECT();
    accm_write_stream(2, data);
    SENSOR_DESELECT();

    if(accm_read_reg(117) != SENSOR_I2C_ADDRESS)
    {
      PRINTF("ERROR: DEVICE ID COULD NOT BE VERIFIED!!!\r\n");
      return;
    }
    else
    {
      // /* set default register values. */
      // accm_write_stream(15, &adxl345_default_settings[0]);
      // accm_write_stream(5, &adxl345_default_settings[15]);
      // accm_write_reg(MPU6050_DATA_FORMAT, adxl345_default_settings[20]);
      // accm_write_reg(MPU6050_FIFO_CTL, adxl345_default_settings[21]);
    }

    // process_start(&accmeter_process, NULL);

    /* Enable msp430 interrupts on the two interrupt pins. */
    // dint();
    // MPU6050_IES &=~ (MPU6050_INT1_PIN | MPU6050_INT2_PIN);   // low to high transition interrupts
    // MPU6050_IE |= (MPU6050_INT1_PIN | MPU6050_INT2_PIN);     // enable interrupts
    // eint();
  // }
}