/*
 *  Copyright (c) 2019 BAE Systems
 *
 *  Author(s):  Roman Kostin  <rqk@stlport.com>
 *
 */

#ifndef __ML_I2C_H__
#define __ML_I2C_H__

#include "main.h"

/* Timing register value is computed with the STM32CubeMX Tool,
  * Fast Mode @400kHz with I2CCLK = 80 MHz,
  * rise time = 100ns, fall time = 10ns
  * Timing Value = (uint32_t)0x00F02B86
  */
#define I2C_TIMING               0x00F02B86

typedef enum {
  I2CS_IDLE,
  I2CS_WADDR,
  I2CS_WADDR0,
  I2CS_WADDR1,
  I2CS_WADDRDONE,
  I2CS_WDATA0,
  I2CS_WDATA1,
  I2CS_WDATA1P,
  I2CS_WDATADONE,
  I2CS_READREG,
  I2CS_READREG1,
  I2CS_READREG2,
  I2CS_RDATA,
  I2CS_RDATAWAIT1,
  I2CS_RDATAWAIT2,
  I2CS_RDATA0,
  I2CS_RDATA0P,
  I2CS_RDATA1,
  I2CS_RDATA1P,
  I2CS_RESETWAIT,
  I2CS_RESET

} I2CS_State_Typedef;

#define SLAVE_BYTE_TO_SEND       (uint8_t)0xA5

#endif // __ML_I2C_H__

