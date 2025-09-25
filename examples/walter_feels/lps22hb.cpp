/**
 * @file lps22hb.cpp
 * @author Daan Pape <daan@dptechnics.com> Arnoud Devoogdt <arnoud@dptechnics.com>
 * @date 25 September 2025
 * @copyright DPTechnics bv
 * @brief Walter Modem library examples
 *
 * @section LICENSE
 *
 * Copyright (C) 2023, DPTechnics bv
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *   3. Neither the name of DPTechnics bv nor the names of its contributors may
 *      be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 *   4. This software, with or without modification, must only be used with a
 *      Walter board from DPTechnics bv.
 *
 *   5. Any software provided in binary form under this license must not be
 *      reverse engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY DPTECHNICS BV “AS IS” AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL DPTECHNICS BV OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @section DESCRIPTION
 *
 * This file contains drivers for the Walter Feels carrier board
 */

#include <Wire.h>

#include "lps22hb.h"

#define LPS22HB_ADDRESS 0x5C

#define LPS22HB_WHO_AM_I_REG 0x0f
#define LPS22HB_CTRL2_REG 0x11
#define LPS22HB_STATUS_REG 0x27
#define LPS22HB_PRESS_OUT_XL_REG 0x28
#define LPS22HB_PRESS_OUT_L_REG 0x29
#define LPS22HB_PRESS_OUT_H_REG 0x2a
#define LPS22HB_TEMP_OUT_L_REG 0x2b
#define LPS22HB_TEMP_OUT_H_REG 0x2c

LPS22HB::LPS22HB(TwoWire& wire) : _wire(&wire), _initialized(false)
{
}

int LPS22HB::begin()
{
  if(i2cRead(LPS22HB_WHO_AM_I_REG) != 0xb1) {
    printf("I died\n");
    end();
    return 0;
  }

  printf("I lived\n");
  _initialized = true;
  return 1;
}

void LPS22HB::end()
{
#if defined(WIRE_HAS_END) && WIRE_HAS_END
  _wire->end();
#endif
  _initialized = false;
}

float LPS22HB::readPressure(int units)
{
  if(_initialized == true) {
    // trigger one shot
    i2cWrite(LPS22HB_CTRL2_REG, 0x01);

    // wait for ONE_SHOT bit to be cleared by the hardware
    while((i2cRead(LPS22HB_CTRL2_REG) & 0x01) != 0) {
      yield();
    }

    float reading = (i2cRead(LPS22HB_PRESS_OUT_XL_REG) | (i2cRead(LPS22HB_PRESS_OUT_L_REG) << 8) |
                     (i2cRead(LPS22HB_PRESS_OUT_H_REG) << 16)) /
                    40960.0;

    if(units == MILLIBAR) { // 1 kPa = 10 millibar
      return reading * 10;
    } else if(units == PSI) { // 1 kPa = 0.145038 PSI
      return reading * 0.145038;
    } else {
      return reading;
    }
  }
  return 0;
}

float LPS22HB::readTemperature(void)
{
  float reading = (i2cRead(LPS22HB_TEMP_OUT_L_REG) << 0) | (i2cRead(LPS22HB_TEMP_OUT_H_REG) << 8);

  return reading / 100;
}

int LPS22HB::i2cRead(uint8_t reg)
{
  _wire->beginTransmission(LPS22HB_ADDRESS);
  _wire->write(reg);
  if(_wire->endTransmission(false) != 0) {
    return -1;
  }

  if(_wire->requestFrom(LPS22HB_ADDRESS, 1) != 1) {
    return -1;
  }

  return _wire->read();
}

int LPS22HB::i2cWrite(uint8_t reg, uint8_t val)
{
  _wire->beginTransmission(LPS22HB_ADDRESS);
  _wire->write(reg);
  _wire->write(val);
  if(_wire->endTransmission() != 0) {
    return 0;
  }

  return 1;
}
