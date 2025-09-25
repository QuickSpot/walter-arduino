/**
 * @file hdc1080.cpp
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

#include "hdc1080.h"

HDC1080::HDC1080()
{
}

void HDC1080::begin()
{
  setResolution(HDC1080_RESOLUTION_14BIT, HDC1080_RESOLUTION_14BIT);
}

void HDC1080::setResolution(HDC1080_MeasurementResolution humidity,
                            HDC1080_MeasurementResolution temperature)
{
  HDC1080_Registers reg;
  reg.HumidityMeasurementResolution = 0;
  reg.TemperatureMeasurementResolution = 0;

  if(temperature == HDC1080_RESOLUTION_11BIT)
    reg.TemperatureMeasurementResolution = 0x01;

  switch(humidity) {
  case HDC1080_RESOLUTION_8BIT:
    reg.HumidityMeasurementResolution = 0x02;
    break;
  case HDC1080_RESOLUTION_11BIT:
    reg.HumidityMeasurementResolution = 0x01;
    break;
  default:
    break;
  }

  writeRegister(reg);
}

HDC1080_SerialNumber HDC1080::readSerialNumber()
{
  HDC1080_SerialNumber sernum;
  sernum.serialFirst = readData(HDC1080_SERIAL_ID_FIRST);
  sernum.serialMid = readData(HDC1080_SERIAL_ID_MID);
  sernum.serialLast = readData(HDC1080_SERIAL_ID_LAST);
  return sernum;
}

HDC1080_Registers HDC1080::readRegister()
{
  HDC1080_Registers reg;
  reg.rawData = (readData(HDC1080_CONFIGURATION) >> 8);
  return reg;
}

void HDC1080::writeRegister(HDC1080_Registers reg)
{
  Wire.beginTransmission(HDC1080_I2C_ADDRESS);
  Wire.write(HDC1080_CONFIGURATION);
  Wire.write(reg.rawData);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(10);
}

void HDC1080::heatUp(uint8_t seconds)
{
  HDC1080_Registers reg = readRegister();
  reg.Heater = 1;
  reg.ModeOfAcquisition = 1;
  writeRegister(reg);

  uint8_t buf[4];
  for(int i = 1; i < (seconds * 66); i++) {
    Wire.beginTransmission(HDC1080_I2C_ADDRESS);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(20);
    Wire.requestFrom(HDC1080_I2C_ADDRESS, (uint8_t) 4);
    Wire.readBytes(buf, (size_t) 4);
  }
  reg.Heater = 0;
  reg.ModeOfAcquisition = 0;
  writeRegister(reg);
}

double HDC1080::readT()
{
  return readTemperature();
}

double HDC1080::readTemperature()
{
  uint16_t rawT = readData(HDC1080_TEMPERATURE);
  return (rawT / pow(2, 16)) * 165.0 - 40.0;
}

double HDC1080::readH()
{
  return readHumidity();
}

double HDC1080::readHumidity()
{
  uint16_t rawH = readData(HDC1080_HUMIDITY);
  return (rawH / pow(2, 16)) * 100.0;
}

uint16_t HDC1080::readManufacturerId()
{
  return readData(HDC1080_MANUFACTURER_ID);
}

uint16_t HDC1080::readDeviceId()
{
  return readData(HDC1080_DEVICE_ID);
}

uint16_t HDC1080::readData(uint8_t pointer)
{
  Wire.beginTransmission(HDC1080_I2C_ADDRESS);
  Wire.write(pointer);
  Wire.endTransmission();

  delay(15);
  Wire.requestFrom(HDC1080_I2C_ADDRESS, (uint8_t) 2);

  byte msb = Wire.read();
  byte lsb = Wire.read();

  return msb << 8 | lsb;
}
