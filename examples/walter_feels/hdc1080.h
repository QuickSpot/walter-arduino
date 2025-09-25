/**
 * @file hdc1080.h
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

#ifndef _HDC1080_h

#define _HDC1080_h
#include <Arduino.h>

/**
 * @brief The i2c address of the hdc1080 sensor.
 */
#define HDC1080_I2C_ADDRESS 0x40

typedef enum {
  HDC1080_RESOLUTION_8BIT,
  HDC1080_RESOLUTION_11BIT,
  HDC1080_RESOLUTION_14BIT,
} HDC1080_MeasurementResolution;

typedef enum {
  HDC1080_TEMPERATURE = 0x00,
  HDC1080_HUMIDITY = 0x01,
  HDC1080_CONFIGURATION = 0x02,
  HDC1080_MANUFACTURER_ID = 0xFE,
  HDC1080_DEVICE_ID = 0xFF,
  HDC1080_SERIAL_ID_FIRST = 0xFB,
  HDC1080_SERIAL_ID_MID = 0xFC,
  HDC1080_SERIAL_ID_LAST = 0xFD,
} HDC1080_Pointers;

typedef union {
  uint8_t rawData[6];
  struct {
    uint16_t serialFirst;
    uint16_t serialMid;
    uint16_t serialLast;
  };
} HDC1080_SerialNumber;

typedef union {
  uint8_t rawData;
  struct {
    uint8_t HumidityMeasurementResolution : 2;
    uint8_t TemperatureMeasurementResolution : 1;
    uint8_t BatteryStatus : 1;
    uint8_t ModeOfAcquisition : 1;
    uint8_t Heater : 1;
    uint8_t ReservedAgain : 1;
    uint8_t SoftwareReset : 1;
  };
} HDC1080_Registers;

class HDC1080
{
public:
  HDC1080();

  void begin();
  uint16_t readManufacturerId(); // 0x5449 ID of Texas Instruments
  uint16_t readDeviceId();       // 0x1050 ID of the device

  HDC1080_Registers readRegister();
  void writeRegister(HDC1080_Registers reg);

  HDC1080_SerialNumber readSerialNumber();

  void heatUp(uint8_t seconds);

  void setResolution(HDC1080_MeasurementResolution humidity,
                     HDC1080_MeasurementResolution temperature);

  double readTemperature();
  double readHumidity();

  double readT(); // short-cut for readTemperature
  double readH(); // short-cut for readHumidity

private:
  uint8_t _address;
  uint16_t readData(uint8_t pointer);
};

#endif
