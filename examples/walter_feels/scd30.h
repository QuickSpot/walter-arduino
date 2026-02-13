/**
 * @file scd30.h
 * @author Daan Pape <daan@dptechnics.com>
 * @author Arnoud Devoogdt <arnoud@dptechnics.com>
 * @date 16 January 2026
 * @version 1.5.0
 * @copyright DPTechnics bv <info@dptechnics.com>
 * @brief Walter Modem library examples
 *
 * @section LICENSE
 *
 * Copyright (C) 2026, DPTechnics bv
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

#ifndef _SCD30_H
#define _SCD30_H

#include "Arduino.h"
#include <Wire.h>

// The default I2C address for the SCD30 is 0x61.
#define SCD30_ADDRESS 0x61

// Available commands

#define COMMAND_CONTINUOUS_MEASUREMENT 0x0010
#define COMMAND_SET_MEASUREMENT_INTERVAL 0x4600
#define COMMAND_GET_DATA_READY 0x0202
#define COMMAND_READ_MEASUREMENT 0x0300
#define COMMAND_AUTOMATIC_SELF_CALIBRATION 0x5306
#define COMMAND_SET_FORCED_RECALIBRATION_FACTOR 0x5204
#define COMMAND_SET_TEMPERATURE_OFFSET 0x5403
#define COMMAND_SET_ALTITUDE_COMPENSATION 0x5102
#define COMMAND_RESET 0xD304 // Soft reset
#define COMMAND_STOP_MEAS 0x0104
#define COMMAND_READ_FW_VER 0xD100

typedef union {
  uint8_t array[4];
  float value;
} ByteToFl; // paulvha

class SCD30
{
public:
  SCD30(void);

  bool begin(bool autoCalibrate) { return begin(Wire, autoCalibrate); }
  bool begin(TwoWire& wirePort = Wire, bool autoCalibrate = false, bool measBegin = true);

  bool isConnected();
  void enableDebugging(Stream& debugPort = Serial); // Turn on debug printing. If user doesn't
                                                    // specify then Serial will be used.

  bool beginMeasuring(uint16_t pressureOffset);
  bool beginMeasuring(void);
  bool StopMeasurement(void); // paulvha

  bool setAmbientPressure(uint16_t pressure_mbar);

  bool getSettingValue(uint16_t registerAddress, uint16_t* val);
  bool getFirmwareVersion(uint16_t* val) { return (getSettingValue(COMMAND_READ_FW_VER, val)); }
  uint16_t getCO2(void);
  float getHumidity(void);
  float getTemperature(void);

  uint16_t getMeasurementInterval(void);
  bool getMeasurementInterval(uint16_t* val)
  {
    return (getSettingValue(COMMAND_SET_MEASUREMENT_INTERVAL, val));
  }
  bool setMeasurementInterval(uint16_t interval);

  uint16_t getAltitudeCompensation(void);
  bool getAltitudeCompensation(uint16_t* val)
  {
    return (getSettingValue(COMMAND_SET_ALTITUDE_COMPENSATION, val));
  }
  bool setAltitudeCompensation(uint16_t altitude);

  bool getAutoSelfCalibration(void);
  bool setAutoSelfCalibration(bool enable);

  bool getForcedRecalibration(uint16_t* val)
  {
    return (getSettingValue(COMMAND_SET_FORCED_RECALIBRATION_FACTOR, val));
  }
  bool setForcedRecalibrationFactor(uint16_t concentration);

  float getTemperatureOffset(void);
  bool getTemperatureOffset(uint16_t* val)
  {
    return (getSettingValue(COMMAND_SET_TEMPERATURE_OFFSET, val));
  }
  bool setTemperatureOffset(float tempOffset);

  bool dataAvailable();
  bool readMeasurement();

  void reset();

  bool sendCommand(uint16_t command, uint16_t arguments);
  bool sendCommand(uint16_t command);

  uint16_t readRegister(uint16_t registerAddress);

  uint8_t computeCRC8(uint8_t data[], uint8_t len);

  void useStaleData(bool enable) { _useStaleData = enable; }

private:
  // Variables
  TwoWire* _i2cPort;
  // Global main datums
  float co2 = 0;
  float temperature = 0;
  float humidity = 0;
  bool _useStaleData = false; // If true, stale data is returned instead of zeros

  // These track the staleness of the current data
  // This allows us to avoid calling readMeasurement() every time individual datums are requested
  bool co2HasBeenReported = true;
  bool humidityHasBeenReported = true;
  bool temperatureHasBeenReported = true;

  // Debug
  Stream* _debugPort;          // The stream to send debug messages to if enabled. Usually Serial.
  boolean _printDebug = false; // Flag to print debugging variables
};
#endif
