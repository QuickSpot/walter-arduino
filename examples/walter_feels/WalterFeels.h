/**
 * @file WalterFeels.h
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

#ifndef WALTER_FEELS_H
#define WALTER_FEELS_H

#include <Arduino.h>
#include <FreeRTOS.h>

/**
 * @defgroup WFEELS_PIN The Walter Feels board PIN definitions.
 * @{
 */
#define WFEELS_PIN_PWR_3V3_EN 0
#define WFEELS_PIN_PWR_12V_EN 43
#define WFEELS_PIN_I2C_BUS_PWR_EN 1
#define WFEELS_PIN_CAN_EN 44
#define WFEELS_PIN_SDI12_TX_EN 10
#define WFEELS_PIN_SDI12_RX_EN 9
#define WFEELS_PIN_RS232_TX_EN 17
#define WFEELS_PIN_RS232_RX_EN 16
#define WFEELS_PIN_RS485_TX_EN 18
#define WFEELS_PIN_RS485_RX_EN 8
#define WFEELS_PIN_CO2_EN 13
#define WFEELS_PIN_I2C_SDA 42
#define WFEELS_PIN_I2C_SCL 2
#define WFEELS_PIN_SD_CMD 6
#define WFEELS_PIN_SD_CLK 5
#define WFEELS_PIN_SD_DAT0 4
#define WFEELS_PIN_GPIO_A 39
#define WFEELS_PIN_GPIO_B 38
#define WFEELS_PIN_SER_RX 41
#define WFEELS_PIN_SER_TX 40
#define WFEELS_PIN_CAN_RX 7
#define WFEELS_PIN_CAN_TX 15
#define WFEELS_PIN_CO2_SDA 12
#define WFEELS_PIN_CO2_SCL 11
/**
 * @}
 */

/**
 * @brief This enumeration represents the modes in which the serial port on
 * Walter Feels can be configured. A single UART peripheral is connected to all
 * transceivers on the board. By means of multiplexing the UART can be
 * dynamically switched between the different physical serial protocols.
 */
typedef enum {
  WFEELS_SERIAL_OFF,
  WFEELS_SERIAL_RS232,
  WFEELS_SERIAL_RS485_TX,
  WFEELS_SERIAL_RS485_RX,
  WFEELS_SERIAL_SDI12_TX,
  WFEELS_SERIAL_SDI12_RX,
} WalterFeelsSerialMode;

/**
 * @brief The WalterFeels class provides easy access to the functionality of the
 * Walter Feels carrier board.
 */
class WalterFeels
{
private:
  /**
   * @brief This flag is set to true when the NMEA subsystem is initialized.
   */
  static inline bool _initialized = false;

  /**
   * @brief Initialize the board's pins, it is lazy called if required.
   *
   * This function will configure the pins of Walter as they are routed on the
   * Walter Feels carrier board.
   */
  static void _init();

  static bool _softStart(uint32_t start_duty, uint32_t target_duty, int max_fade_time_ms);

public:
  /**
   * @brief Set the state of the 3.3V on the board.
   *
   * This function will switch on or off the 3.3V coming from Walter. The 3.3V
   * must be on to use the various peripherals and/or sensors on the Walter
   * Feels board.
   *
   * @param on True to switch the 3.3V on, false for off.
   *
   * @return None.
   */
  static void set3v3(bool on = true);

  /**
   * @brief Set the state of the 12V output.
   *
   * This function will switch on or off the 12V output.
   *
   * @param on True to switch the 12V on, false for off.
   *
   * @return None.
   */
  static void set12v(bool on = true);

  /**
   * @brief Set the state of the CO2 sensor.
   *
   * This function will switch on or off the power to the CO2 sensor.
   *
   * @param on True to switch the CO2 sensor's power on, false for off.
   *
   * @return None.
   */
  static void setCo2(bool on = true);

  /**
   * @brief Set the state of the CAN transceiver.
   *
   * This function will switch on or off the CAN transceiver.
   *
   * @param on True to enable the CAN transceiver, false for disable.
   *
   * @return None.
   */
  static void setCan(bool on = true);

  /**
   * @brief Set the state of the I2C bus power.
   *
   * The I2C bus on Walter has 45.3k pull-ups which are powered by an I/O pin
   * of Walter. If I2C is not required you can save 140uA of leakage current
   * by switching off the bus power.
   *
   * @param on True to enable I2C pull-up power, false to disable.
   *
   * @return None.
   */
  static void setI2cBusPower(bool on = true);

  /**
   * @brief Set the Walter Feels board in a certain serial mode.
   *
   * This function will configure the multiplexer system of the Walter Feels
   * module into a certain mode. The default argument will switch off all
   * serial communication to save most power.
   *
   * @param mode The mode to put the serial port in.
   *
   * @return None.
   */
  static void setSerialMode(WalterFeelsSerialMode mode = WFEELS_SERIAL_OFF);

  /**
   * @brief Prepare the WalterFeels module to go into deep sleep.
   *
   * This function will prepare the Walter Feels module to go into deep sleep
   * by powering off all the external peripherals and configuring the I/O pins
   * to keep off during deep sleep.
   *
   * @param pow3v3 When true the 3.3V will remain on during deep sleep.
   * @param pow12v When true the 12V will remain on during deep sleep.
   * @param powCo2 When true the CO2 sensor will remain on during deep sle
   *
   * @return None.
   */
  static void prepareDeepSleep(bool pow3v3 = false, bool pow12v = false, bool powCo2 = false);
};

#endif