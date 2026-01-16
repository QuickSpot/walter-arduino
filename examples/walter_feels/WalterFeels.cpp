/**
 * @file WalterFeels.cpp
 * @author Daan Pape <daan@dptechnics.com> Arnoud Devoogdt <arnoud@dptechnics.com>
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

#include <Arduino.h>
#include <Wire.h>
#include <driver/ledc.h>

#include "WalterFeels.h"

#define PWM_CHANNEL LEDC_CHANNEL_0
#define PWM_TIMER LEDC_TIMER_0
#define PWM_RESOLUTION LEDC_TIMER_5_BIT
#define PWM_FREQUENCY 1000000

void WalterFeels::_init()
{
  if(_initialized) {
    return;
  }

  /* Disable output holds, to make sure the pins are configurable */
  gpio_hold_dis((gpio_num_t) WFEELS_PIN_PWR_3V3_EN);
  gpio_hold_dis((gpio_num_t) WFEELS_PIN_PWR_12V_EN);
  gpio_hold_dis((gpio_num_t) WFEELS_PIN_I2C_BUS_PWR_EN);
  gpio_hold_dis((gpio_num_t) WFEELS_PIN_CAN_EN);
  gpio_hold_dis((gpio_num_t) WFEELS_PIN_SDI12_TX_EN);
  gpio_hold_dis((gpio_num_t) WFEELS_PIN_SDI12_RX_EN);
  gpio_hold_dis((gpio_num_t) WFEELS_PIN_RS232_TX_EN);
  gpio_hold_dis((gpio_num_t) WFEELS_PIN_RS232_RX_EN);
  gpio_hold_dis((gpio_num_t) WFEELS_PIN_RS485_TX_EN);
  gpio_hold_dis((gpio_num_t) WFEELS_PIN_RS485_RX_EN);
  gpio_hold_dis((gpio_num_t) WFEELS_PIN_CO2_EN);
  gpio_hold_dis((gpio_num_t) WFEELS_PIN_I2C_SDA);
  gpio_hold_dis((gpio_num_t) WFEELS_PIN_I2C_SCL);
  gpio_hold_dis((gpio_num_t) WFEELS_PIN_SD_CMD);
  gpio_hold_dis((gpio_num_t) WFEELS_PIN_SD_CLK);
  gpio_hold_dis((gpio_num_t) WFEELS_PIN_SD_DAT0);
  gpio_hold_dis((gpio_num_t) WFEELS_PIN_GPIO_A);
  gpio_hold_dis((gpio_num_t) WFEELS_PIN_GPIO_B);
  gpio_hold_dis((gpio_num_t) WFEELS_PIN_SER_RX);
  gpio_hold_dis((gpio_num_t) WFEELS_PIN_SER_TX);
  gpio_hold_dis((gpio_num_t) WFEELS_PIN_CAN_RX);
  gpio_hold_dis((gpio_num_t) WFEELS_PIN_CAN_TX);
  gpio_hold_dis((gpio_num_t) WFEELS_PIN_CO2_SDA);
  gpio_hold_dis((gpio_num_t) WFEELS_PIN_CO2_SCL);

  /* Configure IO pins */
  // gpio_set_direction((gpio_num_t)WFEELS_PIN_PWR_3V3_EN, GPIO_MODE_OUTPUT);
  gpio_set_direction((gpio_num_t) WFEELS_PIN_PWR_12V_EN, GPIO_MODE_OUTPUT);
  gpio_set_direction((gpio_num_t) WFEELS_PIN_I2C_BUS_PWR_EN, GPIO_MODE_OUTPUT);
  gpio_set_direction((gpio_num_t) WFEELS_PIN_CAN_EN, GPIO_MODE_OUTPUT);
  gpio_set_direction((gpio_num_t) WFEELS_PIN_SDI12_TX_EN, GPIO_MODE_OUTPUT);
  gpio_set_direction((gpio_num_t) WFEELS_PIN_SDI12_RX_EN, GPIO_MODE_OUTPUT);
  gpio_set_direction((gpio_num_t) WFEELS_PIN_RS232_TX_EN, GPIO_MODE_OUTPUT);
  gpio_set_direction((gpio_num_t) WFEELS_PIN_RS232_RX_EN, GPIO_MODE_OUTPUT);
  gpio_set_direction((gpio_num_t) WFEELS_PIN_RS485_TX_EN, GPIO_MODE_OUTPUT);
  gpio_set_direction((gpio_num_t) WFEELS_PIN_RS485_RX_EN, GPIO_MODE_OUTPUT);
  gpio_set_direction((gpio_num_t) WFEELS_PIN_CO2_EN, GPIO_MODE_OUTPUT);
  gpio_set_direction((gpio_num_t) WFEELS_PIN_I2C_SDA, GPIO_MODE_INPUT);
  gpio_set_direction((gpio_num_t) WFEELS_PIN_I2C_SCL, GPIO_MODE_INPUT);
  gpio_set_direction((gpio_num_t) WFEELS_PIN_SD_CMD, GPIO_MODE_INPUT);
  gpio_set_direction((gpio_num_t) WFEELS_PIN_SD_CLK, GPIO_MODE_INPUT);
  gpio_set_direction((gpio_num_t) WFEELS_PIN_SD_DAT0, GPIO_MODE_INPUT);
  gpio_set_direction((gpio_num_t) WFEELS_PIN_GPIO_A, GPIO_MODE_INPUT);
  gpio_set_direction((gpio_num_t) WFEELS_PIN_GPIO_B, GPIO_MODE_INPUT);
  gpio_set_direction((gpio_num_t) WFEELS_PIN_SER_RX, GPIO_MODE_INPUT);
  gpio_set_direction((gpio_num_t) WFEELS_PIN_SER_TX, GPIO_MODE_INPUT);
  gpio_set_direction((gpio_num_t) WFEELS_PIN_CAN_RX, GPIO_MODE_INPUT);
  gpio_set_direction((gpio_num_t) WFEELS_PIN_CAN_TX, GPIO_MODE_INPUT);
  gpio_set_direction((gpio_num_t) WFEELS_PIN_CO2_SDA, GPIO_MODE_INPUT);
  gpio_set_direction((gpio_num_t) WFEELS_PIN_CO2_SCL, GPIO_MODE_OUTPUT);

  gpio_set_pull_mode((gpio_num_t) WFEELS_PIN_PWR_3V3_EN, GPIO_FLOATING);
  gpio_set_pull_mode((gpio_num_t) WFEELS_PIN_PWR_12V_EN, GPIO_FLOATING);
  gpio_set_pull_mode((gpio_num_t) WFEELS_PIN_I2C_BUS_PWR_EN, GPIO_FLOATING);
  gpio_set_pull_mode((gpio_num_t) WFEELS_PIN_CAN_EN, GPIO_PULLUP_ONLY);
  gpio_set_pull_mode((gpio_num_t) WFEELS_PIN_SDI12_TX_EN, GPIO_FLOATING);
  gpio_set_pull_mode((gpio_num_t) WFEELS_PIN_SDI12_RX_EN, GPIO_FLOATING);
  gpio_set_pull_mode((gpio_num_t) WFEELS_PIN_RS232_TX_EN, GPIO_FLOATING);
  gpio_set_pull_mode((gpio_num_t) WFEELS_PIN_RS232_RX_EN, GPIO_FLOATING);
  gpio_set_pull_mode((gpio_num_t) WFEELS_PIN_RS485_TX_EN, GPIO_FLOATING);
  gpio_set_pull_mode((gpio_num_t) WFEELS_PIN_RS485_RX_EN, GPIO_FLOATING);
  gpio_set_pull_mode((gpio_num_t) WFEELS_PIN_CO2_EN, GPIO_FLOATING);
  gpio_set_pull_mode((gpio_num_t) WFEELS_PIN_I2C_SDA, GPIO_FLOATING);
  gpio_set_pull_mode((gpio_num_t) WFEELS_PIN_I2C_SCL, GPIO_FLOATING);
  gpio_set_pull_mode((gpio_num_t) WFEELS_PIN_SD_CMD, GPIO_FLOATING);
  gpio_set_pull_mode((gpio_num_t) WFEELS_PIN_SD_CLK, GPIO_FLOATING);
  gpio_set_pull_mode((gpio_num_t) WFEELS_PIN_SD_DAT0, GPIO_FLOATING);
  gpio_set_pull_mode((gpio_num_t) WFEELS_PIN_GPIO_A, GPIO_FLOATING);
  gpio_set_pull_mode((gpio_num_t) WFEELS_PIN_GPIO_B, GPIO_FLOATING);
  gpio_set_pull_mode((gpio_num_t) WFEELS_PIN_SER_RX, GPIO_FLOATING);
  gpio_set_pull_mode((gpio_num_t) WFEELS_PIN_SER_TX, GPIO_FLOATING);
  gpio_set_pull_mode((gpio_num_t) WFEELS_PIN_CAN_RX, GPIO_FLOATING);
  gpio_set_pull_mode((gpio_num_t) WFEELS_PIN_CAN_TX, GPIO_FLOATING);
  gpio_set_pull_mode((gpio_num_t) WFEELS_PIN_CO2_SDA, GPIO_FLOATING);
  gpio_set_pull_mode((gpio_num_t) WFEELS_PIN_CO2_SCL, GPIO_FLOATING);

  /* Disable all peripherals */
  gpio_set_level((gpio_num_t) WFEELS_PIN_PWR_3V3_EN, 1);
  gpio_set_level((gpio_num_t) WFEELS_PIN_PWR_12V_EN, 0);
  gpio_set_level((gpio_num_t) WFEELS_PIN_I2C_BUS_PWR_EN, 0);
  gpio_set_level((gpio_num_t) WFEELS_PIN_CAN_EN, 1);
  gpio_set_level((gpio_num_t) WFEELS_PIN_SDI12_TX_EN, 0);
  gpio_set_level((gpio_num_t) WFEELS_PIN_SDI12_RX_EN, 0);
  gpio_set_level((gpio_num_t) WFEELS_PIN_RS232_TX_EN, 0);
  gpio_set_level((gpio_num_t) WFEELS_PIN_RS232_RX_EN, 1);
  gpio_set_level((gpio_num_t) WFEELS_PIN_RS485_TX_EN, 0);
  gpio_set_level((gpio_num_t) WFEELS_PIN_RS485_RX_EN, 1);
  gpio_set_level((gpio_num_t) WFEELS_PIN_CO2_EN, 1);

  ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = PWM_RESOLUTION,
    .timer_num = PWM_TIMER,
    .freq_hz = PWM_FREQUENCY,
    .clk_cfg = LEDC_AUTO_CLK,
    .deconfigure = false,
  };

  ledc_timer_config(&ledc_timer);

  ledc_channel_config_t ledc_channel = { .gpio_num = (gpio_num_t) WFEELS_PIN_PWR_3V3_EN,
                                         .speed_mode = LEDC_LOW_SPEED_MODE,
                                         .channel = PWM_CHANNEL,
                                         .intr_type = LEDC_INTR_DISABLE,
                                         .timer_sel = PWM_TIMER,
                                         .duty = 0,
                                         .hpoint = 0,
                                         // .sleep_mode = LEDC_SLEEP_MODE_KEEP_ALIVE,
                                         .flags = { .output_invert = 1 } };
  ledc_channel_config(&ledc_channel);
  ledc_fade_func_install(0);

  ESP_LOGI("Feels", "Walter Feels initialized");
  vTaskDelay(pdMS_TO_TICKS(5000));
  _initialized = true;
}

bool WalterFeels::_softStart(uint32_t start_duty, uint32_t target_duty, int fade_time_ms)
{
  ledc_mode_t speed_mode = LEDC_LOW_SPEED_MODE;

  uint32_t max_duty = (1 << PWM_RESOLUTION) - 1;

  if((target_duty == max_duty) && (max_duty != 1)) {
    target_duty = max_duty + 1;
  } else if((start_duty == max_duty) && (max_duty != 1)) {
    start_duty = max_duty + 1;
  }

  if(ledc_set_duty_and_update(speed_mode, PWM_CHANNEL, start_duty, 0) != ESP_OK) {
    return false;
  }

  while(ledc_get_duty(speed_mode, PWM_CHANNEL) != start_duty)
    ;

  if(ledc_set_fade_time_and_start(speed_mode, PWM_CHANNEL, target_duty, fade_time_ms,
                                  LEDC_FADE_NO_WAIT) != ESP_OK) {
    return false;
  }

  return true;
}

void WalterFeels::set3v3(bool on)
{
  _init();

  if(on) {
    _softStart(0, 31, 100);

  } else {
    ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE, PWM_CHANNEL, 0, 0);
    // gpio_set_level((gpio_num_t)WFEELS_PIN_PWR_3V3_EN, 0);
  }
}

void WalterFeels::set12v(bool on)
{
  _init();
  gpio_set_level((gpio_num_t) WFEELS_PIN_PWR_12V_EN, on ? 1 : 0);
}

void WalterFeels::setCo2(bool on)
{
  _init();
  gpio_set_level((gpio_num_t) WFEELS_PIN_CO2_EN, on ? 0 : 1);
}

void WalterFeels::setCan(bool on)
{
  _init();
  gpio_set_level((gpio_num_t) WFEELS_PIN_CAN_EN, on ? 0 : 1);
}

void WalterFeels::setI2cBusPower(bool on)
{
  _init();
  gpio_set_level((gpio_num_t) WFEELS_PIN_I2C_BUS_PWR_EN, on ? 1 : 0);
}

void WalterFeels::setSerialMode(WalterFeelsSerialMode mode)
{
  _init();

  switch(mode) {
  case WFEELS_SERIAL_OFF:
    gpio_set_level((gpio_num_t) WFEELS_PIN_SDI12_TX_EN, 0);
    gpio_set_level((gpio_num_t) WFEELS_PIN_SDI12_RX_EN, 0);
    gpio_set_level((gpio_num_t) WFEELS_PIN_RS232_TX_EN, 0);
    gpio_set_level((gpio_num_t) WFEELS_PIN_RS232_RX_EN, 1);
    gpio_set_level((gpio_num_t) WFEELS_PIN_RS485_TX_EN, 0);
    gpio_set_level((gpio_num_t) WFEELS_PIN_RS485_RX_EN, 1);
    break;

  case WFEELS_SERIAL_RS232:
    set3v3(true);
    gpio_set_level((gpio_num_t) WFEELS_PIN_SDI12_TX_EN, 0);
    gpio_set_level((gpio_num_t) WFEELS_PIN_SDI12_RX_EN, 0);
    gpio_set_level((gpio_num_t) WFEELS_PIN_RS485_TX_EN, 0);
    gpio_set_level((gpio_num_t) WFEELS_PIN_RS485_RX_EN, 1);

    gpio_set_level((gpio_num_t) WFEELS_PIN_RS232_TX_EN, 1);
    gpio_set_level((gpio_num_t) WFEELS_PIN_RS232_RX_EN, 0);
    break;

  case WFEELS_SERIAL_RS485_TX:
    set3v3(true);
    gpio_set_level((gpio_num_t) WFEELS_PIN_SDI12_TX_EN, 0);
    gpio_set_level((gpio_num_t) WFEELS_PIN_SDI12_RX_EN, 0);
    gpio_set_level((gpio_num_t) WFEELS_PIN_RS232_TX_EN, 0);
    gpio_set_level((gpio_num_t) WFEELS_PIN_RS232_RX_EN, 1);
    gpio_set_level((gpio_num_t) WFEELS_PIN_RS485_RX_EN, 1);

    gpio_set_level((gpio_num_t) WFEELS_PIN_RS485_TX_EN, 1);
    break;

  case WFEELS_SERIAL_RS485_RX:
    set3v3(true);
    gpio_set_level((gpio_num_t) WFEELS_PIN_SDI12_TX_EN, 0);
    gpio_set_level((gpio_num_t) WFEELS_PIN_SDI12_RX_EN, 0);
    gpio_set_level((gpio_num_t) WFEELS_PIN_RS232_TX_EN, 0);
    gpio_set_level((gpio_num_t) WFEELS_PIN_RS232_RX_EN, 1);
    gpio_set_level((gpio_num_t) WFEELS_PIN_RS485_TX_EN, 0);

    gpio_set_level((gpio_num_t) WFEELS_PIN_RS485_RX_EN, 0);
    break;

  case WFEELS_SERIAL_SDI12_TX:
    set3v3(true);
    gpio_set_level((gpio_num_t) WFEELS_PIN_RS232_TX_EN, 0);
    gpio_set_level((gpio_num_t) WFEELS_PIN_RS232_RX_EN, 1);
    gpio_set_level((gpio_num_t) WFEELS_PIN_RS485_TX_EN, 0);
    gpio_set_level((gpio_num_t) WFEELS_PIN_RS485_RX_EN, 1);
    gpio_set_level((gpio_num_t) WFEELS_PIN_SDI12_RX_EN, 0);

    gpio_set_level((gpio_num_t) WFEELS_PIN_SDI12_TX_EN, 1);
    break;

  case WFEELS_SERIAL_SDI12_RX:
    set3v3(true);
    gpio_set_level((gpio_num_t) WFEELS_PIN_RS232_TX_EN, 0);
    gpio_set_level((gpio_num_t) WFEELS_PIN_RS232_RX_EN, 1);
    gpio_set_level((gpio_num_t) WFEELS_PIN_RS485_TX_EN, 0);
    gpio_set_level((gpio_num_t) WFEELS_PIN_RS485_RX_EN, 1);
    gpio_set_level((gpio_num_t) WFEELS_PIN_SDI12_TX_EN, 0);

    gpio_set_level((gpio_num_t) WFEELS_PIN_SDI12_RX_EN, 1);
    break;
  }
}

void WalterFeels::prepareDeepSleep(bool pow3v3, bool pow12v, bool powCo2)
{
  _init();

  set3v3(pow3v3);
  set12v(pow12v);
  setCo2(powCo2);
  setCan(false);
  setI2cBusPower(false);
  setSerialMode(WFEELS_SERIAL_OFF);
  setCan(false);

  gpio_hold_en((gpio_num_t) WFEELS_PIN_PWR_3V3_EN);
  gpio_hold_en((gpio_num_t) WFEELS_PIN_PWR_12V_EN);
  gpio_hold_en((gpio_num_t) WFEELS_PIN_I2C_BUS_PWR_EN);
  gpio_hold_en((gpio_num_t) WFEELS_PIN_CAN_EN);
  gpio_hold_en((gpio_num_t) WFEELS_PIN_SDI12_TX_EN);
  gpio_hold_en((gpio_num_t) WFEELS_PIN_SDI12_RX_EN);
  gpio_hold_en((gpio_num_t) WFEELS_PIN_RS232_TX_EN);
  gpio_hold_en((gpio_num_t) WFEELS_PIN_RS232_RX_EN);
  gpio_hold_en((gpio_num_t) WFEELS_PIN_RS485_TX_EN);
  gpio_hold_en((gpio_num_t) WFEELS_PIN_RS485_RX_EN);
  gpio_hold_en((gpio_num_t) WFEELS_PIN_CO2_EN);
  gpio_deep_sleep_hold_en();
}