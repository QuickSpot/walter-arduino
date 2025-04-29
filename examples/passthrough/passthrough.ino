/**
 * @file passthrough.ino
 * @author Daan Pape <daan@dptechnics.com>
 * @date 21 Mar 2023
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
 * This file contains a sketch which enables to talk to the Sequans Monarch 2 
 * modem via the USB-C uart on Walter.
 */

/**
 * @brief The RX pin on which modem data is received.
 */
#define WALTER_MODEM_PIN_RX 14

/**
 * @brief The TX to which modem data must be transmitted.
 */
#define WALTER_MODEM_PIN_TX 48

/**
 * @brief The RTS pin on the ESP32 side.
 */
#define WALTER_MODEM_PIN_RTS 21

/**
 * @brief The CTS pin on the ESP32 size.
 */
#define WALTER_MODEM_PIN_CTS 47

/**
 * @brief The active low modem reset pin.
 */
#define WALTER_MODEM_PIN_RESET 45

/**
 * @brief The serial interface to talk to the modem.
 */
#define ModemSerial Serial2

/**
 * @brief Reset the modem.
 * 
 * This function will perform a hardware reset of the modem.
 * 
 * @return None.
 */
static void _modem_reset()
{
  gpio_hold_dis((gpio_num_t) WALTER_MODEM_PIN_RESET);
  digitalWrite(WALTER_MODEM_PIN_RESET, LOW);
  delay(1000);
  digitalWrite(WALTER_MODEM_PIN_RESET, HIGH);
  gpio_hold_en((gpio_num_t) WALTER_MODEM_PIN_RESET);
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
  
  pinMode(WALTER_MODEM_PIN_RX, INPUT);
  pinMode(WALTER_MODEM_PIN_TX, OUTPUT);
  pinMode(WALTER_MODEM_PIN_CTS, INPUT);
  pinMode(WALTER_MODEM_PIN_RTS, OUTPUT);
  pinMode(WALTER_MODEM_PIN_RESET, OUTPUT);

  ModemSerial.begin(
    115200,
    SERIAL_8N1,
    WALTER_MODEM_PIN_RX,
    WALTER_MODEM_PIN_TX,
    false,
    WALTER_MODEM_PIN_RTS,
    WALTER_MODEM_PIN_CTS);

  _modem_reset();
}

void loop() {
  static bool rawMode = false;

  if(Serial.available()) {
    int x = Serial.read();
    if(x == '|') {
      rawMode = !rawMode;
    } else {
      if(rawMode) {
        Serial.printf("OUT to modem: %02x  %c\r\n", x, x);
      }
      
      ModemSerial.write(x);
    }
  }

  if(ModemSerial.available()) {
    int x = ModemSerial.read();
    if(rawMode) {
      Serial.printf("IN from modem: %02x  %c\r\n", x, x);
    } else {
      Serial.write(x);
    }
  }
}
