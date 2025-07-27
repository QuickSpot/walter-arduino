/**
 * @file SensorShield.ino
 * @author Daan Pape <daan@dptechnics.com>
 * @date 1 Mar 2023
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
 * This file contains a sketch which reads out environimental sensors and sends
 * a reading of these sensors to the Walter demo server every 60 seconds.
 */

#include <Adafruit_MPL3115A2.h>
#include <HardwareSerial.h>
#include <LTR329ALS01.h>
#include <WalterModem.h>
#include <esp_mac.h>


/**
 * @brief The address of the server to upload the data to.
 */
#define SERV_ADDR "64.225.64.140"

/**
 * @brief The UDP port on which the server is listening.
 */
#define SERV_PORT 1999

/**
 * @brief The I2C address of the Si7006-A20
 */
#define SI7006_ADDR 0x40

/**
 * @brief The SDA pin of the I2C bus.
 */
#define I2C_SDA_PIN 9

/**
 * @brief The SCL pin of the I2C bus.
 */
#define I2C_SCL_PIN 8

/**
 * @brief The modem instance.
 */
WalterModem modem;

/**
 * @brief Response object containing command response information.
 */
WalterModemRsp rsp;

/**
 * @brief Lux sensor instance.
 */
LTR329ALS01 ltr329(9, 8);

/**
 * @brief Barometric pressure sensor instance.
 */
Adafruit_MPL3115A2 mpl3115a2 = Adafruit_MPL3115A2();

/**
 * @brief The buffer to transmit to the UDP server. The first 6 bytes will be
 * the MAC address of the Walter this code is running on.
 */
uint8_t dataBuf[14] = {0};

/**
 * @brief This function checks if we are connected to the lte network
 *
 * @return True when connected, False otherwise
 */
bool lteConnected() {
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  return (regState == WALTER_MODEM_NETWORK_REG_REGISTERED_HOME ||
          regState == WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING);
}

/**
 * @brief This function waits for the modem to be connected to the Lte network.
 * @return true if the connected, else false on timeout.
 */
bool waitForNetwork() {
  /* Wait for the network to become available */
  int timeout = 0;
  while (!lteConnected()) {
    delay(1000);
    timeout++;
    if (timeout > 300)
      return false;
  }
  Serial.println("Connected to the network");
  return true;
}

/**
 * @brief This function tries to connect the modem to the cellular network.
 * @return true if the connection attempt is successful, else false.
 */
bool lteConnect() {
  if (modem.setOpState(WALTER_MODEM_OPSTATE_NO_RF)) {
    Serial.println("Successfully set operational state to NO RF");
  } else {
    Serial.println("Error: Could not set operational state to NO RF");
    return false;
  }

  /* Create PDP context */
  if (modem.definePDPContext()) {
    Serial.println("Created PDP context");
  } else {
    Serial.println("Error: Could not create PDP context");
    return false;
  }

  /* Set the operational state to full */
  if (modem.setOpState(WALTER_MODEM_OPSTATE_FULL)) {
    Serial.println("Successfully set operational state to FULL");
  } else {
    Serial.println("Error: Could not set operational state to FULL");
    return false;
  }

  /* Set the network operator selection to automatic */
  if (modem.setNetworkSelectionMode(WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC)) {
    Serial.println("Network selection mode to was set to automatic");
  } else {
    Serial.println(
        "Error: Could not set the network selection mode to automatic");
    return false;
  }

  return waitForNetwork();
}

/**
 * @brief Read out the relative humidity.
 *
 * This function reads the relative humidity from the Si7006 sensor.
 *
 * @return The resulting relative humidity.
 */
float getRelHum() {
  uint8_t data[2] = {0};

  Wire.beginTransmission(SI7006_ADDR);
  Wire.write(0xF5);
  Wire.endTransmission();
  delay(500);

  Wire.requestFrom(SI7006_ADDR, 2);
  if (Wire.available() == 2) {
    data[0] = Wire.read();
    data[1] = Wire.read();
  }

  float humidity = ((data[0] * 256.0) + data[1]);
  humidity = ((125 * humidity) / 65536.0) - 6;
  return humidity;
}

/**
 * @brief Read out the temperature.
 *
 * This function reads the temperature from the Si7006 sensor.
 *
 * @return The resulting temperature.
 */
static float getTemp() {
  uint8_t data[2] = {0};

  Wire.beginTransmission(SI7006_ADDR);
  Wire.write(0xF3);
  Wire.endTransmission();
  delay(500);

  Wire.requestFrom(SI7006_ADDR, 2);
  if (Wire.available() == 2) {
    data[0] = Wire.read();
    data[1] = Wire.read();
  }

  float temp = ((data[0] * 256.0) + data[1]);
  float ctemp = ((175.72 * temp) / 65536.0) - 46.85;
  return ctemp;
}

/**
 * @brief Set-up the system.
 *
 * This function will setup the system and initialize the communication buses.
 *
 * @return None.
 */
void setup() {
  Serial.begin(115200);
  delay(5000);

  Serial.println("Walter SensorShield v1.0.0");

  /* Get the MAC address for board validation */
  esp_read_mac(dataBuf, ESP_MAC_WIFI_STA);
  Serial.printf("Walter's MAC is: %02X:%02X:%02X:%02X:%02X:%02X\n", dataBuf[0],
                dataBuf[1], dataBuf[2], dataBuf[3], dataBuf[4], dataBuf[5]);

  /* I2C bus initialisation */
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  /* Lux sensor initialization */
  ltr329.begin();
  ltr329.writetControl(true, LTR329ALS01_ALS_GAIN_x8);
  ltr329.writeMeasRate(LTR329ALS01_ALS_INT_100ms, LTR329ALS01_ALS_RATE_500ms);

  /* Baromether initialization */
  if (!mpl3115a2.begin()) {
    while (1)
      ;
  }
  mpl3115a2.setSeaPressure(1013.26);

  /* Modem initialization */
  if (modem.begin(&Serial2)) {
    Serial.println("Modem initialization OK");
  } else {
    Serial.println("Error: Modem initialization ERROR");
    return;
  }

  /* Connect the modem to the lte network */
  if (!lteConnect()) {
    Serial.println("Error: Could Not Connect to LTE");
    return;
  }

  /* Construct a socket */
  if (!modem.createSocket(&rsp)) {
    Serial.println("Error: Could not create a new socket");
    return;
  }

  /* Connect to the UDP test server */
  if (modem.socketDial(SERV_ADDR, SERV_PORT)) {
    Serial.printf("Connected to UDP server %s:%d\n", ip, port);
  } else {
    Serial.println("Error: Could not connect UDP socket");
    return;
  }
}

void loop() {
  float hum = getRelHum();
  float temp = getTemp();
  float lux = ltr329.readLux();
  float pressure = mpl3115a2.getPressure();

  uint16_t rawHum = hum * 100;
  uint16_t rawTemp = (temp + 50) * 100;
  uint16_t rawLux = lux * 10;
  uint16_t rawPressure = pressure * 10;

  uint16_t dataSize = 14;
  dataBuf[6] = rawHum >> 8;
  dataBuf[7] = rawHum & 0xFF;
  dataBuf[8] = rawTemp >> 8;
  dataBuf[9] = rawTemp & 0xFF;
  dataBuf[10] = rawLux >> 8;
  dataBuf[11] = rawLux & 0xFF;
  dataBuf[12] = rawPressure >> 8;
  dataBuf[13] = rawPressure & 0xFF;

  if (!modem.socketSend(dataBuf, dataSize)) {
    Serial.println("Error: Could not transmit data");
    ESP.restart();
  }

  Serial.printf("Sent data %.01f %%, %.01f C, %.01f lux, %.01f hPa\n", hum,
                temp, lux, pressure);

  delay(60000);
  ESP.restart();
}