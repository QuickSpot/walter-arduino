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

#include <esp_system.h>
#include <LTR329ALS01.h>
#include <WalterModem.h>
#include <HardwareSerial.h>
#include <Adafruit_MPL3115A2.h>

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
uint8_t dataBuf[14] = { 0 };

/**
 * @brief Connect to a network and UDP socket.
 * 
 * This function will set-up the modem and connect an UDP socket.
 * 
 * @param apn The APN to use for the PDP context.
 * @param user The APN username.
 * @param pass The APN password.
 * @param ip The IP address of the server to connect to.
 * @param port The port to connect to.
 * 
 * @return True on success, false on error.
 */
bool connect(const char *apn, const char *user, const char *pass, const char *ip, uint16_t port)
{
  if(!modem.reset()) {
    Serial.println("Could not reset the modem");
    return false;
  }

  if(!modem.checkComm()) {
    Serial.println("Modem communication error");
    return false;
  }

  if(!modem.configCMEErrorReports()) {
    Serial.println("Could not configure CME error reports");
    return false;
  }

  if(!modem.configCEREGReports()) {
    Serial.println("Could not configure CEREG reports");
    return false;
  }

  WalterModemRsp rsp = {};
  if(!modem.getOpState(&rsp)) {
    Serial.println("Could not retrieve modem operational state");
    return false;
  }

  if(modem.getRadioBands(&rsp)) {
    Serial.println("Modem is configured for the following bands:");
    
    for(int i = 0; i < rsp.data.bandSelCfgSet.count; ++i) {
      WalterModemBandSelection *bSel = rsp.data.bandSelCfgSet.config + i;
      Serial.printf("  - Operator '%s' on %s: 0x%05X\n",
        bSel->netOperator.name,
        bSel->rat == WALTER_MODEM_RAT_NBIOT ? "NB-IoT" : "LTE-M",
        bSel->bands);
    }
  } else {
    Serial.println("Could not retrieve configured radio bands");
    return false;
  }

  if(!modem.setOpState(WALTER_MODEM_OPSTATE_NO_RF)) {
    Serial.println("Could not set operational state to NO RF");
    return false;
  }

  /* Give the modem time to detect the SIM */
  delay(2000);

  if(modem.unlockSIM()) {
    Serial.println("Successfully unlocked SIM card");
  } else {
    Serial.println("Could not unlock SIM card");
    return false;
  }

  /* Create PDP context */
  if(user != NULL) {
    if(!modem.createPDPContext(
        apn,
        WALTER_MODEM_PDP_AUTH_PROTO_PAP,
        user,
        pass))
    {
      Serial.println("Could not create PDP context");
      return false;
    }
  } else {
    if(!modem.createPDPContext(apn)) {
      Serial.println("Could not create PDP context");
      return false;
    }
  }

  /* Authenticate the PDP context */
  if(!modem.authenticatePDPContext()) {
    Serial.println("Could not authenticate the PDP context");
    return false;
  }

  /* Set the operational state to full */
  if(!modem.setOpState(WALTER_MODEM_OPSTATE_FULL)) {
    Serial.println("Could not set operational state to FULL");
    return false;
  }

  /* Set the network operator selection to automatic */
  if(!modem.setNetworkSelectionMode(WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC)) {
    Serial.println("Could not set the network selection mode to automatic");
    return false;
  }

  /* Wait for the network to become available */
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  while(!(regState == WALTER_MODEM_NETOWRK_REG_REGISTERED_HOME ||
          regState == WALTER_MODEM_NETOWRK_REG_REGISTERED_ROAMING))
  {
    delay(100);
    regState = modem.getNetworkRegState();
  }
  Serial.println("Connected to the network");

  /* Activate the PDP context */
  if(!modem.setPDPContextActive(true)) {
    Serial.println("Could not activate the PDP context");
    return false;
  }

  /* Attach the PDP context */
  if(!modem.attachPDPContext(true)) {
    Serial.println("Could not attach to the PDP context");
    return false;
  }

  if(modem.getPDPAddress(&rsp)) {
    Serial.println("PDP context address list: ");
    Serial.printf("  - %s\n", rsp.data.pdpAddressList.pdpAddress);
    if(rsp.data.pdpAddressList.pdpAddress2[0] != '\0') {
      Serial.printf("  - %s\n", rsp.data.pdpAddressList.pdpAddress2);
    }
  } else {
    Serial.println("Could not retrieve PDP context addresses");
    return false;
  }

  /* Construct a socket */
  if(!modem.createSocket(&rsp)) {
    Serial.println("Could not create a new socket");
    return false;
  }

  /* Configure the socket */
  if(!modem.configSocket()) {
    Serial.println("Could not configure the socket");
    return false;
  }

  /* Connect to the UDP test server */
  if(modem.connectSocket(ip, port, port)) {
    Serial.printf("Connected to UDP server %s:%d\n", ip, port);
  } else {
    Serial.println("Could not connect UDP socket");
    return false;
  }

  return true;
}

/**
 * @brief Read out the relative humidity.
 * 
 * This function reads the relative humidity from the Si7006 sensor.
 * 
 * @return The resulting relative humidity.
 */
float getRelHum()
{
  uint8_t data[2] = { 0 };

  Wire.beginTransmission(SI7006_ADDR);
  Wire.write(0xF5);
  Wire.endTransmission();
  delay(500);
  
  Wire.requestFrom(SI7006_ADDR, 2);
  if(Wire.available() == 2) {
    data[0] = Wire.read();
    data[1] = Wire.read();
  }
  
  float humidity  = ((data[0] * 256.0) + data[1]);
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
static float getTemp()
{
  uint8_t data[2] = { 0 };

  Wire.beginTransmission(SI7006_ADDR);
  Wire.write(0xF3);
  Wire.endTransmission();
  delay(500);
  
  Wire.requestFrom(SI7006_ADDR, 2);
  if(Wire.available() == 2) {
    data[0] = Wire.read();
    data[1] = Wire.read();
  }

  float temp  = ((data[0] * 256.0) + data[1]);
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
void setup()
{
  Serial.begin(115200);
  delay(5000);

  Serial.println("Walter SensorShield v0.0.1");

  /* Get the MAC address for board validation */
  esp_read_mac(dataBuf, ESP_MAC_WIFI_STA);
  Serial.printf("Walter's MAC is: %02X:%02X:%02X:%02X:%02X:%02X\n",
    dataBuf[0],
    dataBuf[1],
    dataBuf[2],
    dataBuf[3],
    dataBuf[4],
    dataBuf[5]);

  /* I2C bus initialisation */
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  
  /* Lux sensor initialization */
  ltr329.begin();
  ltr329.writetControl(true, LTR329ALS01_ALS_GAIN_x8); 
  ltr329.writeMeasRate(LTR329ALS01_ALS_INT_100ms, LTR329ALS01_ALS_RATE_500ms); 

  /* Baromether initialization */
  if(!mpl3115a2.begin()) {
    while(1);
  }
  mpl3115a2.setSeaPressure(1013.26);

  /* Modem initialization */
  if(modem.begin(&Serial2)) {
    Serial.println("Modem initialization OK");
  } else {
    Serial.println("Modem initialization ERROR");
    return;
  }
}

void loop()
{
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
    
  if(!connect("soracom.io", "sora", "sora", SERV_ADDR, SERV_PORT)) {
    ESP.restart();
  }

  if(!modem.socketSend(dataBuf, dataSize)) {
    Serial.println("Could not transmit data");
    ESP.restart();
  }

  Serial.printf("Sent data %.01f %%, %.01f C, %.01f lux, %.01f hPa\n",
    hum, temp, lux, pressure);

  delay(60000);
  ESP.restart();
}