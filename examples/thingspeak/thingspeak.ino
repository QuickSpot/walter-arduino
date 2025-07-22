/**
 * @file thingspeak.ino
 * @author Daan Pape <daan@dptechnics.com>
 * @date 7 Jul 2025
 * @copyright DPTechnics bv
 * @brief Walter Modem library examples
 *
 * @section LICENSE
 *
 * Copyright (C) 2025, DPTechnics bv
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
 * This file contains a sketch which uses the modem in Walter to send data to 
 * ThingSpeak.
 */

#include <HardwareSerial.h>
#include <WalterModem.h>
#include <esp_mac.h>

/**
 * The ThingSpeak device MQTT username.
 */
#define THINGSPEAK_MQTT_USERNAME ""

/**
 * The ThingSpeak device MQTT client id.
 */
#define THINGSPEAK_MQTT_CLIENT_ID ""

/**
 * The ThingSpeak device MQTT password.
 */
#define THINGSPEAK_MQTT_PASSWORD ""

/**
 * @brief The id of the ThingSpeak channel to publish data to.
 */
#define THINGSPEAK_CHANNEL_ID ""

/**
 * @brief The topic to publish ThingSpeak data to.
 */
#define THINGSPEAK_TOPIC "channels/" THINGSPEAK_CHANNEL_ID "/publish"

/**
 * @brief The modem instance.
 */
WalterModem modem;

/**
 * @brief Response object containing command response information.
 */
WalterModemRsp rsp;

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
    if (timeout > 300) {
      ESP.restart();
      return false;
    }
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

void setup() {
  Serial.begin(115200);
  delay(5000);

  Serial.println("Walter ThingSpeak example");

  if (WalterModem::begin(&Serial2)) {
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

  /* 
   * Configure MQTT for ThingSpeak.
   */
  if (modem.mqttConfig(THINGSPEAK_MQTT_CLIENT_ID, THINGSPEAK_MQTT_USERNAME, THINGSPEAK_MQTT_PASSWORD)) {
    Serial.println("MQTT configuration succeeded");
  } else {
    Serial.println("Error: MQTT configuration failed");
    return;
  }

  /* Connect to the ThingSpeak MQTT broker */
  if (modem.mqttConnect("mqtt3.thingspeak.com", 1883)) {
    Serial.println("MQTT connection succeeded");
  } else {
    Serial.println("Error: MQTT connection failed");
  }
}

void loop() {
  /* Read the temperature of Walter */
  float temp = temperatureRead();
  Serial.printf("Walter's SoC temp: %.02f °C\n", temp);

  /* Get cell information */
  static WalterModemRsp rsp = {};
  if(modem.getCellInformation(WALTER_MODEM_SQNMONI_REPORTS_SERVING_CELL, &rsp)) {
    Serial.printf("Band %u, Operator %s (%u%02u), RSRP: %.2f, RSRQ: %.2f\n",
      rsp.data.cellInformation.band, rsp.data.cellInformation.netName,
      rsp.data.cellInformation.cc, rsp.data.cellInformation.nc,
      rsp.data.cellInformation.rsrp, rsp.data.cellInformation.rsrq);
  } else {
    Serial.println("Could not request cell information");
  }

  /* Publish data to ThingsSpeak over MQTT */
  static char outgoingMsg[64] = { 0 };
  sprintf(outgoingMsg, "field1=%.2f&field2=%.2f&field3=%.2f",
    temp,
    rsp.data.cellInformation.rsrp,
    rsp.data.cellInformation.rsrq);

  Serial.printf("Going to publish '%s' to ThingSpeak\n", outgoingMsg);

  if(modem.mqttPublish(THINGSPEAK_TOPIC, (uint8_t*) outgoingMsg, strlen(outgoingMsg), 0)) {
    Serial.printf("Published to ThingSpeak on topic %s\n", THINGSPEAK_TOPIC);
  } else {
    Serial.print("Could not publish to ThingSpeak\n");
  }

  delay(120000);
}