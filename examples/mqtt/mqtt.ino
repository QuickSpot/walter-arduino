/**
 * @file mqtt.ino
 * @author Jonas Maes <jonas@dptechnics.com>
 * @date 28 Apr 2025
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
 * This file contains a sketch which uses the modem in Walter to subscribe and
 * publish data to an MQTT broker.
 */

#include <esp_mac.h>
#include <WalterModem.h>
#include <HardwareSerial.h>

#define MQTT_PORT 1883
#define MQTT_HOST "broker.emqx.io"
#define MQTT_TOPIC "walter-test-topic"
#define MQTT_CLIENT_ID "walter-client"

/**
 * @brief The modem instance.
 */
WalterModem modem;

/**
 * @brief Response object containing command response information.
 */
WalterModemRsp rsp;

/**
 * @brief Buffer for incoming response
 */
uint8_t incomingBuf[256] = { 0 };

/**
 * @brief MQTT client and message prefix based on mac address
 */
char macString[32];

/**
 * @brief This function checks if we are connected to the LTE network
 *
 * @return true when connected, false otherwise
 */
bool lteConnected()
{
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  return (regState == WALTER_MODEM_NETWORK_REG_REGISTERED_HOME ||
          regState == WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING);
}

/**
 * @brief This function waits for the modem to be connected to the LTE network.
 *
 * @param timeout_sec The amount of seconds to wait before returning a time-out.
 *
 * @return true if connected, false on time-out.
 */
bool waitForNetwork(int timeout_sec = 300)
{
  Serial.print("Connecting to the network...");
  int time = 0;
  while(!lteConnected()) {
    Serial.print(".");
    delay(1000);
    time++;
    if(time > timeout_sec)
      return false;
  }
  Serial.println();
  Serial.println("Connected to the network");
  return true;
}

/**
 * @brief Disconnect from the LTE network.
 *
 * This function will disconnect the modem from the LTE network and block until
 * the network is actually disconnected. After the network is disconnected the
 * GNSS subsystem can be used.
 *
 * @return true on success, false on error.
 */
bool lteDisconnect()
{
  /* Set the operational state to minimum */
  if(modem.setOpState(WALTER_MODEM_OPSTATE_MINIMUM)) {
    Serial.println("Successfully set operational state to MINIMUM");
  } else {
    Serial.println("Error: Could not set operational state to MINIMUM");
    return false;
  }

  /* Wait for the network to become available */
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  while(regState != WALTER_MODEM_NETWORK_REG_NOT_SEARCHING) {
    delay(100);
    regState = modem.getNetworkRegState();
  }

  Serial.println("Disconnected from the network");
  return true;
}

/**
 * @brief This function tries to connect the modem to the cellular network.
 *
 * @return true on success, false on error.
 */
bool lteConnect()
{
  /* Set the operational state to NO RF */
  if(modem.setOpState(WALTER_MODEM_OPSTATE_NO_RF)) {
    Serial.println("Successfully set operational state to NO RF");
  } else {
    Serial.println("Error: Could not set operational state to NO RF");
    return false;
  }

  /* Create PDP context */
  if(modem.definePDPContext()) {
    Serial.println("Created PDP context");
  } else {
    Serial.println("Error: Could not create PDP context");
    return false;
  }

  /* Set the operational state to full */
  if(modem.setOpState(WALTER_MODEM_OPSTATE_FULL)) {
    Serial.println("Successfully set operational state to FULL");
  } else {
    Serial.println("Error: Could not set operational state to FULL");
    return false;
  }

  /* Set the network operator selection to automatic */
  if(modem.setNetworkSelectionMode(WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC)) {
    Serial.println("Network selection mode was set to automatic");
  } else {
    Serial.println("Error: Could not set the network selection mode to automatic");
    return false;
  }

  return waitForNetwork();
}

/**
 * @brief Common routine to publish a message to an MQTT topic.
 */
static bool mqttsPublishMessage(const char* topic, const char* message)
{
  Serial.printf("Publishing to topic '%s': %s\r\n", topic, message);
  if(modem.mqttPublish(topic, (uint8_t*) message, strlen(message))) {
    Serial.println("MQTT publish succeeded");
    return true;
  }
  Serial.println("Error: MQTT publish failed");
  return false;
}

/**
 * @brief Common routine to check for and print incoming MQTT messages.
 */
static void mqttsCheckIncoming(const char* topic)
{
  while(modem.mqttDidRing(topic, incomingBuf, sizeof(incomingBuf), &rsp)) {
    Serial.printf("Incoming MQTT message on '%s'\r\n", topic);
    Serial.printf("  QoS: %d, Message ID: %d, Length: %d\r\n", rsp.data.mqttResponse.qos,
                  rsp.data.mqttResponse.messageId, rsp.data.mqttResponse.length);
    Serial.println("  Payload:");
    for(int i = 0; i < rsp.data.mqttResponse.length; i++) {
      Serial.printf("  '%c' 0x%02X\r\n", incomingBuf[i], incomingBuf[i]);
    }
  }
}

/**
 * @brief The main Arduino setup method.
 */
void setup()
{
  Serial.begin(115200);
  delay(5000);

  Serial.printf("\r\n\r\n=== WalterModem MQTT example ===\r\n\r\n");

  /* Build a unique client ID from the ESP MAC address */
  esp_read_mac(incomingBuf, ESP_MAC_WIFI_STA);
  sprintf(macString, "walter%02X:%02X:%02X:%02X:%02X:%02X", incomingBuf[0], incomingBuf[1],
          incomingBuf[2], incomingBuf[3], incomingBuf[4], incomingBuf[5]);

  /* Start the modem */
  if(WalterModem::begin(&Serial2)) {
    Serial.println("Successfully initialized the modem");
  } else {
    Serial.println("Error: Could not initialize the modem");
    return;
  }

  /* Connect the modem to the LTE network */
  if(!lteConnect()) {
    Serial.println("Error: Could not connect to LTE");
    return;
  }

  /* Configure the MQTT client */
  if(modem.mqttConfig(MQTT_CLIENT_ID)) {
    Serial.println("Successfully configured the MQTT client");
  } else {
    Serial.println("Error: Failed to configure MQTT client");
    return;
  }

  /* Connect to a public MQTT broker */
  if(modem.mqttConnect(MQTT_HOST, MQTT_PORT)) {
    Serial.println("Successfully connected to MQTT broker");
  } else {
    Serial.println("Error: Failed to connect to MQTT broker");
    return;
  }

  /* Subscribe to the test topic */
  if(modem.mqttSubscribe(MQTT_TOPIC)) {
    Serial.printf("Successfully subscribed to '%s'", MQTT_TOPIC);
  } else {
    Serial.println("Error: MQTT subscribe failed");
  }
}

/**
 * @brief The main Arduino loop method.
 */
void loop()
{
  static unsigned long lastPublish = 0;
  const unsigned long publishInterval = 15000; // 15 seconds

  static int seq = 0;
  static char outgoingMsg[64];

  /* Periodically publish a message */
  if(millis() - lastPublish >= publishInterval) {
    lastPublish = millis();
    seq++;

    if(seq % 3 == 0) {
      sprintf(outgoingMsg, "%s-%d", macString, seq);
      if(!mqttsPublishMessage(MQTT_TOPIC, outgoingMsg)) {
        Serial.println("MQTT publish failed, restarting...");
        delay(1000);
        ESP.restart();
      }
      Serial.println();
    }
  }

  /* Check for incoming messages */
  mqttsCheckIncoming(MQTT_TOPIC);
}
