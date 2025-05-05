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
 * This file contains a sketch which uses the modem in Walter to make a
 * http request and show the result.
 */

#include <HardwareSerial.h>
#include <WalterModem.h>
#include <esp_mac.h>

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
uint8_t incomingBuf[256] = {0};

/**
 * @brief MQTT client and message prefix based on mac address
 */
char macString[32];

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
    delay(100);
    timeout += 100;
    if (timeout > 300000)
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

void setup() {
  Serial.begin(115200);
  delay(5000);

  Serial.println("Walter modem MQTT example v1.0.0\r\n");

  esp_read_mac(incomingBuf, ESP_MAC_WIFI_STA);
  sprintf(macString, "walter%02X:%02X:%02X:%02X:%02X:%02X", incomingBuf[0],
          incomingBuf[1], incomingBuf[2], incomingBuf[3], incomingBuf[4],
          incomingBuf[5]);

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

  /* configure the mqtt client */
  if (modem.mqttConfig("walter-mqtt-test-topic")) {
    Serial.println("MQTT configuration succeeded");
  } else {
    Serial.println("Error: MQTT configuration failed");
    return;
  }

  /* other public mqtt broker with web client: mqtthq.com */
  if (modem.mqttConnect("test.mosquitto.org", 1883)) {
    Serial.println("MQTT connection succeeded");
  } else {
    Serial.println("Error: MQTT connection failed");
  }

  if (modem.mqttSubscribe("walter-mqtt-test-topic")) {
    Serial.println("MQTT subscribed to topic 'walter-mqtt-test-topic'");
  } else {
    Serial.println("Error: MQTT subscribe failed");
  }
}

void loop() {
  delay(15000);

  static int seq = 0;
  static char outgoingMsg[64];
  seq++;
  if (seq % 3 == 0) {
    sprintf(outgoingMsg, "%s-%d", macString, seq);
    if (modem.mqttPublish("walter-mqtt-test-topic", (uint8_t *)outgoingMsg,
                          strlen(outgoingMsg))) {
      Serial.printf("published '%s' on topic 'walter-mqtt-test-topic'\r\n",
                    outgoingMsg);
    } else {
      Serial.print("MQTT publish failed\r\n");
    }
  }

  while (modem.mqttDidRing("walter-mqtt-test-topic", incomingBuf,
                           sizeof(incomingBuf), &rsp)) {
    Serial.printf("incoming: qos=%d msgid=%d len=%d:\r\n",
                  rsp.data.mqttResponse.qos, rsp.data.mqttResponse.messageId,
                  rsp.data.mqttResponse.length);
    for (int i = 0; i < rsp.data.mqttResponse.length; i++) {
      Serial.printf("'%c' 0x%02x\r\n", incomingBuf[i], incomingBuf[i]);
    }
  }
}
