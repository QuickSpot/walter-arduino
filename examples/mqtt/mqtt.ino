/**
 * @file mqtt.ino
 * @author Jonas Maes <jonas@dptechnics.com>
 * @author Arnoud Devoogdt <arnoud@dptechnics.com>
 * @date 12 Jan 2026
 * @copyright DPTechnics bv
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
 * This file contains a sketch which uses the modem in Walter to subscribe and
 * publish data to an MQTT broker.
 */

#include <HardwareSerial.h>
#include <WalterModem.h>
#include <esp_mac.h>

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
walter_modem_rsp_t rsp;

/**
 * @brief Flag indicating whether to publish a message.
 */
bool mqtt_connected = false;

/**
 * @brief The buffer to transmit to the MQTT server.
 */
uint8_t out_buf[32] = { 0 };

/**
 * @brief The buffer to receive from the MQTT server.
 * @note Make sure this is sufficiently large enough for incoming data. (Up to 4096 bytes supported
 * by Sequans)
 */
uint8_t in_buf[4096] = { 0 };

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
 * @brief The network registration event handler.
 *
 * You can use this handler to get notified of network registration state changes. For this example,
 * we use polling to get the network registration state. You can use this to implement your own
 * reconnection logic.
 *
 * @note Make sure to keep this handler as lightweight as possible to avoid blocking the event
 * processing task.
 *
 * @param[out] state The network registration state.
 * @param[out] args User arguments.
 *
 * @return void
 */
static void myNetworkEventHandler(WalterModemNetworkRegState state, void* args)
{
  switch(state) {
  case WALTER_MODEM_NETWORK_REG_REGISTERED_HOME:
    Serial.println("Network registration: Registered (home)");
    break;

  case WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING:
    Serial.println("Network registration: Registered (roaming)");
    break;

  case WALTER_MODEM_NETWORK_REG_NOT_SEARCHING:
    Serial.println("Network registration: Not searching");
    break;

  case WALTER_MODEM_NETWORK_REG_SEARCHING:
    Serial.println("Network registration: Searching");
    break;

  case WALTER_MODEM_NETWORK_REG_DENIED:
    Serial.println("Network registration: Denied");
    break;

  case WALTER_MODEM_NETWORK_REG_UNKNOWN:
    Serial.println("Network registration: Unknown");
    break;

  default:
    break;
  }
}

/**
 * @brief The MQTT event handler.
 *
 * This function will be called on various MQTT events such as connection, disconnection,
 * subscription, publication and incoming messages. You can modify this handler to implement your
 * own logic based on the events received.
 *
 * @note Make sure to keep this handler as lightweight as possible to avoid blocking the event
 * processing task.
 *
 * @param[out] event The type of MQTT event.
 * @param[out] data The data associated with the event.
 * @param[out] args User arguments.
 *
 * @return void
 */
static void myMQTTEventHandler(WMMQTTEventType event, WMMQTTEventData data, void* args)
{
  switch(event) {
  case WALTER_MODEM_MQTT_EVENT_CONNECTED:
    if(data.rc != 0) {
      Serial.printf("MQTT: Connection could not be established. (code: %d)\r\n", data.rc);
    } else {
      Serial.printf("MQTT: Connected successfully\r\n");

      /* Subscribe to the test topic */
      if(modem.mqttSubscribe(MQTT_TOPIC)) {
        Serial.printf("Subscribing to '%s'...\r\n", MQTT_TOPIC);
      } else {
        Serial.println("Subscribing failed");
      }
    }
    break;

  case WALTER_MODEM_MQTT_EVENT_DISCONNECTED:
    if(data.rc != 0) {
      Serial.printf("MQTT: Connection was interrupted (code: %d)\r\n", data.rc);
    } else {
      Serial.printf("MQTT: Disconnected\r\n");
    }
    mqtt_connected = false;
    break;

  case WALTER_MODEM_MQTT_EVENT_SUBSCRIBED:
    if(data.rc != 0) {
      Serial.printf("MQTT: Could not subscribe to topic. (code: %d)\r\n", data.rc);
    } else {
      Serial.printf("MQTT: Successfully subscribed to topic '%s'\r\n", data.topic);
      mqtt_connected = true;
    }
    break;

  case WALTER_MODEM_MQTT_EVENT_PUBLISHED:
    if(data.rc != 0) {
      Serial.printf("MQTT: Could not publish message (id: %d) to topic. (code: %d)\r\n", data.mid,
                    data.rc);
    } else {
      Serial.printf("MQTT: Successfully published message (id: %d)\r\n", data.mid);
    }
    break;

  case WALTER_MODEM_MQTT_EVENT_MESSAGE:
    Serial.printf("MQTT: Message (id: %d) received on topic '%s' (size: %ld bytes)\r\n", data.mid,
                  data.topic, data.msg_length);

    /* Receive the MQTT message from the modem buffer */
    memset(in_buf, 0, sizeof(in_buf));
    if(modem.mqttReceive(data.topic, data.mid, in_buf, data.msg_length)) {
      Serial.printf("Received message: %s\r\n", in_buf);
    } else {
      Serial.println("Could not receive MQTT message");
    }
    break;

  case WALTER_MODEM_MQTT_EVENT_MEMORY_FULL:
    Serial.println("MQTT: Memory full");
    break;
  }
}

/**
 * @brief Common routine to publish a message to an MQTT topic.
 */
static bool mqttPublishMessage(const char* topic, const char* message)
{
  Serial.printf("Publishing to topic '%s': %s ...\r\n", topic, message);
  if(!modem.mqttPublish(topic, (uint8_t*) message, strlen(message))) {
    Serial.println("Publishing failed");
    return false;
  }
  return true;
}

/**
 * @brief The main Arduino setup method.
 */
void setup()
{
  Serial.begin(115200);
  delay(2000);

  Serial.printf("\r\n\r\n=== WalterModem MQTT example (v1.5.0) ===\r\n\r\n");

  uint8_t mac[6] = { 0 };
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  sprintf((char*) out_buf, "walter%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3],
          mac[4], mac[5]);

  /* Start the modem */
  if(modem.begin(&Serial2)) {
    Serial.println("Successfully initialized the modem");
  } else {
    Serial.println("Error: Could not initialize the modem");
    return;
  }

  /* Set the network registration event handler (optional) */
  modem.setRegistrationEventHandler(myNetworkEventHandler, NULL);

  /* Set the MQTT event handler */
  modem.setMQTTEventHandler(myMQTTEventHandler, NULL);

  /* Configure the MQTT client */
  if(modem.mqttConfig(MQTT_CLIENT_ID)) {
    Serial.println("Successfully configured the MQTT client");
  } else {
    Serial.println("Error: Failed to configure MQTT client");
    return;
  }
}

/**
 * @brief The main Arduino loop method.
 */
void loop()
{
  static int seq = 0;
  static char out_msg[64];
  seq++;

  if(!lteConnected() && !lteConnect()) {
    Serial.println("Error: Failed to register to network");
    delay(1000);
    ESP.restart();
  }

  while(!mqtt_connected) {

    /* Connect to a public MQTT broker */
    if(modem.mqttConnect(MQTT_HOST, MQTT_PORT)) {
      Serial.println("Connecting to MQTT broker...");
    } else {
      Serial.println("Error: Failed to connect to MQTT broker");
      delay(1000);
      ESP.restart();
    }

    delay(5000);
  }

  Serial.println();
  sprintf(out_msg, "%s-%d", out_buf, seq);
  if(!mqttPublishMessage(MQTT_TOPIC, out_msg)) {
    Serial.println("MQTT publish failed");
    mqtt_connected = false;
  }

  delay(15000);
}