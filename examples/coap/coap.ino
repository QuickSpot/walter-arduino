/**
 * @file coap.ino
 * @author Dries Vandenbussche <dries@dptechnics.com>
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
 * This file contains a sketch which communicates with the coap.me
 * CoAP test server.
 */

#include <HardwareSerial.h>
#include <WalterModem.h>
#include <esp_mac.h>

/**
 * @brief The CoAP profile identifier.
 */
#define MODEM_COAP_PROFILE 1

/**
 * @brief The modem instance.
 */
WalterModem modem;

/**
 * @brief Response object containing command response information.
 */
WalterModemRsp rsp = {};

/**
 * @brief The buffer to transmit to the CoAP server.
 */
uint8_t out_buf[8] = { 0 };

/**
 * @brief The buffer to receive from the CoAP server.
 * @note Make sure this is sufficiently large enough for incoming data. (Up to 1024 bytes supported
 * by Sequans)
 */
uint8_t in_buf[1024] = { 0 };

/**
 * @brief The counter used in the ping packets.
 */
uint16_t counter = 0;

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
 * This function will be called when network registration state changes or when
 * eDRX parameters are received from the network.
 *
 * @note Make sure to keep this handler as lightweight as possible to avoid blocking
 * the event processing task.
 *
 * @param[out] event The network registration state event.
 * @param[out] data The registration event data including state and PSM info.
 * @param[out] args User arguments.
 *
 * @return void
 */
static void myNetworkEventHandler(WMNetworkEventType event, const WMNetworkEventData* data,
                                  void* args)
{
  if(event == WALTER_MODEM_NETWORK_EVENT_REG_STATE_CHANGE) {
    switch(data->cereg.state) {
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
}

/**
 * @brief The CoAP event handler.
 *
 * @note Make sure to keep this handler as lightweight as possible to avoid blocking the event
 * processing task.
 *
 * @param[out] event The type of CoAP event.
 * @param[out] data The data associated with the event.
 * @param[out] args User arguments.
 *
 * @return void
 */
static void myCoAPEventHandler(WMCoAPEventType event, const WMCoAPEventData* data, void* args)
{
  switch(event) {
  case WALTER_MODEM_COAP_EVENT_CONNECTED:
    Serial.printf("CoAP: Connected successfully (profile %d)\r\n", data->profile_id);
    break;

  case WALTER_MODEM_COAP_EVENT_CLOSED:
    Serial.printf("CoAP: Disconnected (profile %d) reason: %s\r\n", data->profile_id, data->reason);
    break;

  case WALTER_MODEM_COAP_EVENT_RING:
    Serial.printf("CoAP: Message received on profile %d. (id: %d | %s | type: %d | code: %u | "
                  "size: %u)\r\n",
                  data->profile_id, data->msg_id, data->req_rsp ? "response" : "request",
                  data->type, data->rsp_code, data->data_len);

    /* Receive the CoAP message from the modem buffer */
    memset(in_buf, 0, sizeof(in_buf));
    if(modem.coapReceive(data->profile_id, data->msg_id, in_buf, data->data_len)) {
      if(data->data_len > 0) {
        Serial.printf("Received message for profile %d: %s\r\n", data->profile_id, in_buf);
      } else {
        Serial.printf("Received empty message for profile %d\r\n", data->profile_id);
      }
    } else {
      Serial.printf("Could not receive CoAP message for profile %d\r\n", data->profile_id);
    }
    break;
  }
}

/**
 * @brief The main Arduino setup method.
 */
void setup()
{
  Serial.begin(115200);
  delay(2000);

  Serial.printf("\r\n\r\n=== WalterModem CoAP example (Arduino v1.5.0) ===\r\n\r\n");

  /* Get the MAC address for board validation */
  esp_read_mac(out_buf, ESP_MAC_WIFI_STA);
  Serial.printf("Walter's MAC is: %02X:%02X:%02X:%02X:%02X:%02X\r\n", out_buf[0], out_buf[1],
                out_buf[2], out_buf[3], out_buf[4], out_buf[5]);

  if(modem.begin(&Serial2)) {
    Serial.println("Modem initialization OK");
  } else {
    Serial.println("Error: Modem initialization ERROR");
    return;
  }

  /* Set the network event handler (optional) */
  modem.setNetworkEventHandler(myNetworkEventHandler, NULL);

  /* Set the CoAP event handler */
  modem.setCoAPEventHandler(myCoAPEventHandler, NULL);
}

/**
 * @brief The main Arduino loop method.
 */
void loop()
{
  out_buf[6] = counter >> 8;
  out_buf[7] = counter & 0xFF;
  counter++;

  if(!lteConnected() && !lteConnect()) {
    Serial.println("Error: Failed to register to network");
    delay(1000);
    ESP.restart();
  }

  if(modem.coapCreateContext(MODEM_COAP_PROFILE, "coap.me", 5683)) {
    Serial.println("Successfully created or refreshed CoAP context");
  } else {
    Serial.println("Error: Could not create CoAP context.");
    return;
  }

  if(modem.coapSetHeader(MODEM_COAP_PROFILE, counter)) {
    Serial.printf("Set CoAP header with message id %d\r\n", counter);
  } else {
    Serial.println("Error: Could not set CoAP header");
    delay(1000);
    ESP.restart();
  }

  if(modem.coapSendData(MODEM_COAP_PROFILE, WALTER_MODEM_COAP_SEND_TYPE_CON,
                        WALTER_MODEM_COAP_SEND_METHOD_GET, 8, out_buf)) {
    Serial.println("Sent CoAP datagram");
  } else {
    Serial.println("Error: Could not send CoAP datagram");
    delay(1000);
    ESP.restart();
  }

  delay(15000);
  Serial.println();
}
