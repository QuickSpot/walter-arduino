/**
 * @file http.ino
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
 * This file contains a sketch which uses the modem in Walter to make a
 * HTTP GET/POST request and show the result.
 */

#include <HardwareSerial.h>
#include <WalterModem.h>
#include <esp_mac.h>

#define HTTP_PORT 80
#define HTTP_HOST "quickspot.io"
#define HTTP_GET_ENDPOINT "/hello/get"
#define HTTP_POST_ENDPOINT "/hello/post"

/**
 * @brief HTTP profile
 */
#define MODEM_HTTP_PROFILE 1

/**
 * @brief The modem instance.
 */
WalterModem modem;

/**
 * @brief Response object containing command response information.
 */
WalterModemRsp rsp = {};

/**
 * @brief The buffer to receive from the HTTP server.
 * @note Make sure this is sufficiently large enough for incoming data. (Up to 1500 bytes supported
 * by Sequans)
 */
uint8_t in_buf[1500] = { 0 };

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
 * @brief The HTTP event handler.
 *
 * This function will be called on various HTTP events such as connection, disconnection,
 * ring, etc. You can modify this handler to implement your own logic based on the events received.
 *
 * @note Make sure to keep this handler as lightweight as possible to avoid blocking the event
 * processing task.
 *
 * @param[out] event The type of HTTP event.
 * @param[out] data The data associated with the event.
 * @param[out] args User arguments.
 *
 * @return void
 */
static void myHTTPEventHandler(WMHTTPEventType event, const WMHTTPEventData* data, void* args)
{
  switch(event) {
  case WALTER_MODEM_HTTP_EVENT_CONNECTED:
    if(data->rc != 0) {
      Serial.printf("HTTP: Connection (profile %d) could not be established. (CURL: %d)\r\n",
                    data->profile_id, data->rc);
    } else {
      Serial.printf("HTTP: Connected successfully (profile %d)\r\n", data->profile_id);
    }
    break;

  case WALTER_MODEM_HTTP_EVENT_DISCONNECTED:
    Serial.printf("HTTP: Disconnected successfully (profile %d)\r\n", data->profile_id);
    break;

  case WALTER_MODEM_HTTP_EVENT_CONNECTION_CLOSED:
    Serial.printf("HTTP: Connection (profile %d) was interrupted (CURL: %d)\r\n", data->profile_id,
                  data->rc);
    break;

  case WALTER_MODEM_HTTP_EVENT_RING:
    Serial.printf(
        "HTTP: Message received on profile %d. (status: %d | content-type: %s | size: %u)\r\n",
        data->profile_id, data->status, data->content_type, data->data_len);

    /* Receive the HTTP message from the modem buffer */
    memset(in_buf, 0, sizeof(in_buf));
    if(modem.httpReceive(data->profile_id, in_buf, data->data_len)) {
      Serial.printf("Received message for profile %d: %s\r\n", data->profile_id, in_buf);
    } else {
      Serial.printf("Could not receive HTTP message for profile %d\r\n", data->profile_id);
    }
    break;
  }
}

/**
 * @brief Perform an HTTP GET request.
 */
bool httpGet(const char* path)
{
  char ctBuf[32] = { 0 };

  Serial.printf("Sending HTTP GET to %s%s\r\n", HTTP_HOST, path);
  if(!modem.httpQuery(MODEM_HTTP_PROFILE, path, WALTER_MODEM_HTTP_QUERY_CMD_GET, ctBuf,
                      sizeof(ctBuf))) {
    Serial.println("Error: HTTP GET query failed");
    return false;
  }
  Serial.println("HTTP GET successfully sent");
  return true;
}

/**
 * @brief Perform an HTTP POST request with a body.
 */
bool httpPost(const char* path, const uint8_t* body, size_t bodyLen,
              const char* mimeType = "application/json")
{
  char ctBuf[32] = { 0 };

  Serial.printf("Sending HTTP POST to %s%s (content-type: %s | size: %d)\r\n", HTTP_HOST, path,
                mimeType, (int) bodyLen);
  if(!modem.httpSend(MODEM_HTTP_PROFILE, path, (uint8_t*) body, (uint16_t) bodyLen,
                     WALTER_MODEM_HTTP_SEND_CMD_POST, WALTER_MODEM_HTTP_POST_PARAM_JSON, ctBuf,
                     sizeof(ctBuf))) {
    Serial.println("Error: HTTP POST failed");
    return false;
  }
  Serial.println("HTTP POST successfully sent");
  return true;
}

/**
 *@brief The main Arduino setup method
 */
void setup()
{
  Serial.begin(115200);
  delay(2000);

  Serial.printf("\r\n\r\n=== WalterModem HTTP example (Arduino v1.5.0) ===\r\n\r\n");

  /* Start the modem */
  if(modem.begin(&Serial2)) {
    Serial.println("Successfully initialized the modem");
  } else {
    Serial.println("Error: Could not initialize the modem");
    return;
  }

  /* Set the network registration event handler (optional) */
  modem.setRegistrationEventHandler(myNetworkEventHandler, NULL);

  /* Set the HTTP event handler */
  modem.setHTTPEventHandler(myHTTPEventHandler, NULL);

  /* Configure the HTTP profile */
  if(modem.httpConfigProfile(MODEM_HTTP_PROFILE, HTTP_HOST, HTTP_PORT)) {
    Serial.println("Successfully configured the HTTP profile");
  } else {
    Serial.println("Error: Failed to configure HTTP profile");
    return;
  }
}

/**
 * @brief The main Arduino loop method
 */
void loop()
{
  if(!lteConnected() && !lteConnect()) {
    Serial.println("Error: Failed to register to network");
    delay(1000);
    ESP.restart();
  }

  // Example GET
  if(!httpGet(HTTP_GET_ENDPOINT)) {
    Serial.println("HTTP GET failed, restarting...");
    delay(1000);
    ESP.restart();
  }

  delay(5000);
  Serial.println();

  // Example POST
  const char jsonBody[] = "{\"hello\":\"quickspot\"}";
  if(!httpPost(HTTP_POST_ENDPOINT, (const uint8_t*) jsonBody, strlen(jsonBody),
               "application/json")) {
    Serial.println("HTTP POST failed, restarting...");
    delay(1000);
    ESP.restart();
  }

  delay(15000);
  Serial.println();
}
