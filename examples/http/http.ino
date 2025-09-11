/**
 * @file http.ino
 * @author Arnoud Devoogdt <arnoud@dptechnics.com>
 * @date 11 September 2025
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
 * HTTP GET/POST request and show the result.
 */

#include <esp_mac.h>
#include <WalterModem.h>
#include <HardwareSerial.h>

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
 * @brief Buffer for incoming HTTP response
 */
uint8_t incomingBuf[1024] = { 0 };

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
 * @brief Common routine to wait for and print an HTTP response.
 */
static bool waitForHttpResponse(uint8_t profile, const char* contentType)
{
  Serial.print("Waiting for reply...");
  const uint16_t maxPolls = 30;
  for(uint16_t i = 0; i < maxPolls; i++) {
    Serial.print(".");
    if(modem.httpDidRing(profile, incomingBuf, sizeof(incomingBuf), &rsp)) {
      Serial.println();
      Serial.printf("HTTP status code (Modem): %d\r\n", rsp.data.httpResponse.httpStatus);
      Serial.printf("Content type: %s\r\n", contentType);
      Serial.printf("Payload:\r\n%s\r\n", incomingBuf);
      return true;
    }
    delay(1000);
  }
  Serial.println();
  Serial.println("Error: HTTP response timeout");
  return false;
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
  Serial.println("HTTP GET Successfully sent");
  return waitForHttpResponse(MODEM_HTTP_PROFILE, ctBuf);
}

/**
 * @brief Perform an HTTP POST request with a body.
 */
bool httpPost(const char* path, const uint8_t* body, size_t bodyLen,
              const char* mimeType = "application/json")
{
  char ctBuf[32] = { 0 };

  Serial.printf("Sending HTTP POST to %s%s (%u bytes, type %s)\r\n", HTTP_HOST, path,
                (unsigned) bodyLen, mimeType);
  if(!modem.httpSend(MODEM_HTTP_PROFILE, path, (uint8_t*) body, (uint16_t) bodyLen,
                     WALTER_MODEM_HTTP_SEND_CMD_POST, WALTER_MODEM_HTTP_POST_PARAM_JSON, ctBuf,
                     sizeof(ctBuf))) {
    Serial.println("Error: HTTP POST failed");
    return false;
  }
  Serial.println("HTTP POST Successfully sent");
  return waitForHttpResponse(MODEM_HTTP_PROFILE, ctBuf);
}

/**
 *@brief The main Arduino setup method
 */
void setup()
{
  Serial.begin(115200);
  delay(5000);

  Serial.printf("\r\n\r\n=== WalterModem HTTP Example ===\r\n\r\n");

  /* Start the modem */
  if(WalterModem::begin(&Serial2)) {
    Serial.println("Successfully initialized the modem");
  } else {
    Serial.println("Error: Could not initialize the modem");
    return;
  }

  /* Connect the modem to the lte network */
  if(!lteConnect()) {
    Serial.println("Error: Could not Connect to LTE");
    return;
  }

  /* Configure the HTTP profile */
  if(modem.httpConfigProfile(MODEM_HTTP_PROFILE, HTTP_HOST, HTTP_PORT)) {
    Serial.println("Successfully configured the HTTP profile");
  } else {
    Serial.println("Error: Failed to configure HTTP profile");
  }
}

/**
 * @brief The main Arduino loop method
 */
void loop()
{
  static unsigned long lastRequest = 0;
  const unsigned long requestInterval = 10000; // 10 seconds

  if(millis() - lastRequest >= requestInterval) {
    lastRequest = millis();

    // Example GET
    if(!httpGet(HTTP_GET_ENDPOINT)) {
      Serial.println("HTTP GET failed, restarting...");
      delay(1000);
      ESP.restart();
    }

    Serial.println();
    delay(2000);

    // Example POST
    const char jsonBody[] = "{\"hello\":\"walter\"}";
    if(!httpPost(HTTP_POST_ENDPOINT, (const uint8_t*) jsonBody, strlen(jsonBody),
                 "application/json")) {
      Serial.println("HTTP POST failed, restarting...");
      delay(1000);
      ESP.restart();
    }

    Serial.println();
  }
}
