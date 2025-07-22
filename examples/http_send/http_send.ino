/**
 * @file http_send.ino
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
 * http post request and show the result.
 */

#include <HardwareSerial.h>
#include <WalterModem.h>
#include <esp_mac.h>

/**
 * @brief HTTP profile
 */
#define HTTP_PROFILE 1

/**
 * @brief The modem instance.
 */
WalterModem modem;

/**
 * @brief The buffer to transmit to the UDP server. The first 6 bytes will be
 * the MAC address of the Walter this code is running on.
 */
uint8_t dataBuf[8] = {0};

/**
 * @brief Buffer for incoming HTTP response
 */
uint8_t incomingBuf[256] = {0};

/**
 * @brief The counter used in the ping packets.
 */
uint16_t counter = 0;

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

void setup() {
  Serial.begin(115200);
  delay(5000);

  Serial.println("Walter modem test v1.0.0");

  /* Get the MAC address for board validation */
  esp_read_mac(dataBuf, ESP_MAC_WIFI_STA);
  Serial.printf("Walter's MAC is: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                dataBuf[0], dataBuf[1], dataBuf[2], dataBuf[3], dataBuf[4],
                dataBuf[5]);

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

  /* Configure http profile for a simple test */
  if (modem.httpConfigProfile(HTTP_PROFILE, "coap.bluecherry.io", 80)) {
    Serial.println("Successfully configured the http profile");
  } else {
    Serial.println("Error: Failed to configure HTTP profile");
  }
}

void loop() {
  dataBuf[6] = counter >> 8;
  dataBuf[7] = counter & 0xFF;

  WalterModemRsp rsp = {};

  /* HTTP test */
  static short httpReceiveAttemptsLeft = 0;
  static char ctbuf[32];

  if (!httpReceiveAttemptsLeft) {
    if (modem.httpSend(
            HTTP_PROFILE, "/", dataBuf, 8, WALTER_MODEM_HTTP_SEND_CMD_POST,
            WALTER_MODEM_HTTP_POST_PARAM_OCTET_STREAM, ctbuf, sizeof(ctbuf))) {
      Serial.println("http query performed");
      httpReceiveAttemptsLeft = 3;
    } else {
      Serial.println("Error: http query failed");
      delay(1000);
    }
  } else {
    httpReceiveAttemptsLeft--;

    if (modem.httpDidRing(HTTP_PROFILE, incomingBuf, sizeof(incomingBuf),
                          &rsp)) {
      httpReceiveAttemptsLeft = 0;

      Serial.printf("http status code: %d\r\n",
                    rsp.data.httpResponse.httpStatus);
      Serial.printf("content type: %s\r\n", ctbuf);
      Serial.printf("[%s]\r\n", incomingBuf);
    } else {
      Serial.println("HTTP response not yet received");
    }
  }

  delay(10000);
}
