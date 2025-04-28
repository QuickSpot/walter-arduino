/**
 * @file coap.ino
 * @author Dries Vandenbussche <dries@dptechnics.com>
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
 * This file contains a sketch which communicates with the coap.me
 * COAP test server.
 */

#include <HardwareSerial.h>
#include <WalterModem.h>
#include <esp_mac.h>


/**
 * @brief COAP profile used for COAP tests
 */
#define COAP_PROFILE 1

/**
 * @brief The modem instance.
 */
WalterModem modem;

/**
 * @brief Response object containing command response information.
 */
WalterModemRsp rsp = {};

/**
 * @brief The buffer to transmit to the COAP server.
 */
uint8_t dataBuf[8] = {0};

/**
 * @brief Buffer for incoming COAP response
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

  Serial.println("Error: Walter modem coap example v1.0.0");

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
}

void loop() {
  delay(10000);

  dataBuf[6] = counter >> 8;
  dataBuf[7] = counter & 0xFF;

  counter++;

  static short receiveAttemptsLeft = 0;

  if (!modem.coapCreateContext(COAP_PROFILE, "coap.me", 5683)) {
    Serial.println(
        "Error: Could not create COAP context. Better luck next iteration?");
    return;
  } else {
    Serial.println("Successfully created or refreshed COAP context");
  }

  if (!receiveAttemptsLeft) {
    if (modem.coapSetHeader(COAP_PROFILE, counter)) {
      Serial.printf("Set COAP header with message id %d\r\n", counter);
    } else {
      Serial.println("Error: Could not set COAP header");
      delay(1000);
      ESP.restart();
    }

    if (modem.coapSendData(COAP_PROFILE, WALTER_MODEM_COAP_SEND_TYPE_CON,
                           WALTER_MODEM_COAP_SEND_METHOD_GET, 8, dataBuf)) {
      Serial.println("Sent COAP datagram");
      receiveAttemptsLeft = 3;
    } else {
      Serial.println("Error: Could not send COAP datagram");
      delay(1000);
      ESP.restart();
    }
  } else {
    receiveAttemptsLeft--;
    Serial.println("Checking for incoming COAP message or response");

    while (modem.coapDidRing(COAP_PROFILE, incomingBuf, sizeof(incomingBuf),
                             &rsp)) {
      receiveAttemptsLeft = 0;

      Serial.println("COAP incoming:");
      Serial.printf("profileId: %d (profile ID used by us: %d)\r\n",
                    rsp.data.coapResponse.profileId, COAP_PROFILE);
      Serial.printf("Message id: %d\r\n", rsp.data.coapResponse.messageId);
      Serial.printf("Send type (CON, NON, ACK, RST): %d\r\n",
                    rsp.data.coapResponse.sendType);
      Serial.printf("Method or response code: %d\r\n",
                    rsp.data.coapResponse.methodRsp);
      Serial.printf("Data (%d bytes):\r\n", rsp.data.coapResponse.length);

      for (size_t i = 0; i < rsp.data.coapResponse.length; i++) {
        Serial.printf("[%02x  %c] ", incomingBuf[i], incomingBuf[i]);
      }
      Serial.println("");
    }
  }
}
