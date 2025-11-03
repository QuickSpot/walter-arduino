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
 * @brief This function checks if we are connected to the lte network
 *
 * @return True when connected, False otherwise
 */
bool lteConnected()
{
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  return (regState == WALTER_MODEM_NETWORK_REG_REGISTERED_HOME ||
          regState == WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING);
}

/**
 * @brief This function waits for the modem to be connected to the Lte network.
 * @return true if the connected, else false on timeout.
 */
bool waitForNetwork()
{
  /* Wait for the network to become available */
  int timeout = 0;
  while(!lteConnected()) {
    delay(1000);
    timeout++;
    if(timeout > 300)
      return false;
  }
  Serial.println("Connected to the network");
  return true;
}

/**
 * @brief This function tries to connect the modem to the cellular network.
 * @return true if the connection attempt is successful, else false.
 */
bool lteConnect()
{
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
    Serial.println("Network selection mode to was set to automatic");
  } else {
    Serial.println("Error: Could not set the network selection mode to automatic");

    return false;
  }

  return waitForNetwork();
}

/**
 * @brief Modem URC event handler
 *
 * Handles unsolicited result codes (URC) from the modem.
 *
 * @note This method should not block for too long. Consider moving heavy processing and blocking
 * functions to your main application thread.
 *
 * @param ev Pointer to the URC event data.
 * @param args User argument pointer passed to urcSetEventHandler
 */
static void myURCHandler(const WalterModemURCEvent* ev, void* args)
{
  Serial.printf("URC received at %lld\n", ev->timestamp);
  switch(ev->type) {
  case WM_URC_TYPE_COAP:
    if(ev->coap.event == WALTER_MODEM_COAP_EVENT_RING) {
      Serial.printf(
          "CoAP Ring Received for profile: %d Length: %u Type: %u Message ID: %u Code: %u\n",
          ev->coap.profileId, ev->coap.dataLen, ev->coap.type, ev->coap.msgId, ev->coap.rspCode);
      if(modem.coapReceiveMessage(ev->coap.profileId, ev->coap.msgId, in_buf, ev->coap.dataLen)) {
        Serial.printf("Payload:\n");
        for(int i = 0; i < ev->coap.dataLen; i++) {
          Serial.printf("%c", in_buf[i]);
        }
        Serial.printf("\n");
      }
    }
    break;
  default:
    /* Unhandled event */
    break;
  }
}

void setup()
{
  Serial.begin(115200);
  delay(5000);

  Serial.printf("\r\n\r\n=== WalterModem CoAP example ===\r\n\r\n");

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

  modem.urcSetEventHandler(myURCHandler, NULL);

  /* Connect the modem to the lte network */
  if(!lteConnect()) {
    Serial.println("Error: Could Not Connect to LTE");
    return;
  }
}

void loop()
{
  static unsigned long lastPublish = 0;
  const unsigned long publishInterval = 15000;

  /* Periodically publish a message */
  if(millis() - lastPublish >= publishInterval) {
    lastPublish = millis();

    out_buf[6] = counter >> 8;
    out_buf[7] = counter & 0xFF;

    counter++;

    if(!modem.coapCreateContext(COAP_PROFILE, "coap.me", 5683)) {
      Serial.println("Error: Could not create COAP context.");
      return;
    } else {
      Serial.println("Successfully created or refreshed COAP context");
    }

    if(modem.coapSetHeader(COAP_PROFILE, counter)) {
      Serial.printf("Set COAP header with message id %d\r\n", counter);
    } else {
      Serial.println("Error: Could not set COAP header");
      delay(1000);
      ESP.restart();
    }

    if(modem.coapSendData(COAP_PROFILE, WALTER_MODEM_COAP_SEND_TYPE_CON,
                          WALTER_MODEM_COAP_SEND_METHOD_GET, 8, out_buf)) {
      Serial.println("Sent COAP datagram");
    } else {
      Serial.println("Error: Could not send COAP datagram");
      delay(1000);
      ESP.restart();
    }
  }

  delay(10);
}
