/**
 * @file ModemHttpSendTest.ino
 * @author Jonas Maes <jonas@dptechnics.com>
 * @date Oct 2023
 * @copyright DPTechnics bv
 * @brief Walter Modem library examples
 *
 * @section LICENSE
 *
 * Copyright (C) 2023, DPTechnics bv
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

#include <esp_mac.h>
#include <WalterModem.h>
#include <HardwareSerial.h>

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
uint8_t dataBuf[8] = { 0 };

/**
 * @brief Buffer for incoming HTTP response
 */
uint8_t incomingBuf[256] = { 0 };

/**
 * @brief The counter used in the ping packets. 
 */
uint16_t counter = 0;

void setup() {
  Serial.begin(115200);
  delay(5000);

  Serial.print("Walter modem test v0.0.1\r\n");

  /* Get the MAC address for board validation */
  esp_read_mac(dataBuf, ESP_MAC_WIFI_STA);
  Serial.printf("Walter's MAC is: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
    dataBuf[0],
    dataBuf[1],
    dataBuf[2],
    dataBuf[3],
    dataBuf[4],
    dataBuf[5]);

  if(WalterModem::begin(&Serial2)) {
    Serial.print("Modem initialization OK\r\n");
  } else {
    Serial.print("Modem initialization ERROR\r\n");
    return;
  }

  if(modem.checkComm()) {
    Serial.print("Modem communication is ok\r\n");
  } else {
    Serial.print("Modem communication error\r\n");
    return;
  }

  WalterModemRsp rsp = {};
  if(modem.getOpState(&rsp)) {
    Serial.printf("Modem operational state: %d\r\n", rsp.data.opState);
  } else {
    Serial.print("Could not retrieve modem operational state\r\n");
    return;
  }

  if(modem.getRadioBands(&rsp)) {
    Serial.print("Modem is configured for the following bands:\r\n");
    
    for(int i = 0; i < rsp.data.bandSelCfgSet.count; ++i) {
      WalterModemBandSelection *bSel = rsp.data.bandSelCfgSet.config + i;
      Serial.printf("  - Operator '%s' on %s: 0x%05X\r\n",
        bSel->netOperator.name,
        bSel->rat == WALTER_MODEM_RAT_NBIOT ? "NB-IoT" : "LTE-M",
        bSel->bands);
    }
  } else {
    Serial.print("Could not retrieve configured radio bands\r\n");
    return;
  }

  if(modem.setOpState(WALTER_MODEM_OPSTATE_NO_RF)) {
    Serial.print("Successfully set operational state to NO RF\r\n");
  } else {
    Serial.print("Could not set operational state to NO RF\r\n");
    return;
  }

  /* Give the modem time to detect the SIM */
  delay(2000);

  if(modem.unlockSIM()) {
    Serial.print("Successfully unlocked SIM card\r\n");
  } else {
    Serial.print("Could not unlock SIM card\r\n");
    return;
  }

  /* Create PDP context */
  if(modem.createPDPContext("soracom.io", WALTER_MODEM_PDP_AUTH_PROTO_PAP, "sora", "sora"))
  {
    Serial.print("Created PDP context\r\n");
  } else {
    Serial.print("Could not create PDP context\r\n");
    return;
  }

  /* Authenticate the PDP context */
  if(modem.authenticatePDPContext()) {
    Serial.print("Authenticated the PDP context\r\n");
  } else {
    Serial.print("Could not authenticate the PDP context\r\n");
    return;
  }

  /* Set the operational state to full */
  if(modem.setOpState(WALTER_MODEM_OPSTATE_FULL)) {
    Serial.print("Successfully set operational state to FULL\r\n");
  } else {
    Serial.print("Could not set operational state to FULL\r\n");
    return;
  }

  /* Set the network operator selection to automatic */
  if(modem.setNetworkSelectionMode(WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC)) {
    Serial.print("Network selection mode to was set to automatic\r\n");
  } else {
    Serial.print("Could not set the network selection mode to automatic\r\n");
    return;
  }

  /* Wait for the network to become available */
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  while(!(regState == WALTER_MODEM_NETWORK_REG_REGISTERED_HOME ||
          regState == WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING))
  {
    delay(100);
    regState = modem.getNetworkRegState();
  }
  Serial.print("Connected to the network\r\n");

  /* Activate the PDP context */
  if(modem.setPDPContextActive(true)) {
    Serial.print("Activated the PDP context\r\n");
  } else {
    Serial.print("Could not activate the PDP context\r\n");
    return;
  }

  /* Attach the PDP context */
  if(modem.attachPDPContext(true)) {
    Serial.print("Attached to the PDP context\r\n");
  } else {
    Serial.print("Could not attach to the PDP context\r\n");
    return;
  }

  if(modem.getPDPAddress(&rsp)) {
    Serial.print("PDP context address list:\r\n");
    Serial.printf("  - %s\r\n", rsp.data.pdpAddressList.pdpAddress);
    if(rsp.data.pdpAddressList.pdpAddress2[0] != '\0') {
      Serial.printf("  - %s\r\n", rsp.data.pdpAddressList.pdpAddress2);
    }
  } else {
    Serial.print("Could not retrieve PDP context addresses\r\n");
    return;
  }

  /* Configure http profile for a simple test */
  if(modem.httpConfigProfile(HTTP_PROFILE, "coap.bluecherry.io", 80)) {
    Serial.print("Successfully configured the http profile\r\n");
  } else {
    Serial.print("Failed to configure HTTP profile\r\n");
  }
}

void loop() {
  dataBuf[6] = counter >> 8;
  dataBuf[7] = counter & 0xFF;

  WalterModemRsp rsp = {};

  /* HTTP test */
  static short httpReceiveAttemptsLeft = 0;
  static char ctbuf[32];

  if(!httpReceiveAttemptsLeft) {
    if(modem.httpSend(HTTP_PROFILE, "/", dataBuf, 8, WALTER_MODEM_HTTP_SEND_CMD_POST, WALTER_MODEM_HTTP_POST_PARAM_OCTET_STREAM, ctbuf, sizeof(ctbuf))) {
      Serial.print("http query performed\r\n");
      httpReceiveAttemptsLeft = 3;
    } else {
      Serial.print("http query failed\r\n");
      delay(1000);
    }
  } else {
    httpReceiveAttemptsLeft--;

    if(modem.httpDidRing(HTTP_PROFILE, incomingBuf, sizeof(incomingBuf), &rsp)) {
      httpReceiveAttemptsLeft = 0;

      Serial.printf("http status code: %d\r\n", rsp.data.httpResponse.httpStatus);
      Serial.printf("content type: %s\r\n", ctbuf);
      Serial.printf("[%s]\r\n", incomingBuf);
    } else {
      Serial.print("HTTP response not yet received\r\n");
    }
  }

  delay(10000);
}
