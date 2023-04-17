/**
 * @file ModemTest.ino
 * @author Daan Pape <daan@dptechnics.com>
 * @date 15 Feb 2023
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
 * connection to a network and upload counter values to the Walter demo server.
 */

#include <esp_system.h>
#include <WalterModem.h>
#include <HardwareSerial.h>

/**
 * @brief The address of the server to upload the data to. 
 */
#define SERV_ADDR "64.225.64.140"

/**
 * @brief The UDP port on which the server is listening.
 */
#define SERV_PORT 1999

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
 * @brief The counter used in the ping packets. 
 */
uint16_t counter = 0;

void setup() {
  Serial.begin(115200);
  delay(5000);

  Serial.println("Walter modem test v0.0.1");

  /* Get the MAC address for board validation */
  esp_read_mac(dataBuf, ESP_MAC_WIFI_STA);
  Serial.printf("Walter's MAC is: %02X:%02X:%02X:%02X:%02X:%02X\n",
    dataBuf[0],
    dataBuf[1],
    dataBuf[2],
    dataBuf[3],
    dataBuf[4],
    dataBuf[5]);

  if(WalterModem::begin(&Serial2)) {
    Serial.println("Modem initialization OK");
  } else {
    Serial.println("Modem initialization ERROR");
    return;
  }

  if(modem.checkComm()) {
    Serial.println("Modem communication is ok");
  } else {
    Serial.println("Modem communication error");
    return;
  }

  WalterModemRsp rsp = {};
  if(modem.getOpState(&rsp)) {
    Serial.printf("Modem operational state: %d\n", rsp.data.opState);
  } else {
    Serial.println("Could not retrieve modem operational state");
    return;
  }

  if(modem.getRadioBands(&rsp)) {
    Serial.println("Modem is configured for the following bands:");
    
    for(int i = 0; i < rsp.data.bandSelCfgSet.count; ++i) {
      WalterModemBandSelection *bSel = rsp.data.bandSelCfgSet.config + i;
      Serial.printf("  - Operator '%s' on %s: 0x%05X\n",
        bSel->netOperator.name,
        bSel->rat == WALTER_MODEM_RAT_NBIOT ? "NB-IoT" : "LTE-M",
        bSel->bands);
    }
  } else {
    Serial.println("Could not retrieve configured radio bands");
    return;
  }

  if(modem.setOpState(WALTER_MODEM_OPSTATE_NO_RF)) {
    Serial.println("Successfully set operational state to NO RF");
  } else {
    Serial.println("Could not set operational state to NO RF");
    return;
  }

  /* Give the modem time to detect the SIM */
  delay(2000);

  if(modem.unlockSIM()) {
    Serial.println("Successfully unlocked SIM card");
  } else {
    Serial.println("Could not unlock SIM card");
    return;
  }

  /* Create PDP context */
  if(modem.createPDPContext(
    "soracom.io",
    WALTER_MODEM_PDP_AUTH_PROTO_PAP,
    "sora",
    "sora"))
  {
    Serial.println("Created PDP context");
  } else {
    Serial.println("Could not create PDP context");
    return;
  }

  /* Authenticate the PDP context */
  if(modem.authenticatePDPContext()) {
    Serial.println("Authenticated the PDP context");
  } else {
    Serial.println("Could not authenticate the PDP context");
    return;
  }

  /* Set the operational state to full */
  if(modem.setOpState(WALTER_MODEM_OPSTATE_FULL)) {
    Serial.println("Successfully set operational state to FULL");
  } else {
    Serial.println("Could not set operational state to FULL");
    return;
  }

  /* Set the network operator selection to automatic */
  if(modem.setNetworkSelectionMode(WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC)) {
    Serial.println("Network selection mode to was set to automatic");
  } else {
    Serial.println("Could not set the network selection mode to automatic");
    return;
  }

  /* Wait for the network to become available */
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  while(!(regState == WALTER_MODEM_NETOWRK_REG_REGISTERED_HOME ||
          regState == WALTER_MODEM_NETOWRK_REG_REGISTERED_ROAMING))
  {
    delay(100);
    regState = modem.getNetworkRegState();
  }
  Serial.println("Connected to the network");

  /* Activate the PDP context */
  if(modem.setPDPContextActive(true)) {
    Serial.println("Activated the PDP context");
  } else {
    Serial.println("Could not activate the PDP context");
    return;
  }

  /* Attach the PDP context */
  if(modem.attachPDPContext(true)) {
    Serial.println("Attached to the PDP context");
  } else {
    Serial.println("Could not attach to the PDP context");
    return;
  }

  if(modem.getPDPAddress(&rsp)) {
    Serial.println("PDP context address list: ");
    Serial.printf("  - %s\n", rsp.data.pdpAddressList.pdpAddress);
    if(rsp.data.pdpAddressList.pdpAddress2[0] != '\0') {
      Serial.printf("  - %s\n", rsp.data.pdpAddressList.pdpAddress2);
    }
  } else {
    Serial.println("Could not retrieve PDP context addresses");
    return;
  }

  /* Construct a socket */
  if(modem.createSocket(&rsp)) {
    Serial.println("Created a new socket");
  } else {
    Serial.println("Could not create a new socket");
  }

  /* Configure the socket */
  if(modem.configSocket()) {
    Serial.println("Successfully configured the socket");
  } else {
    Serial.println("Could not configure the socket");
  }

  /* Connect to the UDP test server */
  if(modem.connectSocket(SERV_ADDR, SERV_PORT, SERV_PORT)) {
    Serial.printf("Connected to UDP server %s:%d\n", SERV_ADDR, SERV_PORT);
  } else {
    Serial.println("Could not connect UDP socket");
  }
}

void loop() {
  dataBuf[6] = counter >> 8;
  dataBuf[7] = counter & 0xFF;

  if(modem.socketSend(dataBuf, 8)) {
    Serial.printf("Transmitted counter value %d\n", counter);
    counter += 1;
  } else {
    Serial.println("Could not transmit data");
    delay(1000);
    ESP.restart();
  }

  delay(10000);
}