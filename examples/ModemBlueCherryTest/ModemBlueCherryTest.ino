/**
 * @file ModemBlueCherryTest.ino
 * @author Dries Vandenbussche <dries@dptechnics.com>
 * @date 06 Jun 2023
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
 * This file contains a sketch which sends and receives mqtt data
 * using the DPTechnics BlueCherry cloud platform. It also supports
 * OTA updates which are scheduled through the BlueCherry web interface.
 */

#include <esp_system.h>
#include <WalterModem.h>
#include <HardwareSerial.h>

WalterModem modem;

uint8_t dataBuf[256] = { 0 };
uint8_t otaBuffer[SPI_FLASH_BLOCK_SIZE];
uint8_t counter = 0;

void waitForNetwork()
{
  /* Wait for the network to become available */
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  while(!(regState == WALTER_MODEM_NETOWRK_REG_REGISTERED_HOME ||
          regState == WALTER_MODEM_NETOWRK_REG_REGISTERED_ROAMING))
  {
    delay(100);
    regState = modem.getNetworkRegState();
  }
  Serial.print("Connected to the network\r\n");
}

void setup()
{
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
  if(modem.createPDPContext("", WALTER_MODEM_PDP_AUTH_PROTO_PAP, "sora", "sora"))
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

  waitForNetwork();

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

  modem.initBlueCherry("coap.bluecherry.io", 65534, 0x21, otaBuffer);
  Serial.print("BlueCherry cloud platform link initialized\r\n");
}

void loop()
{
  WalterModemRsp rsp = {};
  bool moreDataAvailable;

  delay(15000);

  dataBuf[6] = counter++;
  modem.blueCherryPublish(0x84, 7, dataBuf);

  do {
    if(!modem.blueCherrySynchronize()) {
      Serial.print("Error communicating with BlueCherry cloud platform!\r\n");
      Serial.print("Rebooting modem after BlueCherry sync failure (CoAP stack may be broken)\r\n");
      modem.reset();
      modem.setOpState(WALTER_MODEM_OPSTATE_FULL);
      waitForNetwork();
      return;
    }

    Serial.print("Synchronized with the BlueCherry cloud platform, awaiting ACK\r\n");

    while(!modem.blueCherryDidRing(&moreDataAvailable, &rsp)) {
      Serial.print("Awaiting ring... ");
      delay(100);
    }
    Serial.print("\r\n");

    if(rsp.data.blueCherry.nak) {
      Serial.print("Rebooting modem after timeout waiting for ACK (workaround modem bug)\r\n");
      modem.reset();
      modem.setOpState(WALTER_MODEM_OPSTATE_FULL);
      waitForNetwork();
      delay(5000);
      Serial.print("Continuing\r\n");
      return;
    }

    Serial.printf("Successfully sent message. Nr incoming msgs: %d\r\n",
      rsp.data.blueCherry.messageCount);

    for(uint8_t msgIdx = 0; msgIdx < rsp.data.blueCherry.messageCount; msgIdx++) {
      if(rsp.data.blueCherry.messages[msgIdx].topic == 0) {
        Serial.printf("Incoming message %d/%d is a BlueCherry management message\r\n",
          msgIdx + 1, rsp.data.blueCherry.messageCount);
      } else {
        Serial.printf("Incoming message %d/%d:\r\n", msgIdx + 1, rsp.data.blueCherry.messageCount);
        Serial.printf("topic: %02x\r\n", rsp.data.blueCherry.messages[msgIdx].topic);
        Serial.printf("data size: %d\r\n", rsp.data.blueCherry.messages[msgIdx].dataSize);

        for(uint8_t byteIdx = 0; byteIdx < rsp.data.blueCherry.messages[msgIdx].dataSize; byteIdx++) {
          Serial.printf("%02x ", rsp.data.blueCherry.messages[msgIdx].data[byteIdx]);
        }

        Serial.print("\r\n");
      }
    }

    if(moreDataAvailable) {
      Serial.print("(got some incoming data but more is waiting to be fetched: doing another sync call)\r\n");
    }
  } while(moreDataAvailable);
}