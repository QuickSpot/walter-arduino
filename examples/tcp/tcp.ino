/**
 * @file tcp.ino
 * @author Daan Pape <daan@dptechnics.com>
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
 * connection to a network and upload data packets to the Walter demo server.
 */

#include <HardwareSerial.h>
#include <WalterModem.h>
#include <esp_mac.h>

#define TCP_PORT 1999
#define TCP_HOST "walterdemo.quickspot.io"

#define BASIC_INFO_PACKET_SIZE 24
#define COUNTER_PACKET_SIZE 8

/**
 * @brief The Socket profile to use (1..6)
 *
 * @note At least one socket should be available/reserved for BlueCherry.
 */
#define MODEM_SOCKET_PROFILE 1

/**
 * @brief The modem instance.
 */
WalterModem modem;

/**
 * @brief Response object containing command response information.
 */
walter_modem_rsp_t rsp = {};

/**
 * @brief The buffer to transmit to the TCP server. The first 6 bytes will be
 * the MAC address of the Walter this code is running on.
 */
uint8_t out_buf[8] = { 0 };

/**
 * @brief The buffer to receive from the TCP server.
 * @note Make sure this is sufficiently large enough for incoming data. (Up to 1500 bytes supported
 * by Sequans)
 */
uint8_t in_buf[1500] = { 0 };

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
 * @brief Send a basic info packet to walterdemo
 */
bool tcpSendBasicInfoPacket()
{
  uint16_t packet_size = COUNTER_PACKET_SIZE;

  out_buf[6] = counter >> 8;
  out_buf[7] = counter & 0xFF;

  /* Only send the full packet if cellinfo is valid */
  if(rsp.data.cellInformation.cc != 0 || rsp.data.cellInformation.nc != 0 ||
     rsp.data.cellInformation.tac != 0 || rsp.data.cellInformation.cid != 0) {
    packet_size = BASIC_INFO_PACKET_SIZE;

    /* Read the temperature of Walter */
    float temp = temperatureRead();
    uint16_t rawTemp = (temp + 50) * 100;

    uint8_t rat = -1;
    if(modem.getRAT(&rsp)) {
      rat = (uint8_t) rsp.data.rat;
    }

    /* Construct the basic info packet */
    out_buf[8] = rawTemp >> 8;
    out_buf[9] = rawTemp & 0xFF;
    out_buf[10] = rsp.data.cellInformation.cc >> 8;
    out_buf[11] = rsp.data.cellInformation.cc & 0xFF;
    out_buf[12] = rsp.data.cellInformation.nc >> 8;
    out_buf[13] = rsp.data.cellInformation.nc & 0xFF;
    out_buf[14] = rsp.data.cellInformation.tac >> 8;
    out_buf[15] = rsp.data.cellInformation.tac & 0xFF;
    out_buf[16] = (rsp.data.cellInformation.cid >> 24) & 0xFF;
    out_buf[17] = (rsp.data.cellInformation.cid >> 16) & 0xFF;
    out_buf[18] = (rsp.data.cellInformation.cid >> 8) & 0xFF;
    out_buf[19] = rsp.data.cellInformation.cid & 0xFF;
    out_buf[20] = (uint8_t) (rsp.data.cellInformation.rsrp * -1);
    out_buf[21] = (uint8_t) (rsp.data.cellInformation.rsrq * -1);
    out_buf[22] = rat;
    out_buf[23] = 0xFF;
  }

  Serial.println("Sending packet...");

  if(!modem.socketSend(MODEM_SOCKET_PROFILE, out_buf, packet_size)) {
    Serial.println("Error: TCP send packet failed");
    return false;
  }

  delay(2000);

  /* Attempt to get the latest cell information (for next packet) */
  modem.getCellInformation(WALTER_MODEM_SQNMONI_REPORTS_SERVING_CELL, &rsp);

  Serial.println("TCP send basic packet succeeded");
  return true;
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
static void myURCHandler(const WalterModemEvent* ev, void* args)
{
  Serial.printf("URC received at %lld\n", ev->timestamp);
  switch(ev->type) {
  case WM_URC_TYPE_SOCKET:
    if(ev->socket.event == WALTER_MODEM_SOCKET_EVENT_RING) {
      Serial.printf("Socket Ring Received for profile %d: Length: %u\n", ev->socket.profileId,
                    ev->socket.dataLen);
      if(modem.socketReceiveMessage(ev->socket.profileId, in_buf, ev->socket.dataLen)) {
        for(int i = 0; i < ev->socket.dataLen; i++) {
          Serial.printf("%c", in_buf[i]);
        }
        Serial.printf("\n");
      }
    } else if(ev->socket.event == WALTER_MODEM_SOCKET_EVENT_DISCONNECTED) {
      Serial.printf("Socket was closed for profile %d", ev->socket.profileId);
    }
    break;
  default:
    /* Unhandled event */
    break;
  }
}

/**
 * @brief The main Arduino setup method.
 */
void setup()
{
  Serial.begin(115200);
  delay(5000);

  Serial.printf("\r\n\r\n=== WalterModem TCP example ===\r\n\r\n");

  /* Start the modem */
  if(modem.begin(&Serial2)) {
    Serial.println("Successfully initialized the modem");
  } else {
    Serial.println("Error: Could not initialize the modem");
    return;
  }

  /* Set the modem URC event handler */
  modem.urcSetEventHandler(myURCHandler, NULL);

  /* Connect the modem to the LTE network */
  if(!lteConnect()) {
    Serial.println("Error: Could not connect to LTE");
    return;
  }

  /* Retrieve and print the board MAC address */
  esp_read_mac(out_buf, ESP_MAC_WIFI_STA);
  Serial.printf("Board MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n", out_buf[0], out_buf[1], out_buf[2],
                out_buf[3], out_buf[4], out_buf[5]);

  /* Configure a new socket */
  if(modem.socketConfig(MODEM_SOCKET_PROFILE)) {
    Serial.println("Successfully configured a new socket");
  } else {
    Serial.println("Error: Could not configure a new socket");
    return;
  }

  /* Disable TLS (the demo TCP server does not use it) */
  if(modem.socketConfigSecure(MODEM_SOCKET_PROFILE, false)) {
    Serial.println("Successfully set socket to insecure mode");
  } else {
    Serial.println("Error: Could not disable socket TLS");
    return;
  }

  /* Connect (dial) to the TCP test server */
  if(modem.socketDial(MODEM_SOCKET_PROFILE, WALTER_MODEM_SOCKET_PROTO_TCP, TCP_PORT, TCP_HOST)) {
    Serial.printf("Successfully dialed TCP server %s:%d\r\n", TCP_HOST, TCP_PORT);
  } else {
    Serial.println("Error: Could not dial TCP server");
    return;
  }
}

/**
 * @brief The main Arduino loop method.
 */
void loop()
{
  if(!tcpSendBasicInfoPacket()) {
    Serial.println("TCP send failed, restarting...");
    delay(1000);
    ESP.restart();
  }

  counter++;
  Serial.println();
  delay(15000);
}
