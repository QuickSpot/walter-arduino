/**
 * @file udp.ino
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

#define UDP_PORT 1999
#define UDP_HOST "walterdemo.quickspot.io"

#define BASIC_INFO_PACKET_SIZE 24

/**
 * @brief The modem instance.
 */
WalterModem modem;

/**
 * @brief Response object containing command response information.
 */
WalterModemRsp rsp = {};

/**
 * @brief The socket identifier (1-6)
 *
 * @note At least one socket should be available/reserved for BlueCherry.
 */
uint8_t socketId = -1;

/**
 * @brief The buffer to transmit to the UDP server. The first 6 bytes will be
 * the MAC address of the Walter this code is running on.
 */
uint8_t dataBuf[8] = { 0 };

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
 * @brief Socket event handler
 *
 * Handles status changes and incoming UDP messages.
 * @note This callback is invoked from the modem driver’s event context.
 *       It must never block or call modem methods directly.
 *       Use it only to set flags or copy data for later processing.
 *
 * @param ev          Event type (e.g. WALTER_MODEM_SOCKET_EVENT_RING for incoming messages)
 * @param socketId    ID of the socket that triggered the event
 * @param dataReceived Number of bytes received
 * @param dataBuffer  Pointer to received data
 * @param args        User argument pointer passed to socketSetEventHandler
 */
void udpSocketEventHandler(WalterModemSocketEvent ev, int socketId, uint16_t dataReceived,
                           uint8_t* dataBuffer, void* args)
{
  if(ev == WALTER_MODEM_SOCKET_EVENT_RING) {
    Serial.printf("Received UDP message (%u bytes) on socket %d\r\n", dataReceived, socketId);
    Serial.printf("Payload:\r\n%.*s\r\n", dataReceived, reinterpret_cast<const char*>(dataBuffer));
  }
}

/**
 * @brief Send a basic info packet to walterdemo
 */
bool udpSendBasicInfoPacket()
{
  /* Read the temperature of Walter */
  float temp = temperatureRead();
  uint16_t rawTemp = (temp + 50) * 100;

  uint8_t rat = -1;
  if(modem.getRAT(&rsp)) {
    rat = (uint8_t) rsp.data.rat;
  }

  /* Construct the Basic info Packet */
  dataBuf[6] = rawTemp >> 8;
  dataBuf[7] = rawTemp & 0xFF;
  dataBuf[8] = counter >> 8;
  dataBuf[9] = counter & 0xFF;
  dataBuf[10] = rsp.data.cellInformation.cc >> 8;
  dataBuf[11] = rsp.data.cellInformation.cc & 0xFF;
  dataBuf[12] = rsp.data.cellInformation.nc >> 8;
  dataBuf[13] = rsp.data.cellInformation.nc & 0xFF;
  dataBuf[14] = rsp.data.cellInformation.tac >> 8;
  dataBuf[15] = rsp.data.cellInformation.tac & 0xFF;
  dataBuf[16] = (rsp.data.cellInformation.cid >> 24) & 0xFF;
  dataBuf[17] = (rsp.data.cellInformation.cid >> 16) & 0xFF;
  dataBuf[18] = (rsp.data.cellInformation.cid >> 8) & 0xFF;
  dataBuf[19] = rsp.data.cellInformation.cid & 0xFF;
  dataBuf[20] = (uint8_t) (rsp.data.cellInformation.rsrp * -1);
  dataBuf[21] = (uint8_t) (rsp.data.cellInformation.rsrq * -1);
  dataBuf[22] = rat;
  dataBuf[23] = 0xFF;

  Serial.println("Sending basic info packet");

  if(!modem.socketSend(dataBuf, BASIC_INFO_PACKET_SIZE)) {
    Serial.println("Error: UDP send basic info packet failed");
    return false;
  }

  /* Attempt to get the latest cell information (for next packet) */
  modem.getCellInformation(WALTER_MODEM_SQNMONI_REPORTS_SERVING_CELL, &rsp);

  Serial.println("UDP send basic info packet succeeded");
  return true;
}

/**
 * @brief The main Arduino setup method.
 */
void setup()
{
  Serial.begin(115200);
  delay(5000);

  Serial.printf("\r\n\r\n=== WalterModem UDP example ===\r\n\r\n");

  /* Start the modem */
  if(WalterModem::begin(&Serial2)) {
    Serial.println("Successfully initialized the modem");
  } else {
    Serial.println("Error: Could not initialize the modem");
    return;
  }

  /* Connect the modem to the LTE network */
  if(!lteConnect()) {
    Serial.println("Error: Could not connect to LTE");
    return;
  }

  /* Retrieve and print the board MAC address */
  esp_read_mac(dataBuf, ESP_MAC_WIFI_STA);
  Serial.printf("Board MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n", dataBuf[0], dataBuf[1], dataBuf[2],
                dataBuf[3], dataBuf[4], dataBuf[5]);

  /* Set the UDP socket event handler */
  modem.socketSetEventHandler(udpSocketEventHandler, NULL);

  /* Configure a new socket */
  if(modem.socketConfig(&rsp)) {
    Serial.println("Successfully configured a new socket");

    /* Utilize the socket id if you have more then one socket */
    /* If not specified in the methods, the modem will use the previous socket id */
    socketId = rsp.data.socketId;
  } else {
    Serial.println("Error: Could not configure a new socket");
    return;
  }

  /* Disable TLS (the demo UDP server does not use it) */
  if(modem.socketConfigSecure(false)) {
    Serial.println("Successfully set socket to insecure mode");
  } else {
    Serial.println("Error: Could not disable socket TLS");
    return;
  }

  /* Connect (dial) to the UDP test server */
  if(modem.socketDial(UDP_HOST, UDP_PORT, 0, NULL, NULL, NULL, WALTER_MODEM_SOCKET_PROTO_UDP)) {
    Serial.printf("Successfully dialed UDP server %s:%d\r\n", UDP_HOST, UDP_PORT);
  } else {
    Serial.println("Error: Could not dial UDP server");
    return;
  }
}

/**
 * @brief The main Arduino loop method.
 */
void loop()
{
  static unsigned long lastSend = 0;
  const unsigned long sendInterval = 30000;

  if(millis() - lastSend >= sendInterval) {
    lastSend = millis();

    if(!udpSendBasicInfoPacket()) {
      Serial.println("UDP send failed, restarting...");
      delay(1000);
      ESP.restart();
    }
    counter++;
    Serial.println();
  }
}
