/**
 * @file Positioning.ino
 * @author Daan Pape <daan@dptechnics.com>
 * @date 2 Sep 2024
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
 * This file contains a sketch which connects to LTE to download GNSS assistance
 * data, gets a GNSS fix and uploads the position to the Walter demo server.
 */

#include <inttypes.h>
#include <esp_mac.h>
#include <WalterModem.h>

/**
 * @brief The address of the server to upload the data to. 
 */
#define SERV_ADDR "64.225.64.140"

/**
 * @brief The UDP port on which the server is listening.
 */
#define SERV_PORT 1999

/**
 * @brief The size in bytes of a minimal sensor + GNSS + cell info packet.
 */
#define PACKET_SIZE 29

/**
 * @brief All fixes with a confidence below this number are considered ok.
 */
#define MAX_GNSS_CONFIDENCE 100.0

/**
 * @brief The serial interface to talk to the modem.
 */
#define ModemSerial Serial2

/**
 * @brief The radio access technology to use - LTEM or NBIOT.
 */
#define RADIO_TECHNOLOGY WALTER_MODEM_RAT_LTEM

/**
 * @brief The modem instance.
 */
WalterModem modem;

/**
 * @brief Flag used to signal when a fix is received.
 */
volatile bool fixRcvd = false;

/**
 * @brief The last received GNSS fix.
 */
WalterModemGNSSFix posFix = {};

/**
 * @brief The buffer to transmit to the UDP server. The first 6 bytes will be
 * the MAC address of the Walter this code is running on.
 */
uint8_t dataBuf[PACKET_SIZE] = { 0 };

/**
 * @brief Connect to the LTE network.
 * 
 * This function will connect the modem to the LTE network. This function will
 * block until the modem is attached.
 * 
 * @return True on success, false on error.
 */
bool lteConnect()
{
  /* Set the operational state to full */
  if(!modem.setOpState(WALTER_MODEM_OPSTATE_FULL)) {
    Serial.print("Could not set operational state to FULL\r\n");
    return false;
  }

  /* Set the network operator selection to automatic */
  if(!modem.setNetworkSelectionMode(WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC)) {
    Serial.print("Could not set the network selection mode to automatic\r\n");
    return false;
  }

  /* Wait for the network to become available */
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  while(!(regState == WALTER_MODEM_NETWORK_REG_REGISTERED_HOME ||
          regState == WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING))
  {
    delay(1000);
    regState = modem.getNetworkRegState();
  }

  /* Stabilization time */
  Serial.print("Connected to the network\r\n");
  return true;
}

/**
 * @brief Disconnect from the LTE network.
 * 
 * This function will disconnect the modem from the LTE network and block until
 * the network is actually disconnected. After the network is disconnected the
 * GNSS subsystem can be used.
 * 
 * @return True on success, false on error.
 */
bool lteDisconnect()
{
  /* Set the operational state to minimum */
  if(!modem.setOpState(WALTER_MODEM_OPSTATE_MINIMUM)) {
    Serial.print("Could not set operational state to MINIMUM\r\n");
    return false;
  }

  /* Wait for the network to become available */
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  while(regState != WALTER_MODEM_NETWORK_REG_NOT_SEARCHING)
  {
    delay(100);
    regState = modem.getNetworkRegState();
  }

  Serial.print("Disconnected from the network\r\n");
  return true;
}

/**
 * @brief Check the assistance data in the modem response.
 * 
 * This function checks the availability of assistance data in the modem's 
 * response. This function also sets a flag if any of the assistance databases
 * should be updated.
 * 
 * @param rsp The modem response to check.
 * @param updateAlmanac Pointer to the flag to set when the almanac should be
 * updated. 
 * @param updateEphemeris Pointer to the flag to set when ephemeris should be
 * updated.
 * 
 * @return None.
 */
void checkAssistanceData(
  WalterModemRsp *rsp,
  bool *updateAlmanac = NULL,
  bool *updateEphemeris = NULL)
{
  if(updateAlmanac != NULL) {
    *updateAlmanac = false;
  }

  if(updateEphemeris != NULL) {
    *updateEphemeris = false;
  }

  Serial.print("Almanac data is ");
  if(rsp->data.gnssAssistance.almanac.available) {
    Serial.printf("available and should be updated within %ds\r\n",
      rsp->data.gnssAssistance.almanac.timeToUpdate);
    if(updateAlmanac != NULL) {
      *updateAlmanac = rsp->data.gnssAssistance.almanac.timeToUpdate <= 0;
    }
  } else {
    Serial.print("not available.\r\n");
    if(updateAlmanac != NULL) {
      *updateAlmanac = true;
    }
  }

  Serial.print("Real-time ephemeris data is ");
  if(rsp->data.gnssAssistance.realtimeEphemeris.available) {
    Serial.printf("available and should be updated within %ds\r\n",
      rsp->data.gnssAssistance.realtimeEphemeris.timeToUpdate);
    if(updateEphemeris != NULL) {
      *updateEphemeris = rsp->data.gnssAssistance.realtimeEphemeris.timeToUpdate <= 0;
    }
  } else {
    Serial.print("not available.\r\n");
    if(updateEphemeris != NULL) {
      *updateEphemeris = true;
    }
  }
}

/**
 * @brief This function will update GNSS assistance data when needed.
 * 
 * This funtion will check if the current real-time ephemeris data is good
 * enough to get a fast GNSS fix. If not the function will attach to the LTE 
 * network to download newer assistance data.
 * 
 * @return True on success, false on error.
 */
bool updateGNSSAssistance()
{
  bool lteConnected = false;
  WalterModemRsp rsp = {};

  lteDisconnect();

  /* Even with valid assistance data the system clock could be invalid */
  modem.gnssGetUTCTime(&rsp);

  if(rsp.data.clock.epochTime <= 4) {
    /* The system clock is invalid, connect to LTE network to sync time */
    if(!lteConnect()) {
      Serial.print("Could not connect to LTE network\r\n");
      return false;
    }

    lteConnected = true;

    /* 
     * Wait for the modem to synchronize time with the LTE network, try 5 times
     * with a delay of 500ms.
     */
    for(int i = 0; i < 5; ++i) {
      modem.gnssGetUTCTime(&rsp);

      if(rsp.data.clock.epochTime > 4) {
        Serial.printf("Synchronized clock with network: %"PRIi64"\r\n",
          rsp.data.clock.epochTime);
        break;
      } else if(i == 4) {
        Serial.print("Could not sync time with network\r\n");
        return false;
      }

      delay(500);
    }
  }

  /* Check the availability of assistance data */
  if(!modem.gnssGetAssistanceStatus(&rsp) ||
     rsp.type != WALTER_MODEM_RSP_DATA_TYPE_GNSS_ASSISTANCE_DATA)
  {
    Serial.print("Could not request GNSS assistance status\r\n");
    return false;
  }

  bool updateAlmanac = false;
  bool updateEphemeris = false;
  checkAssistanceData(&rsp, &updateAlmanac, &updateEphemeris);

  if(!(updateAlmanac || updateEphemeris)) {
    if(lteConnected) {
      if(!lteDisconnect()) {
        Serial.print("Could not disconnect from the LTE network\r\n");
        return false;
      }
    }

    return true;
  }

  if(!lteConnected) {
    if(!lteConnect()) {
      Serial.print("Could not connect to LTE network\r\n");
      return false;
    }
  }

  if(updateAlmanac) {
    if(!modem.gnssUpdateAssistance(WALTER_MODEM_GNSS_ASSISTANCE_TYPE_ALMANAC)) {
      Serial.print("Could not update almanac data\r\n");
      return false;
    }
  }

  if(updateEphemeris) {
    if(!modem.gnssUpdateAssistance(
      WALTER_MODEM_GNSS_ASSISTANCE_TYPE_REALTIME_EPHEMERIS))
    {
      Serial.print("Could not update real-time ephemeris data\r\n");
      return false;
    }
  }

  if(!modem.gnssGetAssistanceStatus(&rsp) ||
    rsp.type != WALTER_MODEM_RSP_DATA_TYPE_GNSS_ASSISTANCE_DATA)
  {
    Serial.print("Could not request GNSS assistance status\r\n");
    return false;
  }

  checkAssistanceData(&rsp);
  
  if(!lteDisconnect()) {
    Serial.print("Could not disconnect from the LTE network\r\n");
    return false;
  }

  return true;
}

/**
 * @brief Connect to an UDP socket.
 * 
 * This function will set-up the modem and connect an UDP socket. The LTE 
 * connection must be active before this function can be called.
 * 
 * @param ip The IP address of the server to connect to.
 * @param port The port to connect to.
 * 
 * @return True on success, false on error.
 */
bool socketConnect(const char *ip, uint16_t port)
{
  /* Construct a socket */
  if(modem.socketConfig()) {
    Serial.print("Created a new socket\r\n");
  } else {
    Serial.print("Could not create a new socket\r\n");
    return false;
  }

  /* disable socket tls as the demo server does not use it */
  if(modem.socketConfigSecure(false)) {
    Serial.print("Configured TLS\r\n");
  } else {
    Serial.print("Could not configure TLS\r\n");
    return false;
  }

  /* Connect to the UDP test server */
  if(modem.socketDial(ip, port)) {
    Serial.printf("Connected to UDP server %s:%d\r\n", ip, port);
  } else {
    Serial.print("Could not connect UDP socket\r\n");
    return false;
  }

  return true;
}

/**
 * @brief This function is called when a fix attempt finished.
 * 
 * This function is called by Walter's modem library as soon as a fix attempt
 * has finished. This function should be handled as an interrupt and should be
 * as short as possible as it is called within the modem data thread.
 * 
 * @param fix The fix data.
 * @param args Optional arguments, a NULL pointer in this case.
 * 
 * @return None.
 */
void fixHandler(const WalterModemGNSSFix *fix, void *args)
{
  memcpy(&posFix, fix, sizeof(WalterModemGNSSFix));
  fixRcvd = true;
}

/**
 * @brief Set up the system.
 * 
 * This function will set up the system and initialize the modem.
 * 
 * @return None.
 */
void setup()
{
  Serial.begin(115200);
  delay(5000);

  Serial.print("Walter Positioning v0.0.2\r\n");

  /* Get the MAC address for board validation */
  esp_read_mac(dataBuf, ESP_MAC_WIFI_STA);
  Serial.printf("Walter's MAC is: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
    dataBuf[0],
    dataBuf[1],
    dataBuf[2],
    dataBuf[3],
    dataBuf[4],
    dataBuf[5]);

  /* Modem initialization */
  if(modem.begin(&ModemSerial)) {
    Serial.print("Modem initialization OK\r\n");
  } else {
    Serial.print("Modem initialization ERROR\r\n");
    delay(1000);
    ESP.restart();
    return;
  }

  WalterModemRsp rsp = {};
  if(modem.getIdentity(&rsp)) {
    Serial.print("Modem identity:\r\n");
    Serial.printf(" -IMEI: %s\r\n", rsp.data.identity.imei);
    Serial.printf(" -IMEISV: %s\r\n", rsp.data.identity.imeisv);
    Serial.printf(" -SVN: %s\r\n", rsp.data.identity.svn);
  }

  if(modem.getRAT(&rsp)) {
    if(rsp.data.rat != RADIO_TECHNOLOGY) {
      modem.setRAT(RADIO_TECHNOLOGY);
      Serial.println("Switched modem radio technology");
    }
  } else {
    Serial.println("Could not retrieve radio access technology");
  }

  if(!modem.definePDPContext()) {
    Serial.println("Could not create PDP context");
    delay(1000);
    ESP.restart();
    return;
  }

  if(!modem.setOpState(WALTER_MODEM_OPSTATE_NO_RF)) {
    Serial.print("Could not set operational state to NO RF\r\n");
    delay(1000);
    ESP.restart();
    return;
  }

  delay(500);

  if(modem.getSIMCardID(&rsp)) {
    Serial.print("SIM card identity:\r\n");
    Serial.printf(" -ICCID: %s\r\n", rsp.data.simCardID.iccid);
    Serial.printf(" -eUICCID: %s\r\n", rsp.data.simCardID.euiccid);
  }

  if(modem.getSIMCardIMSI(&rsp)) {
    Serial.printf("Active IMSI: %s\r\n", rsp.data.imsi);
  }

  if(!modem.gnssConfig()) {
    Serial.println("Could not configure the GNSS subsystem");
    delay(1000);
    ESP.restart();
    return;
  }

  modem.gnssSetEventHandler(fixHandler);
}

void loop()
{
  static WalterModemRsp rsp = {};

  /* Check clock and assistance data, update if required */
  if(!updateGNSSAssistance()) {
    Serial.print("Could not update GNSS assistance data\r\n");
    delay(1000);
    ESP.restart();
    return;
  }

  /* Try up to 5 times to get a good fix */
  for(int i = 0; i < 5; ++i) {
    fixRcvd = false;
    if(!modem.gnssPerformAction()) {
      Serial.print("Could not request GNSS fix\r\n");
      delay(1000);
      ESP.restart();
      return;
    }
    Serial.print("Started GNSS fix\r\n");

    int j = 0;
    while(!fixRcvd) {
        Serial.print(".");
        if (j >= 300) {
            Serial.println("");
            Serial.println("Timed out while waiting for GNSS fix");
            delay(1000);
            ESP.restart();
            break;
        }
        j++;
        delay(500);
    }
    
    Serial.println("");

    if(posFix.estimatedConfidence <= MAX_GNSS_CONFIDENCE) {
      break;
    }
  }

  uint8_t abovedBTreshold = 0;
  for(int i = 0; i < posFix.satCount; ++i) {
    if(posFix.sats[i].signalStrength >= 30) {
      abovedBTreshold += 1;
    }
  }

  Serial.printf("GNSS fix attempt finished:\r\n"
    "  Confidence: %.02f\r\n"
    "  Latitude: %.06f\r\n"
    "  Longitude: %.06f\r\n"
    "  Satcount: %d\r\n"
    "  Good sats: %d\r\n",
    posFix.estimatedConfidence,
    posFix.latitude,
    posFix.longitude,
    posFix.satCount,
    abovedBTreshold);

  /* Read the temperature of Walter */
  float temp = temperatureRead();
  Serial.printf("The temperature of Walter is %.02f degrees Celsius\r\n", temp);

  float lat = posFix.latitude;
  float lon = posFix.longitude;
  
  if(posFix.estimatedConfidence > MAX_GNSS_CONFIDENCE) {
    posFix.satCount = 0xFF;
    lat = 0.0;
    lon = 0.0;
    Serial.print("Could not get a valid fix\r\n");
  }

  /* Construct the minimal sensor + GNSS */
  uint16_t rawTemp = (temp + 50) * 100;
  dataBuf[6] = 0x02;
  dataBuf[7] = rawTemp >> 8;
  dataBuf[8] = rawTemp & 0xFF;
  dataBuf[9] = posFix.satCount;
  memcpy(dataBuf + 10, &lat, 4);
  memcpy(dataBuf + 14, &lon, 4);

  /* Transmit the packet */
  if(!lteConnect()) {
    Serial.print("Could not connect to the LTE network\r\n");
    delay(1000);
    ESP.restart();
    return;
  }

  if(!modem.getCellInformation(WALTER_MODEM_SQNMONI_REPORTS_SERVING_CELL, &rsp)) {
    Serial.println("Could not request cell information");
  } else {
    Serial.printf("Connected on band %u using operator %s (%u%02u)",
      rsp.data.cellInformation.band, rsp.data.cellInformation.netName,
      rsp.data.cellInformation.cc, rsp.data.cellInformation.nc);
    Serial.printf(" and cell ID %u.\r\n",
      rsp.data.cellInformation.cid);
    Serial.printf("Signal strength: RSRP: %.2f, RSRQ: %.2f.\r\n",
      rsp.data.cellInformation.rsrp, rsp.data.cellInformation.rsrq);
  }

  dataBuf[18] = rsp.data.cellInformation.cc >> 8;
  dataBuf[19] = rsp.data.cellInformation.cc & 0xFF;
  dataBuf[20] = rsp.data.cellInformation.nc >> 8;
  dataBuf[21] = rsp.data.cellInformation.nc & 0xFF;
  dataBuf[22] = rsp.data.cellInformation.tac >> 8;
  dataBuf[23] = rsp.data.cellInformation.tac & 0xFF;
  dataBuf[24] = (rsp.data.cellInformation.cid >> 24) & 0xFF;
  dataBuf[25] = (rsp.data.cellInformation.cid >> 16) & 0xFF;
  dataBuf[26] = (rsp.data.cellInformation.cid >> 8) & 0xFF;
  dataBuf[27] = rsp.data.cellInformation.cid & 0xFF;
  dataBuf[28] = (uint8_t) (rsp.data.cellInformation.rsrp * -1);

  if(!socketConnect(SERV_ADDR, SERV_PORT)) {
    Serial.print("Could not connect to UDP server socket\r\n");
    delay(1000);
    ESP.restart();
    return;
  }

  if(!modem.socketSend(dataBuf, PACKET_SIZE)) {
    Serial.print("Could not transmit data\r\n");
    delay(1000);
    ESP.restart();
    return;
  }

  delay(5000);

  if(!modem.socketClose()) {
    Serial.print("Could not close the socket\r\n");
    delay(1000);
    ESP.restart();
    return;
  }

  lteDisconnect();
}
