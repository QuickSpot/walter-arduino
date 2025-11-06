/**
 * @file Positioning.ino
 * @author Daan Pape <daan@dptechnics.com> Arnoud Devoogdt <arnoud@dptechnics.com>
 * @date 17 Sep 2025
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
 * data (if available), gets a GNSS fix and uploads the position to the Walter demo server.
 * walterdemo.quickspot.io
 */

#include <WalterModem.h>
#include <inttypes.h>
#include <esp_mac.h>

/**
 * @brief The Socket profile to use (1..6)
 *
 * @note At least one socket should be available/reserved for BlueCherry.
 */
#define MODEM_SOCKET_PROFILE 1

/**
 * @brief The address of the server to upload the data to.
 */
#define SERV_ADDR "walterdemo.quickspot.io"

/**
 * @brief The UDP port on which the server is listening.
 */
#define SERV_PORT 1999

/**
 * @brief The size in bytes of a minimal sensor + GNSS + cellinfo packet.
 */
#define PACKET_SIZE 30

/**
 * @brief The APN of your cellular provider.
 *
 * @note If your SIM card supports it, you can also leave this empty.
 */
#define CELLULAR_APN ""

/**
 * @brief All fixes with a confidence below this number are considered ok.
 */
#define MAX_GNSS_CONFIDENCE 100.0

/**
 * @brief The radio access technology to use - LTEM or NBIOT.
 *
 * @note WALTER_MODEM_RAT_LTEM or WALTER_MODEM_RAT_NBIOT are recommended.
 */
#define RADIO_TECHNOLOGY WALTER_MODEM_RAT_LTEM

/**
 * @brief The modem instance.
 */
WalterModem modem;

/**
 * @brief Flag used to signal when a fix is received.
 */
volatile bool gnssFixRcvd = false;

/**
 * @brief The last received GNSS fix.
 */
WalterModemGNSSFix latestGnssFix = {};

/**
 * @brief The buffer to transmit to the UDP server. The first 6 bytes will be
 * the MAC address of the Walter this code is running on.
 */
uint8_t out_buf[PACKET_SIZE] = { 0 };

/**
 * @brief The buffer to receive from the UDP server.
 * @note Make sure this is sufficiently large enough for incoming data. (Up to 1500 bytes supported
 * by Sequans)
 */
uint8_t in_buf[1500] = { 0 };

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
  if(modem.definePDPContext(1, CELLULAR_APN)) {
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
 * @brief Inspect GNSS assistance status and optionally set update flags.
 *
 * Prints the availability and recommended update timing for the
 * almanac and real-time ephemeris databases.  If update flags are provided,
 * they are set to:
 *   - true  : update is required (data missing or time-to-update <= 0)
 *   - false : no update required
 *
 * @param rsp Pointer to modem response object.
 * @param updateAlmanac   Optional pointer to bool receiving almanac update.
 * @param updateEphemeris Optional pointer to bool receiving ephemeris update.
 *
 * @return true  If assistance status was successfully retrieved and parsed.
 * @return false If the assistance status could not be retrieved.
 */
bool checkAssistanceStatus(walter_modem_rsp_t* rsp, bool* updateAlmanac = nullptr,
                           bool* updateEphemeris = nullptr)
{
  /* Check assistance data status */
  if(!modem.gnssGetAssistanceStatus(rsp) ||
     rsp->type != WALTER_MODEM_RSP_DATA_TYPE_GNSS_ASSISTANCE_DATA) {
    Serial.println("Could not request GNSS assistance status");
    return false;
  }

  /* Default output flags to false if provided */
  if(updateAlmanac)
    *updateAlmanac = false;
  if(updateEphemeris)
    *updateEphemeris = false;

  /* Lambda to reduce repetition for each data type */
  auto report = [](const char* name, const auto& data, bool* updateFlag) {
    Serial.printf("%s data is ", name);
    if(data.available) {
      Serial.printf("available and should be updated within %ds\r\n", data.timeToUpdate);
      if(updateFlag)
        *updateFlag = (data.timeToUpdate <= 0);
    } else {
      Serial.println("not available.");
      if(updateFlag)
        *updateFlag = true;
    }
  };

  /* Check both data sets */
  report("Almanac", rsp->data.gnssAssistance.almanac, updateAlmanac);
  report("Real-time ephemeris", rsp->data.gnssAssistance.realtimeEphemeris, updateEphemeris);

  Serial.println("GNSS assistance data is up to date");
  return true;
}

/**
 * @brief Ensure the GNSS subsystem clock is valid, syncing with LTE if needed.
 *
 * If the clock is invalid, this function will attempt to connect to LTE
 * (if not already connected) and sync the clock up to 5 times.
 *
 * @param rsp Pointer to modem response object.
 *
 * @return true If the clock is valid or successfully synchronized.
 * @return false If synchronization fails or LTE connection fails.
 */
bool validateGNSSClock(walter_modem_rsp_t* rsp)
{
  /* Validate the GNSS subsystem clock */
  modem.gnssGetUTCTime(rsp);
  if(rsp->data.clock.epochTime > 4) {
    return true;
  }

  Serial.println("System clock invalid, LTE time sync required");

  /* Connect to LTE (required for time sync) */
  if(!lteConnected() && !lteConnect()) {
    Serial.println("Error: Could not connect to LTE network");
    return false;
  }

  /* Attempt sync clock up to 5 times */
  for(int i = 0; i < 5; ++i) {
    /* Validate the GNSS subsystem clock */
    modem.gnssGetUTCTime(rsp);
    if(rsp->data.clock.epochTime > 4) {
      Serial.printf("Clock synchronized: %" PRIi64 "\r\n", rsp->data.clock.epochTime);
      return true;
    }
    delay(2000);
  }

  Serial.println("Error: Could not sync time with network. Does the network support NITZ?");
  return false;
}

/**
 * @brief Update GNSS assistance data if required.
 *
 * Steps performed:
 *   1. Ensure the system clock is valid (sync with LTE if needed).
 *   2. Check the status of GNSS assistance data (almanac & ephemeris).
 *   3. Connect to LTE (if not already) and download any missing data.
 *
 * LTE is only connected when necessary.
 *
 * @param rsp Pointer to modem response object.
 *
 * @return true  Assistance data is valid (or successfully updated).
 * @return false Failure to sync time, connect LTE, or update assistance data.
 */
bool updateGNSSAssistance(walter_modem_rsp_t* rsp)
{
  bool updateAlmanac = false;
  bool updateEphemeris = false;

  /* Get the latest assistance data */
  if(!checkAssistanceStatus(rsp, &updateAlmanac, &updateEphemeris)) {
    Serial.println("Error: Could not check GNSS assistance status");
    return false;
  }

  /* No update needed */
  if(!updateAlmanac && !updateEphemeris) {
    return true;
  }

  /* Connect to LTE to download assistance data */
  if(!lteConnected() && !lteConnect()) {
    Serial.println("Could not connect to LTE network");
    return false;
  }

  /* Update almanac data if needed */
  if(updateAlmanac && !modem.gnssUpdateAssistance(WALTER_MODEM_GNSS_ASSISTANCE_TYPE_ALMANAC)) {
    Serial.println("Could not update almanac data");
    return false;
  }

  /* Update real-time ephemeris data if needed */
  if(updateEphemeris &&
     !modem.gnssUpdateAssistance(WALTER_MODEM_GNSS_ASSISTANCE_TYPE_REALTIME_EPHEMERIS)) {
    Serial.println("Could not update real-time ephemeris data");
    return false;
  }

  /* Recheck assistance data to ensure its valid */
  if(!checkAssistanceStatus(rsp)) {
    Serial.println("Error: Could not check GNSS assistance status");
    return false;
  }

  return true;
}

/**
 * @brief Attempt to obtain a GNSS position fix with acceptable confidence.
 *
 * This function:
 *   1. Updates GNSS assistance data if needed.
 *   2. Requests a GNSS fix up to 5 times.
 *   3. Waits for each fix attempt to complete or time out.
 *   4. Checks the final fix confidence against MAX_GNSS_CONFIDENCE.
 *
 * @return true  If a valid GNSS fix was obtained within the confidence threshold.
 * @return false If assistance update fails, a fix cannot be requested,
 *               a timeout occurs, or the final confidence is too low.
 */
bool attemptGNSSFix()
{
  walter_modem_rsp_t rsp = {};

  if(!validateGNSSClock(&rsp)) {
    Serial.println("Error: Could not validate GNSS clock");
    return false;
  }

  /* Ensure assistance data is current */
  if(!updateGNSSAssistance(&rsp)) {
    Serial.println(
        "Warning: Could not update GNSS assistance data. Continuing without assistance.");
  }

  /* Disconnect from the network (Required for GNSS) */
  if(lteConnected && !lteDisconnect()) {
    Serial.println("Error: Could not disconnect from the LTE network");
    return false;
  }

  /* Optional: Reconfigure GNSS with last valid fix - This might speed up consecutive fixes */
  if(latestGnssFix.estimatedConfidence <= MAX_GNSS_CONFIDENCE) {
    /* Reconfigure GNSS for potential quick fix */
    if(modem.gnssConfig(WALTER_MODEM_GNSS_SENS_MODE_HIGH, WALTER_MODEM_GNSS_ACQ_MODE_HOT_START)) {
      Serial.println("GNSS reconfigured for potential quick fix");
    } else {
      Serial.println("Error: Could not reconfigure GNSS");
    }
  }

  /* Attempt up to 5 GNSS fixes */
  const int maxAttempts = 5;
  for(int attempt = 0; attempt < maxAttempts; ++attempt) {
    gnssFixRcvd = false;

    /* Request a GNSS fix */
    if(!modem.gnssPerformAction()) {
      Serial.println("Error: Could not request GNSS fix");
      return false;
    }

    Serial.printf("Started GNSS fix (attempt %d/%d)\r\n", attempt + 1, maxAttempts);

    /* For this example, we block here until the GNSS event handler sets the flag */
    /* Feel free to build your application code asynchronously */
    while(!gnssFixRcvd) {
      Serial.print(".");
      delay(500);
    }
    Serial.println();

    /* If confidence is acceptable, stop trying. Otherwise, try again */
    if(latestGnssFix.estimatedConfidence <= MAX_GNSS_CONFIDENCE) {
      Serial.println("Successfully obtained a valid GNSS fix");
      return true;
    } else {
      Serial.printf("GNSS fix confidence %.02f too low, retrying...\r\n",
                    latestGnssFix.estimatedConfidence);
    }
  }
  return false;
}

/**
 * @brief GNSS event handler
 *
 * Handles GNSS fix events.
 * @note This callback is invoked from the modem driver’s event context.
 *       It must never block or call modem methods directly.
 *       Use it only to set flags or copy data for later processing.
 *
 * @param fix The fix data.
 * @param args User argument pointer passed to gnssSetEventHandler
 *
 * @return None.
 */
void gnssEventHandler(const WalterModemGNSSFix* fix, void* args)
{
  memcpy(&latestGnssFix, fix, sizeof(WalterModemGNSSFix));

  /* Count satellites with good signal strength */
  uint8_t goodSatCount = 0;
  for(int i = 0; i < latestGnssFix.satCount; ++i) {
    if(latestGnssFix.sats[i].signalStrength >= 30) {
      ++goodSatCount;
    }
  }
  Serial.println();
  Serial.printf("GNSS fix received:"
                "  Confidence: %.02f"
                "  Latitude: %.06f"
                "  Longitude: %.06f"
                "  Satcount: %d"
                "  Good sats: %d\r\n",
                latestGnssFix.estimatedConfidence, latestGnssFix.latitude, latestGnssFix.longitude,
                latestGnssFix.satCount, goodSatCount);

  gnssFixRcvd = true;
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
static void myURCHandler(const walter_modem_urc_event_t* ev, void* args)
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

  Serial.printf("\r\n\r\n=== WalterModem Positioning example ===\r\n\r\n");

  /* Get the MAC address for board validation */
  esp_read_mac(out_buf, ESP_MAC_WIFI_STA);
  Serial.printf("Walter's MAC is: %02X:%02X:%02X:%02X:%02X:%02X\r\n", out_buf[0], out_buf[1],
                out_buf[2], out_buf[3], out_buf[4], out_buf[5]);

  /* Start the modem */
  if(modem.begin(&Serial2)) {
    Serial.println("Successfully initialized the modem");
  } else {
    Serial.println("Error: Could not initialize the modem");
    delay(5000);
    ESP.restart();
  }

  /* Set the modem URC event handler */
  modem.urcSetEventHandler(myURCHandler, NULL);

  /* Set the GNSS fix event handler */
  /* This method will be merged with the URC handler in a future release */
  modem.gnssSetEventHandler(gnssEventHandler, NULL);

  walter_modem_rsp_t rsp = {};

  /* Ensure we are using the preferred RAT */
  /* This is a reboot-persistent setting */
  if(modem.getRAT(&rsp)) {
    if(rsp.data.rat != RADIO_TECHNOLOGY) {
      modem.setRAT(RADIO_TECHNOLOGY);
      Serial.println("Switched modem radio technology");
    }
  } else {
    Serial.println("Error: Could not retrieve radio access technology");
  }

  /* Print some modem information */
  if(modem.getIdentity(&rsp)) {
    Serial.println("Modem identity:");
    Serial.printf(" -IMEI: %s\r\n", rsp.data.identity.imei);
    Serial.printf(" -IMEISV: %s\r\n", rsp.data.identity.imeisv);
    Serial.printf(" -SVN: %s\r\n", rsp.data.identity.svn);
  }

  /* Get the SIM card ID */
  if(modem.getSIMCardID(&rsp)) {
    Serial.println("SIM card identity:");
    Serial.printf(" -ICCID: %s\r\n", rsp.data.simCardID.iccid);
    Serial.printf(" -eUICCID: %s\r\n", rsp.data.simCardID.euiccid);
  }

  /* Get the SIM card IMSI */
  if(modem.getSIMCardIMSI(&rsp)) {
    Serial.printf("Active IMSI: %s\r\n", rsp.data.imsi);
  }

  /* Configure the GNSS subsystem */
  if(!modem.gnssConfig()) {
    Serial.println("Error: Could not configure the GNSS subsystem");
    delay(1000);
    ESP.restart();
    return;
  }

  /* Configure a new socket */
  if(modem.socketConfig(MODEM_SOCKET_PROFILE)) {
    Serial.println("Successfully configured a new socket");
  } else {
    Serial.println("Error: Could not configure a new socket");
    return;
  }

  /* Disable TLS (the demo UDP server does not use it) */
  if(modem.socketConfigSecure(MODEM_SOCKET_PROFILE, false)) {
    Serial.println("Successfully set socket to insecure mode");
  } else {
    Serial.println("Error: Could not disable socket TLS");
    return;
  }
}

/**
 * @brief The main Arduino loop method.
 */
void loop()
{
  walter_modem_rsp_t rsp = {};

  attemptGNSSFix();

  /* Get the RAT */
  uint8_t rat = -1;
  if(modem.getRAT(&rsp)) {
    rat = (uint8_t) rsp.data.rat;
  }

  /* Force reconnect to the network to get the latest cell information */
  if(!lteConnect()) {
    Serial.println("Error: Could not connect to the LTE network");
    delay(1000);
    ESP.restart();
    return;
  }

  if(!modem.getCellInformation(WALTER_MODEM_SQNMONI_REPORTS_SERVING_CELL, &rsp)) {
    Serial.println("Error: Could not request cell information");
  } else {
    Serial.printf("Connected on band %u using operator %s (%u%02u)", rsp.data.cellInformation.band,
                  rsp.data.cellInformation.netName, rsp.data.cellInformation.cc,
                  rsp.data.cellInformation.nc);
    Serial.printf(" and cell ID %u.\r\n", rsp.data.cellInformation.cid);
    Serial.printf("Signal strength: RSRP: %.2f, RSRQ: %.2f.\r\n", rsp.data.cellInformation.rsrp,
                  rsp.data.cellInformation.rsrq);
  }

  /* Read the temperature of Walter */
  float temp = temperatureRead();
  Serial.printf("The temperature of Walter is %.02f degrees Celsius\r\n", temp);

  float lat32 = (float) latestGnssFix.latitude;
  float lon32 = (float) latestGnssFix.longitude;

  /* Construct the minimal sensor + GNSS + Cellinfo */
  uint16_t rawTemp = (temp + 50) * 100;
  out_buf[6] = 0x02;
  out_buf[7] = rawTemp >> 8;
  out_buf[8] = rawTemp & 0xFF;
  out_buf[9] = latestGnssFix.satCount;
  memcpy(out_buf + 10, &lat32, 4);
  memcpy(out_buf + 14, &lon32, 4);
  out_buf[18] = rsp.data.cellInformation.cc >> 8;
  out_buf[19] = rsp.data.cellInformation.cc & 0xFF;
  out_buf[20] = rsp.data.cellInformation.nc >> 8;
  out_buf[21] = rsp.data.cellInformation.nc & 0xFF;
  out_buf[22] = rsp.data.cellInformation.tac >> 8;
  out_buf[23] = rsp.data.cellInformation.tac & 0xFF;
  out_buf[24] = (rsp.data.cellInformation.cid >> 24) & 0xFF;
  out_buf[25] = (rsp.data.cellInformation.cid >> 16) & 0xFF;
  out_buf[26] = (rsp.data.cellInformation.cid >> 8) & 0xFF;
  out_buf[27] = rsp.data.cellInformation.cid & 0xFF;
  out_buf[28] = (uint8_t) (rsp.data.cellInformation.rsrp * -1);
  out_buf[29] = rat;

  /* Connect (dial) to the WalterDemo server */
  if(modem.socketDial(MODEM_SOCKET_PROFILE, WALTER_MODEM_SOCKET_PROTO_UDP, SERV_PORT, SERV_ADDR)) {
    Serial.printf("Successfully dialed UDP server %s:%d\r\n", SERV_ADDR, SERV_PORT);
  } else {
    Serial.println("Error: Could not dial UDP server");
    return;
  }

  /* Transmit the packet */
  if(!modem.socketSend(MODEM_SOCKET_PROFILE, out_buf, PACKET_SIZE)) {
    Serial.println("Error: Could not transmit data");
    delay(1000);
    ESP.restart();
    return;
  }

  delay(2000);

  /* Close the socket */
  if(!modem.socketClose(MODEM_SOCKET_PROFILE)) {
    Serial.println("Error: Could not close the socket");
    delay(1000);
    ESP.restart();
    return;
  }

  lteDisconnect();
  delay(60000);
}
