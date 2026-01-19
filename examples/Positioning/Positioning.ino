/**
 * @file positioning.ino
 * @author Daan Pape <daan@dptechnics.com>
 * @author Arnoud Devoogdt <arnoud@dptechnics.com>
 * @date 16 January 2026
 * @version 1.5.0
 * @copyright DPTechnics bv <info@dptechnics.com>
 * @brief Walter Modem library examples
 *
 * @section LICENSE
 *
 * Copyright (C) 2026, DPTechnics bv
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
#define MODEM_SOCKET_ID 1

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
volatile bool gnss_fix_received = false;

/**
 * @brief Flag used to signal when an assistance update event is received.
 */
bool assistance_update_received = false;

/**
 * @brief The last received GNSS fix.
 */
WMGNSSFixEvent latestGnssFix = {};

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
 * @brief The network registration event handler.
 *
 * You can use this handler to get notified of network registration state changes. For this example,
 * we use polling to get the network registration state. You can use this to implement your own
 * reconnection logic.
 *
 * @note Make sure to keep this handler as lightweight as possible to avoid blocking the event
 * processing task.
 *
 * @param[out] state The network registration state.
 * @param[out] args User arguments.
 *
 * @return void
 */
static void myNetworkEventHandler(WalterModemNetworkRegState state, void* args)
{
  switch(state) {
  case WALTER_MODEM_NETWORK_REG_REGISTERED_HOME:
    Serial.println("Network registration: Registered (home)");
    break;

  case WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING:
    Serial.println("Network registration: Registered (roaming)");
    break;

  case WALTER_MODEM_NETWORK_REG_NOT_SEARCHING:
    Serial.println("Network registration: Not searching");
    break;

  case WALTER_MODEM_NETWORK_REG_SEARCHING:
    Serial.println("Network registration: Searching");
    break;

  case WALTER_MODEM_NETWORK_REG_DENIED:
    Serial.println("Network registration: Denied");
    break;

  case WALTER_MODEM_NETWORK_REG_UNKNOWN:
    Serial.println("Network registration: Unknown");
    break;

  default:
    break;
  }
}

/**
 * @brief The Socket event handler.
 *
 * This function will be called on various Socket events such as connection, disconnection,
 * ring, etc. You can modify this handler to implement your own logic based on the events received.
 *
 * @note Make sure to keep this handler as lightweight as possible to avoid blocking the event
 * processing task.
 *
 * @param[out] event The type of Socket event.
 * @param[out] data The data associated with the event.
 * @param[out] args User arguments.
 *
 * @return void
 */
static void mySocketEventHandler(WMSocketEventType event, WMSocketEventData data, void* args)
{
  switch(event) {
  case WALTER_MODEM_SOCKET_EVENT_DISCONNECTED:
    Serial.printf("SOCKET: Disconnected (id %d)\r\n", data.conn_id);
    break;

  case WALTER_MODEM_SOCKET_EVENT_RING:
    Serial.printf("SOCKET: Message received on socket %d (size: %lu)\r\n", data.conn_id,
                  data.data_len);

    /* Receive the HTTP message from the modem buffer */
    memset(in_buf, 0, sizeof(in_buf));
    if(modem.socketReceive(data.conn_id, in_buf, data.data_len)) {
      Serial.printf("Received message on socket %d: %s\r\n", data.conn_id, in_buf);
    } else {
      Serial.printf("Could not receive message for socket %d\r\n", data.conn_id);
    }
    break;

  default:
    break;
  }
}

/**
 * @brief GNSS event handler
 *
 * This function will be called on various GNSS events such as fix received or assistance update.
 *
 * @note Make sure to keep this handler as lightweight as possible to avoid blocking the event
 * processing task.
 *
 * @param[out] type The type of GNSS event.
 * @param[out] data The data associated with the GNSS event.
 * @param[out] args User argument pointer passed to gnssSetEventHandler
 *
 * @return None.
 */
void myGNSSEventHandler(WMGNSSEventType type, WMGNSSEventData data, void* args)
{
  uint8_t goodSatCount = 0;

  switch(type) {
  case WALTER_MODEM_GNSS_EVENT_FIX:
    memcpy(&latestGnssFix, &data.gnssfix, sizeof(WMGNSSFixEvent));

    /* Count satellites with good signal strength */
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
                  latestGnssFix.estimatedConfidence, latestGnssFix.latitude,
                  latestGnssFix.longitude, latestGnssFix.satCount, goodSatCount);

    gnss_fix_received = true;
    break;

  case WALTER_MODEM_GNSS_EVENT_ASSISTANCE:
    if(data.assistance == WALTER_MODEM_GNSS_ASSISTANCE_TYPE_ALMANAC) {
      Serial.println("GNSS Assistance: Almanac updated");
    } else if(data.assistance == WALTER_MODEM_GNSS_ASSISTANCE_TYPE_REALTIME_EPHEMERIS) {
      Serial.println("GNSS Assistance: Real-time ephemeris updated");
    } else if(data.assistance == WALTER_MODEM_GNSS_ASSISTANCE_TYPE_PREDICTED_EPHEMERIS) {
      Serial.println("GNSS Assistance: Predicted ephemeris updated");
    }

    assistance_update_received = true;
    break;

  default:
    break;
  }
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
 * @param[in] rsp Pointer to modem response object.
 * @param[out] updateAlmanac   Optional pointer to bool receiving almanac update.
 * @param[out] updateEphemeris Optional pointer to bool receiving ephemeris update.
 *
 * @return True if assistance status was successfully retrieved and parsed. False on error.
 */
bool checkAssistanceStatus(WalterModemRsp* rsp, bool* updateAlmanac = nullptr,
                           bool* updateEphemeris = nullptr)
{
  /* Request assistance status */
  if(!modem.gnssGetAssistanceStatus(rsp) ||
     rsp->type != WALTER_MODEM_RSP_DATA_TYPE_GNSS_ASSISTANCE_DATA) {
    Serial.println("Could not request GNSS assistance status");
    return false;
  }

  /* Default output flags */
  if(updateAlmanac)
    *updateAlmanac = false;
  if(updateEphemeris)
    *updateEphemeris = false;

  /* Helper lambda */
  auto report = [](const char* name, const WMGNSSAssistance& data, bool* updateFlag) {
    Serial.printf("%s data is ", name);

    if(data.available) {
      Serial.printf("available and should be updated within %lds\r\n", data.timeToUpdate);

      if(updateFlag)
        *updateFlag = (data.timeToUpdate <= 0);
    } else {
      Serial.println("not available.");

      if(updateFlag)
        *updateFlag = true;
    }
  };

  const auto& almanac = rsp->data.gnssAssistance[WALTER_MODEM_GNSS_ASSISTANCE_TYPE_ALMANAC];
  const auto& rtEph =
      rsp->data.gnssAssistance[WALTER_MODEM_GNSS_ASSISTANCE_TYPE_REALTIME_EPHEMERIS];

  report("Almanac", almanac, updateAlmanac);
  report("Real-time ephemeris", rtEph, updateEphemeris);
  return true;
}

/**
 * @brief Ensure the GNSS subsystem clock is valid, syncing with LTE if needed.
 *
 * If the clock is invalid, this function will attempt to connect to LTE
 * (if not already connected) and sync the clock up to 5 times.
 *
 * @param[in] rsp Pointer to modem response object.
 *
 * @return True if the clock is valid or successfully synchronized. False on error.
 */
bool validateGNSSClock(WalterModemRsp* rsp)
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
 * @param[in] rsp Pointer to modem response object.
 *
 * @return True if assistance data is valid (or successfully updated). False on error.
 */
bool updateGNSSAssistance(WalterModemRsp* rsp)
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
  assistance_update_received = false;
  if(updateAlmanac && !modem.gnssUpdateAssistance(WALTER_MODEM_GNSS_ASSISTANCE_TYPE_ALMANAC)) {
    Serial.println("Could not update almanac data");
    return false;
  }

  /* Wait for assistance update event */
  while(updateAlmanac && !assistance_update_received) {
    delay(200);
  }

  /* Update real-time ephemeris data if needed */
  assistance_update_received = false;
  if(updateEphemeris &&
     !modem.gnssUpdateAssistance(WALTER_MODEM_GNSS_ASSISTANCE_TYPE_REALTIME_EPHEMERIS)) {
    Serial.println("Could not update real-time ephemeris data");
    return false;
  }

  /* Wait for assistance update event */
  while(updateEphemeris && !assistance_update_received) {
    delay(200);
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
 * @return True f a valid GNSS fix was obtained within the confidence threshold. False if assistance
 * update fails, a fix cannot be requested, a timeout occurs, or the final confidence is too low.
 */
bool attemptGNSSFix()
{
  WalterModemRsp rsp = {};

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
  if(latestGnssFix.estimatedConfidence <= MAX_GNSS_CONFIDENCE &&
     latestGnssFix.estimatedConfidence > 0) {
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
    gnss_fix_received = false;

    /* Request a GNSS fix */
    if(!modem.gnssPerformAction()) {
      Serial.println("Error: Could not request GNSS fix");
      return false;
    }

    Serial.printf("Started GNSS fix (attempt %d/%d)\r\n", attempt + 1, maxAttempts);

    /* For this example, we block here until the GNSS event handler sets the flag */
    /* Feel free to build your application code asynchronously */
    while(!gnss_fix_received) {
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
 * @brief The main Arduino setup method.
 */
void setup()
{
  Serial.begin(115200);
  delay(2000);

  Serial.printf("\r\n\r\n=== WalterModem Positioning example (v1.5.0) ===\r\n\r\n");

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

  /* Set the network registration event handler (optional) */
  modem.setRegistrationEventHandler(myNetworkEventHandler, NULL);

  /* Set the Socket event handler */
  modem.setSocketEventHandler(mySocketEventHandler, NULL);

  /* Set the GNSS event handler */
  modem.setGNSSEventHandler(myGNSSEventHandler, NULL);

  WalterModemRsp rsp = {};

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
  if(modem.socketConfig(MODEM_SOCKET_ID)) {
    Serial.println("Successfully configured a new socket");
  } else {
    Serial.println("Error: Could not configure a new socket");
    return;
  }

  /* Disable TLS (the demo UDP server does not use it) */
  if(modem.socketConfigSecure(MODEM_SOCKET_ID, false)) {
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
  WalterModemRsp rsp = {};

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
  if(modem.socketDial(MODEM_SOCKET_ID, WALTER_MODEM_SOCKET_PROTO_UDP, SERV_PORT, SERV_ADDR)) {
    Serial.printf("Successfully dialed UDP server %s:%d\r\n", SERV_ADDR, SERV_PORT);
  } else {
    Serial.println("Error: Could not dial UDP server");
    return;
  }

  /* Transmit the packet */
  if(!modem.socketSend(MODEM_SOCKET_ID, out_buf, PACKET_SIZE)) {
    Serial.println("Error: Could not transmit data");
    delay(1000);
    ESP.restart();
    return;
  }

  delay(2000);

  /* Close the socket */
  if(!modem.socketClose(MODEM_SOCKET_ID)) {
    Serial.println("Error: Could not close the socket");
    delay(1000);
    ESP.restart();
    return;
  }

  lteDisconnect();
  delay(60000);
}
