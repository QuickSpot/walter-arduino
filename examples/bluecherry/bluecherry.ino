/**
 * @file bluecherry.ino
 * @author Jonas Maes <jonas@dptechnics.com>
 * @author Arnoud Devoogdt <arnoud@dptechnics.com>
 * @date 21 September 2026
 * @version 1.5.1
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
 * This sketch sends and receives mqtt data using the DPTechnics BlueCherry cloud
 * platform. It also supports OTA updates which are scheduled through the BlueCherry web interface.
 */

#include <WalterBlueCherry.h>
#include <Arduino.h>
#include <esp_heap_caps.h>

// The cellular Access Point Name
// Leave blank for autodetection
#define CELLULAR_APN ""

// The BlueCherry device type this firmware belongs to, used for Zero-Touch Provisioning
#define BC_DEVICE_TYPE "walter01"

// The modem TLS profile BlueCherry may use
#define BC_TLS_PROFILE 1

// The size of the buffer holding messages waiting to be published
#define BC_PUBLISH_BUFFER_SIZE 4096

/**
 * @brief The modem instance.
 */
WalterModem modem;

/**
 * @brief The BlueCherry cloud client.
 */
WalterBlueCherry bc;

/**
 * @brief Flag used to signal when BlueCherry has nothing left to do.
 */
volatile bool bc_synchronized = false;

/**
 * @brief The binary configuration settings for PSM.
 * These can be calculated using e.g.
 * https://www.soracom.io/psm-calculation-tool/
 */
const char* psmActive = "00000001";
const char* psmTAU = "00000110";

/**
 * @brief The binary configuration settings for eDRX.
 */
const char* edrxValue = "1101";
const char* edrxPagingTimeWindow = "0000";

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
  /* Configure power saving mode */
  if(modem.configPSM(WALTER_MODEM_PSM_ENABLE, psmTAU, psmActive)) {
    Serial.println("Successfully configured PSM");
  } else {
    Serial.println("Error: Could not configure PSM");
  }

  /* Configure eDRX */
  if(modem.configEDRX(WALTER_MODEM_EDRX_ENABLE_WITH_RESULT, edrxValue, edrxPagingTimeWindow)) {
    Serial.println("Successfully configured eDRX");
  } else {
    Serial.println("Error: Could not configure eDRX");
  }

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

/** * @brief The network registration event handler.
 *
 * This function will be called when network registration state changes or when
 * eDRX parameters are received from the network.
 *
 * @note Make sure to keep this handler as lightweight as possible to avoid blocking
 * the event processing task.
 *
 * @param[out] event The network registration state event.
 * @param[out] data The registration event data including state and PSM info.
 * @param[out] args User arguments.
 *
 * @return void
 */
static void myNetworkEventHandler(WMNetworkEventType event, const WMNetworkEventData* data,
                                  void* args)
{
  if(event == WALTER_MODEM_NETWORK_EVENT_REG_STATE_CHANGE) {
    switch(data->cereg.state) {
    case WALTER_MODEM_NETWORK_REG_REGISTERED_HOME:
      Serial.println("Network registration: Registered (home)");
      if(data->cereg.hasPsmInfo) {
        Serial.printf("PSM Active Time: %s, TAU: %s\r\n", data->cereg.activeTime,
                      data->cereg.periodicTau);
      }
      break;

    case WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING:
      Serial.println("Network registration: Registered (roaming)");
      if(data->cereg.hasPsmInfo) {
        Serial.printf("PSM Active Time: %s, TAU: %s\r\n", data->cereg.activeTime,
                      data->cereg.periodicTau);
      }
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
  } else if(event == WALTER_MODEM_NETWORK_EVENT_EDRX_RECEIVED) {
    Serial.printf(
        "Network event: eDRX received (ACT: %d) Requested: %s, NW-Provided: %s, PTW: %s\r\n",
        data->edrx.actType, data->edrx.requestedEdrx, data->edrx.nwProvidedEdrx,
        data->edrx.pagingTimeWindow);
  }
}

/**
 * @brief Handle a message the cloud sent down.
 *
 * Runs on the BlueCherry synchronisation task, so it must not block. The payload is a pointer into
 * the receive buffer, so copy anything that has to outlive the call.
 *
 * @param topic The single byte topic index the cloud maps to an MQTT topic.
 * @param len The number of bytes in data.
 * @param data The payload, valid only for the duration of the call.
 * @param args User arguments.
 *
 * @return void
 */
void myMessageHandler(uint8_t topic, uint16_t len, const uint8_t* data, void* args)
{
  Serial.printf("Incoming message on topic 0x%02x (%u bytes)\r\n", topic, len);
  Serial.printf("%.*s\r\n", len, (const char*) data);
}

/**
 * @brief Handle a firmware update event, taking both update decisions in the application.
 *
 * Runs on the BlueCherry synchronisation task, so it must not block.
 *
 * Two events carry a decision: AVAILABLE, where otaStart accepts the offer, and COMPLETE, where
 * the image is installed and only the restart is left. Returning false hands either back to the
 * library, which then downloads and restarts on its own. The other three are notifications.
 *
 * @param event The event that occurred.
 * @param info Details for the event, valid only for the duration of the call.
 * @param args User arguments.
 *
 * @return True when this handler took the event's decision.
 */
bool myOtaHandler(WalterModemBlueCherryOtaEvent event, const WalterModemBlueCherryOtaInfo* info,
                  void* args)
{
  switch(event) {
  case BLUECHERRY_OTA_EVENT_AVAILABLE:
    Serial.printf("OTA: firmware v%d available (%lu bytes), accepting\r\n", info->version,
                  (unsigned long) info->size);
    bc.otaStart();
    return true;

  case BLUECHERRY_OTA_EVENT_STARTED:
    Serial.printf("OTA: downloading firmware v%d\r\n", info->version);
    break;

  case BLUECHERRY_OTA_EVENT_PROGRESS:
    Serial.printf("OTA: %lu / %lu bytes\r\n", (unsigned long) info->bytes_received,
                  (unsigned long) info->size);
    break;

  case BLUECHERRY_OTA_EVENT_COMPLETE:
    Serial.printf("OTA: firmware v%d installed, restarting\r\n", info->version);
    Serial.flush();
    ESP.restart();
    return true;

  case BLUECHERRY_OTA_EVENT_FAILED:
    Serial.printf("OTA: firmware v%d failed with error %u\r\n", info->version, info->error_code);
    break;
  }

  return false;
}

/**
 * @brief Report what the BlueCherry connection is doing.
 *
 * Runs on the BlueCherry synchronisation task, so it must not block. Publish and sync report the
 * transitions they make themselves, so it can also run on the calling task. IDLE is the only state
 * in which it is safe to sleep.
 *
 * @param state The state that was just entered.
 * @param args User arguments.
 *
 * @return void
 */
void myStateHandler(WalterModemBlueCherryState state, void* args)
{
  switch(state) {
  case BLUECHERRY_STATE_NOT_PROVISIONED:
    Serial.println("BlueCherry holds no credentials yet, provisioning...");
    break;

  case BLUECHERRY_STATE_AWAIT_CONNECTION:
    Serial.println("BlueCherry is connecting...");
    break;

  case BLUECHERRY_STATE_IDLE:
    Serial.println("Synchronized with the BlueCherry cloud platform");
    bc_synchronized = true;
    break;

  default:
    break;
  }
}

/**
 * @brief Give BlueCherry its buffers and handlers.
 *
 * Runs on every boot: it resumes a session that survived deep sleep and re-registers the handlers
 * and buffers, which do not survive one. Talks to the modem, never to the cloud, so it needs no
 * network and there is nothing to retry.
 *
 * @return True when BlueCherry is ready to be published to.
 */
static bool initializeBlueCherry()
{
  /* PSRAM, falling back to a library-allocated one in internal RAM. This descriptor is read during
   * init and not kept, so only the buffer it points at has to outlive the call. PSRAM has to be
   * enabled in the board menu ("PSRAM: QSPI PSRAM") for the allocation to succeed. */
  WalterModemBlueCherryPublishBuffer publishBuffer = {};
  publishBuffer.buffer = (uint8_t*) heap_caps_malloc(BC_PUBLISH_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
  publishBuffer.size = BC_PUBLISH_BUFFER_SIZE;

  if(publishBuffer.buffer == NULL) {
    Serial.println("Warning: No PSRAM available, letting BlueCherry allocate its own publish "
                   "buffer");
    publishBuffer.size = 0;
  }

  if(!bc.init(BC_TLS_PROFILE, BC_DEVICE_TYPE,
              publishBuffer.buffer != NULL ? &publishBuffer : NULL)) {
    Serial.println("Error: Could not initialize BlueCherry");
    return false;
  }

  /* Optional, but recommended: the only way this application sees its downlink. The library has
   * no default for it, the payloads being application data it cannot interpret. */
  bc.setMsgHandler(myMessageHandler, NULL);

  /* Both optional: without them the library takes the update decisions itself, and bc.getState()
   * answers what the state handler reports. */
  bc.setOtaHandler(myOtaHandler, NULL);
  bc.setStateHandler(myStateHandler, NULL);

  Serial.println("Successfully initialized BlueCherry");
  return true;
}

/**
 * @brief The main Arduino setup method.
 */
void setup()
{
  WalterModemRsp rsp = {};
  Serial.begin(115200);
  delay(2000);

  Serial.printf("\r\n\r\n=== WalterModem BlueCherry example (v1.5.1) ===\r\n\r\n");

  /* 1. Start the modem. */
  if(modem.begin(&Serial2)) {
    Serial.println("Successfully initialized the modem");
  } else {
    Serial.println("Error: Could not initialize the modem");
    return;
  }

  /* 2. Register the network event handler, before anything can change the registration state. */
  modem.setNetworkEventHandler(myNetworkEventHandler, NULL);

  /* 3. Hand BlueCherry its buffers and handlers. Nothing here needs the network. */
  if(!initializeBlueCherry()) {
    return;
  }

  /* 4. Read the sensors. Both monitors are modem-local, so they need no network either. */
  /* Enable temperature monitoring */
  if(modem.configTemperatureMonitor(WALTER_MODEM_TEMP_MONITOR_MODE_ON)) {
    Serial.println("Successfully enabled temperature monitoring");
  } else {
    Serial.println("Warning: Could not enable temperature monitoring");
  }

  /* Get temperature reading */
  int8_t temperature = 0;
  if(modem.getTemperature(&rsp)) {
    if(rsp.type == WALTER_MODEM_RSP_DATA_TYPE_TEMPERATURE) {
      temperature = rsp.data.temperature.temperature;
      Serial.printf("Current temperature: %d°C (status: %d)\r\n", temperature,
                    rsp.data.temperature.status);
    }
  } else {
    Serial.println("Warning: Could not get temperature reading");
  }

  /* Disable temperature monitoring */
  if(modem.configTemperatureMonitor(WALTER_MODEM_TEMP_MONITOR_MODE_OFF)) {
    Serial.println("Successfully disabled temperature monitoring");
  } else {
    Serial.println("Warning: Could not disable temperature monitoring");
  }

  /* Enable voltage monitoring */
  if(modem.configVoltageMonitor(WALTER_MODEM_VOLTAGE_MONITOR_MODE_ACTIVE)) {
    Serial.println("Successfully enabled voltage monitoring");
  } else {
    Serial.println("Warning: Could not enable voltage monitoring");
  }

  /* Get voltage reading */
  uint16_t voltage = 0;
  if(modem.getVoltage(&rsp)) {
    if(rsp.type == WALTER_MODEM_RSP_DATA_TYPE_VOLTAGE) {
      voltage = rsp.data.voltage.voltage;
      Serial.printf("Current voltage: %dmV (status: %d)\r\n", voltage, rsp.data.voltage.status);
    }
  } else {
    Serial.println("Warning: Could not get voltage reading");
  }

  /* Disable voltage monitoring */
  if(modem.configVoltageMonitor(WALTER_MODEM_VOLTAGE_MONITOR_MODE_DISABLED)) {
    Serial.println("Successfully disabled voltage monitoring");
  } else {
    Serial.println("Warning: Could not disable voltage monitoring");
  }

  /* 5. Queue a message. Never touches the network; it goes out on the next synchronisation. */
  char msg[128];
  snprintf(msg, sizeof(msg),
           "{\"message\":\"Hello from Walter Modem!\",\"temperature\":%d,\"voltage\":%d}",
           temperature, voltage);
  Serial.printf("Publishing to BlueCherry: %s\r\n", msg);
  bc.publish(0x84, strlen(msg), (const uint8_t*) msg);

  /* 6. Connect to the cellular network. Everything above this line is local to the board, so the
   * radio is only asked for once there is something to send. */
  if(!lteConnected() && !lteConnect()) {
    Serial.println("Error: Unable to connect to cellular network, restarting Walter "
                   "in 10 seconds");
    delay(10000);
    ESP.restart();
  }

  /* 7. Synchronise. A sleepy device drives this itself rather than with setAutoSync, so it decides
   * how many exchanges run before it sleeps. The flag is cleared here and not next to the publish:
   * an idle reported before this line answers an exchange the task ran on its own. */
  bc_synchronized = false;
  bc.sync();

  /* No time-out on purpose: an unsettled exchange is still retrying, or still pulling a firmware
   * update down, and sleeping through either costs the transfer its progress. */
  while(!bc_synchronized) {
    delay(100);
  }

  /* The modem stays powered, so the session survives and the next boot resumes it. */
  Serial.println("I'm tired, I'm going to deep sleep now for 300 seconds");
  Serial.flush();
  modem.sleep(60 * 5);
}

/**
 * @brief The main Arduino loop method.
 */
void loop()
{
  // Nothing to do here
}
