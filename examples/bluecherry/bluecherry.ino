/**
 * @file bluecherry.ino
 * @author Jonas Maes <jonas@dptechnics.com>
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
 * This sketch sends and receives mqtt data using the DPTechnics BlueCherry cloud
 * platform. It also supports OTA updates which are scheduled through the BlueCherry web interface.
 */
#include <BlueCherryZTP.h>
#include <WalterModem.h>
#include <Arduino.h>
#include <esp_mac.h>

// The cellular Access Point Name
// Leave blank for autodetection
#define CELLULAR_APN ""

// Define BlueCherry cloud device ID
#define BC_DEVICE_TYPE "walter01"

// Define modem TLS profile used for BlueCherry cloud platform
#define BC_TLS_PROFILE 1

// Rename Serial2 to ModemSerial
#define ModemSerial Serial2

// Buffer to store OTA firmware messages in
byte ota_buffer[SPI_FLASH_BLOCK_SIZE] = { 0 };

// The modem instance
WalterModem modem;

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

// The BlueCherry CA root + intermediate certificate used for CoAP DTLS
// communication
const char* bc_ca_cert = "-----BEGIN CERTIFICATE-----\r\n\
MIIBlTCCATqgAwIBAgICEAAwCgYIKoZIzj0EAwMwGjELMAkGA1UEBhMCQkUxCzAJ\r\n\
BgNVBAMMAmNhMB4XDTI0MDMyNDEzMzM1NFoXDTQ0MDQwODEzMzM1NFowJDELMAkG\r\n\
A1UEBhMCQkUxFTATBgNVBAMMDGludGVybWVkaWF0ZTBZMBMGByqGSM49AgEGCCqG\r\n\
SM49AwEHA0IABJGFt28UrHlbPZEjzf4CbkvRaIjxDRGoeHIy5ynfbOHJ5xgBl4XX\r\n\
hp/r8zOBLqSbu6iXGwgjp+wZJe1GCDi6D1KjZjBkMB0GA1UdDgQWBBR/rtuEomoy\r\n\
49ovMAnj5Hpmk2gTGjAfBgNVHSMEGDAWgBR3Vw0Y1sUvMhkX7xySsX55tvsu8TAS\r\n\
BgNVHRMBAf8ECDAGAQH/AgEAMA4GA1UdDwEB/wQEAwIBhjAKBggqhkjOPQQDAwNJ\r\n\
ADBGAiEApN7DmuufC/aqyt6g2Y8qOWg6AXFUyTcub8/Y28XY3KgCIQCs2VUXCPwn\r\n\
k8jR22wsqNvZfbndpHthtnPqI5+yFXrY4A==\r\n\
-----END CERTIFICATE-----\r\n\
-----BEGIN CERTIFICATE-----\r\n\
MIIBmDCCAT+gAwIBAgIUDjfXeosg0fphnshZoXgQez0vO5UwCgYIKoZIzj0EAwMw\r\n\
GjELMAkGA1UEBhMCQkUxCzAJBgNVBAMMAmNhMB4XDTI0MDMyMzE3MzU1MloXDTQ0\r\n\
MDQwNzE3MzU1MlowGjELMAkGA1UEBhMCQkUxCzAJBgNVBAMMAmNhMFkwEwYHKoZI\r\n\
zj0CAQYIKoZIzj0DAQcDQgAEB00rHNthOOYyKj80cd/DHQRBGSbJmIRW7rZBNA6g\r\n\
fbEUrY9NbuhGS6zKo3K59zYc5R1U4oBM3bj6Q7LJfTu7JqNjMGEwHQYDVR0OBBYE\r\n\
FHdXDRjWxS8yGRfvHJKxfnm2+y7xMB8GA1UdIwQYMBaAFHdXDRjWxS8yGRfvHJKx\r\n\
fnm2+y7xMA8GA1UdEwEB/wQFMAMBAf8wDgYDVR0PAQH/BAQDAgGGMAoGCCqGSM49\r\n\
BAMDA0cAMEQCID7AcgACnXWzZDLYEainxVDxEJTUJFBhcItO77gcHPZUAiAu/ZMO\r\n\
VYg4UI2D74WfVxn+NyVd2/aXTvSBp8VgyV3odA==\r\n\
-----END CERTIFICATE-----\r\n";

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

// This function will poll the BlueCherry cloud platform to check if there is an
// incoming MQTT message or new firmware version available. If a new firmware
// version is available, the device automatically downloads and reboots with the
// new firmware.
void syncBlueCherry()
{
  WalterModemRsp rsp = {};
  int attempt = 0;
  bool fail = false;

  do {
    if(!modem.blueCherrySync(&rsp)) {
      Serial.printf("Error during BlueCherry cloud platform synchronisation: %d\r\n",
                    rsp.data.blueCherry.state);
      modem.reset();
      lteConnect();
      attempt++;
      fail = true;
    } else {
      attempt = 0;
      fail = false;
      for(uint8_t msgIdx = 0; msgIdx < rsp.data.blueCherry.messageCount; msgIdx++) {
        if(rsp.data.blueCherry.messages[msgIdx].topic == 0) {
          Serial.println("Downloading new firmware version");
          break;
        } else {
          Serial.printf("Incoming message %d/%d:\r\n", msgIdx + 1,
                        rsp.data.blueCherry.messageCount);
          Serial.printf("Topic: %02x\r\n", rsp.data.blueCherry.messages[msgIdx].topic);
          Serial.printf("Data size: %d\r\n", rsp.data.blueCherry.messages[msgIdx].dataSize);

          for(uint8_t byteIdx = 0; byteIdx < rsp.data.blueCherry.messages[msgIdx].dataSize;
              byteIdx++) {
            Serial.printf("%c", rsp.data.blueCherry.messages[msgIdx].data[byteIdx]);
          }

          Serial.print("\r\n");
        }
      }
    }
  } while(!rsp.data.blueCherry.syncFinished || (fail && attempt < 3));

  Serial.println("Synchronized with BlueCherry cloud platform");
  return;
}

bool configureBluecherry()
{
  WalterModemRsp rsp = {};
  unsigned short attempt = 0;
  while(!modem.blueCherryInit(BC_TLS_PROFILE, ota_buffer, &rsp)) {
    if(rsp.data.blueCherry.state == WALTER_MODEM_BLUECHERRY_STATUS_NOT_PROVISIONED &&
       attempt <= 2) {
      Serial.println("Device is not provisioned for BlueCherry communication, starting ZTP...");

      if(attempt == 0) {
        if(!BlueCherryZTP::begin(BC_DEVICE_TYPE, BC_TLS_PROFILE, bc_ca_cert, &modem)) {
          Serial.println("Error: Failed to initialize ZTP");
          continue;
        }

        // Fetch MAC address
        uint8_t mac[8] = { 0 };
        esp_read_mac(mac, ESP_MAC_WIFI_STA);
        if(!BlueCherryZTP::addDeviceIdParameter(BLUECHERRY_ZTP_DEVICE_ID_TYPE_MAC, mac)) {
          Serial.println("Error: Could not add MAC address as ZTP device ID parameter");
        }

        // Fetch IMEI number
        if(!modem.getIdentity(&rsp)) {
          Serial.println("Error: Could not fetch IMEI number from modem");
        }

        if(!BlueCherryZTP::addDeviceIdParameter(BLUECHERRY_ZTP_DEVICE_ID_TYPE_IMEI,
                                                rsp.data.identity.imei)) {
          Serial.println("Error: Could not add IMEI as ZTP device ID parameter");
        }
      }
      attempt++;

      // Request the BlueCherry device ID
      if(!BlueCherryZTP::requestDeviceId()) {
        Serial.println("Error: Could not request device ID");
        continue;
      }

      // Generate the private key and CSR
      if(!BlueCherryZTP::generateKeyAndCsr()) {
        Serial.println("Error: Could not generate private key");
      }
      delay(1000);

      // Request the signed certificate
      if(!BlueCherryZTP::requestSignedCertificate()) {
        Serial.println("Error: Could not request signed certificate");
        continue;
      }

      // Store BlueCherry TLS certificates + private key in the modem
      if(!modem.blueCherryProvision(BlueCherryZTP::getCert(), BlueCherryZTP::getPrivKey(),
                                    bc_ca_cert)) {
        Serial.println("Error: Failed to upload the DTLS certificates");
        continue;
      }
    } else {
      return false;
    }
  }
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

  Serial.printf("\r\n\r\n=== WalterModem BlueCherry example (v1.5.0) ===\r\n\r\n");

  /* Start the modem */
  if(modem.begin(&Serial2)) {
    Serial.println("Successfully initialized the modem");
  } else {
    Serial.println("Error: Could not initialize the modem");
    return;
  }

  /* Register network event handler */
  modem.setNetworkEventHandler(myNetworkEventHandler, NULL);

  /* Connect to cellular network */
  if(!lteConnected() && !lteConnect()) {
    Serial.println("Error: Unable to connect to cellular network, restarting Walter "
                   "in 10 seconds");
    delay(10000);
    ESP.restart();
  }

  /* Configure BlueCherry on first boot */
  if(esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED) {
    if(configureBluecherry()) {
      Serial.println("Successfully configured BlueCherry");
    } else {
      Serial.println("Error: Could not configure BlueCherry");
      return;
    }
  }

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

  /* Send a message to BlueCherry with sensor data */
  char msg[128];
  snprintf(msg, sizeof(msg),
           "{\"message\":\"Hello from Walter Modem!\",\"temperature\":%d,\"voltage\":%d}",
           temperature, voltage);
  Serial.printf("Publishing to BlueCherry: %s\r\n", msg);
  modem.blueCherryPublish(0x84, strlen(msg), (uint8_t*) msg);

  /* Poll BlueCherry platform if an incoming message or firmware update is available */
  syncBlueCherry();

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
