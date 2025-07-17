/**
 * @file bluecherry.ino
 * @author Jonas Maes <jonas@dptechnics.com>
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
 * This sketch sends and receives mqtt data using the DPTechnics BlueCherry cloud
 * platform. It also supports OTA updates which are scheduled through the BlueCherry web interface.
 */

#include <Arduino.h>
#include "esp_mac.h"

#include "WalterModem.h"
#include "BlueCherryZTP.h"

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
byte otaBuffer[SPI_FLASH_BLOCK_SIZE] = {0};

// The modem instance
WalterModem modem;

/**
 * @brief The binary configuration settings for PSM.
 * These can be calculated using e.g.
 * https://www.soracom.io/psm-calculation-tool/
 */
const char *psmActive = "00000001";
const char *psmTAU = "00000110";

// The BlueCherry CA root + intermediate certificate used for CoAP DTLS
// communication
const char *bc_ca_cert = "-----BEGIN CERTIFICATE-----\r\n\
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


// This function checks if the modem has a cellular connection to the network.
// Returns true if the modem is connected, else false.
bool lteConnected() {
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  return (regState == WALTER_MODEM_NETWORK_REG_REGISTERED_HOME ||
          regState == WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING);
}


// This function tries to connect the modem to the cellular network.
// Returns true if the connection attempt is successful, else false.
bool lteConnect() {
  // Set the functionality level of the modem to minimum
  if (!modem.setOpState(WALTER_MODEM_OPSTATE_MINIMUM)) {
    Serial.println("Could not set the modem to minimum functionality level");
    return false;
  }
  delay(500);

  // Create a PDP context with specified APN
  if (!modem.definePDPContext(1, CELLULAR_APN)) {
    Serial.println("Could not create PDP context");
    return false;
  }
  Serial.println("Attempting to connect to the network...");

  // Set the functionality level of the modem to full
  if (!modem.setOpState(WALTER_MODEM_OPSTATE_FULL)) {
    Serial.println("Error: Could not set the modem to full functionality level");
    return false;
  }

  delay(1000);

  // Set the network operator selection to automatic
  if (modem.setNetworkSelectionMode(WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC)) {
      Serial.println("Network selection mode to was set to automatic");
  } else {
      Serial.println("Error: Could not set the network selection mode to automatic");
      return false;
  }

  // Wait (maximum 5 minutes) until successfully registered to the network
  unsigned short timeout = 300;
  unsigned short i = 0;
  while (!lteConnected() && i < timeout) {
    i++;
    delay(1000);
  }
  if (i >= timeout) {
    return false;
  }

  // Show cellular connection information
  WalterModemRsp rsp = {};
  if (modem.getRAT(&rsp)) {
    Serial.printf("Connected to %s ",
                  rsp.data.rat == WALTER_MODEM_RAT_NBIOT ? "NB-IoT" : "LTE-M");
  }
  if (modem.getCellInformation(WALTER_MODEM_SQNMONI_REPORTS_SERVING_CELL,
                               &rsp)) {
    Serial.printf("on band %u using operator %s (%u%02u)\r\n",
                  rsp.data.cellInformation.band,
                  rsp.data.cellInformation.netName, rsp.data.cellInformation.cc,
                  rsp.data.cellInformation.nc);
    Serial.printf("Signal strength: RSRP: %.2f, RSRQ: %.2f\r\n",
                  rsp.data.cellInformation.rsrp, rsp.data.cellInformation.rsrq);
  }

  return true;
}


// This function will poll the BlueCherry cloud platform to check if there is an
// incoming MQTT message or new firmware version available. If a new firmware
// version is available, the device automatically downloads and reboots with the
// new firmware.
void syncBlueCherry() {
  WalterModemRsp rsp = {};

  do {
    if (!modem.blueCherrySync(&rsp)) {
      Serial.printf(
          "Error during BlueCherry cloud platform synchronisation: %d\r\n",
          rsp.data.blueCherry.state);
      modem.reset();
      lteConnect();
      return;
    }

    for(uint8_t msgIdx = 0; msgIdx < rsp.data.blueCherry.messageCount; msgIdx++) {
      if(rsp.data.blueCherry.messages[msgIdx].topic == 0) {
        Serial.println("Downloading new firmware version");
        break;
      } else {
        Serial.printf("Incoming message %d/%d:\r\n", msgIdx + 1, rsp.data.blueCherry.messageCount);
        Serial.printf("Topic: %02x\r\n", rsp.data.blueCherry.messages[msgIdx].topic);
        Serial.printf("Data size: %d\r\n", rsp.data.blueCherry.messages[msgIdx].dataSize);
      
        for(uint8_t byteIdx = 0; byteIdx < rsp.data.blueCherry.messages[msgIdx].dataSize; byteIdx++) {
          Serial.printf("%c", rsp.data.blueCherry.messages[msgIdx].data[byteIdx]);
        }

        Serial.print("\r\n");
      }
    }
  } while (!rsp.data.blueCherry.syncFinished);

  Serial.println("Synchronized with BlueCherry cloud platform");
  return;
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Walter BlueCherry ZTP + OTA + PSM example.");

  // Modem initialization
  if (!modem.begin(&ModemSerial)) {
    Serial.println(
        "Error: Could not initialize the modem, restarting Walter in 10 seconds");
    delay(10000);
    ESP.restart();
  }

  modem.configPSM(WALTER_MODEM_PSM_ENABLE,psmTAU,psmActive);

  // Connect to cellular network
  if (!lteConnected() && !lteConnect()) {
    Serial.println("Error: Unable to connect to cellular network, restarting Walter "
                  "in 10 seconds");
   delay(10000);
  ESP.restart();
  }


  WalterModemRsp rsp = {};

  // If this is the first boot, set up bluecherry and ZTP.
  if(esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED) {
    // Initialize the BlueCherry connection
    unsigned short attempt = 0;
    while (!modem.blueCherryInit(BC_TLS_PROFILE, otaBuffer, &rsp)) {
      if (rsp.data.blueCherry.state ==
              WALTER_MODEM_BLUECHERRY_STATUS_NOT_PROVISIONED &&
          attempt <= 2) {
        Serial.println("Device is not provisioned for BlueCherry "
                      "communication, starting Zero Touch Provisioning");

        if (attempt == 0) {
          // Device is not provisioned yet, initialize BlueCherry zero touch
          // provisioning
          if (!BlueCherryZTP::begin(BC_DEVICE_TYPE, BC_TLS_PROFILE, bc_ca_cert,
                                    &modem)) {
            Serial.println("Error: Failed to initialize ZTP");
            continue;
          }

          // Fetch MAC address
          uint8_t mac[8] = {0};
          esp_read_mac(mac, ESP_MAC_WIFI_STA);
          if (!BlueCherryZTP::addDeviceIdParameter(
                  BLUECHERRY_ZTP_DEVICE_ID_TYPE_MAC, mac)) {
            Serial.println(
                "Error: Could not add MAC address as ZTP device ID parameter");
          }

          // Fetch IMEI number
          if (!modem.getIdentity(&rsp)) {
            Serial.println("Error: Could not fetch IMEI number from modem");
          }
          if (!BlueCherryZTP::addDeviceIdParameter(
                  BLUECHERRY_ZTP_DEVICE_ID_TYPE_IMEI, rsp.data.identity.imei)) {
            Serial.println("Error: Could not add IMEI as ZTP device ID parameter");
          }
        }
        attempt++;

        // Request the BlueCherry device ID
        if (!BlueCherryZTP::requestDeviceId()) {
          Serial.println("Error: Could not request device ID");
          continue;
        }

        // Generate the private key and CSR
        if (!BlueCherryZTP::generateKeyAndCsr()) {
          Serial.println("Error: Could not generate private key");
        }
        delay(1000);

        // Request the signed certificate
        if (!BlueCherryZTP::requestSignedCertificate()) {
          Serial.println("Error: Could not request signed certificate");
          continue;
        }

        // Store BlueCherry TLS certificates + private key in the modem
        if (!modem.blueCherryProvision(BlueCherryZTP::getCert(),
                                      BlueCherryZTP::getPrivKey(), bc_ca_cert)) {
          Serial.println("Error: Failed to upload the DTLS certificates");
          continue;
        }
      } else {
        Serial.println("Error: Failed to initialize BlueCherry cloud platform, "
                      "restarting Walter in 10 seconds");
        delay(10000);
        ESP.restart();
      }
    }
    Serial.println("Successfully initialized BlueCherry cloud platform");
  }
}

void loop() {
  // Check if the modem is connected to the cellular network, else try to
  // reconnect
   if (!lteConnected() && !lteConnect()) {
    Serial.println("Error: Unable to connect to cellular network, restarting Walter "
                   "in 10 seconds");
    delay(10000);
    ESP.restart();
  }

  WalterModemRsp rsp = {};

  // Send a message containing the measured RSRP value to an MQTT topic
  if (modem.getCellInformation(WALTER_MODEM_SQNMONI_REPORTS_SERVING_CELL,
                               &rsp)) {
    char msg[18];
    snprintf(msg, sizeof(msg), "{\"RSRP\": %7.2f}", rsp.data.cellInformation.rsrp);
    modem.blueCherryPublish(0x84, sizeof(msg)-1, (uint8_t *)msg);
  }

  // Poll BlueCherry platform if an incoming message or firmware update is available
  syncBlueCherry();

  // Go sleep for 5 minutes
  modem.sleep(60 * 5);
}
