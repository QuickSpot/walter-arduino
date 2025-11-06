/**
 * @file aws_mqtt.ino
 * @author Arnoud Devoogdt <arnoud@dptechnics.com>
 * @date 9 September 2025
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
 * mqtts connection to AWS.
 */

#include <HardwareSerial.h>
#include <WalterModem.h>
#include <pgmspace.h>
#include <esp_mac.h>

#define AWS_MQTTS_PORT 8883
#define AWS_MQTTS_ENDPOINT ""
#define AWS_MQTTS_TOPIC ""
#define AWS_MQTTS_CLIENT_ID ""

/**
 * @brief Root CA certificate in PEM format.
 *
 * Used to validate the server's TLS certificate.
 */
const char ca_cert[] PROGMEM = R"EOF(
  -----BEGIN CERTIFICATE-----\n...\n-----END CERTIFICATE-----)EOF";

/**
 * @brief Client certificate in PEM format.
 *
 * Used by the modem to authenticate itself to the server.
 */
const char client_cert[] PROGMEM = R"EOF(
  -----BEGIN CERTIFICATE-----\n...\n-----END CERTIFICATE-----)EOF";

/**
 * @brief Client private key in PEM format.
 *
 * Used by the modem for TLS mutual authentication.
 * Keep this private and secure.
 */
const char client_private_key[] PROGMEM = R"EOF(
  -----BEGIN RSA PRIVATE KEY-----\n...\n-----END RSA PRIVATE KEY-----)EOF";

/**
 * The TLS profile to use for the application (1 is reserved for BlueCherry)
 */
#define AWS_MQTT_TLS_PROFILE 2

/**
 * @brief The modem instance.
 */
WalterModem modem;

/**
 * @brief Response object containing command response information.
 */
walter_modem_rsp_t rsp;

/**
 * @brief Buffer for incoming response
 */
uint8_t in_buf[256] = { 0 };

/**
 * @brief MQTT client and message prefix based on mac address
 */
char macString[32];

/**
 * @brief This function checks if we are connected to the lte network
 * @return True when connected, False otherwise
 */
bool lteConnected()
{
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  return (regState == WALTER_MODEM_NETWORK_REG_REGISTERED_HOME ||
          regState == WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING);
}

/**
 * @brief This function waits for the modem to be connected to the Lte network.
 * @return true if the connected, else false on timeout.
 */
bool waitForNetwork()
{
  /* Wait for the network to become available */
  int timeout = 0;
  while(!lteConnected()) {
    delay(1000);
    timeout++;
    if(timeout > 300)
      return false;
  }
  Serial.println("Connected to the network");
  return true;
}

/**
 * @brief This function tries to connect the modem to the cellular network.
 * @return true if the connection attempt is successful, else false.
 */
bool lteConnect()
{
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
    Serial.println("Network selection mode to was set to automatic");
  } else {
    Serial.println("Error: Could not set the network selection mode to automatic");
    return false;
  }

  return waitForNetwork();
}

/**
 * @brief Writes TLS credentials to the modem's NVS and configures the TLS profile.
 *
 * This function stores the provided TLS certificates and private keys into the modem's
 * non-volatile storage (NVS), and then sets up a TLS profile for secure communication.
 * These configuration changes are persistent across reboots.
 *
 * @note
 * - Certificate indexes 0–10 are reserved for Sequans and BlueCherry internal usage.
 * - Private key index 1 is reserved for BlueCherry internal usage.
 * - Do not attempt to override or use these reserved indexes.
 *
 * @return
 * - true if the credentials were successfully written and the profile configured.
 * - false otherwise.
 */
bool setupTLSProfile(void)
{

  if(!modem.tlsWriteCredential(false, 12, ca_cert)) {
    Serial.println("Error: CA cert upload failed");
    return false;
  }
  if(!modem.tlsWriteCredential(false, 11, client_cert)) {
    Serial.println("Error: client cert upload failed");
    return false;
  }
  if(!modem.tlsWriteCredential(true, 1, client_private_key)) {
    Serial.println("Error: private key upload failed");
    return false;
  }

  if(modem.tlsConfigProfile(AWS_MQTT_TLS_PROFILE, WALTER_MODEM_TLS_VALIDATION_CA,
                            WALTER_MODEM_TLS_VERSION_12, 12, 11, 1)) {
    Serial.println("TLS profile configured");
  } else {
    Serial.println("Error: TLS profile configuration failed");
    return false;
  }

  return true;
}

void setup()
{
  Serial.begin(115200);
  delay(5000);

  Serial.println("Walter modem AWS IoT example v1.0.0\r\n");

  esp_read_mac(in_buf, ESP_MAC_WIFI_STA);
  sprintf(macString, "walter%02X:%02X:%02X:%02X:%02X:%02X", in_buf[0], in_buf[1], in_buf[2],
          in_buf[3], in_buf[4], in_buf[5]);

  if(WalterModem::begin(&Serial2)) {
    Serial.println("Modem initialization OK");
  } else {
    Serial.println("Error: Modem initialization ERROR");
    return;
  }
  /* Connect the modem to the lte network */
  if(!lteConnect()) {
    Serial.println("Error: Could Not Connect to LTE");
    return;
  }

  if(setupTLSProfile()) {
    Serial.println("TLS Profile setup succeeded");
  } else {
    Serial.println("Error: TLS Profile setup failed");
    return;
  }

  /* Configure MQTT with TLS profile (MQTTS) */
  if(modem.mqttConfig(AWS_MQTTS_CLIENT_ID, 0, 0, AWS_MQTT_TLS_PROFILE)) {
    Serial.println("MQTTS configuration succeeded");
  } else {
    Serial.println("Error: MQTTS configuration failed");
    return;
  }

  if(modem.mqttConnect(AWS_MQTTS_ENDPOINT, AWS_MQTTS_PORT)) {
    Serial.println("MQTTS connection succeeded");
  } else {
    Serial.println("Error: MQTTS connection failed");
  }

  if(modem.mqttSubscribe(AWS_MQTTS_TOPIC)) {
    Serial.println("MQTTS subscribed to topic");
  } else {
    Serial.println("Error: MQTTS subscribe failed");
  }
}

void loop()
{
  delay(15000);

  static int seq = 0;
  static char outgoingMsg[64];
  seq++;
  if(seq % 3 == 0) {
    sprintf(outgoingMsg, "%s-%d", macString, seq);
    if(modem.mqttPublish(AWS_MQTTS_TOPIC, (uint8_t*) outgoingMsg, strlen(outgoingMsg))) {
      Serial.printf("published '%s' on topic", outgoingMsg);
    } else {
      Serial.print("MQTTS publish failed\r\n");
    }
  }

  while(modem.mqttDidRing(AWS_MQTTS_TOPIC, in_buf, sizeof(in_buf), &rsp)) {
    Serial.printf("incoming: qos=%d msgid=%d len=%d:\r\n", rsp.data.mqttResponse.qos,
                  rsp.data.mqttResponse.messageId, rsp.data.mqttResponse.length);
    for(int i = 0; i < rsp.data.mqttResponse.length; i++) {
      Serial.printf("'%c' 0x%02x\r\n", in_buf[i], in_buf[i]);
    }
  }
}
