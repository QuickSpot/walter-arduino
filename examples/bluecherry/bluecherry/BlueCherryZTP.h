/**
 * @file BlueCherryZTP.h
 * @author Daan Pape <daan@dptechnics.com>
 * @author Thibo Verheyde <thibo@dptechnics.com>
 * @date 14 Jan 2025
 * @copyright DPTechnics bv
 * @brief BlueCherry ZTP (Zero Touch Provisioning) library.
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
 * This file contains the headers of the BlueCherry ZTP (Zero Touch
 * Provisioning) library.
 */

#ifndef BLUECHERRY_ZTP_H
#define BLUECHERRY_ZTP_H

#include <mbedtls/ctr_drbg.h>
#include <mbedtls/entropy.h>
#include <mbedtls/pk.h>
#include <mbedtls/x509_crt.h>
#include <mbedtls/x509_csr.h>

#include <WalterModem.h>

/**
 * @brief The number of characters in a BlueCherry Type ID or Device ID.
 */
#define BLUECHERRY_ZTP_ID_LEN 8

/**
 * @brief The size of the private key buffer.
 */
#define BLUECHERRY_ZTP_PKEY_BUF_SIZE 256

/**
 * @brief The size of the CSR/certificate buffer.
 */
#define BLUECHERRY_ZTP_CERT_BUF_SIZE 576

/**
 * @brief The size of the CSR subject buffer.
 */
#define BLUECHERRY_ZTP_SUBJ_BUF_SIZE 32

/**
 * @brief The length of a MAC address in bytes.
 */
#define BLUECHERRY_ZTP_MAC_LEN 6

/**
 * @brief The length of an IMEI number.
 */
#define BLUECHERRY_ZTP_IMEI_LEN 15

/**
 * @brief The maximum number of device identification parameters.
 */
#define BLUECHERRY_ZTP_MAX_DEVICE_ID_PARAMS 3

/**
 * @brief The maximum time in seconds to wait for a CoAP ring.
 */
#define BLUECHERRY_ZTP_COAP_TIMEOUT 30

/**
 * @brief This enumeration list all different types of device identification
 * parameters.
 */
typedef enum {
  BLUECHERRY_ZTP_DEVICE_ID_TYPE_MAC = 0,
  BLUECHERRY_ZTP_DEVICE_ID_TYPE_IMEI = 1,
  BLUECHERRY_ZTP_DEVICE_ID_TYPE_OOB_CHALLENGE = 2
} BlueCherryZtpDeviceIdType;

typedef union {
  /**
   * @brief Pointer to the BlueCherry Type ID, as this is always programmed in
   * the application, no extra memory is required.
   */
  const char* bcTypeId;

  /**
   * @brief A MAC address used for authentication.
   */
  unsigned char mac[BLUECHERRY_ZTP_MAC_LEN];

  /**
   * @brief An IMEI number in ASCII format + 0-terminator.
   */
  char imei[BLUECHERRY_ZTP_IMEI_LEN + 1];

  /**
   * @brief A 64-bit OOB challenge.
   */
  unsigned long long oobChallenge;
} BlueCherryZtpDeviceIdValue;

/**
 * @brief This structure represents a device identifier.
 */
typedef struct {
  /**
   * @brief The type of device identifier.
   */
  BlueCherryZtpDeviceIdType type;

  /**
   * @brief The value of the device identifier.
   */
  BlueCherryZtpDeviceIdValue value;
} BlueCherryZtpDeviceIdParam;

/**
 * @brief This structure represents a buffer and length of a CSR stored in PEM
 * format.
 */
typedef struct {
  /**
   * @brief The buffer used to store a CSR.
   */
  unsigned char buffer[BLUECHERRY_ZTP_CERT_BUF_SIZE] = {};

  /**
   * @brief The data length of the CSR.
   */
  size_t length;
} BlueCherryZtpCsr;

/**
 * @brief This structure represents the device identification parameters.
 */
typedef struct {
  /**
   * @brief The array of device identification parameters.
   */
  BlueCherryZtpDeviceIdParam param[BLUECHERRY_ZTP_MAX_DEVICE_ID_PARAMS];

  /**
   * @brief The number of parameters in the list.
   */
  int count;
} BlueCherryZtpDeviceId;

/**
 * @brief This class implements the BlueCherry ZTP (Zero Touch Provisioning)
 * protocol.
 */
class BlueCherryZTP
{
private:
  static inline const WalterModem* _modem;

  /**
   * @brief The BlueCherry type ID associated with this firmware.
   */
  static inline const char* _bcTypeId = nullptr;

  /**
   * @brief The modem security profile in use for BlueCherry communication.
   */
  static inline uint8_t _tlsProfileId = 0;

  /**
   * @brief The BlueCherry device ID received from the server.
   */
  static inline char _bcDevId[BLUECHERRY_ZTP_ID_LEN + 1] = {};

  /**
   * @brief The buffer used to store a private key.
   */
  static inline char _pkeyBuf[BLUECHERRY_ZTP_PKEY_BUF_SIZE] = {};

  /**
   * @brief The buffer used to store a certificate.
   */
  static inline char _certBuf[BLUECHERRY_ZTP_CERT_BUF_SIZE] = {};

  /**
   * @brief The size of the buffer used for the CSR subject.
   */
  static inline char _subjBuf[BLUECHERRY_ZTP_SUBJ_BUF_SIZE] = {};

  /**
   * @brief The CSR context.
   */
  static inline BlueCherryZtpCsr _csr;

  /**
   * @brief Mbed TLS private key context.
   */
  static inline mbedtls_pk_context _mbKey;

  /**
   * @brief Mbed TLS entropy context.
   */
  static inline mbedtls_entropy_context _mbEntropy;

  /**
   * @brief Mbed TLS counter-mode deterministic random bit generator context.
   */
  static inline mbedtls_ctr_drbg_context _mbCtrDrbg;

  /**
   * @brief Mbed TLS CSR creation object.
   */
  static inline mbedtls_x509write_csr _mbCsr;

  /**
   * @brief Mbed TLS certificate creation object.
   */
  static inline mbedtls_x509_crt _mbCrt;

  /**
   * @brief The device ZTP identification data.
   */
  static inline BlueCherryZtpDeviceId _devIdParams = {};

  /**
   * @brief The entropy function used by Mbed TLS.
   *
   * This function makes use of the true random number generator found in the
   * ESP32-S3 hardware.
   *
   * @param data Pointer to the entropy context.
   * @param output Buffer to store the true random data in.
   * @param len The requested number of random bytes.
   *
   * @return 0 on success.
   */
  static int _hardwareRandomEntropyFunc(void* data, unsigned char* output, size_t len);

  /**
   * @brief Seed the counter-mode deterministic random bit generator.
   *
   * This function will seed the counter-mode deterministic random bit generator
   * with numbers from the true random number generator in the ESP32-S3.
   *
   * @param rfEnabled True when WiFi/BLE is enabled, false if not.
   *
   * @return True on success, false on error.
   */
  static bool _seedRandom(bool rfEnabled);

  /**
   * @brief Finish private key and CSR generation.
   *
   * This function will finish the private kay and CSR generation function by
   * cleaning up Mbed TLS memory and setting the private key and CSR buffers to
   * zero length strings in case result was false.
   *
   * @param result The result of the key/CSR generation routines.
   *
   * @return The value of the result parameter.
   */
  static bool _finishCsrGen(bool result);

public:
  /**
   * @brief Initialize the library and configure the type id.
   *
   * This function will initialize the BlueCherry ZTP (Zero Touch Provisioning)
   * library. The BlueCherry Type ID is the 8 character identification code
   * which identifies this firmware/device.
   *
   * @param typeId The BlueCherry Type ID as a constant string, this is not
   * copied and the library expects the Type ID to be always available.
   *
   * @return True on success, false if the Type ID is not valid.
   */
  static bool begin(const char* typeId, const uint8_t tls_profile_id, const char* caCert,
                    const WalterModem* modem);

  /**
   * @brief Get the last generated private key as a string.
   *
   * This function will return the last generated private key as a string. The
   * result of this function should not leave the device under any circumstance.
   *
   * @return The private key as a string.
   */
  static const char* getPrivKey();

  /**
   * @brief Get the last generated CSR.
   *
   * This function will return the last generated certificate signing request.
   *
   * @return The CSR as an unsigned char array.
   */
  static const unsigned char* getCsr();

  /**
   * @brief Get the last generated CSR length.
   *
   * This function will return the length of the last generated certificate
   * signing request.
   *
   * @return The CSR length as a size_t.
   */
  static size_t getCsrLen();

  /**
   * @brief Get the generated device certificate.
   *
   * This function will return the last generated device certificate in PEM
   * format.
   *
   * @return The certificate as a string.
   */
  static const char* getCert();

  /**
   * @brief Clear all device ID parameters.
   *
   * This function will reset all device ID parameters.
   *
   * @return None.
   */
  static void resetDeviceId();

  /**
   * @brief Add a string to the device ID parameter list.
   *
   * This function will add a string to the device ID parameter list. A hard
   * copy of the string will be created.
   *
   * @param type The parameter type.
   * @param str The string to add.
   *
   * @return True on success, false if the internal array is full.
   */
  static bool addDeviceIdParameter(BlueCherryZtpDeviceIdType type, const char* str);

  /**
   * @brief Add a blob to the device ID parameter list.
   *
   * This function will add a blob to the device ID parameter list. The length
   * of the blob is determined from the device ID parameter type. A hard copy of
   * the blob will be created.
   *
   * @param type The parameter type.
   * @param blob The blob to add.
   *
   * @return True on success, false if the internal array is full.
   */
  static bool addDeviceIdParameter(BlueCherryZtpDeviceIdType type, const unsigned char* blob);

  /**
   * @brief Add a number to the device ID parameter list.
   *
   * This function will add a number to the device ID parameter list.
   *
   * @param type The parameter type.
   * @param number The number to add.
   *
   * @return True on success, false if the internal array is full.
   */
  static bool addDeviceIdParameter(BlueCherryZtpDeviceIdType type, unsigned long long number);

  /**
   * @brief Request a provisional BlueCherry Device ID from the ZTP server.
   *
   * This function will connect to the BlueCherry ZTP server and request a new
   * device ID. The server certificate will be validated to make sure that the
   * response is valid.
   *
   * @return True on success, false on error.
   */
  static bool requestDeviceId();

  /**
   * @brief Generate a new BlueCherry private key and CSR.
   *
   * This function starts by generating a new private key for a BlueCherry
   * connection. The generated key should never leave the device and is
   * generated by using the hardware random number generator of the ESP32 which
   * requires the RF (WiFi/BLE) to be enabled or the SAR ADC to be configured
   * in random number generation mode. If RF is not enabled this function will
   * briefly configure the ADC to entropy mode. This function can therefore
   * not be used together with ADC conversions.
   *
   * In the second step this function will generate a new CSR (Certificate
   * Signing Request) to send to the ZTP server to obtain the device
   * certificate.
   *
   * This function is stateful and the last generated private key and CSR can
   * be retrieved using the designated functions in this library.
   *
   * @param rfEnabled True when the WiFi or Bluetooth radio is activated.
   *
   * @return The resulting CSR as a constant string or NULL on error.
   */
  static bool generateKeyAndCsr(bool rfEnabled = false);

  /**
   * @brief Request a signed client certificate from the ZTP server.
   *
   * @return True on success, false on error.
   */
  static bool requestSignedCertificate();
};

#endif
