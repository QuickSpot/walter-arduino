/**
 * @file BlueCherryZTP.cpp
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
 * This file contains the implementation of the BlueCherry ZTP (Zero Touch
 * Provisioning) library.
 */

#include <bootloader_random.h>
#include <esp_random.h>
#include <mbedtls/ctr_drbg.h>
#include <mbedtls/entropy.h>
#include <mbedtls/pem.h>
#include <mbedtls/pk.h>
#include <mbedtls/x509_crt.h>
#include <mbedtls/x509_csr.h>
#include <stdio.h>
#include <string.h>

#include "BlueCherryZTP.h"
#include "BlueCherryZTP_CBOR.h"
#include "WalterModem.h"

#ifdef ARDUINO
#define DELAY(delayMs) delay(delayMs)
#else
#define DELAY(delayMs) vTaskDelay(pdMS_TO_TICKS(delayMs))
#endif
#define COAP_PROFILE 2
#define ZTP_SERV_ADDR "coap.bluecherry.io"
#define ZTP_SERV_PORT 5688

#define ZTP_SERV_API_VERSION "v1"
#define ZTP_SERV_DEVID_PATH "devid"
#define ZTP_SERV_CSR_PATH "sign"

static bool _coapRingReceived = false;
static uint16_t _coapRingMsgId = 0;

static void ztpURCHandler(const WalterModemEvent* ev, void* args)
{
  switch(ev->type) {
  case WM_URC_TYPE_COAP:
    if(ev->coap.event == WALTER_MODEM_COAP_EVENT_RING) {
      _coapRingMsgId = ev->coap.msg_id;
      _coapRingReceived = true;
    }
    break;
  default:
    /* Unhandled event */
    break;
  }
}

int BlueCherryZTP::_hardwareRandomEntropyFunc(void* data, unsigned char* output, size_t len)
{
  esp_fill_random(output, len);
  return 0;
}

bool BlueCherryZTP::_finishCsrGen(bool result)
{
  mbedtls_pk_free(&_mbKey);
  mbedtls_entropy_free(&_mbEntropy);
  mbedtls_ctr_drbg_free(&_mbCtrDrbg);
  mbedtls_x509write_csr_free(&_mbCsr);

  if(!result) {
    _pkeyBuf[0] = '\0';
    _certBuf[0] = '\0';
  }

  return result;
}

bool BlueCherryZTP::_seedRandom(bool rfEnabled)
{
  if(!rfEnabled) {
    bootloader_random_enable();
  }

  int ret = mbedtls_ctr_drbg_seed(&_mbCtrDrbg, _hardwareRandomEntropyFunc, &_mbEntropy, nullptr, 0);

  if(!rfEnabled) {
    bootloader_random_disable();
  }
  return ret == 0;
}

bool BlueCherryZTP::begin(const char* typeId, const uint8_t tls_profile_id, const char* caCert,
                          const WalterModem* modem)
{
  if(typeId == nullptr || strlen(typeId) != BLUECHERRY_ZTP_ID_LEN || tls_profile_id == 0 ||
     tls_profile_id == 0 || tls_profile_id > 6 || caCert == nullptr || modem == nullptr) {
    return false;
  }

  if(!modem->blueCherryProvision(nullptr, nullptr, caCert)) {
    return false;
  }

  _bcTypeId = typeId;
  _tlsProfileId = tls_profile_id;
  _modem = modem;

  if(!modem->tlsConfigProfile(tls_profile_id, WALTER_MODEM_TLS_VALIDATION_CA,
                              WALTER_MODEM_TLS_VERSION_12, 6)) {
    return false;
  }

  return true;
}

const char* BlueCherryZTP::getPrivKey()
{
  return _pkeyBuf;
}

const unsigned char* BlueCherryZTP::getCsr()
{
  return _csr.buffer;
}

size_t BlueCherryZTP::getCsrLen()
{
  return _csr.length;
}

const char* BlueCherryZTP::getCert()
{
  return _certBuf;
}

void BlueCherryZTP::resetDeviceId()
{
  _devIdParams.count = 0;
}

bool BlueCherryZTP::addDeviceIdParameter(BlueCherryZtpDeviceIdType type, const char* str)
{
  if(str == nullptr || _devIdParams.count >= BLUECHERRY_ZTP_MAX_DEVICE_ID_PARAMS) {
    return false;
  }

  switch(type) {
  case BLUECHERRY_ZTP_DEVICE_ID_TYPE_IMEI:
    _devIdParams.param[_devIdParams.count].type = BLUECHERRY_ZTP_DEVICE_ID_TYPE_IMEI;
    strncpy(_devIdParams.param[_devIdParams.count].value.imei, str, BLUECHERRY_ZTP_IMEI_LEN);
    _devIdParams.param[_devIdParams.count].value.imei[BLUECHERRY_ZTP_IMEI_LEN] = '\0';
    _devIdParams.count += 1;
    break;

  default:
    return false;
  }

  return true;
}

bool BlueCherryZTP::addDeviceIdParameter(BlueCherryZtpDeviceIdType type, const unsigned char* blob)
{
  if(blob == nullptr || _devIdParams.count >= BLUECHERRY_ZTP_MAX_DEVICE_ID_PARAMS) {
    return false;
  }

  switch(type) {
  case BLUECHERRY_ZTP_DEVICE_ID_TYPE_MAC:
    _devIdParams.param[_devIdParams.count].type = BLUECHERRY_ZTP_DEVICE_ID_TYPE_MAC;
    memcpy(_devIdParams.param[_devIdParams.count].value.mac, blob, BLUECHERRY_ZTP_MAC_LEN);
    _devIdParams.count += 1;
    break;

  default:
    return false;
  }

  return true;
}

bool BlueCherryZTP::addDeviceIdParameter(BlueCherryZtpDeviceIdType type, unsigned long long number)
{
  if(_devIdParams.count >= BLUECHERRY_ZTP_MAX_DEVICE_ID_PARAMS) {
    return false;
  }

  switch(type) {
  case BLUECHERRY_ZTP_DEVICE_ID_TYPE_OOB_CHALLENGE:
    _devIdParams.param[_devIdParams.count].type = BLUECHERRY_ZTP_DEVICE_ID_TYPE_OOB_CHALLENGE;
    _devIdParams.param[_devIdParams.count].value.oobChallenge = number;
    _devIdParams.count += 1;
    break;

  default:
    return false;
  }

  return true;
}

bool BlueCherryZTP::requestDeviceId()
{
  int ret;
  walter_modem_rsp_t rsp = {};
  uint8_t cborBuf[256];
  uint8_t coapData[16];
  ZTP_CBOR cbor;

  if(ztp_cbor_init(&cbor, cborBuf, sizeof(cborBuf)) < 0) {
    printf("Failed to init CBOR buffer\n");
    return false;
  };

  // Start the CBOR array
  if(ztp_cbor_start_array(&cbor, 2) < 0) {
    printf("Failed to start CBOR array\n");
    return false;
  }

  // Encode type ID value
  if(ztp_cbor_encode_string(&cbor, _bcTypeId) < 0) {
    printf("Failed to encode typeId value\n");
    return false;
  }

  // Start the CBOR map (key-value pairs)
  if(ztp_cbor_start_map(&cbor, _devIdParams.count) < 0) {
    printf("Failed to start CBOR map\n");
    return false;
  }

  for(size_t i = 0; i < _devIdParams.count; i++) {

    int type = (int) _devIdParams.param[i].type;
    if(ztp_cbor_encode_int(&cbor, type) < 0) {
      printf("Failed to encode param type (%u)\n", type);
      return false;
    }

    switch(_devIdParams.param[i].type) {
    case BLUECHERRY_ZTP_DEVICE_ID_TYPE_IMEI: {
      // Encode IMEI number (15 characters)
      uint64_t imei = strtoull(_devIdParams.param[i].value.imei, NULL, 10);
      if(ztp_cbor_encode_uint64(&cbor, imei) < 0) {
        printf("Failed to encode IMEI number\n");
        return false;
      }
    } break;

    case BLUECHERRY_ZTP_DEVICE_ID_TYPE_MAC: {
      // Encode MAC address (6 bytes)
      if(ztp_cbor_encode_bytes(&cbor, (uint8_t*) _devIdParams.param[i].value.mac, 6) < 0) {
        printf("Failed to encode MAC address\n");
        return false;
      }
    } break;

    case BLUECHERRY_ZTP_DEVICE_ID_TYPE_OOB_CHALLENGE: {
      // Encode OOB challenge (64 bit unsigned int)
      uint64_t oobChallenge = _devIdParams.param[0].value.oobChallenge;
      if(ztp_cbor_encode_uint64(&cbor, oobChallenge) < 0) {
        printf("Failed to encode OOB challenge\n");
        return false;
      }
    } break;

    default:
      break;
    }
  }

  _modem->urcSetEventHandler(ztpURCHandler, NULL);

  // Send first CoAP
  if(!_modem->coapCreateContext(COAP_PROFILE, ZTP_SERV_ADDR, ZTP_SERV_PORT, _tlsProfileId)) {
    printf("Failed to create ZTP CoAP context\n");
    return false;
  }

  if(!_modem->coapSetOptions(COAP_PROFILE, WALTER_MODEM_COAP_OPT_SET,
                             WALTER_MODEM_COAP_OPT_CODE_URI_PATH, ZTP_SERV_API_VERSION)) {
    printf("Failed to configure ZTP CoAP URI path for API version\n");
  }

  if(!_modem->coapSetOptions(COAP_PROFILE, WALTER_MODEM_COAP_OPT_EXTEND,
                             WALTER_MODEM_COAP_OPT_CODE_URI_PATH, ZTP_SERV_DEVID_PATH)) {
    printf("Failed to configure ZTP CoAP URI path for device id\n");
  }

  if(!_modem->coapSendData(COAP_PROFILE, WALTER_MODEM_COAP_SEND_TYPE_CON,
                           WALTER_MODEM_COAP_SEND_METHOD_GET, ztp_cbor_size(&cbor), cborBuf)) {
    printf("Failed to send ZTP CoAP datagram\n");
    return false;
  }

  _coapRingReceived = false;
  int i = BLUECHERRY_ZTP_COAP_TIMEOUT;
  printf("Awaiting ZTP CoAP ring.");
  while(i && !_coapRingReceived) {
    printf(".");
    DELAY(1000);
    i--;
  }
  printf("\n");

  if(!_modem->coapReceive(COAP_PROFILE, _coapRingMsgId, coapData, sizeof(coapData), &rsp)) {
    printf("Failed to receive ZTP CoAP message\n");
    return false;
  }

  if(i < 1) {
    printf("Failed to receive response from ZTP COAP server\n");
    return false;
  }

  ret =
      ztp_cbor_decode_device_id(coapData, rsp.data.coapResponse.length, _bcDevId, sizeof(_bcDevId));
  if(ret < 0) {
    printf("Failed to decode device id: %d\n", ret);
    return false;
  }

  return true;
}

bool BlueCherryZTP::generateKeyAndCsr(bool rfEnabled)
{
  int ret;
  uint8_t csrBuf[BLUECHERRY_ZTP_CERT_BUF_SIZE];

  if(_bcTypeId == nullptr || strlen(_bcTypeId) != BLUECHERRY_ZTP_ID_LEN ||
     strlen(_bcDevId) != BLUECHERRY_ZTP_ID_LEN) {
    return false;
  }

  mbedtls_pk_init(&_mbKey);
  mbedtls_entropy_init(&_mbEntropy);
  mbedtls_ctr_drbg_init(&_mbCtrDrbg);
  mbedtls_x509write_csr_init(&_mbCsr);

  if(!_seedRandom(rfEnabled)) {
    return _finishCsrGen(false);
  }

  if(mbedtls_pk_setup(&_mbKey, mbedtls_pk_info_from_type(MBEDTLS_PK_ECKEY)) != 0) {
    return _finishCsrGen(false);
  }

  if(mbedtls_ecp_gen_key(MBEDTLS_ECP_DP_SECP256R1, mbedtls_pk_ec(_mbKey), mbedtls_ctr_drbg_random,
                         &_mbCtrDrbg) != 0) {
    return _finishCsrGen(false);
  }

  if(mbedtls_pk_write_key_pem(&_mbKey, (unsigned char*) _pkeyBuf, BLUECHERRY_ZTP_PKEY_BUF_SIZE) !=
     0) {
    return _finishCsrGen(false);
  }

  mbedtls_x509write_csr_set_md_alg(&_mbCsr, MBEDTLS_MD_SHA256);
  mbedtls_x509write_csr_set_key(&_mbCsr, &_mbKey);

  snprintf(_subjBuf, BLUECHERRY_ZTP_SUBJ_BUF_SIZE, "C=BE,CN=%s.%s", _bcTypeId, _bcDevId);
  if(mbedtls_x509write_csr_set_subject_name(&_mbCsr, _subjBuf) != 0) {
    return _finishCsrGen(false);
  }

  ret = mbedtls_x509write_csr_der(&_mbCsr, csrBuf, BLUECHERRY_ZTP_CERT_BUF_SIZE,
                                  mbedtls_ctr_drbg_random, &_mbCtrDrbg);
  if(ret < 0) {
    printf("Failed to write CSR: -0x%04X\n", -ret);
    return _finishCsrGen(false);
  }

  size_t offset = BLUECHERRY_ZTP_CERT_BUF_SIZE - ret;
  _csr.length = ret;
  memcpy(_csr.buffer, csrBuf + offset, _csr.length);

  return _finishCsrGen(true);
}

bool BlueCherryZTP::requestSignedCertificate()
{
  int ret;
  walter_modem_rsp_t rsp = {};
  uint8_t buf[BLUECHERRY_ZTP_CERT_BUF_SIZE];
  uint8_t coapData[BLUECHERRY_ZTP_CERT_BUF_SIZE];
  ZTP_CBOR cbor;

  ztp_cbor_init(&cbor, buf, BLUECHERRY_ZTP_CERT_BUF_SIZE);
  mbedtls_x509_crt_init(&_mbCrt);

  if(ztp_cbor_encode_bytes(&cbor, _csr.buffer, _csr.length) < 0) {
    printf("Failed to encode CSR\n");
    return false;
  }

  _modem->urcSetEventHandler(ztpURCHandler, NULL);

  // Send second CoAP
  if(!_modem->coapSetOptions(COAP_PROFILE, WALTER_MODEM_COAP_OPT_SET,
                             WALTER_MODEM_COAP_OPT_CODE_URI_PATH, ZTP_SERV_API_VERSION)) {
    printf("Failed to configure ZTP CoAP URI path for API version\n");
  }

  if(!_modem->coapSetOptions(COAP_PROFILE, WALTER_MODEM_COAP_OPT_EXTEND,
                             WALTER_MODEM_COAP_OPT_CODE_URI_PATH, ZTP_SERV_CSR_PATH)) {
    printf("Failed to configure ZTP CoAP URI path for CSR signing\n");
  }

  if(!_modem->coapSendData(COAP_PROFILE, WALTER_MODEM_COAP_SEND_TYPE_CON,
                           WALTER_MODEM_COAP_SEND_METHOD_GET, ztp_cbor_size(&cbor), buf)) {
    printf("Failed to send ZTP CoAP datagram\n");
    return false;
  }

  _coapRingReceived = false;
  int i = BLUECHERRY_ZTP_COAP_TIMEOUT;
  printf("Awaiting ZTP CoAP ring.");
  while(i && !_coapRingReceived) {
    printf(".");
    DELAY(1000);
    i--;
  }
  printf("\n");

  if(!_modem->coapReceive(COAP_PROFILE, _coapRingMsgId, coapData, sizeof(coapData), &rsp)) {
    printf("Failed to receive ZTP CoAP message\n");
    return false;
  }

  if(i < 1) {
    printf("Failed to receive response from ZTP COAP server\n");
    return false;
  }

  size_t decodedSize;
  ret = ztp_cbor_decode_certificate(coapData, rsp.data.coapResponse.length, buf, &decodedSize);
  if(ret < 0) {
    printf("Failed to decode certificate: %d\n", ret);
    return false;
  }

  // Parse the DER-encoded certificate
  ret = mbedtls_x509_crt_parse_der(&_mbCrt, buf, decodedSize);
  if(ret < 0) {
    printf("Failed to parse DER certificate, error code: -0x%x\n", -ret);
    mbedtls_x509_crt_free(&_mbCrt);
    return false;
  }

  // Convert the certificate to PEM format
  size_t pemLen;
  ret = mbedtls_pem_write_buffer("-----BEGIN CERTIFICATE-----\n", "-----END CERTIFICATE-----\n",
                                 _mbCrt.raw.p, _mbCrt.raw.len, buf, BLUECHERRY_ZTP_CERT_BUF_SIZE,
                                 &pemLen);
  if(ret < 0) {
    printf("Failed to write PEM: -0x%04X\n", -ret);
    mbedtls_x509_crt_free(&_mbCrt);
    return false;
  }

  memcpy(_certBuf, buf, pemLen);
  _certBuf[pemLen] = '\0';

  mbedtls_x509_crt_free(&_mbCrt);
  return true;
}
