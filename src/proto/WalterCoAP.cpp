/**
 * @file WalterCoAP.cpp
 * @author Daan Pape <daan@dptechnics.com>
 * @date 28 Mar 2025
 * @copyright DPTechnics bv
 * @brief Walter Modem library
 *
 * @section LICENSE
 *
 * Copyright (C) 2023, DPTechnics bv
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice, this list of
 *      conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above copyright notice, this list of
 *      conditions and the following disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *
 *   3. Neither the name of DPTechnics bv nor the names of its contributors may be used to endorse
 *      or promote products derived from this software without specific prior written permission.
 *
 *   4. This software, with or without modification, must only be used with a Walter board from
 *      DPTechnics bv.
 *
 *   5. Any software provided in binary form under this license must not be reverse engineered,
 *      decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY DPTECHNICS BV “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, NONINFRINGEMENT, AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL DPTECHNICS BV OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @section DESCRIPTION
 *
 * This file contains the headers of Walter's modem library.
 */

#include <WalterDefines.h>

#if CONFIG_WALTER_MODEM_ENABLE_COAP
#pragma region PRIVATE_METHODS
void WalterModem::_dispatchEvent(WalterModemCoapEvent event, int profileId)
{
  WalterModemEventHandler* handler = _eventHandlers + WALTER_MODEM_EVENT_TYPE_COAP;
  if(handler->coapHandler == nullptr) {
    return;
  }

  auto start = std::chrono::steady_clock::now();
  handler->coapHandler(event, profileId, handler->args);
  _checkEventDuration(start);
}
#pragma endregion

#pragma region PUBLIC_METHODS
bool WalterModem::coapDidRing(uint8_t profileId, uint8_t* targetBuf, uint16_t targetBufSize,
                              WalterModemRsp* rsp)
{
  ESP_LOGW("DEPRECATION",
           "this coapDidRing method is deprecated and will be removed in future releases. Use "
           "coapReceiveMessage instead.");

  int receive_count = 0;
  return coapReceiveMessage(profileId, targetBuf, targetBufSize, &receive_count, rsp, NULL, NULL);
}

bool WalterModem::coapReceiveMessage(uint8_t profile_id, uint8_t* buf, uint16_t buf_size,
                                     int* receive_count, WalterModemRsp* rsp, walterModemCb cb,
                                     void* args)
{
  if(profile_id == 0) {
    _returnState(WALTER_MODEM_STATE_ERROR);
  }

  if(profile_id >= WALTER_MODEM_MAX_COAP_PROFILES) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
  }

  uint8_t ringIdx;
  for(ringIdx = 0; ringIdx < WALTER_MODEM_COAP_MAX_PENDING_RINGS; ringIdx++) {
    if(_coapContextSet[profile_id].rings[ringIdx].messageId) {
      break;
    }
  }

  if(ringIdx == WALTER_MODEM_COAP_MAX_PENDING_RINGS) {
    _returnState(WALTER_MODEM_STATE_NO_DATA);
  }
  if(_coapContextSet[profile_id].rings[ringIdx].length == 0) {
    _returnState(WALTER_MODEM_STATE_OK);
  }
  WalterModemBuffer* stringsBuffer = _getFreeBuffer();
  stringsBuffer->size += sprintf((char*) stringsBuffer->data, "AT+SQNCOAPRCV=%d,%u,%u", profile_id,
                                 _coapContextSet[profile_id].rings[ringIdx].messageId,
                                 _coapContextSet[profile_id].rings[ringIdx].length);

  _runCmd(arr((const char*) stringsBuffer->data), "OK", rsp, cb, args, NULL, NULL,
          WALTER_MODEM_CMD_TYPE_TX_WAIT, buf, buf_size, stringsBuffer);

  _returnAfterReply();
}

bool WalterModem::coapCreateContext(uint8_t profileId, const char* serverName, int port,
                                    uint8_t tlsProfileId, int localPort, WalterModemRsp* rsp,
                                    walterModemCb cb, void* args)
{
  if(profileId >= WALTER_MODEM_MAX_COAP_PROFILES) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
  }

  if(_coapContextSet[profileId].connected) {
    _returnState(WALTER_MODEM_STATE_OK);
  }

  WalterModemBuffer* stringsBuffer = _getFreeBuffer();
  stringsBuffer->size += sprintf((char*) stringsBuffer->data, "AT+SQNCOAPCREATE=%d,\"%s\",%d,",
                                 profileId, serverName, port);

  if(localPort > -1) {
    stringsBuffer->size +=
        sprintf((char*) stringsBuffer->data + stringsBuffer->size, "%d", localPort);
  }

  stringsBuffer->size +=
      sprintf((char*) stringsBuffer->data + stringsBuffer->size, ",%d,60", tlsProfileId != 0);

  if(tlsProfileId) {
    stringsBuffer->size +=
        sprintf((char*) stringsBuffer->data + stringsBuffer->size, ",,%d", tlsProfileId);
  }

  _runCmd(arr((const char*) stringsBuffer->data), "+SQNCOAPCONNECTED: ", rsp, cb, args, NULL, NULL,
          WALTER_MODEM_CMD_TYPE_TX_WAIT, NULL, 0, stringsBuffer);

  _returnAfterReply();
}

bool WalterModem::coapClose(uint8_t profileId, WalterModemRsp* rsp, walterModemCb cb, void* args)
{
  if(profileId == 0) {
    _returnState(WALTER_MODEM_STATE_ERROR);
  }

  if(profileId >= WALTER_MODEM_MAX_COAP_PROFILES) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
  }

  _runCmd(arr("AT+SQNCOAPCLOSE=", _atNum(profileId)), "+SQNCOAPCLOSED: ", rsp, cb, args);
  _returnAfterReply();
}

bool WalterModem::coapGetContextStatus(uint8_t profileId)
{
  if(profileId >= WALTER_MODEM_MAX_COAP_PROFILES) {
    return false;
  }

  return _coapContextSet[profileId].connected;
}

bool WalterModem::coapSetHeader(uint8_t profileId, int messageId, const char* token,
                                WalterModemRsp* rsp, walterModemCb cb, void* args)
{
  _runCmd(arr("AT+SQNCOAPHDR=", _atNum(profileId), ",", _atNum(messageId), ",\"", token, "\""),
          "OK", rsp, cb, args);
  _returnAfterReply();
}

bool WalterModem::coapSetOptions(uint8_t profileId, WalterModemCoapOptAction action,
                                 WalterModemCoapOptCode code, const char* const values,
                                 WalterModemRsp* rsp, walterModemCb cb, void* args)
{
  if(profileId == 0) {
    _returnState(WALTER_MODEM_STATE_ERROR);
  }

  if(action == WALTER_MODEM_COAP_OPT_READ) {
    /* not yet supported - add together with incoming socket con/coap response */
    _returnState(WALTER_MODEM_STATE_ERROR);
  }

  WalterModemBuffer* stringsBuffer = _getFreeBuffer();

  if(action == WALTER_MODEM_COAP_OPT_SET || action == WALTER_MODEM_COAP_OPT_EXTEND) {
    if(values && *values) {
      stringsBuffer->size += sprintf((char*) stringsBuffer->data, "AT+SQNCOAPOPT=%d,%d,%d,\"%s\"",
                                     profileId, action, code, values);
    } else {
      stringsBuffer->size +=
          sprintf((char*) stringsBuffer->data, "AT+SQNCOAPOPT=%d,%d,%d", profileId, action, code);
    }
  } else if(action == WALTER_MODEM_COAP_OPT_DELETE) {
    stringsBuffer->size +=
        sprintf((char*) stringsBuffer->data, "AT+SQNCOAPOPT=%d,%d,%d", profileId, action, code);
  } else {
    /* make sure something sane is in the buffer if wrong action */
    stringsBuffer->size += sprintf((char*) stringsBuffer->data, "AT");
  }

  _runCmd(arr((const char*) stringsBuffer->data), "OK", rsp, cb, args, NULL, NULL,
          WALTER_MODEM_CMD_TYPE_TX_WAIT, NULL, 0, stringsBuffer);
  _returnAfterReply();
}

bool WalterModem::coapSendData(uint8_t profileId, WalterModemCoapSendType type,
                               WalterModemCoapSendMethodRsp methodRsp, int length, uint8_t* payload,
                               WalterModemRsp* rsp, walterModemCb cb, void* args)
{
  _runCmd(arr("AT+SQNCOAPSEND=", _atNum(profileId), ",", _atNum(type), ",", _atNum(methodRsp), ",",
              _atNum(length)),
          "OK", rsp, cb, args, NULL, NULL, WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT, payload, length);
  _returnAfterReply();
}

void WalterModem::coapSetEventHandler(walterModemCoAPEventHandler handler, void* args)
{
  _eventHandlers[WALTER_MODEM_EVENT_TYPE_COAP].coapHandler = handler;
  _eventHandlers[WALTER_MODEM_EVENT_TYPE_COAP].args = args;
}
#pragma endregion
#endif