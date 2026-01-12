/**
 * @file WalterCoAP.cpp
 * @author Daan Pape <daan@dptechnics.com>
 * @author Arnoud Devoogdt <arnoud@dptechnics.com>
 * @date 5 Nov 2025
 * @copyright DPTechnics bv <info@dptechnics.com>
 * @brief Walter Modem library
 *
 * @section LICENSE
 *
 * Copyright (C) 2025, DPTechnics bv
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
#pragma endregion

#pragma region PUBLIC_METHODS
bool WalterModem::coapReceive(int profile_id, int message_id, uint8_t* buf, size_t buf_size,
                              walter_modem_rsp_t* rsp, walter_modem_cb_t cb, void* args)
{
  if(profile_id >= WALTER_MODEM_MAX_COAP_PROFILES) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
  }

  size_t readable_size = (buf_size > 1024) ? 1024 : buf_size;

  walter_modem_buffer_t* stringsBuffer = _getFreeBuffer();
  stringsBuffer->size += sprintf((char*) stringsBuffer->data, "AT+SQNCOAPRCV=%d,%u,%u", profile_id,
                                 message_id, readable_size);

  _runCmd(arr((const char*) stringsBuffer->data), "OK", rsp, cb, args, NULL, NULL,
          WALTER_MODEM_CMD_TYPE_TX_WAIT, buf, readable_size, stringsBuffer);
  _returnAfterReply();
}

bool WalterModem::coapCreateContext(int profile_id, const char* hostname, int port,
                                    int tls_profile_id, int local_port, walter_modem_rsp_t* rsp,
                                    walter_modem_cb_t cb, void* args)
{
  if(profile_id >= WALTER_MODEM_MAX_COAP_PROFILES) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
  }

  if(_coapContextSet[profile_id].connected) {
    _returnState(WALTER_MODEM_STATE_OK);
  }

  walter_modem_buffer_t* stringsBuffer = _getFreeBuffer();
  stringsBuffer->size += sprintf((char*) stringsBuffer->data, "AT+SQNCOAPCREATE=%d,\"%s\",%d,",
                                 profile_id, hostname, port);

  if(local_port > -1) {
    stringsBuffer->size +=
        sprintf((char*) stringsBuffer->data + stringsBuffer->size, "%d", local_port);
  }

  stringsBuffer->size +=
      sprintf((char*) stringsBuffer->data + stringsBuffer->size, ",%d,60", tls_profile_id != 0);

  if(tls_profile_id) {
    stringsBuffer->size +=
        sprintf((char*) stringsBuffer->data + stringsBuffer->size, ",,%d", tls_profile_id);
  }

  _runCmd(arr((const char*) stringsBuffer->data), "OK", rsp, cb, args, NULL, NULL,
          WALTER_MODEM_CMD_TYPE_TX_WAIT, NULL, 0, stringsBuffer);
  _returnAfterReply();
}

bool WalterModem::coapClose(int profile_id, walter_modem_rsp_t* rsp, walter_modem_cb_t cb,
                            void* args)
{
  if(profile_id == 0) {
    _returnState(WALTER_MODEM_STATE_ERROR);
  }

  if(profile_id >= WALTER_MODEM_MAX_COAP_PROFILES) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
  }

  _runCmd(arr("AT+SQNCOAPCLOSE=", _atNum(profile_id)), "OK", rsp, cb, args);
  _returnAfterReply();
}

bool WalterModem::coapSetHeader(int profile_id, int message_id, const char* token,
                                walter_modem_rsp_t* rsp, walter_modem_cb_t cb, void* args)
{
  _runCmd(arr("AT+SQNCOAPHDR=", _atNum(profile_id), ",", _atNum(message_id), ",\"", token, "\""),
          "OK", rsp, cb, args);
  _returnAfterReply();
}

bool WalterModem::coapSetOptions(int profile_id, WalterModemCoapOptAction action,
                                 WalterModemCoapOptCode code, const char* const values,
                                 walter_modem_rsp_t* rsp, walter_modem_cb_t cb, void* args)
{
  if(profile_id == 0) {
    _returnState(WALTER_MODEM_STATE_ERROR);
  }

  if(action == WALTER_MODEM_COAP_OPT_READ) {
    /* not yet supported - add together with incoming socket con/coap response */
    _returnState(WALTER_MODEM_STATE_ERROR);
  }

  walter_modem_buffer_t* stringsBuffer = _getFreeBuffer();

  if(action == WALTER_MODEM_COAP_OPT_SET || action == WALTER_MODEM_COAP_OPT_EXTEND) {
    if(values && *values) {
      stringsBuffer->size += sprintf((char*) stringsBuffer->data, "AT+SQNCOAPOPT=%d,%d,%d,\"%s\"",
                                     profile_id, action, code, values);
    } else {
      stringsBuffer->size +=
          sprintf((char*) stringsBuffer->data, "AT+SQNCOAPOPT=%d,%d,%d", profile_id, action, code);
    }
  } else if(action == WALTER_MODEM_COAP_OPT_DELETE) {
    stringsBuffer->size +=
        sprintf((char*) stringsBuffer->data, "AT+SQNCOAPOPT=%d,%d,%d", profile_id, action, code);
  } else {
    /* make sure something sane is in the buffer if wrong action */
    stringsBuffer->size += sprintf((char*) stringsBuffer->data, "AT");
  }

  _runCmd(arr((const char*) stringsBuffer->data), "OK", rsp, cb, args, NULL, NULL,
          WALTER_MODEM_CMD_TYPE_TX_WAIT, NULL, 0, stringsBuffer);
  _returnAfterReply();
}

bool WalterModem::coapSendData(int profile_id, WalterModemCoapSendType type,
                               WalterModemCoapSendMethodRsp method_rsp, int buf_size, uint8_t* buf,
                               walter_modem_rsp_t* rsp, walter_modem_cb_t cb, void* args)
{
  _runCmd(arr("AT+SQNCOAPSEND=", _atNum(profile_id), ",", _atNum(type), ",", _atNum(method_rsp),
              ",", _atNum(buf_size)),
          "OK", rsp, cb, args, NULL, NULL, WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT, buf, buf_size);
  _returnAfterReply();
}

void WalterModem::setCoAPEventHandler(walterModemCoAPEventHandler handler, void* args)
{
  _eventHandlers[WALTER_MODEM_EVENT_TYPE_COAP].coapHandler = handler;
  _eventHandlers[WALTER_MODEM_EVENT_TYPE_COAP].args = args;
}

#pragma endregion
#pragma region DEPRICATION
bool WalterModem::coapGetContextStatus(int profileId)
{
  if(profileId >= WALTER_MODEM_MAX_COAP_PROFILES) {
    return false;
  }

  ESP_LOGW(
      "DEPRECATION",
      "this coapGetContextStatus method is deprecated and will be removed in future releases.");

  return _coapContextSet[profileId].connected;
}

bool WalterModem::coapDidRing(uint8_t profileId, uint8_t* targetBuf, uint16_t targetBufSize,
                              walter_modem_rsp_t* rsp)
{
  ESP_LOGW("DEPRECATION",
           "coapDidRing method is deprecated and will be removed in future releases. Use "
           "coapReceive(...) instead.");

  return coapReceive(profileId, -1, targetBuf, (size_t) targetBufSize, rsp, NULL, NULL);
}

void WalterModem::coapSetEventHandler(walterModemCoAPEventHandler handler, void* args)
{
  ESP_LOGW("DEPRECATION",
           "coapSetEventHandler method is deprecated and will be removed in future releases. "
           "Use setCoAPEventHandler(...) instead.");

  setCoAPEventHandler(handler, args);
}
#pragma endregion
#endif