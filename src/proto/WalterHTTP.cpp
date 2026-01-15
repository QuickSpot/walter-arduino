/**
 * @file WalterHTTP.cpp
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
#if CONFIG_WALTER_MODEM_ENABLE_HTTP
#pragma region PRIVATE_METHODS
#pragma endregion

#pragma region PUBLIC_METHODS
bool WalterModem::httpConfigProfile(int profile_id, const char* hostname, uint16_t port,
                                    uint8_t tls_profile_id, bool use_basic_auth,
                                    const char* auth_user, const char* auth_pass,
                                    uint16_t max_timeout, uint16_t cnx_timeout,
                                    uint8_t inactivity_timeout, walter_modem_rsp_t* rsp,
                                    walter_modem_cb_t cb, void* args)
{
  if(profile_id >= WALTER_MODEM_MAX_HTTP_PROFILES) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
  }

  if(tls_profile_id && port == 80) {
    port = 443;
  }

  walter_modem_buffer_t* stringsBuffer = _getFreeBuffer();
  stringsBuffer->size +=
      sprintf((char*) stringsBuffer->data, "AT+SQNHTTPCFG=%d,\"%s\",%d,%d,\"%s\",\"%s\"",
              profile_id, hostname, port, use_basic_auth, auth_user, auth_pass);

  if(tls_profile_id) {
    stringsBuffer->size += sprintf((char*) stringsBuffer->data + stringsBuffer->size, ",1");
  } else {
    stringsBuffer->size += sprintf((char*) stringsBuffer->data + stringsBuffer->size, ",0");
  }

  /**
   * cnxTimeout needs to be larger then maxTimout, otherwise modem will return error.
   */
  if(cnx_timeout > max_timeout) {
    _returnState(WALTER_MODEM_STATE_ERROR);
  }

  stringsBuffer->size +=
      sprintf((char*) stringsBuffer->data + stringsBuffer->size, ",%u,,", max_timeout);

  if(tls_profile_id) {
    stringsBuffer->size +=
        sprintf((char*) stringsBuffer->data + stringsBuffer->size, "%u,", tls_profile_id);
  } else {
    stringsBuffer->size += sprintf((char*) stringsBuffer->data + stringsBuffer->size, ",");
  }

  stringsBuffer->size += sprintf((char*) stringsBuffer->data + stringsBuffer->size, "%u,%u",
                                 cnx_timeout, inactivity_timeout);

  _runCmd(arr((const char*) stringsBuffer->data), "OK", rsp, cb, args, NULL, NULL,
          WALTER_MODEM_CMD_TYPE_TX_WAIT, NULL, 0, stringsBuffer);
  _returnAfterReply();
}

bool WalterModem::httpConnect(int profile_id, walter_modem_rsp_t* rsp, walter_modem_cb_t cb,
                              void* args)
{
  if(profile_id >= WALTER_MODEM_MAX_HTTP_PROFILES) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
  }

  if(_httpContextSet[profile_id].connected) {
    _returnState(WALTER_MODEM_STATE_OK);
  }

  _runCmd(arr("AT+SQNHTTPCONNECT=", _atNum(profile_id)), "+SQNHTTPCONNECT: ", rsp, cb, args);
  _returnAfterReply();
}

bool WalterModem::httpClose(int profile_id, walter_modem_rsp_t* rsp, walter_modem_cb_t cb,
                            void* args)
{
  if(profile_id >= WALTER_MODEM_MAX_HTTP_PROFILES) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
  }

  _runCmd(arr("AT+SQNHTTPDISCONNECT=", _atNum(profile_id)), "OK", rsp, cb, args);
  _returnAfterReply();
}

bool WalterModem::httpQuery(int profile_id, const char* uri, WalterModemHttpQueryCmd http_query_cmd,
                            char* content_type_buf, uint16_t content_type_buf_size,
                            const char* extra_header_line, walter_modem_rsp_t* rsp,
                            walter_modem_cb_t cb, void* args)
{
  if(profile_id >= WALTER_MODEM_MAX_HTTP_PROFILES) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
  }

  walter_modem_buffer_t* stringsBuffer = _getFreeBuffer();
  stringsBuffer->size += sprintf((char*) stringsBuffer->data, "AT+SQNHTTPQRY=%d,%d,\"%s\"",
                                 profile_id, http_query_cmd, uri);

  if(extra_header_line != NULL && strlen(extra_header_line) > 0) {
    stringsBuffer->size +=
        sprintf((char*) stringsBuffer->data + stringsBuffer->size, ",\"%s\"", extra_header_line);
  }

  _runCmd(arr((const char*) stringsBuffer->data), "OK", rsp, cb, args, NULL, NULL,
          WALTER_MODEM_CMD_TYPE_TX_WAIT, NULL, 0, stringsBuffer);
  _returnAfterReply();
}

bool WalterModem::httpSend(int profile_id, const char* uri, uint8_t* buf, uint16_t buf_size,
                           WalterModemHttpSendCmd http_send_cmd,
                           WalterModemHttpPostParam http_post_param, char* content_type_buf,
                           uint16_t content_type_buf_size, walter_modem_rsp_t* rsp,
                           walter_modem_cb_t cb, void* args)
{
  if(profile_id >= WALTER_MODEM_MAX_HTTP_PROFILES) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
  }

  walter_modem_buffer_t* stringsBuffer = _getFreeBuffer();
  if(http_post_param == WALTER_MODEM_HTTP_POST_PARAM_UNSPECIFIED) {
    stringsBuffer->size += sprintf((char*) stringsBuffer->data, "AT+SQNHTTPSND=%d,%d,\"%s\",%d",
                                   profile_id, http_send_cmd, uri, buf_size);
  } else {
    stringsBuffer->size +=
        sprintf((char*) stringsBuffer->data, "AT+SQNHTTPSND=%d,%d,\"%s\",%d,\"%d\"", profile_id,
                http_send_cmd, uri, buf_size, http_post_param);
  }

  _runCmd(arr((const char*) stringsBuffer->data), "OK", rsp, cb, args, NULL, NULL,
          WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT, buf, buf_size, stringsBuffer);
  _returnAfterReply();
}

bool WalterModem::httpDidRing(uint8_t profileId, uint8_t* targetBuf, uint16_t targetBufSize,
                              walter_modem_rsp_t* rsp)
{
  ESP_LOGW("DEPRECATION", "The httpDidRing method is deprecated and will be removed in future "
                          "releases. Use httpReceive(...) instead.");

  return httpReceive(profileId, targetBuf, (size_t) targetBufSize, rsp, NULL, NULL);
}

bool WalterModem::httpReceive(int profile_id, uint8_t* buf, size_t buf_size,
                              walter_modem_rsp_t* rsp, walter_modem_cb_t cb, void* args)
{
  if(profile_id >= WALTER_MODEM_MAX_HTTP_PROFILES) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
  }

  size_t readable_size = (buf_size > 1500) ? 1500 : buf_size;

  // Known bug: CME ERROR 4 when attempting to receive a HTTP payload with a fixed size.
  // Omit size for now and let rsp processor handle payload size
  walter_modem_buffer_t* stringsBuffer = _getFreeBuffer();
  stringsBuffer->size += sprintf((char*) stringsBuffer->data, "AT+SQNHTTPRCV=%d", profile_id);

  _runCmd(arr((const char*) stringsBuffer->data), "OK", rsp, cb, args, NULL, NULL,
          WALTER_MODEM_CMD_TYPE_TX_WAIT, buf, readable_size, stringsBuffer);
  _returnAfterReply();
}

void WalterModem::setHTTPEventHandler(walterModemHttpEventHandler handler, void* args)
{
  _eventHandlers[WALTER_MODEM_EVENT_TYPE_HTTP].httpHandler = handler;
  _eventHandlers[WALTER_MODEM_EVENT_TYPE_HTTP].args = args;
}

#pragma endregion
#pragma region DEPRICATION
bool WalterModem::httpGetContextStatus(uint8_t profile_id)
{
  if(profile_id >= WALTER_MODEM_MAX_HTTP_PROFILES) {
    return false;
  }

  ESP_LOGW("DEPRECATION",
           "The httpGetContextStatus method is deprecated and will be removed in future releases");

  return _httpContextSet[profile_id].connected;
}

void WalterModem::httpSetEventHandler(walterModemHttpEventHandler handler, void* args)
{
  ESP_LOGW("DEPRECATION",
           "httpSetEventHandler is deprecated and will be removed in future releases. Use "
           "setHTTPEventHandler(...) instead.");

  _eventHandlers[WALTER_MODEM_EVENT_TYPE_HTTP].httpHandler = handler;
  _eventHandlers[WALTER_MODEM_EVENT_TYPE_HTTP].args = args;
}

#pragma endregion
#endif