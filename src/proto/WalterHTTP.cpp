/**
 * @file WalterHTTP.cpp
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
#if CONFIG_WALTER_MODEM_ENABLE_HTTP
#pragma region PRIVATE_METHODS
void WalterModem::_dispatchEvent(WalterModemHttpEvent event, int profileId)
{
  WalterModemEventHandler* handler = _eventHandlers + WALTER_MODEM_EVENT_TYPE_HTTP;
  if(handler->httpHandler == nullptr) {
    return;
  }

  auto start = std::chrono::steady_clock::now();
  handler->httpHandler(event, profileId, handler->args);
  _checkEventDuration(start);
}
#pragma endregion

#pragma region PUBLIC_METHODS
bool WalterModem::httpConfigProfile(uint8_t profileId, const char* serverName, uint16_t port,
                                    uint8_t tlsProfileId, bool useBasicAuth, const char* authUser,
                                    const char* authPass, uint16_t maxTimeout, uint16_t cnxTimeout,
                                    uint8_t inactivityTimeout, WalterModemRsp* rsp,
                                    walterModemCb cb, void* args)
{
  if(profileId >= WALTER_MODEM_MAX_HTTP_PROFILES) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
  }

  if(tlsProfileId && port == 80) {
    port = 443;
  }

  WalterModemBuffer* stringsBuffer = _getFreeBuffer();
  stringsBuffer->size +=
      sprintf((char*) stringsBuffer->data, "AT+SQNHTTPCFG=%d,\"%s\",%d,%d,\"%s\",\"%s\"", profileId,
              serverName, port, useBasicAuth, authUser, authPass);

  if(tlsProfileId) {
    stringsBuffer->size += sprintf((char*) stringsBuffer->data + stringsBuffer->size, ",1");
  } else {
    stringsBuffer->size += sprintf((char*) stringsBuffer->data + stringsBuffer->size, ",0");
  }

  /**
   * cnxTimeout needs to be larger then maxTimout, otherwise modem will return error.
   */
  if(cnxTimeout > maxTimeout) {
    _returnState(WALTER_MODEM_STATE_ERROR);
  }

  stringsBuffer->size +=
      sprintf((char*) stringsBuffer->data + stringsBuffer->size, ",%u,,", maxTimeout);

  if(tlsProfileId) {
    stringsBuffer->size +=
        sprintf((char*) stringsBuffer->data + stringsBuffer->size, "%u,", tlsProfileId);
  } else {
    stringsBuffer->size += sprintf((char*) stringsBuffer->data + stringsBuffer->size, ",");
  }

  stringsBuffer->size += sprintf((char*) stringsBuffer->data + stringsBuffer->size, "%u,%u",
                                 cnxTimeout, inactivityTimeout);

  _runCmd(arr((const char*) stringsBuffer->data), "OK", rsp, cb, args, NULL, NULL,
          WALTER_MODEM_CMD_TYPE_TX_WAIT, NULL, 0, stringsBuffer);

  _returnAfterReply();
}

bool WalterModem::httpConnect(uint8_t profileId, WalterModemRsp* rsp, walterModemCb cb, void* args)
{
  if(profileId >= WALTER_MODEM_MAX_HTTP_PROFILES) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
  }

  if(_httpContextSet[profileId].connected) {
    _returnState(WALTER_MODEM_STATE_OK);
  }

  _runCmd(arr("AT+SQNHTTPCONNECT=", _atNum(profileId)), "+SQNHTTPCONNECT: ", rsp, cb, args);
  _returnAfterReply();
}

bool WalterModem::httpClose(uint8_t profileId, WalterModemRsp* rsp, walterModemCb cb, void* args)
{
  if(profileId >= WALTER_MODEM_MAX_HTTP_PROFILES) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
  }

  _runCmd(arr("AT+SQNHTTPDISCONNECT=", _atNum(profileId)), "OK", rsp, cb, args);
  _returnAfterReply();
}

bool WalterModem::httpGetContextStatus(uint8_t profileId)
{
  if(profileId >= WALTER_MODEM_MAX_HTTP_PROFILES) {
    return false;
  }

  /* note: in my observation the SQNHTTPCONNECT command is to be avoided.
   * if the connection is closed by the server, you will not even
   * receive a +SQNHTTPSH disconnected message (you will on the next
   * connect attempt). reconnect will be impossible even if you try
   * to manually disconnect.
   * and a SQNHTTPQRY will still work and create its own implicit connection.
   *
   * (too bad: according to the docs SQNHTTPCONNECT is mandatory for
   * TLS connections)
   */
  return _httpContextSet[profileId].connected;
}

bool WalterModem::httpQuery(uint8_t profileId, const char* uri,
                            WalterModemHttpQueryCmd httpQueryCmd, char* contentTypeBuf,
                            uint16_t contentTypeBufSize, const char* extraHeaderLine,
                            WalterModemRsp* rsp, walterModemCb cb, void* args)
{
  if(profileId >= WALTER_MODEM_MAX_HTTP_PROFILES) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
  }

  if(_httpContextSet[profileId].state != WALTER_MODEM_HTTP_CONTEXT_STATE_IDLE) {
    _returnState(WALTER_MODEM_STATE_BUSY);
  }

  _httpContextSet[profileId].contentType = contentTypeBuf;
  _httpContextSet[profileId].contentTypeSize = contentTypeBufSize;

  auto completeHandler = [](WalterModemCmd* cmd, WalterModemState result) {
    WalterModemHttpContext* ctx = (WalterModemHttpContext*) cmd->completeHandlerArg;

    if(result == WALTER_MODEM_STATE_OK) {
      ctx->state = WALTER_MODEM_HTTP_CONTEXT_STATE_EXPECT_RING;
    }
  };

  WalterModemBuffer* stringsBuffer = _getFreeBuffer();
  stringsBuffer->size += sprintf((char*) stringsBuffer->data, "AT+SQNHTTPQRY=%d,%d,\"%s\"",
                                 profileId, httpQueryCmd, uri);

  if(extraHeaderLine != NULL && strlen(extraHeaderLine) > 0) {
    stringsBuffer->size +=
        sprintf((char*) stringsBuffer->data + stringsBuffer->size, ",\"%s\"", extraHeaderLine);
  }

  _runCmd(arr((const char*) stringsBuffer->data), "OK", rsp, cb, args, completeHandler,
          (void*) (_httpContextSet + profileId), WALTER_MODEM_CMD_TYPE_TX_WAIT, NULL, 0,
          stringsBuffer);

  _returnAfterReply();
}

bool WalterModem::httpSend(uint8_t profileId, const char* uri, uint8_t* data, uint16_t dataSize,
                           WalterModemHttpSendCmd httpSendCmd,
                           WalterModemHttpPostParam httpPostParam, char* contentTypeBuf,
                           uint16_t contentTypeBufSize, WalterModemRsp* rsp, walterModemCb cb,
                           void* args)
{
  if(profileId >= WALTER_MODEM_MAX_HTTP_PROFILES) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
  }

  if(_httpContextSet[profileId].state != WALTER_MODEM_HTTP_CONTEXT_STATE_IDLE) {
    _returnState(WALTER_MODEM_STATE_BUSY);
  }

  _httpContextSet[profileId].contentType = contentTypeBuf;
  _httpContextSet[profileId].contentTypeSize = contentTypeBufSize;

  auto completeHandler = [](WalterModemCmd* cmd, WalterModemState result) {
    WalterModemHttpContext* ctx = (WalterModemHttpContext*) cmd->completeHandlerArg;

    if(result == WALTER_MODEM_STATE_OK) {
      ctx->state = WALTER_MODEM_HTTP_CONTEXT_STATE_EXPECT_RING;
    }
  };

  WalterModemBuffer* stringsBuffer = _getFreeBuffer();
  if(httpPostParam == WALTER_MODEM_HTTP_POST_PARAM_UNSPECIFIED) {
    stringsBuffer->size += sprintf((char*) stringsBuffer->data, "AT+SQNHTTPSND=%d,%d,\"%s\",%d",
                                   profileId, httpSendCmd, uri, dataSize);
  } else {
    stringsBuffer->size +=
        sprintf((char*) stringsBuffer->data, "AT+SQNHTTPSND=%d,%d,\"%s\",%d,\"%d\"", profileId,
                httpSendCmd, uri, dataSize, httpPostParam);
  }

  _runCmd(arr((const char*) stringsBuffer->data), "OK", rsp, cb, args, completeHandler,
          (void*) (_httpContextSet + profileId), WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT, data, dataSize,
          stringsBuffer);

  _returnAfterReply();
}

bool WalterModem::httpDidRing(uint8_t profileId, uint8_t* targetBuf, uint16_t targetBufSize,
                              WalterModemRsp* rsp)
{
  /* this is by definition a blocking call without callback.
   * it is only used when the arduino user is not taking advantage of
   * the (TBI) ring notification events.
   */
  walterModemCb cb = NULL;
  void* args = NULL;

  if(_httpCurrentProfile != 0xff) {
    _returnState(WALTER_MODEM_STATE_ERROR);
  }

  if(profileId >= WALTER_MODEM_MAX_HTTP_PROFILES) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
  }

  if(_httpContextSet[profileId].state == WALTER_MODEM_HTTP_CONTEXT_STATE_IDLE) {
    _returnState(WALTER_MODEM_STATE_NOT_EXPECTING_RING);
  }

  if(_httpContextSet[profileId].state == WALTER_MODEM_HTTP_CONTEXT_STATE_EXPECT_RING) {
    _returnState(WALTER_MODEM_STATE_AWAITING_RING);
  }

  if(_httpContextSet[profileId].state != WALTER_MODEM_HTTP_CONTEXT_STATE_GOT_RING) {
    _returnState(WALTER_MODEM_STATE_ERROR);
  }

  /* ok, got ring. http context fields have been filled.
   * http status 0 means: timeout (or also disconnected apparently) */
  if(_httpContextSet[profileId].httpStatus == 0) {
    _httpContextSet[profileId].state = WALTER_MODEM_HTTP_CONTEXT_STATE_IDLE;
    _returnState(WALTER_MODEM_STATE_ERROR);
  }

  /* in the case of chunked data contentLenght can be zero! */

  _httpCurrentProfile = profileId;

  auto completeHandler = [](WalterModemCmd* cmd, WalterModemState result) {
    _httpContextSet[_httpCurrentProfile].state = WALTER_MODEM_HTTP_CONTEXT_STATE_IDLE;
    _httpCurrentProfile = 0xff;
  };
  _runCmd(arr("AT+SQNHTTPRCV=", _atNum(profileId)), "OK", rsp, cb, args, completeHandler, NULL,
          WALTER_MODEM_CMD_TYPE_TX_WAIT, targetBuf, targetBufSize);
  _returnAfterReply();
}

void WalterModem::httpSetEventHandler(walterModemHttpEventHandler handler, void* args)
{
  _eventHandlers[WALTER_MODEM_EVENT_TYPE_HTTP].httpHandler = handler;
  _eventHandlers[WALTER_MODEM_EVENT_TYPE_HTTP].args = args;
}
#pragma endregion
#endif