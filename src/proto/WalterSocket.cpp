/**
 * @file WalterSocket.cpp
 * @author Daan Pape <daan@dptechnics.com>
 * @date 24 Apr 2025
 * @copyright DPTechnics bv
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
 * This file contains the Socket implementation of the Walter Modem library.
 */

#include <WalterDefines.h>
#include <WalterModem.h>
#include <esp_log.h>

#if CONFIG_WALTER_MODEM_ENABLE_SOCKETS
#pragma region PRIVATE_METHODS
WalterModemSocket* WalterModem::_socketReserve()
{
  WalterModemSocket* sock = NULL;

  for(int i = 0; i < WALTER_MODEM_MAX_SOCKETS; ++i) {
    if(_socketSet[i].state == WALTER_MODEM_SOCKET_STATE_FREE) {
      sock = _socketSet + i;
      sock->state = WALTER_MODEM_SOCKET_STATE_RESERVED;
      sock->id = i + 1;
      break;
    }
  }

  return sock;
}

WalterModemSocket* WalterModem::_socketGet(int id)
{
  for(int i = 0; i < WALTER_MODEM_MAX_SOCKETS; ++i) {
    if(_socketSet[i].id == id) {
      return _socketSet + i;
    }
  }

  return NULL;
}

bool WalterModem::_socketUpdateStates()
{
  WalterModemRsp* rsp = NULL;
  walterModemCb cb = NULL;
  void* args = NULL;

  _runCmd({ "AT+SQNSS?" }, "OK", rsp, cb, args);
  _returnAfterReply();
}
#pragma endregion

#pragma region PUBLIC_METHODS
bool WalterModem::socketConfig(int profile_id, int pdp_ctx_id, uint16_t mtu,
                               uint16_t exchange_timeout, uint16_t conn_timeout,
                               uint16_t send_delay_ms, WalterModemRsp* rsp, walterModemCb cb,
                               void* args)
{
  WalterModemSocket* sock = _socketGet(profile_id);

  if(sock == NULL) {
    _returnState(WALTER_MODEM_STATE_NO_FREE_SOCKET);
  }

  sock->pdpContextId = pdp_ctx_id;
  sock->mtu = mtu;
  sock->exchangeTimeout = exchange_timeout;
  sock->connTimeout = conn_timeout;
  sock->sendDelayMs = send_delay_ms;

  auto completeHandler = [](WalterModemCmd* cmd, WalterModemState result) {
    WalterModemSocket* sock = (WalterModemSocket*) cmd->completeHandlerArg;
    if(result == WALTER_MODEM_STATE_OK) {
      sock->state = WALTER_MODEM_SOCKET_STATE_READY;
    }
  };

  _runCmd(arr("AT+SQNSCFG=", _digitStr(profile_id), ",", _digitStr(pdp_ctx_id), ",", _atNum(mtu),
              ",", _atNum(exchange_timeout), ",", _atNum(conn_timeout * 10), ",",
              _atNum(send_delay_ms / 100)),
          "OK", rsp, cb, args, completeHandler, sock);
  _returnAfterReply();
}

bool WalterModem::socketConfigExtended(int profile_id, WalterModemSocketRingMode ring_mode,
                                       WalterModemSocketRecvMode recv_mode, int keep_alive,
                                       WalterModemSocketListenMode listen_mode,
                                       WalterModemsocketSendMode send_mode, WalterModemRsp* rsp,
                                       walterModemCb cb, void* args)
{
  WalterModemSocket* sock = _socketGet(profile_id);
  if(sock == NULL) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
  }

  _runCmd(arr("AT+SQNSCFGEXT=", _digitStr(sock->id), ",", _digitStr(ring_mode), ",",
              _digitStr(recv_mode), ",", _digitStr(keep_alive), ",", _digitStr(listen_mode), ",",
              _digitStr(send_mode)),
          "OK", rsp, cb, args, NULL, sock);
  _returnAfterReply();
}

bool WalterModem::socketConfigSecure(int profile_id, bool enable_tls, int tls_profile_id,
                                     WalterModemRsp* rsp, walterModemCb cb, void* args)
{
  WalterModemSocket* sock = _socketGet(profile_id);
  if(sock == NULL) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
  }

  _runCmd(arr("AT+SQNSSCFG=", _digitStr(sock->id), ",", enable_tls ? "1" : "0", ",",
              _digitStr(tls_profile_id)),
          "OK", rsp, cb, args, NULL);
  _returnAfterReply();
}

bool WalterModem::socketDial(const char* remoteHost, uint16_t remotePort, uint16_t localPort,
                             WalterModemRsp* rsp, walterModemCb cb, void* args,
                             WalterModemSocketProto protocol,
                             WalterModemSocketAcceptAnyRemote acceptAnyRemote, int profileId)
{
  WalterModemSocket* sock = _socketGet(profileId);
  if(sock == NULL) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
  }

  sock->protocol = protocol;
  sock->acceptAnyRemote = acceptAnyRemote;
  _strncpy_s(sock->remoteHost, remoteHost, WALTER_MODEM_HOSTNAME_MAX_SIZE);
  sock->remotePort = remotePort;
  sock->localPort = localPort;

  auto completeHandler = [](WalterModemCmd* cmd, WalterModemState result) {
    WalterModemSocket* sock = (WalterModemSocket*) cmd->completeHandlerArg;

    if(result == WALTER_MODEM_STATE_OK) {
      sock->state = WALTER_MODEM_SOCKET_STATE_OPENED;
    }
  };

  _runCmd(arr("AT+SQNSD=", _digitStr(sock->id), ",", _digitStr(sock->protocol), ",",
              _atNum(sock->remotePort), ",", _atStr(sock->remoteHost), ",0,",
              _atNum(sock->localPort), ",1,", _digitStr(sock->acceptAnyRemote), ",0"),
          "OK", rsp, cb, args, completeHandler, sock);
  _returnAfterReply();
}

bool WalterModem::socketClose(WalterModemRsp* rsp, walterModemCb cb, void* args, int profileId)
{
  WalterModemSocket* sock = _socketGet(profileId);
  if(sock == NULL) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
  }

  auto completeHandler = [](WalterModemCmd* cmd, WalterModemState result) {
    WalterModemSocket* sock = (WalterModemSocket*) cmd->completeHandlerArg;

    if(result == WALTER_MODEM_STATE_OK) {
      _socketRelease(sock);
    }
  };

  _runCmd(arr("AT+SQNSH=", _digitStr(sock->id)), "OK", rsp, cb, args, completeHandler, sock);
  _returnAfterReply();
}

bool WalterModem::socketSend(uint8_t* data, uint32_t dataSize, WalterModemRsp* rsp,
                             walterModemCb cb, void* args, WalterModemRAI rai, int profileId)
{
  /* AT+SQNSSEND is a legacy AT command! */
  WalterModemSocket* sock = _socketGet(profileId);
  if(sock == NULL) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
  }

  _runCmd(arr("AT+SQNSSENDEXT=", _digitStr(sock->id), ",", _atNum(dataSize), ",", _digitStr(rai)),
          "OK", rsp, cb, args, NULL, NULL, WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT, data, dataSize);
  _returnAfterReply();
}

bool WalterModem::socketSend(char* str, WalterModemRsp* rsp, walterModemCb cb, void* args,
                             WalterModemRAI rai, int profileId)
{
  return socketSend((uint8_t*) str, strlen(str), rsp, cb, args, rai, profileId);
}

bool WalterModem::socketAccept(WalterModemRsp* rsp, walterModemCb cb, void* args, int profileId,
                               bool commandMode)
{
  WalterModemSocket* sock = _socketGet(profileId);
  if(sock == NULL) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
  }

  _runCmd(arr("AT+SQNSA=", _digitStr(sock->id), ",", _digitStr(commandMode)), "OK", rsp, cb, args,
          NULL, NULL, WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT);
  _returnAfterReply();
}

bool WalterModem::socketListen(WalterModemRsp* rsp, walterModemCb cb, void* args, int profileId,
                               WalterModemSocketProto protocol,
                               WalterModemSocketListenState listenState, int socketListenPort)
{
  if(protocol == WALTER_MODEM_SOCKET_PROTO_TCP) {
    _runCmd(arr("AT+SQNSL=", _digitStr(profileId), ",", _digitStr(listenState), ",",
                _atNum(socketListenPort)),
            "OK", rsp, cb, args, NULL, NULL, WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT);
    _returnAfterReply();
  } else {
    _runCmd(arr("AT+SQNSLUDP=", _digitStr(profileId), ",", _digitStr(listenState), ",",
                _atNum(socketListenPort)),
            "OK", rsp, cb, args, NULL, NULL, WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT);
    _returnAfterReply();
  }
}

uint16_t WalterModem::socketAvailable(int profileId)
{
  ESP_LOGW("DEPRECATION",
           "socketAvailable method is deprecated and will be removed in future releases.");
  return 0;
}

bool WalterModem::socketReceive(uint16_t receiveCount, size_t targetBufSize, uint8_t* targetBuf,
                                int profileId, WalterModemRsp* rsp)
{
  ESP_LOGW("DEPRECATION",
           "this socketReceive method is deprecated and will be removed in future releases. Use "
           "socketReceiveMessage(int profile_id, uint8_t* buf, size_t buf_size, "
           "WalterModemRsp* rsp, walterModemCb cb, void* args) instead.");

  return socketReceiveMessage(profileId, targetBuf, targetBufSize, rsp, NULL, NULL);
}

bool WalterModem::socketReceiveMessage(int profile_id, uint8_t* buf, size_t buf_size,
                                       WalterModemRsp* rsp, walterModemCb cb, void* args)
{
  WalterModemSocket* sock = _socketGet(profile_id);
  if(sock == NULL) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
  }

  size_t toRead = (buf_size > 1500) ? 1500 : buf_size;

  _runCmd(arr("AT+SQNSRECV=", _digitStr(sock->id), ",", _atNum(toRead)), "OK", rsp, cb, args, NULL,
          NULL, WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT, buf, toRead);
  _returnAfterReply();
}

WalterModemSocketState WalterModem::socketGetState(int profileId)
{
  if(!_socketUpdateStates()) {
    ESP_LOGW("WalterModem", "Could not update socket states");
  }

  if(profileId == 0) {
    return WalterModemSocketState::WALTER_MODEM_SOCKET_STATE_FREE;
  }

  WalterModemSocket* sock = _socketGet(profileId);
  if(sock == NULL) {
    return WalterModemSocketState::WALTER_MODEM_SOCKET_STATE_FREE;
  }

  return sock->state;
}

bool WalterModem::socketResume(int profileId, WalterModemRsp* rsp, walterModemCb cb, void* args)
{
  WalterModemSocket* sock = _socketGet(profileId);
  if(sock == NULL) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
  }

  _runCmd(arr("AT+SQNSO", _digitStr(sock->id)), "OK", rsp, cb, args);
  _returnAfterReply();
}

#pragma endregion
#pragma region DEPRECATION
bool WalterModem::socketConfig(WalterModemRsp* rsp, walterModemCb cb, void* args, int pdpCtxId,
                               uint16_t mtu, uint16_t exchangeTimeout, uint16_t connTimeout,
                               uint16_t sendDelayMs, int profileId)
{
  ESP_LOGW("DEPRECATION",
           "Use socketConfig(profile_id, pdp_ctx_id, mtu, exchange_timeout, conn_timeout, "
           "send_delay_ms, rsp, cb, args) instead");
  return socketConfig(profileId, pdpCtxId, mtu, exchangeTimeout, connTimeout, sendDelayMs, rsp, cb,
                      args);
}

bool WalterModem::socketConfigExtended(WalterModemRsp* rsp, walterModemCb cb, void* args,
                                       int profileId, WalterModemSocketRingMode ringMode,
                                       WalterModemSocketRecvMode recvMode, int keepAlive,
                                       WalterModemSocketListenMode listenMode,
                                       WalterModemsocketSendMode sendMode)
{
  ESP_LOGW("DEPRECATION", "Use socketConfigExtended(profile_id, ring_mode, recv_mode, keep_alive, "
                          "listen_mode, send_mode, rsp, cb, args) instead");
  return socketConfigExtended(profileId, ringMode, recvMode, keepAlive, listenMode, sendMode, rsp,
                              cb, args);
}

bool WalterModem::socketConfigSecure(bool enableTLS, int tlsProfileId, int profileId,
                                     WalterModemRsp* rsp, walterModemCb cb, void* args)
{
  ESP_LOGW("DEPRECATION",
           "Use socketConfigSecure(profile_id, enable_tls, tls_profile_id, rsp, cb, args) instead");
  return socketConfigSecure(profileId, enableTLS, tlsProfileId, rsp, cb, args);
}

#pragma endregion
#endif