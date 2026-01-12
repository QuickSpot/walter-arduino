/**
 * @file WalterSocket.cpp
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
 * This file contains the Socket implementation of the Walter Modem library.
 */

#include <WalterDefines.h>
#include <WalterModem.h>
#include <esp_log.h>

#if CONFIG_WALTER_MODEM_ENABLE_SOCKETS

static uint8_t _curr_sock_id;

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
  walter_modem_rsp_t* rsp = NULL;
  walter_modem_cb_t cb = NULL;
  void* args = NULL;

  _runCmd({ "AT+SQNSS?" }, "OK", rsp, cb, args);
  _returnAfterReply();
}
#pragma endregion

#pragma region PUBLIC_METHODS
bool WalterModem::socketConfig(int profile_id, int pdp_ctx_id, uint16_t mtu,
                               uint16_t exchange_timeout, uint16_t conn_timeout,
                               uint16_t send_delay_ms, walter_modem_rsp_t* rsp,
                               walter_modem_cb_t cb, void* args)
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

  auto completeHandler = [](walter_modem_cmd_t* cmd, WalterModemState result) {
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
                                       WalterModemsocketSendMode send_mode, walter_modem_rsp_t* rsp,
                                       walter_modem_cb_t cb, void* args)
{
  WalterModemSocket* sock = _socketGet(profile_id);
  if(sock == NULL) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
  }

  _runCmd(arr("AT+SQNSCFGEXT=", _digitStr(profile_id), ",", _digitStr(ring_mode), ",",
              _digitStr(recv_mode), ",", _digitStr(keep_alive), ",", _digitStr(listen_mode), ",",
              _digitStr(send_mode)),
          "OK", rsp, cb, args, NULL, sock);
  _returnAfterReply();
}

bool WalterModem::socketConfigSecure(int profile_id, bool enable_tls, int tls_profile_id,
                                     walter_modem_rsp_t* rsp, walter_modem_cb_t cb, void* args)
{
  WalterModemSocket* sock = _socketGet(profile_id);
  if(sock == NULL) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
  }

  _runCmd(arr("AT+SQNSSCFG=", _digitStr(profile_id), ",", enable_tls ? "1" : "0", ",",
              _digitStr(tls_profile_id)),
          "OK", rsp, cb, args, NULL);
  _returnAfterReply();
}

bool WalterModem::socketDial(int profile_id, WalterModemSocketProto protocol, uint16_t remote_port,
                             const char* remote_host, uint16_t local_port,
                             WalterModemSocketAcceptAnyRemote accept_any_remote,
                             walter_modem_rsp_t* rsp, walter_modem_cb_t cb, void* args)
{
  WalterModemSocket* sock = _socketGet(profile_id);
  if(sock == NULL) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
  }

  sock->protocol = protocol;
  sock->acceptAnyRemote = accept_any_remote;
  _strncpy_s(sock->remoteHost, remote_host, WALTER_MODEM_HOSTNAME_MAX_SIZE);
  sock->remotePort = remote_port;
  sock->localPort = local_port;

  auto completeHandler = [](walter_modem_cmd_t* cmd, WalterModemState result) {
    WalterModemSocket* sock = (WalterModemSocket*) cmd->completeHandlerArg;
    if(result == WALTER_MODEM_STATE_OK) {
      sock->state = WALTER_MODEM_SOCKET_STATE_OPENED;
    }
  };

  _runCmd(arr("AT+SQNSD=", _digitStr(profile_id), ",", _digitStr(protocol), ",",
              _atNum(remote_port), ",", _atStr(remote_host), ",0,", _atNum(local_port), ",1,",
              _digitStr(accept_any_remote), ",0"),
          "OK", rsp, cb, args, completeHandler, sock);
  _returnAfterReply();
}

bool WalterModem::socketClose(int profile_id, walter_modem_rsp_t* rsp, walter_modem_cb_t cb,
                              void* args)
{

  WalterModemSocket* sock = _socketGet(profile_id);
  if(sock == NULL) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
  }

  auto completeHandler = [](walter_modem_cmd_t* cmd, WalterModemState result) {
    WalterModemSocket* sock = (WalterModemSocket*) cmd->completeHandlerArg;
    if(result == WALTER_MODEM_STATE_OK) {
      sock->state = WALTER_MODEM_SOCKET_STATE_FREE;
    }
  };

  _runCmd(arr("AT+SQNSH=", _digitStr(profile_id)), "OK", rsp, cb, args, completeHandler, sock);
  _returnAfterReply();
}

bool WalterModem::socketSend(int profile_id, uint8_t* buf, uint16_t buf_size, WalterModemRAI rai,
                             walter_modem_rsp_t* rsp, walter_modem_cb_t cb, void* args)
{

  WalterModemSocket* sock = _socketGet(profile_id);
  if(sock == NULL) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
  }

  _runCmd(arr("AT+SQNSSENDEXT=", _digitStr(profile_id), ",", _atNum(buf_size), ",", _digitStr(rai)),
          "OK", rsp, cb, args, NULL, NULL, WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT, buf, buf_size);
  _returnAfterReply();
}

bool WalterModem::socketListen(int profile_id, WalterModemSocketProto protocol,
                               WalterModemSocketListenState listen_state, int listen_port,
                               walter_modem_rsp_t* rsp, walter_modem_cb_t cb, void* args)
{
  if(protocol == WALTER_MODEM_SOCKET_PROTO_TCP) {
    _runCmd(arr("AT+SQNSL=", _digitStr(profile_id), ",", _digitStr(listen_state), ",",
                _atNum(listen_port)),
            "OK", rsp, cb, args, NULL, NULL, WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT);
    _returnAfterReply();
  } else {
    _runCmd(arr("AT+SQNSLUDP=", _digitStr(profile_id), ",", _digitStr(listen_state), ",",
                _atNum(listen_port)),
            "OK", rsp, cb, args, NULL, NULL, WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT);
    _returnAfterReply();
  }
}

bool WalterModem::socketAccept(int profile_id, bool connection_mode, walter_modem_rsp_t* rsp,
                               walter_modem_cb_t cb, void* args)
{
  WalterModemSocket* sock = _socketGet(profile_id);
  if(sock == NULL) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
  }

  _runCmd(arr("AT+SQNSA=", _digitStr(profile_id), ",", _digitStr(connection_mode)), "OK", rsp, cb,
          args, NULL, NULL, WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT);
  _returnAfterReply();
}

bool WalterModem::socketReceive(int profile_id, uint8_t* buf, size_t buf_size,
                                walter_modem_rsp_t* rsp, walter_modem_cb_t cb, void* args)
{
  WalterModemSocket* sock = _socketGet(profile_id);
  if(sock == NULL) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
  }

  size_t readable_size = (buf_size > 1500) ? 1500 : buf_size;

  _runCmd(arr("AT+SQNSRECV=", _digitStr(profile_id), ",", _atNum(readable_size)), "OK", rsp, cb,
          args, NULL, NULL, WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT, buf, readable_size);
  _returnAfterReply();
}

WalterModemSocketState WalterModem::socketGetState(int profile_id)
{
  if(!_socketUpdateStates()) {
    ESP_LOGW("WalterModem", "Could not update socket states");
  }

  if(profile_id == 0) {
    return WalterModemSocketState::WALTER_MODEM_SOCKET_STATE_FREE;
  }

  WalterModemSocket* sock = _socketGet(profile_id);
  if(sock == NULL) {
    return WalterModemSocketState::WALTER_MODEM_SOCKET_STATE_FREE;
  }

  return sock->state;
}

bool WalterModem::socketResume(int profile_id, walter_modem_rsp_t* rsp, walter_modem_cb_t cb,
                               void* args)
{
  WalterModemSocket* sock = _socketGet(profile_id);
  if(sock == NULL) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
  }

  _runCmd(arr("AT+SQNSO", _digitStr(profile_id)), "OK", rsp, cb, args);
  _returnAfterReply();
}

void WalterModem::socketSetEventHandler(walterModemSocketEventHandler handler, void* args)
{
  _eventHandlers[WALTER_MODEM_EVENT_TYPE_SOCKET].socketHandler = handler;
  _eventHandlers[WALTER_MODEM_EVENT_TYPE_SOCKET].args = args;
}

#pragma endregion
#pragma region DEPRECATION
bool WalterModem::socketConfig(walter_modem_rsp_t* rsp, walter_modem_cb_t cb, void* args,
                               int pdp_ctx_id, uint16_t mtu, uint16_t exchangeTimeout,
                               uint16_t connTimeout, uint16_t sendDelayMs, int profileId)
{
  if(profileId == -1) {
    _curr_sock_id = _socketReserve()->id;
    profileId = _curr_sock_id;
  } else {
    _curr_sock_id = profileId;
  }

  if(rsp) {
    rsp->data.profileId = _curr_sock_id;
  }

  ESP_LOGW("DEPRECATION",
           "Use socketConfig(profile_id, pdp_ctx_id, mtu, exchange_timeout, conn_timeout, "
           "send_delay_ms, rsp, cb, args) instead");
  return socketConfig(profileId, pdp_ctx_id, mtu, exchangeTimeout, connTimeout, sendDelayMs, rsp,
                      cb, args);
}

bool WalterModem::socketConfigExtended(walter_modem_rsp_t* rsp, walter_modem_cb_t cb, void* args,
                                       int profileId, WalterModemSocketRingMode ringMode,
                                       WalterModemSocketRecvMode recvMode, int keepAlive,
                                       WalterModemSocketListenMode listenMode,
                                       WalterModemsocketSendMode sendMode)
{
  if(profileId == -1) {
    profileId = _curr_sock_id;
  } else {
    _curr_sock_id = profileId;
  }

  if(rsp) {
    rsp->data.profileId = _curr_sock_id;
  }

  ESP_LOGW("DEPRECATION", "Use socketConfigExtended(profile_id, ring_mode, recv_mode, keep_alive, "
                          "listen_mode, send_mode, rsp, cb, args) instead");
  return socketConfigExtended(profileId, ringMode, recvMode, keepAlive, listenMode, sendMode, rsp,
                              cb, args);
}

bool WalterModem::socketConfigSecure(bool enableTLS, int tls_profile_id, int profileId,
                                     walter_modem_rsp_t* rsp, walter_modem_cb_t cb, void* args)
{
  if(profileId == -1) {
    profileId = _curr_sock_id;
  } else {
    _curr_sock_id = profileId;
  }

  if(rsp) {
    rsp->data.profileId = _curr_sock_id;
  }

  ESP_LOGW("DEPRECATION",
           "Use socketConfigSecure(profile_id, enable_tls, tls_profile_id, rsp, cb, args) instead");
  return socketConfigSecure(profileId, enableTLS, tls_profile_id, rsp, cb, args);
}

bool WalterModem::socketDial(const char* remoteHost, uint16_t remotePort, uint16_t localPort,
                             walter_modem_rsp_t* rsp, walter_modem_cb_t cb, void* args,
                             WalterModemSocketProto protocol,
                             WalterModemSocketAcceptAnyRemote acceptAnyRemote, int profileId)
{
  if(profileId == -1) {
    profileId = _curr_sock_id;
  } else {
    _curr_sock_id = profileId;
  }

  if(rsp) {
    rsp->data.profileId = _curr_sock_id;
  }

  ESP_LOGW("DEPRECATION",
           "Use socketDial(profile_id, protocol, remote_port, remote_host, local_port, "
           "accept_any_remote, rsp, cb, args) instead");
  return socketDial(profileId, protocol, remotePort, remoteHost, localPort, acceptAnyRemote, rsp,
                    cb, args);
}

bool WalterModem::socketClose(walter_modem_rsp_t* rsp, walter_modem_cb_t cb, void* args,
                              int profileId)
{
  if(profileId == -1) {
    profileId = _curr_sock_id;
  } else {
    _curr_sock_id = profileId;
  }

  if(rsp) {
    rsp->data.profileId = _curr_sock_id;
  }

  ESP_LOGW("DEPRECATION", "Use socketClose(profile_id, rsp, cb, args) instead");
  return socketClose(profileId, rsp, cb, args);
}

bool WalterModem::socketSend(uint8_t* data, uint32_t dataSize, walter_modem_rsp_t* rsp,
                             walter_modem_cb_t cb, void* args, WalterModemRAI rai, int profileId)
{
  if(profileId == -1) {
    profileId = _curr_sock_id;
  } else {
    _curr_sock_id = profileId;
  }

  if(rsp) {
    rsp->data.profileId = _curr_sock_id;
  }

  ESP_LOGW("DEPRECATION", "Use socketSend(profile_id, buf, buf_size, rai, rsp, cb, args) instead");
  return socketSend(profileId, data, dataSize, rai, rsp, cb, args);
}

bool WalterModem::socketSend(char* str, walter_modem_rsp_t* rsp, walter_modem_cb_t cb, void* args,
                             WalterModemRAI rai, int profileId)
{
  if(profileId == -1) {
    profileId = _curr_sock_id;
  } else {
    _curr_sock_id = profileId;
  }

  if(rsp) {
    rsp->data.profileId = _curr_sock_id;
  }

  ESP_LOGW("DEPRECATION", "Use socketSend(profile_id, buf, buf_size, rai, rsp, cb, args) instead");
  return socketSend(profileId, (uint8_t*) str, strlen(str), rai, rsp, cb, args);
}

bool WalterModem::socketListen(walter_modem_rsp_t* rsp, walter_modem_cb_t cb, void* args,
                               int profileId, WalterModemSocketProto protocol,
                               WalterModemSocketListenState listenState, int socketListenPort)
{
  if(profileId == -1) {
    profileId = _curr_sock_id;
  } else {
    _curr_sock_id = profileId;
  }

  if(rsp) {
    rsp->data.profileId = _curr_sock_id;
  }

  ESP_LOGW("DEPRECATION", "Use socketListen(profile_id, protocol, listen_state, listen_port, "
                          "rsp, cb, args) instead");
  return socketListen(profileId, protocol, listenState, socketListenPort, rsp, cb, args);
}

bool WalterModem::socketAccept(walter_modem_rsp_t* rsp, walter_modem_cb_t cb, void* args,
                               int profileId, bool commandMode)
{
  if(profileId == -1) {
    profileId = _curr_sock_id;
  } else {
    _curr_sock_id = profileId;
  }

  if(rsp) {
    rsp->data.profileId = _curr_sock_id;
  }

  ESP_LOGW("DEPRECATION", "Use socketAccept(profile_id, connection_mode, rsp, cb, args) instead");
  return socketAccept(profileId, commandMode, rsp, cb, args);
}

uint16_t WalterModem::socketAvailable(int profileId)
{
  ESP_LOGW("DEPRECATION", "This method is deprecated and will be removed in a future release.");
  return 0;
}

bool WalterModem::socketReceive(uint16_t receiveCount, size_t targetBufSize, uint8_t* targetBuf,
                                int profileId, walter_modem_rsp_t* rsp)
{
  if(profileId == -1) {
    profileId = _curr_sock_id;
  } else {
    _curr_sock_id = profileId;
  }

  if(rsp) {
    rsp->data.profileId = _curr_sock_id;
  }

  ESP_LOGW("DEPRECATION", "Use socketReceive(profile_id, buf, buf_size, rsp, cb, args) instead");
  return socketReceive(profileId, targetBuf, targetBufSize, rsp, NULL, NULL);
}

#pragma endregion
#endif