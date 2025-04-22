/**
 * @file WalterSocket.cpp
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
#if CONFIG_WALTER_MODEM_ENABLE_SOCKETS
#pragma region PRIVATE_METHODS
WalterModemSocket *WalterModem::_socketReserve()
{
    WalterModemSocket *sock = NULL;

    for (int i = 0; i < WALTER_MODEM_MAX_SOCKETS; ++i)
    {
        if (_socketSet[i].state == WALTER_MODEM_SOCKET_STATE_FREE)
        {
            sock = _socketSet + i;
            sock->state = WALTER_MODEM_SOCKET_STATE_RESERVED;
            sock->id = i + 1;
            break;
        }
    }

    if (sock != NULL)
    {
        _socket = sock;
    }

    return sock;
}

WalterModemSocket *WalterModem::_socketGet(int id)
{
    if (id < 0)
    {
        return _socket;
    }

    for (int i = 0; i < WALTER_MODEM_MAX_SOCKETS; ++i)
    {
        if (_socketSet[i].state != WALTER_MODEM_SOCKET_STATE_FREE && _socketSet[i].id == id)
        {
            _socket = _socketSet + i;
            return _socketSet + i;
        }
    }

    return NULL;
}
void WalterModem::_socketRelease(WalterModemSocket *sock)
{
    if (sock == NULL)
    {
        return;
    }

    sock->state = WALTER_MODEM_SOCKET_STATE_FREE;
}

#pragma endregion

#pragma region PUBLIC_METHODS
bool WalterModem::createSocket(
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args,
    int pdpCtxId,
    uint16_t mtu,
    uint16_t exchangeTimeout,
    uint16_t connTimeout,
    uint16_t sendDelayMs)
{
    WalterModemPDPContext *ctx = _pdpContextGet(pdpCtxId);
    if (ctx == NULL || pdpCtxId < 0)
    {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_PDP_CONTEXT);
    }

    WalterModemSocket *sock = _socketReserve();
    if (sock == NULL)
    {
        _returnState(WALTER_MODEM_STATE_NO_FREE_SOCKET);
    }

    if(ctx->state != WALTER_MODEM_PDP_CONTEXT_STATE_ATTACHED){
        _returnState(WALTER_MODEM_STATE_ERROR);
    }

    sock->pdpContextId = ctx->id;
    sock->mtu = mtu;
    sock->exchangeTimeout = exchangeTimeout;
    sock->connTimeout = connTimeout;
    sock->sendDelayMs = sendDelayMs;

    auto completeHandler = [](WalterModemCmd *cmd, WalterModemState result)
    {
        WalterModemSocket *sock = (WalterModemSocket *)cmd->completeHandlerArg;

        cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_SOCKET_ID;
        cmd->rsp->data.socketId = sock->id;

        if (result == WALTER_MODEM_STATE_OK)
        {
            sock->state = WALTER_MODEM_SOCKET_STATE_CREATED;
        }
    };

    _runCmd(arr(
                "AT+SQNSCFG=",
                _digitStr(sock->id), ",",
                _digitStr(sock->pdpContextId), ",",
                _atNum(sock->mtu), ",",
                _atNum(sock->exchangeTimeout), ",",
                _atNum(sock->connTimeout * 10), ",",
                _atNum(sock->sendDelayMs / 100)),
            "OK", rsp, cb, args, completeHandler, sock);
    _returnAfterReply();
}

bool WalterModem::configSocket(
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args,
    int socketId)
{
    WalterModemSocket *sock = _socketGet(socketId);
    if (sock == NULL)
    {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
    }

    auto completeHandler = [](WalterModemCmd *cmd, WalterModemState result)
    {
        WalterModemSocket *sock = (WalterModemSocket *)cmd->completeHandlerArg;

        if (result == WALTER_MODEM_STATE_OK)
        {
            sock->state = WALTER_MODEM_SOCKET_STATE_CONFIGURED;
        }
    };

    _runCmd(arr(
                "AT+SQNSCFGEXT=",
                _digitStr(sock->id), ",2,0,0,0,0,0"),
            "OK", rsp, cb, args, completeHandler, sock);
    _returnAfterReply();
}

bool WalterModem::connectSocket(
    const char *remoteHost,
    uint16_t remotePort,
    uint16_t localPort,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args,
    WalterModemSocketProto protocol,
    WalterModemSocketAcceptAnyRemote acceptAnyRemote,
    int socketId)
{
    WalterModemSocket *sock = _socketGet(socketId);
    if (sock == NULL)
    {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
    }

    sock->protocol = protocol;
    sock->acceptAnyRemote = acceptAnyRemote;
    _strncpy_s(sock->remoteHost, remoteHost, WALTER_MODEM_HOSTNAME_MAX_SIZE);
    sock->remotePort = remotePort;
    sock->localPort = localPort;

    auto completeHandler = [](WalterModemCmd *cmd, WalterModemState result)
    {
        WalterModemSocket *sock = (WalterModemSocket *)cmd->completeHandlerArg;

        if (result == WALTER_MODEM_STATE_OK)
        {
            sock->state = WALTER_MODEM_SOCKET_STATE_OPENED;
        }
    };

    _runCmd(arr(
                "AT+SQNSD=",
                _digitStr(sock->id), ",",
                _digitStr(sock->protocol), ",",
                _atNum(sock->remotePort), ",",
                _atStr(sock->remoteHost), ",0,",
                _atNum(sock->localPort), ",1,",
                _digitStr(sock->acceptAnyRemote), ",0"),
            "OK", rsp, cb, args, completeHandler, sock);
    _returnAfterReply();
}

bool WalterModem::closeSocket(
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args,
    int socketId)
{
    WalterModemSocket *sock = _socketGet(socketId);
    if (sock == NULL)
    {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
    }

    auto completeHandler = [](WalterModemCmd *cmd, WalterModemState result)
    {
        WalterModemSocket *sock = (WalterModemSocket *)cmd->completeHandlerArg;

        if (result == WALTER_MODEM_STATE_OK)
        {
            _socketRelease(sock);
        }
    };

    _runCmd(arr(
                "AT+SQNSH=",
                _digitStr(sock->id)),
            "OK", rsp, cb, args, completeHandler, sock);
    _returnAfterReply();
}

bool WalterModem::socketSend(
    uint8_t *data,
    uint16_t dataSize,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args,
    WalterModemRAI rai,
    int socketId)
{
    WalterModemSocket *sock = _socketGet(socketId);
    if (sock == NULL)
    {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
    }

    _runCmd(arr(
                "AT+SQNSSENDEXT=",
                _digitStr(sock->id), ",",
                _atNum(dataSize), ",",
                _digitStr(rai)),
            "OK", rsp, cb, args, NULL, NULL, WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT, data, dataSize);
    _returnAfterReply();
}

bool WalterModem::socketSend(
    char *str,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args,
    WalterModemRAI rai,
    int socketId)
{
    return socketSend((uint8_t *)str, strlen(str), rsp, cb, args, rai, socketId);
}

#pragma endregion
#endif