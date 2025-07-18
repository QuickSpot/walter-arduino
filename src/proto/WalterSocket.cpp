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
WalterModemSocket *WalterModem::_socketReserve()
{
    WalterModemSocket *sock = NULL;

    for (int i = 0; i < WALTER_MODEM_MAX_SOCKETS; ++i) {
        if (_socketSet[i].state == WALTER_MODEM_SOCKET_STATE_FREE) {
            sock = _socketSet + i;
            sock->state = WALTER_MODEM_SOCKET_STATE_RESERVED;
            sock->id = i + 1;
            break;
        }
    }

    if (sock != NULL) {
        _socket = sock;
    }

    return sock;
}

WalterModemSocket *WalterModem::_socketGet(int id)
{
    if (id < 0) {
        return _socket;
    }

    for (int i = 0; i < WALTER_MODEM_MAX_SOCKETS; ++i) {
        if (_socketSet[i].state != WALTER_MODEM_SOCKET_STATE_FREE && _socketSet[i].id == id) {
            _socket = _socketSet + i;
            return _socketSet + i;
        }
    }

    return NULL;
}
void WalterModem::_socketRelease(WalterModemSocket *sock)
{
    if (sock == NULL) {
        return;
    }

    sock->state = WALTER_MODEM_SOCKET_STATE_FREE;
}

bool WalterModem::_socketUpdateStates()
{
    WalterModemRsp *rsp = NULL;
    walterModemCb cb = NULL;
    void *args = NULL;

    _runCmd({"AT+SQNSS?"}, "OK", rsp, cb, args);
    _returnAfterReply();
}
void WalterModem::_ringQueueProcessingTask(void *args)
{
    WalterModemSocketRing ring{};
    TickType_t blockTime = pdMS_TO_TICKS(1000);
    uint8_t data[1500];
    while(true) 
    {
        if (xQueueReceive(_ringQueue.handle, &ring, blockTime) == pdTRUE) {
            socketReceive(ring.ringSize, sizeof(data), data, ring.profileId);
            _dispatchEvent(WALTER_MODEM_SOCKET_EVENT_RING, ring.profileId,ring.ringSize, data);
#ifdef CONFIG_WALTER_MODEM_ENABLE_BLUECHERRY
            if(ring.profileId == _blueCherry.bcSocketId) {
                _blueCherrySocketEventHandler(WALTER_MODEM_SOCKET_EVENT_RING, ring.ringSize, data);
            }
#endif
        }
    }
}

void WalterModem::_dispatchEvent(
    WalterModemSocketEvent event, int socketId, uint16_t dataReceived, uint8_t *dataBuffer)
{
    WalterModemEventHandler *handler = _eventHandlers + WALTER_MODEM_EVENT_TYPE_SOCKET;
    if (handler->socketHandler == nullptr) {
        return;
    }

    auto start = std::chrono::steady_clock::now();
    handler->socketHandler(event, socketId, dataReceived, dataBuffer, handler->args);
    _checkEventDuration(start);
}
    #pragma endregion

    #pragma region PUBLIC_METHODS
bool WalterModem::socketConfig(
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args,
    int pdpCtxId,
    uint16_t mtu,
    uint16_t exchangeTimeout,
    uint16_t connTimeout,
    uint16_t sendDelayMs,
    int socketId)
{
    WalterModemSocket *sock = NULL;

    if (socketId == -1) {
        sock = _socketReserve();
    } else {
        sock = _socketGet(socketId);
    }

    if (sock == NULL) {
        _returnState(WALTER_MODEM_STATE_NO_FREE_SOCKET);
    }


    sock->pdpContextId = pdpCtxId;
    sock->mtu = mtu;
    sock->exchangeTimeout = exchangeTimeout;
    sock->connTimeout = connTimeout;
    sock->sendDelayMs = sendDelayMs;

    auto completeHandler = [](WalterModemCmd *cmd, WalterModemState result) {
        WalterModemSocket *sock = (WalterModemSocket *)cmd->completeHandlerArg;

        cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_SOCKET_ID;
        cmd->rsp->data.socketId = sock->id;

        if (result == WALTER_MODEM_STATE_OK) {
            sock->state = WALTER_MODEM_SOCKET_STATE_CONFIGURED;
        }
    };

    _runCmd(
        arr("AT+SQNSCFG=",
            _digitStr(sock->id),
            ",",
            _digitStr(sock->pdpContextId),
            ",",
            _atNum(sock->mtu),
            ",",
            _atNum(sock->exchangeTimeout),
            ",",
            _atNum(sock->connTimeout * 10),
            ",",
            _atNum(sock->sendDelayMs / 100)),
        "OK",
        rsp,
        cb,
        args,
        completeHandler,
        sock);
    _returnAfterReply();
}

bool WalterModem::socketConfigExtended(
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args,
    int socketId,
    WalterModemSocketRingMode ringMode,
    WalterModemSocketRecvMode recvMode,
    int keepAlive,
    WalterModemSocketListenMode listenMode,
    WalterModemsocketSendMode sendMode)
{
    WalterModemSocket *sock = _socketGet(socketId);
    if (sock == NULL) {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
    }

    _runCmd(
        arr("AT+SQNSCFGEXT=",
            _digitStr(sock->id),
            ",",
            _digitStr(ringMode),
            ",",
            _digitStr(recvMode),
            ",",
            _digitStr(keepAlive),
            ",",
            _digitStr(listenMode),
            ",",
            _digitStr(sendMode)),
        "OK",
        rsp,
        cb,
        args,
        NULL,
        sock);
    _returnAfterReply();
}

bool WalterModem::socketConfigSecure(
    bool enableTLS,
    int profileId,
    int socketId,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    WalterModemSocket *sock = _socketGet(socketId);
    if (sock == NULL) {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
    }

    _runCmd(
        arr("AT+SQNSSCFG=", _digitStr(sock->id), ",", enableTLS ? "1" : "0",
            ",", _digitStr(profileId)),
        "OK",
        rsp,
        cb,
        args,
        NULL);
    _returnAfterReply();
}

bool WalterModem::socketDial(
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
    if (sock == NULL) {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
    }

    sock->protocol = protocol;
    sock->acceptAnyRemote = acceptAnyRemote;
    _strncpy_s(sock->remoteHost, remoteHost, WALTER_MODEM_HOSTNAME_MAX_SIZE);
    sock->remotePort = remotePort;
    sock->localPort = localPort;

    auto completeHandler = [](WalterModemCmd *cmd, WalterModemState result) {
        WalterModemSocket *sock = (WalterModemSocket *)cmd->completeHandlerArg;

        if (result == WALTER_MODEM_STATE_OK) {
            sock->state = WALTER_MODEM_SOCKET_STATE_OPENED;
        }
    };

    _runCmd(
        arr("AT+SQNSD=",
            _digitStr(sock->id),
            ",",
            _digitStr(sock->protocol),
            ",",
            _atNum(sock->remotePort),
            ",",
            _atStr(sock->remoteHost),
            ",0,",
            _atNum(sock->localPort),
            ",1,",
            _digitStr(sock->acceptAnyRemote),
            ",0"),
        "OK",
        rsp,
        cb,
        args,
        completeHandler,
        sock);
    _returnAfterReply();
}

bool WalterModem::socketClose(WalterModemRsp *rsp, walterModemCb cb, void *args, int socketId)
{
    WalterModemSocket *sock = _socketGet(socketId);
    if (sock == NULL) {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
    }

    auto completeHandler = [](WalterModemCmd *cmd, WalterModemState result) {
        WalterModemSocket *sock = (WalterModemSocket *)cmd->completeHandlerArg;

        if (result == WALTER_MODEM_STATE_OK) {
            _socketRelease(sock);
        }
    };

    _runCmd(arr("AT+SQNSH=", _digitStr(sock->id)), "OK", rsp, cb, args, completeHandler, sock);
    _returnAfterReply();
}

bool WalterModem::socketSend(
    uint8_t *data,
    uint32_t dataSize,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args,
    WalterModemRAI rai,
    int socketId)
{
    /* AT+SQNSSEND is a legacy AT command! */
    WalterModemSocket *sock = _socketGet(socketId);
    if (sock == NULL) {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
    }
    
    _runCmd(
        arr("AT+SQNSSENDEXT=", _digitStr(sock->id), ",", _atNum(dataSize), ",", _digitStr(rai)),
        "OK",
        rsp,
        cb,
        args,
        NULL,
        NULL,
        WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT,
        data,
        dataSize);
    _returnAfterReply();
}

bool WalterModem::socketSend(
    char *str, WalterModemRsp *rsp, walterModemCb cb, void *args, WalterModemRAI rai, int socketId)
{
    return socketSend((uint8_t *)str, strlen(str), rsp, cb, args, rai, socketId);
}

bool WalterModem::socketAccept(
    WalterModemRsp *rsp, walterModemCb cb, void *args, int socketId, bool commandMode)
{
    WalterModemSocket *sock = _socketGet(socketId);
    if (sock == NULL) {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
    }

    _runCmd(
        arr("AT+SQNSA=", _digitStr(sock->id), ",", _digitStr(commandMode)),
        "OK",
        rsp,
        cb,
        args,
        NULL,
        NULL,
        WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT);
    _returnAfterReply();
}

bool WalterModem::socketListen(
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args,
    int socketId,
    WalterModemSocketProto protocol,
    WalterModemSocketListenState listenState,
    int socketListenPort)
{
    WalterModemSocket *sock = _socketGet(socketId);
    if (sock == NULL) {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
    }
    sock->protocol = protocol;
    if (sock->protocol == WALTER_MODEM_SOCKET_PROTO_TCP) {
        _runCmd(
            arr("AT+SQNSL=",
                _digitStr(sock->id),
                ",",
                _digitStr(listenState),
                ",",
                _atNum(socketListenPort)),
            "OK",
            rsp,
            cb,
            args,
            NULL,
            NULL,
            WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT);
        _returnAfterReply();
    } else {
        _runCmd(
            arr("AT+SQNSLUDP=",
                _digitStr(sock->id),
                ",",
                _digitStr(listenState),
                ",",
                _atNum(socketListenPort)),
            "OK",
            rsp,
            cb,
            args,
            NULL,
            NULL,
            WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT);
        _returnAfterReply();
    }
}

uint16_t WalterModem::socketAvailable(int socketId)
{
    WalterModemSocket *sock = _socketGet(socketId);
    if (sock == NULL) {
        return 0;
    }
    return  sock->dataAvailable;
}

bool WalterModem::socketReceive(
    uint16_t receiveCount,
    size_t targetBufSize,
    uint8_t *targetBuf,
    int socketId,
    WalterModemRsp *rsp)
{
    /* this is by definition a blocking call without callback.
     * it is only used when the arduino user is not taking advantage of
     * the (TBI) ring notification events.
     */
    walterModemCb cb = NULL;
    void *args = NULL;
    uint16_t dataToRead;
    WalterModemSocket *sock = _socketGet(socketId);
    if (sock == NULL) {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
    }

    if (targetBufSize < receiveCount || receiveCount > 1500) {
        _returnState(WALTER_MODEM_STATE_NO_MEMORY);
    }

    dataToRead = (receiveCount > sock->dataAvailable) ? sock->dataAvailable : receiveCount;

    if (dataToRead == 0) {
        return true;
    }

    sock->dataAvailable -= dataToRead;

    // The number of received bytes attempts to be read from within the RX parser.
    // This will be used as a fallback if it cannot read it.
    _expectedPayloadSize = dataToRead;
    _runCmd(
        arr("AT+SQNSRECV=", _digitStr(sock->id), ",", _atNum(receiveCount)),
        "+SQNSRECV:",
        rsp,
        cb,
        args,
        NULL,
        NULL,
        WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT,
        targetBuf,
        targetBufSize);
    _returnAfterReply();
}

WalterModemSocketState WalterModem::socketGetState(int socketId)
{
    if(!_socketUpdateStates()){
        ESP_LOGW("WalterModem", "Could not update socket states");
    }

    if (socketId == 0) {
        return WalterModemSocketState::WALTER_MODEM_SOCKET_STATE_FREE;
    }

    WalterModemSocket *sock = _socketGet(socketId);
    if (sock == NULL) {
        return WalterModemSocketState::WALTER_MODEM_SOCKET_STATE_FREE;
    }

    return sock->state;
}

bool WalterModem::socketResume(
    int socketId, WalterModemRsp *rsp, walterModemCb cb, void *args)
{
    WalterModemSocket *sock = _socketGet(socketId);
    if (sock == NULL) {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
    }

    _runCmd(arr("AT+SQNSO", _digitStr(sock->id)),"OK",rsp,cb,args);
    _returnAfterReply();
}

void WalterModem::socketSetEventHandler(walterModemSocketEventHandler handler, void *args = NULL)
{
    _eventHandlers[WALTER_MODEM_EVENT_TYPE_SOCKET].socketHandler = handler;
    _eventHandlers[WALTER_MODEM_EVENT_TYPE_SOCKET].args = args;
}

    #pragma endregion
#endif