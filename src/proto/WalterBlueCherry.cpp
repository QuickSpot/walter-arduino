/**
 * @file WalterBlueCherry.cpp
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
 * This file contains the implementation of the BlueCherry cloud protocol in the Walter Modem
 * library. BlueCherry cloud uses CoAP + DTLS to communicate with the cloud.
 */

#include <WalterDefines.h>

#if CONFIG_WALTER_MODEM_ENABLE_BLUE_CHERRY
    #if !CONFIG_WALTER_MODEM_ENABLE_BLUE_CHERRY || !CONFIG_WALTER_MODEM_ENABLE_MOTA
        #error Bluecherry cannot be enabled with OTA or MOTA disabled.
    #endif
#endif

#if CONFIG_WALTER_MODEM_ENABLE_BLUE_CHERRY && !CONFIG_WALTER_MODEM_ENABLE_BLUE_CHERRY
    #error OTA cann only be done when bluecherry is enabled.
#endif

#if CONFIG_WALTER_MODEM_ENABLE_BLUE_CHERRY

#pragma region PRIVATE_METHODS

bool WalterModem::_blueCherryProcessEvent(uint8_t *data, uint8_t len)
{
    switch (data[0]) {
        case WALTER_MODEM_BLUECHERRY_EVENT_TYPE_OTA_INITIALIZE:
            return _processOtaInitializeEvent(data + 1, len - 1);

        case WALTER_MODEM_BLUECHERRY_EVENT_TYPE_OTA_CHUNK:
            return _processOtaChunkEvent(data + 1, len - 1);

        case WALTER_MODEM_BLUECHERRY_EVENT_TYPE_OTA_FINISH:
            return _processOtaFinishEvent();

        case WALTER_MODEM_BLUECHERRY_EVENT_TYPE_MOTA_INITIALIZE:
            return _processMotaInitializeEvent(data + 1, len - 1);

        case WALTER_MODEM_BLUECHERRY_EVENT_TYPE_MOTA_CHUNK:
            return _processMotaChunkEvent(data + 1, len - 1);

        case WALTER_MODEM_BLUECHERRY_EVENT_TYPE_MOTA_FINISH:
            return _processMotaFinishEvent();

        default:
            ESP_LOGD(
                "WalterModem", "Error: invalid BlueCherry event type 0x%x from cloud server", data[0]);
            return true;
    }

    return true;
}

bool WalterModem::_blueCherrySocketConfigure()
{
    if(_blueCherry.bcSocketId == NULL) {
        _blueCherry.bcSocketId = reserveSocketId();

        if(_blueCherry.bcSocketId == NULL) {
            return false;
        }
    }

    if(!socketConfig(NULL, NULL, NULL, 1, 300, 0, 60, 5000, _blueCherry.bcSocketId)) {
        return false;
    }

    if(!socketConfigExtended(NULL, NULL, NULL, _blueCherry.bcSocketId, WALTER_MODEM_SOCKET_RING_MODE_DATA_AMOUNT)) {
        return false;
    }

    if(!socketConfigTLS(_blueCherry.bcSocketId, _blueCherry.tlsProfileId, true)) {
        return false;
    }

    return true;
}

void WalterModem::_blueCherrySocketEventHandler(WalterModemSocketEvent event, uint16_t dataReceived, uint8_t *dataBuffer) {
    switch (event) {
        case WALTER_MODEM_SOCKET_EVENT_RING:
            _blueCherry.messageInLen = dataReceived;
            _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_RESPONSE_READY;
            break;
    }
}

void WalterModem::_blueCherrySetCoapHeaders(uint8_t code, uint8_t tokenLen, uint16_t msgId) {
    if (tokenLen > 8) tokenLen = 8;

    _blueCherry.messageOut[0] = 0x40 | tokenLen;             // ver=1, type=0 (CON), token length
    _blueCherry.messageOut[1] = code;                        // BlueCherry replaces request/response code
    _blueCherry.messageOut[2] = msgId >> 8;                  // Message ID high byte
    _blueCherry.messageOut[3] = msgId & 0xFF;                // Message ID low byte
    _blueCherry.messageOut[4] = 0xFF;                        // payload marker
}

bool WalterModem::_blueCherryCoapSend(WalterModemRsp *rsp)
{
    const uint8_t MAX_RETRANSMIT = 4;
    const double ACK_TIMEOUT = 2.0;
    const double ACK_RANDOM_FACTOR = 1.5;

    // Calculate missed messages
    int32_t lastAcked = _blueCherry.lastAckedMessageId;
    if(lastAcked > _blueCherry.curMessageId) {
        lastAcked -= 0xffff;
    }
    uint8_t nrMissed = _blueCherry.curMessageId - lastAcked - 1;

    // Dial the host over the socket if the connection was not yet established or dropped.
    WalterModemSocket *sock = _socketGet(_blueCherry.bcSocketId);
    if (sock->state != WALTER_MODEM_SOCKET_STATE_OPENED) {
        if(!socketDial(WALTER_MODEM_BLUE_CHERRY_HOSTNAME, _blueCherry.port)) {
            return false;
        }
    }

    _blueCherrySetCoapHeaders(nrMissed, 0, _blueCherry.curMessageId);

    double timeout = ACK_TIMEOUT * (1 + (rand() / (RAND_MAX + 1.0)) * (ACK_RANDOM_FACTOR - 1));
    _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_AWAITING_RESPONSE;

    for (uint8_t attempt = 1; attempt <= MAX_RETRANSMIT; ++attempt) {
        _blueCherry.lastTransmissionTime = time(NULL);

        socketSend(_blueCherry.messageOut,
            _blueCherry.messageOutLen,
            NULL,
            NULL,
            NULL,
            WALTER_MODEM_RAI_NO_INFO,
            _blueCherry.bcSocketId);

        while (true) {
            if (_blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_AWAITING_RESPONSE) {
                return true;
            }

            if (difftime(time(NULL), _blueCherry.lastTransmissionTime) >= timeout) {
                break; // retransmit or give up
            }
        }

        timeout *= 2;  // exponential backoff
    }

    _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_TIMED_OUT;
    return false;
}

bool WalterModem::_blueCherryCoapProcessResponse(WalterModemRsp *rsp) {

    if (!socketReceive(_blueCherry.messageInLen, sizeof(_blueCherry.messageIn), _blueCherry.messageIn)) {
        return false;
    }

    uint16_t byteOffset = 0;
    uint8_t recvHeader = _blueCherry.messageIn[byteOffset++];
    uint8_t recvVer = (recvHeader >> 6) & 0x03;

    if (recvVer != 1) {
        return false;
    }

    uint8_t recvType = (recvHeader >> 4) & 0x03;
    uint8_t recvTokenLen = recvHeader & 0x0F;
    byteOffset += recvTokenLen;
    uint8_t recvCode = _blueCherry.messageIn[byteOffset++];
    uint16_t recvMsgId = (_blueCherry.messageIn[byteOffset++] << 8) | _blueCherry.messageIn[byteOffset++];
    byteOffset++; // Skip the 0xff payload marker

    if (recvType == WALTER_MODEM_BLUECHERRY_COAP_SEND_TYPE_ACK) {
            _blueCherry.lastAckedMessageId = recvMsgId;
            /* BlueCherry cloud ack means our last error line can be cleared */
            _blueCherry.emitErrorEvent = false;
    }

    switch (recvCode) {
        case WALTER_MODEM_BLUECHERRY_COAP_RSP_VALID:
            _blueCherry.moreDataAvailable = false;
            break;
        case WALTER_MODEM_BLUECHERRY_COAP_RSP_CONTINUE:
            _blueCherry.moreDataAvailable = true;
            break;
    }

    rsp->data.blueCherry.syncFinished = !_blueCherry.moreDataAvailable;

    while (byteOffset < _blueCherry.messageInLen) {
        uint8_t topic = _blueCherry.messageIn[byteOffset++];
        uint8_t dataLen = _blueCherry.messageIn[byteOffset++];

        /*
            * Topic 0 is reserved for BlueCherry events, which are also visible to walter as mqtt
            * messages on topic id 0
            */
        if (topic == 0) {
            rsp->data.blueCherry.syncFinished = false;

            if (_blueCherryProcessEvent(_blueCherry.messageIn + byteOffset, dataLen)) {
                _blueCherry.emitErrorEvent = true;
                _blueCherry.otaSize = 0;
            }
        }

        if (rsp) {
            WalterModemBlueCherryMessage *msg =
                rsp->data.blueCherry.messages + rsp->data.blueCherry.messageCount;
            msg->topic = topic;
            msg->dataSize = dataLen;
            msg->data = _blueCherry.messageIn + byteOffset;

            rsp->data.blueCherry.messageCount++;
        }

        byteOffset += dataLen;
    }

    return true;
}

#pragma endregion

#pragma region PUBLIC_METHODS

bool WalterModem::blueCherryProvision(
    const char *walterCertificate,
    const char *walterPrivateKey,
    const char *caCertificate,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    WalterModemState result = WALTER_MODEM_STATE_OK;

    if (walterCertificate) {
        if (!tlsWriteCredential(false, 5, walterCertificate)) {
            result = WALTER_MODEM_STATE_ERROR;
        }
    }

    if (walterPrivateKey) {
        if (!tlsWriteCredential(true, 0, walterPrivateKey)) {
            result = WALTER_MODEM_STATE_ERROR;
        }
    }

    if (caCertificate) {
        if (!tlsWriteCredential(false, 6, caCertificate)) {
            result = WALTER_MODEM_STATE_ERROR;
        }
    }

    _returnState(result);
}

bool WalterModem::blueCherryIsProvisioned()
{
    if (!_tlsIsCredentialPresent(false, 5)) {
        return false;
    }

    if (!_tlsIsCredentialPresent(false, 6)) {
        return false;
    }

    if (!_tlsIsCredentialPresent(true, 0)) {
        return false;
    }

    return true;
}

bool WalterModem::blueCherryInit(
    uint8_t tlsProfileId, uint8_t *otaBuffer, WalterModemRsp *rsp, uint16_t ackTimeout)
{
    if (!blueCherryIsProvisioned() ||
        !tlsConfigProfile(
            tlsProfileId,
            WALTER_MODEM_TLS_VALIDATION_URL_AND_CA,
            WALTER_MODEM_TLS_VERSION_12,
            6,
            5,
            0)) {
        _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_NOT_PROVISIONED;

        if(rsp) {
            rsp->type = WALTER_MODEM_RSP_DATA_TYPE_BLUECHERRY;
            rsp->data.blueCherry.state = WALTER_MODEM_BLUECHERRY_STATUS_NOT_PROVISIONED;
            rsp->data.blueCherry.messageCount = 0;
        }

        return false;
    }

    _blueCherry.tlsProfileId = tlsProfileId;

    _blueCherry.messageOutLen = 0x0005; // Reserve space for CoAP headers
    _blueCherry.curMessageId = 0x0001;
    _blueCherry.lastAckedMessageId = 0x0000;
    _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_IDLE;
    _blueCherry.moreDataAvailable = false;

    _blueCherry.emitErrorEvent = false;
    _blueCherry.otaSize = 0;
    _blueCherry.otaBuffer = otaBuffer;
    _blueCherry.ackTimeout = ackTimeout;

    return _blueCherrySocketConfigure();
}

bool WalterModem::blueCherryPublish(uint8_t topic, uint8_t len, uint8_t *data)
{
    if (_blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_IDLE &&
        _blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_TIMED_OUT) {
        return false;
    }

    if (_blueCherry.messageOutLen + len + 2 >= WALTER_MODEM_MAX_OUTGOING_MESSAGE_LEN) {
        return false;
    }

    _blueCherry.messageOut[_blueCherry.messageOutLen] = topic;
    _blueCherry.messageOut[_blueCherry.messageOutLen + 1] = len;
    memcpy(_blueCherry.messageOut + _blueCherry.messageOutLen + 2, data, len);

    _blueCherry.messageOutLen += len + 2;
    return true;
}

bool WalterModem::blueCherrySync(WalterModemRsp *rsp)
{
    walterModemCb cb = NULL;
    void *args = NULL;

    if (_blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_IDLE &&
        _blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_PENDING_MESSAGES &&
        _blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_TIMED_OUT) {
        _returnState(WALTER_MODEM_STATE_BUSY)
    }

    if (rsp) {
        rsp->type = WALTER_MODEM_RSP_DATA_TYPE_BLUECHERRY;
        rsp->data.blueCherry.syncFinished = false;
        rsp->data.blueCherry.messageCount = 0;
    }

    if(!_blueCherryCoapSend(rsp)) {
        _returnState(WALTER_MODEM_STATE_TIMEOUT)
    }

    if(!_blueCherryCoapProcessResponse(rsp)) {
        _returnState(WALTER_MODEM_STATE_ERROR)
    }

    _blueCherry.curMessageId++;
    if (_blueCherry.curMessageId == 0) {
        /* on wrap around, skip msg id 0 which we use as a special/error value */
        _blueCherry.curMessageId++;
    }
    // Reserve initial 5 bytes for CoAP headers
    _blueCherry.messageOutLen = 5;

    if (_blueCherry.emitErrorEvent) {
        uint8_t blueCherryErrorEventCode = WALTER_MODEM_BLUECHERRY_EVENT_TYPE_OTA_ERROR;
        blueCherryPublish(0, 1, &blueCherryErrorEventCode);
    }

    if (_blueCherry.status == WALTER_MODEM_BLUECHERRY_STATUS_TIMED_OUT) {
        rsp->data.blueCherry.state = WALTER_MODEM_BLUECHERRY_STATUS_TIMED_OUT;
        _returnState(WALTER_MODEM_STATE_ERROR)
    }

    if (_blueCherry.moreDataAvailable) {
        _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_PENDING_MESSAGES;
        rsp->data.blueCherry.state = WALTER_MODEM_BLUECHERRY_STATUS_PENDING_MESSAGES;
    } else {
        _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_IDLE;
        rsp->data.blueCherry.state = WALTER_MODEM_BLUECHERRY_STATUS_IDLE;
    }

    _returnState(WALTER_MODEM_STATE_OK);
}

bool WalterModem::blueCherryClose(WalterModemRsp *rsp, walterModemCb cb, void *args)
{
    _runCmd({"AT+SQNCOAPCLOSE=0"}, "+SQNCOAPCLOSED: ", rsp, cb, args);
    _returnAfterReply();
}


size_t WalterModem::blueCherryGetOtaProgressPercentage()
{
    if (_blueCherry.otaSize == 0) {
        return 0; /* NO deviding by ZERO*/
    }
    return (_blueCherry.otaProgress * 100) / _blueCherry.otaSize;
}


size_t WalterModem::blueCherryGetOtaProgressBytes() 
{
    return _blueCherry.otaProgress;
}
size_t WalterModem::blueCherryGetOtaSize()
{
    return _blueCherry.otaSize;
}

#pragma endregion
#endif