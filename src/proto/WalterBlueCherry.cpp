/**
 * @file WalterModem.h
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
#if CONFIG_WALTER_MODEM_ENABLE_BLUE_CHERRY
    #if !CONFIG_WALTER_MODEM_ENABLE_OTA || !CONFIG_WALTER_MODEM_ENABLE_MOTA
        #error Bluecherry cannot be enabled with OTA or MOTA disabled.
    #endif
#endif

#if CONFIG_WALTER_MODEM_ENABLE_OTA && !CONFIG_WALTER_MODEM_ENABLE_BLUE_CHERRY
    #error OTA cann only be done when bluecherry is enabled.
#endif
#if CONFIG_WALTER_MODEM_ENABLE_BLUE_CHERRY
#pragma region PRIVATE_METHODS
bool WalterModem::_processBlueCherryEvent(uint8_t *data, uint8_t len)
{
    switch (data[0])
    {
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
        ESP_LOGD("WalterModem", "Error: invalid BlueCherry event type 0x%x from cloud server",
                 data[0]);
        return true;
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

    if(walterCertificate) {
        if(!tlsWriteCredential(false, 5, walterCertificate)) {
            result = WALTER_MODEM_STATE_ERROR;
        }
    }

    if(walterPrivateKey) {
        if(!tlsWriteCredential(true, 0, walterPrivateKey)) {
            result = WALTER_MODEM_STATE_ERROR;
        }
    }

    if(caCertificate) {
        if(!tlsWriteCredential(false, 6, caCertificate)) {
            result = WALTER_MODEM_STATE_ERROR;
        }
    }

    _returnState(result);
}

bool WalterModem::blueCherryIsProvisioned()
{
    if(!_tlsIsCredentialPresent(false, 5)) {
        return false;
    }

    if(!_tlsIsCredentialPresent(false, 6)) {
        return false;
    }

    if(!_tlsIsCredentialPresent(true, 0)) {
        return false;
    }

    return true;
}

bool WalterModem::blueCherryInit(
    uint8_t tlsProfileId,
    uint8_t *otaBuffer,
    WalterModemRsp *rsp,
    uint16_t ackTimeout)
{
    if(!blueCherryIsProvisioned() ||
       !tlsConfigProfile(tlsProfileId, WALTER_MODEM_TLS_VALIDATION_URL_AND_CA,
        WALTER_MODEM_TLS_VERSION_12, 6, 5, 0)) {
        blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_NOT_PROVISIONED;
        
        if(rsp) {
            rsp->type = WALTER_MODEM_RSP_DATA_TYPE_BLUECHERRY;
            rsp->data.blueCherry.state = WALTER_MODEM_BLUECHERRY_STATUS_NOT_PROVISIONED;
            rsp->data.blueCherry.messageCount = 0;
        }

        return false;
    }

    blueCherry.tlsProfileId = tlsProfileId;

    blueCherry.messageOutLen = 0;
    blueCherry.curMessageId = 0x1;
    blueCherry.lastAckedMessageId = 0x0;
    blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_IDLE;
    blueCherry.moreDataAvailable = false;

    blueCherry.emitErrorEvent = false;
    blueCherry.otaSize = 0;
    blueCherry.otaBuffer = otaBuffer;
    blueCherry.ackTimeout = ackTimeout;

    return true;
}

bool WalterModem::blueCherryPublish(uint8_t topic, uint8_t len, uint8_t *data)
{
    if(blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_IDLE &&
       blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_TIMED_OUT) {
        return false;
    }

    if(blueCherry.messageOutLen + len >= WALTER_MODEM_MAX_OUTGOING_MESSAGE_LEN) {
        return false;
    }

    blueCherry.messageOut[blueCherry.messageOutLen] = topic;
    blueCherry.messageOut[blueCherry.messageOutLen + 1] = len;
    memcpy(blueCherry.messageOut + blueCherry.messageOutLen + 2, data, len);

    blueCherry.messageOutLen += len + 2;
    return true;
}

bool WalterModem::blueCherrySync(WalterModemRsp *rsp)
{
    walterModemCb cb = NULL;
    void *args = NULL;
    uint16_t curOffset = 0;

    if(blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_IDLE &&
       blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_PENDING_MESSAGES &&
       blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_TIMED_OUT) {
        return false;
    }

    if(rsp) {
        rsp->type = WALTER_MODEM_RSP_DATA_TYPE_BLUECHERRY;
        rsp->data.blueCherry.syncFinished = false;
        rsp->data.blueCherry.messageCount = 0;
    }

    if (!coapCreateContext(0, WALTER_MODEM_BLUE_CHERRY_HOSTNAME, blueCherry.port, blueCherry.tlsProfileId)) {
        return false;
    }

    if(!coapSetHeader(0, blueCherry.curMessageId)) {
        return false;
    }

    /* determine nr of messages to recup if we missed any */
    int32_t lastAckedMessageId = blueCherry.lastAckedMessageId;
    if(lastAckedMessageId > blueCherry.curMessageId) {
        lastAckedMessageId -= 0xffff;
    }

    uint8_t nrMissed = blueCherry.curMessageId - lastAckedMessageId - 1;

    if(!coapSendData(0, WALTER_MODEM_COAP_SEND_TYPE_CON, (WalterModemCoapSendMethodRsp) nrMissed,
       blueCherry.messageOutLen, blueCherry.messageOut)) {
        return false;
    }

    blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_AWAITING_RESPONSE;
    blueCherry.lastTransmissionTime = time(NULL);

    while(blueCherry.status == WALTER_MODEM_BLUECHERRY_STATUS_AWAITING_RESPONSE) {
        if(time(NULL) - blueCherry.lastTransmissionTime > blueCherry.ackTimeout) {
            blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_TIMED_OUT;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if(blueCherry.status == WALTER_MODEM_BLUECHERRY_STATUS_RESPONSE_READY) {
        rsp->data.blueCherry.syncFinished = !blueCherry.moreDataAvailable;

        /* BlueCherry cloud ack means our last error line can be cleared */
        blueCherry.emitErrorEvent = false;

        while(curOffset < blueCherry.messageInLen) {
            uint8_t topic = blueCherry.messageIn[curOffset];
            curOffset++;
            uint8_t dataLen = blueCherry.messageIn[curOffset];
            curOffset++;

            /* 
             * Topic 0 is reserved for BlueCherry events, which are also visible to walter as mqtt
             * messages on topic id 0
             */
            if(topic == 0) {
                rsp->data.blueCherry.syncFinished = false;

                if(_processBlueCherryEvent(blueCherry.messageIn + curOffset, dataLen)) {
                    blueCherry.emitErrorEvent = true;
                    blueCherry.otaSize = 0;
                }
            }

            if(rsp) {
                WalterModemBlueCherryMessage *msg =
                    rsp->data.blueCherry.messages + rsp->data.blueCherry.messageCount;
                msg->topic = topic;
                msg->dataSize = dataLen;
                msg->data = blueCherry.messageIn + curOffset;

                rsp->data.blueCherry.messageCount++;
            }

            curOffset += dataLen;
        }
    }

    blueCherry.curMessageId++;
    if(blueCherry.curMessageId == 0) {
        /* on wrap around, skip msg id 0 which we use as a special/error value */
        blueCherry.curMessageId++;
    }
    blueCherry.messageOutLen = 0;

    if(blueCherry.emitErrorEvent) {
        uint8_t blueCherryErrorEventCode = WALTER_MODEM_BLUECHERRY_EVENT_TYPE_OTA_ERROR;
        blueCherryPublish(0, 1, &blueCherryErrorEventCode);
    }

    if(blueCherry.status == WALTER_MODEM_BLUECHERRY_STATUS_TIMED_OUT) {
        rsp->data.blueCherry.state = WALTER_MODEM_BLUECHERRY_STATUS_TIMED_OUT;
        _returnState(WALTER_MODEM_STATE_ERROR)
    }

    if(blueCherry.moreDataAvailable) {
        blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_PENDING_MESSAGES;
        rsp->data.blueCherry.state = WALTER_MODEM_BLUECHERRY_STATUS_PENDING_MESSAGES;
    } else {
        blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_IDLE;
        rsp->data.blueCherry.state = WALTER_MODEM_BLUECHERRY_STATUS_IDLE;
    }

    _returnState(WALTER_MODEM_STATE_OK);
}

bool WalterModem::blueCherryClose(WalterModemRsp *rsp, walterModemCb cb, void *args)
{
    _runCmd({"AT+SQNCOAPCLOSE=0"}, "+SQNCOAPCLOSED: ", rsp, cb, args);
    _returnAfterReply();
}

#pragma endregion

#endif