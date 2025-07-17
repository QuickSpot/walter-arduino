/**
 * @file WalterMQTT.cpp
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

#if CONFIG_WALTER_MODEM_ENABLE_MQTT
    #pragma region PRIVATE_METHODS
static void mqttResubscribeCallback(const WalterModemRsp *rsp, void *args)
{
    /*This is an empty callback so the _runCmd() runs async*/
}

bool WalterModem::_mqttSubscribeRaw(
    const char *topicString, uint8_t qos, WalterModemRsp *rsp, walterModemCb cb, void *args)
{
    _runCmd(
        arr("AT+SQNSMQTTSUBSCRIBE=0,", _atStr(topicString), ",", _atNum(qos)),
        NULL,
        rsp,
        mqttResubscribeCallback,
        args);
    _returnState(WALTER_MODEM_STATE_OK);
}

void WalterModem::_dispatchEvent(WalterModemMQTTEvent event, WalterModemMqttStatus status)
{
    WalterModemEventHandler *handler = _eventHandlers + WALTER_MODEM_EVENT_TYPE_MQTT;
    if (handler->mqttHandler == nullptr) {
        return;
    }

    auto start = std::chrono::steady_clock::now();
    handler->mqttHandler(event, status, handler->args);
    _checkEventDuration(start);
}
    #pragma endregion

    #pragma region PUBLIC_METHODS
WalterModemMqttStatus WalterModem::getMqttStatus()
{
    return _mqttStatus;
}

bool WalterModem::mqttConfig(
    const char *clientId, const char *username, const char *password, uint8_t tlsProfileId)
{
    WalterModemRsp *rsp = NULL;
    walterModemCb cb = NULL;
    void *args = NULL;

    WalterModemBuffer *buf = _getFreeBuffer();
    buf->size += sprintf((char*) buf->data, "AT+SQNSMQTTCFG=0,\"%s\"", clientId);

    if(username && password) {
        buf->size += sprintf((char*)buf->data + buf->size,",\"%s\",\"%s\"", username, password);
    }

    if(tlsProfileId > 0) {
        if(!(username && password)) {
            buf->size += sprintf((char*) buf->data + buf->size, ",,");
        }

        buf->size += sprintf((char*) buf->data + buf->size, ",%u", tlsProfileId);
    }
    
    _runCmd(arr((const char*) buf->data), "OK", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::mqttDisconnect(WalterModemRsp *rsp, walterModemCb cb, void *args)
{
    _runCmd(arr("AT+SQNSMQTTDISCONNECT=0"), "+SQNSMQTTONDISCONNECT:0,", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::mqttConnect(
    const char *serverName,
    uint16_t port,
    uint16_t keepAlive,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    _runCmd(
        arr("AT+SQNSMQTTCONNECT=0,", _atStr(serverName), ",", _atNum(port), ",", _atNum(keepAlive)),
        "+SQNSMQTTONCONNECT:0,",
        rsp,
        cb,
        args);
    _returnAfterReply();
}

bool WalterModem::mqttPublish(
    const char *topicString,
    uint8_t *data,
    uint16_t dataSize,
    uint8_t qos,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    if (getNetworkRegState() != WALTER_MODEM_NETWORK_REG_REGISTERED_HOME &&
        getNetworkRegState() != WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING) {
        ESP_LOGD("WalterModem", "network is not connected!");
        _returnState(WALTER_MODEM_STATE_ERROR);
    }

    _runCmd(
        arr("AT+SQNSMQTTPUBLISH=0,", _atStr(topicString), ",", _atNum(qos), ",", _atNum(dataSize)),
        "+SQNSMQTTONPUBLISH:0,",
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

bool WalterModem::mqttSubscribe(
    const char *topicString, uint8_t qos, WalterModemRsp *rsp, walterModemCb cb, void *args)
{
    int index = -1;

    for (size_t i = 0; i < WALTER_MODEM_MQTT_MAX_TOPICS; i++) {
        if (_mqttTopics[i].free) {
            index = i;
            /*Reserve the topic by setting free to false*/
            _mqttTopics[i].free = false;
            _mqttTopics[i].qos = qos;
            _currentTopic = &_mqttTopics[i];
            _strncpy_s(_mqttTopics[i].topic, topicString, WALTER_MODEM_MQTT_TOPIC_BUF_SIZE);

            break;
        }
    }

    if (index < 0) {
        _currentTopic = NULL;
        rsp->data.mqttResponse.mqttStatus = WALTER_MODEM_MQTT_UNAVAILABLE;
        _returnState(WALTER_MODEM_STATE_ERROR);
    }

    auto completeHandler = [](WalterModemCmd *cmd, WalterModemState result) {
        if (result == WALTER_MODEM_STATE_ERROR) {
            /*If subscription was not succesfull free the topic so we can try again.*/
            _currentTopic->free = true;
        }
    };

    _runCmd(
        arr("AT+SQNSMQTTSUBSCRIBE=0,", _atStr(topicString), ",", _atNum(qos)),
        "+SQNSMQTTONSUBSCRIBE:0,",
        rsp,
        cb,
        args,
        completeHandler);
    _returnAfterReply();
}

bool WalterModem::mqttDidRing(
    const char *topic, uint8_t *targetBuf, uint16_t targetBufSize, WalterModemRsp *rsp)
{
    /* this is by definition a blocking call without callback.
     * it is only used when the arduino user is not taking advantage of
     * the (TBI) ring notification events which give access to the raw
     * buffer (a targetBuf is not needed).
     */
    walterModemCb cb = NULL;
    void *args = NULL;

    uint8_t idx;
    for (idx = 0; idx < WALTER_MODEM_MQTT_MAX_PENDING_RINGS; idx++) {
        if (!strncmp(topic, _mqttRings[idx].topic, strlen(topic)) && !_mqttRings[idx].free) {
            break;
        }
    }

    if (idx == WALTER_MODEM_MQTT_MAX_PENDING_RINGS) {
        _returnState(WALTER_MODEM_STATE_NO_DATA);
    }

    if (targetBufSize < _mqttRings[idx].length) {
        _returnState(WALTER_MODEM_STATE_NO_MEMORY);
    }

    if (_mqttRings[idx].qos == 0) {
        /* no msg id means qos 0 message */
        _runCmd(
            arr("AT+SQNSMQTTRCVMESSAGE=0,", _atStr(topic)),
            "OK",
            rsp,
            cb,
            args,
            NULL,
            (void *)(uintptr_t)idx,
            WALTER_MODEM_CMD_TYPE_TX_WAIT,
            targetBuf,
            _mqttRings[idx].length);

        _returnAfterReply();
    } else {
        _runCmd(
            arr("AT+SQNSMQTTRCVMESSAGE=0,", _atStr(topic), ",", _atNum(_mqttRings[idx].messageId)),
            "OK",
            rsp,
            cb,
            args,
            NULL,
            (void *)(uintptr_t)idx,
            WALTER_MODEM_CMD_TYPE_TX_WAIT,
            targetBuf,
            _mqttRings[idx].length);

        _returnAfterReply();
    }
}

void WalterModem::mqttSetEventHandler(walterModemMQTTEventHandler handler, void *args)
{
    _eventHandlers[WALTER_MODEM_EVENT_TYPE_MQTT].mqttHandler = handler;
    _eventHandlers[WALTER_MODEM_EVENT_TYPE_MQTT].args = args;
}
    #pragma endregion
#endif