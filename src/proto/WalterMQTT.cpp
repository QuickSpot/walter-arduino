/**
 * @file WalterMQTT.cpp
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
#include <esp_log.h>

#if CONFIG_WALTER_MODEM_ENABLE_MQTT
#pragma region PUBLIC_METHODS
bool WalterModem::mqttConfig(const char* client_id, const char* username, const char* password,
                             uint8_t tls_profile_id, walter_modem_rsp_t* rsp, walter_modem_cb_t cb,
                             void* args)
{
  walter_modem_buffer_t* buf = _getFreeBuffer();
  buf->size += sprintf((char*) buf->data, "AT+SQNSMQTTCFG=0,\"%s\"", client_id);

  if(username && password) {
    buf->size += sprintf((char*) buf->data + buf->size, ",\"%s\",\"%s\"", username, password);
  }

  if(tls_profile_id > 0) {
    if(!(username && password)) {
      buf->size += sprintf((char*) buf->data + buf->size, ",,");
    }

    buf->size += sprintf((char*) buf->data + buf->size, ",%u", tls_profile_id);
  }

  _runCmd(arr((const char*) buf->data), "OK", rsp, cb, args, NULL, NULL,
          WALTER_MODEM_CMD_TYPE_TX_WAIT, NULL, 0, buf);
  _returnAfterReply();
}

bool WalterModem::mqttDisconnect(walter_modem_rsp_t* rsp, walter_modem_cb_t cb, void* args)
{
  _runCmd(arr("AT+SQNSMQTTDISCONNECT=0"), "OK", rsp, cb, args);
  _returnAfterReply();
}

bool WalterModem::mqttConnect(const char* hostname, uint16_t port, uint16_t keep_alive,
                              walter_modem_rsp_t* rsp, walter_modem_cb_t cb, void* args)
{
  _runCmd(
      arr("AT+SQNSMQTTCONNECT=0,", _atStr(hostname), ",", _atNum(port), ",", _atNum(keep_alive)),
      "OK", rsp, cb, args);
  _returnAfterReply();
}

bool WalterModem::mqttPublish(const char* topic, uint8_t* buf, uint16_t buf_size, uint8_t qos,
                              walter_modem_rsp_t* rsp, walter_modem_cb_t cb, void* args)
{
  _runCmd(arr("AT+SQNSMQTTPUBLISH=0,", _atStr(topic), ",", _atNum(qos), ",", _atNum(buf_size)),
          "OK", rsp, cb, args, NULL, NULL, WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT, buf, buf_size);
  _returnAfterReply();
}

bool WalterModem::mqttSubscribe(const char* topic, uint8_t qos, walter_modem_rsp_t* rsp,
                                walter_modem_cb_t cb, void* args)
{
  int index = -1;

  for(size_t i = 0; i < WALTER_MODEM_MQTT_MAX_TOPICS; i++) {
    if(_mqttTopics[i].free) {
      index = i;
      /*Reserve the topic by setting free to false*/
      _mqttTopics[i].free = false;
      _mqttTopics[i].qos = qos;
      _currentTopic = &_mqttTopics[i];
      _strncpy_s(_mqttTopics[i].topic, topic, WALTER_MODEM_MQTT_TOPIC_BUF_SIZE);

      break;
    } else if(!strncmp(topic, _mqttTopics[i].topic, WALTER_MODEM_MQTT_TOPIC_BUF_SIZE - 1)) {
      /*Topic already in use*/
      _returnState(WALTER_MODEM_STATE_OK);
      break;
    }
  }

  if(index < 0) {
    _currentTopic = NULL;
    if(rsp) {
      rsp->data.mqttResponse.mqttStatus = WALTER_MODEM_MQTT_UNAVAILABLE;
    }
    _returnState(WALTER_MODEM_STATE_ERROR);
  }

  auto completeHandler = [](walter_modem_cmd_t* cmd, WalterModemState result) {
    if(result == WALTER_MODEM_STATE_ERROR) {
      /*If subscription was not succesfull free the topic so we can try again.*/
      _currentTopic->free = true;
    }
  };

  _runCmd(arr("AT+SQNSMQTTSUBSCRIBE=0,", _atStr(topic), ",", _atNum(qos)), "OK", rsp, cb, args,
          completeHandler);
  _returnAfterReply();
}

bool WalterModem::mqttReceive(const char* topic, int message_id, uint8_t* buf, size_t buf_size,
                              walter_modem_rsp_t* rsp, walter_modem_cb_t cb, void* args)
{

  size_t readable_size = (buf_size > 4096) ? 4096 : buf_size;

  if(message_id == 0) {
    /* no msg id means qos 0 message */
    _runCmd(arr("AT+SQNSMQTTRCVMESSAGE=0,", _atStr(topic)), "OK", rsp, cb, args, NULL, NULL,
            WALTER_MODEM_CMD_TYPE_TX_WAIT, buf, readable_size);
    _returnAfterReply();
  } else {
    _runCmd(arr("AT+SQNSMQTTRCVMESSAGE=0,", _atStr(topic), ",", _atNum(message_id), ",",
                _atNum(readable_size)),
            "OK", rsp, cb, args, NULL, NULL, WALTER_MODEM_CMD_TYPE_TX_WAIT, buf, readable_size);
    _returnAfterReply();
  }
}

void WalterModem::setMQTTEventHandler(walterModemMQTTEventHandler handler, void* args)
{
  _eventHandlers[WALTER_MODEM_EVENT_TYPE_MQTT].mqttHandler = handler;
  _eventHandlers[WALTER_MODEM_EVENT_TYPE_MQTT].args = args;
}

#pragma endregion
#pragma region DEPRICATION
bool WalterModem::mqttDidRing(const char* topic, uint8_t* targetBuf, uint16_t targetBufSize,
                              walter_modem_rsp_t* rsp)
{
  ESP_LOGW("DEPRECATION",
           "this mqttDidRing method is deprecated and will be removed in future releases. Use "
           "mqttReceive(...) instead.");

  return mqttReceive(topic, 0, targetBuf, targetBufSize, rsp, NULL, NULL);
}

WMMQTTConnRC WalterModem::getMqttStatus()
{
  ESP_LOGW("DEPRECATION", "getMqttStatus() is deprecated and will be removed in future releases.");
  return _mqttStatus;
}

void WalterModem::mqttSetEventHandler(walterModemMQTTEventHandler handler, void* args)
{
  ESP_LOGW("DEPRECATION",
           "mqttSetEventHandler is deprecated and will be removed in future releases. Use "
           "setMQTTEventHandler(...) instead.");

  _eventHandlers[WALTER_MODEM_EVENT_TYPE_MQTT].mqttHandler = handler;
  _eventHandlers[WALTER_MODEM_EVENT_TYPE_MQTT].args = args;
}
#pragma endregion
#endif