/**
 * @file WalterBlueCherry.cpp
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
 * This file contains the implementation of the BlueCherry cloud protocol in the Walter Modem
 * library. BlueCherry cloud uses CoAP + DTLS to communicate with the cloud.
 */

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <WalterDefines.h>
#include <esp_task_wdt.h>
#include <esp_log.h>

#if CONFIG_WALTER_MODEM_ENABLE_BLUECHERRY && !CONFIG_WALTER_MODEM_ENABLE_MOTA
#error Bluecherry cannot be enabled with OTA or MOTA disabled.
#endif

#if CONFIG_WALTER_MODEM_ENABLE_BLUECHERRY && !CONFIG_WALTER_MODEM_ENABLE_SOCKETS
#error Bluecherry cannot be enabled with sockets disabled.
#endif

#if CONFIG_WALTER_MODEM_ENABLE_BLUECHERRY

#pragma region PRIVATE_METHODS

bool WalterModem::_blueCherryProcessEvent(uint8_t* data, uint8_t len)
{
  switch(data[0]) {
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
    ESP_LOGD("WalterModem", "Error: invalid BlueCherry event type 0x%x from cloud server", data[0]);
    return true;
  }

  return true;
}

bool WalterModem::_blueCherrySocketConfigure()
{
  if(_blueCherry.bcProfileId == 0) {
    return false;
  }

  bool success = true;

  success &= socketConfig(_blueCherry.bcProfileId);
  success &= socketConfigExtended(_blueCherry.bcProfileId);
  success &= socketConfigSecure(_blueCherry.bcProfileId, true, _blueCherry.tls_profile_id);

  return success;
}

bool WalterModem::_blueCherrySocketConnect()
{

  WalterModem::socketGetState();

  if(_blueCherry.bcProfileId == 0) {
    if(WalterModemSocket* sock = _socketReserve(); sock != NULL) {
      _blueCherry.bcProfileId = sock->id;
    } else {
      return false;
    }
  }

  WalterModemSocket* sock = _socketGet(_blueCherry.bcProfileId);

  for(int attempt = 0; attempt < 5; ++attempt) {
    switch(sock->state) {
    case WALTER_MODEM_SOCKET_STATE_FREE:
    case WALTER_MODEM_SOCKET_STATE_RESERVED:
    case WALTER_MODEM_SOCKET_STATE_CLOSED:
      if(!_blueCherrySocketConfigure()) {
        break;
      }
      continue;

    case WALTER_MODEM_SOCKET_STATE_READY:
      if(!socketDial(_blueCherry.bcProfileId, WALTER_MODEM_SOCKET_PROTO_UDP,
                     WALTER_MODEM_BLUECHERRY_PORT, WALTER_MODEM_BLUECHERRY_HOSTNAME)) {
        break;
      }
      continue;

    case WALTER_MODEM_SOCKET_STATE_OPENED:
    case WALTER_MODEM_SOCKET_STATE_PENDING_NO_DATA:
    case WALTER_MODEM_SOCKET_STATE_PENDING_WITH_DATA:
      return true;

    case WALTER_MODEM_SOCKET_STATE_SUSPENDED:
      break;

    default:
      break;
    }
  }

  _blueCherry.bcProfileId = 0;
  return false;
}

size_t WalterModem::_blueCherryGetCoapHeaderLength(walter_modem_bluecherry_coap_message_t* msg)
{
  // CoAP header: 1 byte (ver/type/tkl) + 1 byte (code) + 2 bytes (message ID)
  // Then token_len bytes of token (0-8)
  size_t length = 4 + msg->token_len;

  // If there's a payload, we'll have a payload marker (0xFF)
  // CoAP uses this to indicate the start of the payload

  return length;
}

bool WalterModem::_blueCherryCoapParse(uint8_t* data, size_t data_len,
                                       walter_modem_bluecherry_coap_message_t* msg)
{
  if(msg == NULL || data == NULL || data_len < 4) {
    return false;
  }

  // Copy the entire buffer into msg->data
  if(data_len > sizeof(msg->data)) {
    return false;
  }
  memcpy(msg->data, data, data_len);
  msg->data_len = data_len;

  size_t byteOffset = 0;

  // Byte 0: Version (2 bits), Type (2 bits), Token Length (4 bits)
  uint8_t recvHeader = data[byteOffset++];
  msg->version = (recvHeader >> 6) & 0x03;
  msg->type = (recvHeader >> 4) & 0x03;
  msg->token_len = recvHeader & 0x0F;

  if(msg->token_len > 8 || data_len < 4 + msg->token_len) {
    return false;
  }

  // Byte 1: Code
  msg->code = data[byteOffset++];

  // Bytes 2-3: Message ID
  msg->message_id = ((uint16_t) data[byteOffset] << 8) | data[byteOffset + 1];
  byteOffset += 2;

  // Token (0-8 bytes)
  if(msg->token_len > 0) {
    memcpy(msg->token, &data[byteOffset], msg->token_len);
  } else {
    memset(msg->token, 0, sizeof(msg->token));
  }

  return true;
}

bool WalterModem::_blueCherryCoapTransmit(walter_modem_bluecherry_coap_message_t* msg)
{
  if(_blueCherry.bcProfileId == 0) {
    return false;
  }

  size_t offset = 0;

  msg->data[offset++] = (msg->version << 6) | (msg->type << 4) | msg->token_len;
  msg->data[offset++] = msg->code;
  msg->data[offset++] = (msg->message_id >> 8) & 0xFF;
  msg->data[offset++] = msg->message_id & 0xFF;

  memcpy(&msg->data[offset], msg->token, msg->token_len);
  offset += msg->token_len;

  if(msg->data_len > offset) {
    msg->data[offset++] = 0xFF;
  }

  return socketSend(_blueCherry.bcProfileId, msg->data, msg->data_len);
}

void WalterModem::_blueCherrySocketEventHandler(WalterModemSocketEvent event, uint16_t dataReceived,
                                                uint8_t* dataBuffer)
{
  switch(event) {
  case WALTER_MODEM_SOCKET_EVENT_RING:
    if(_blueCherryCoapParse(dataBuffer, dataReceived, &_bc_msg_in)) {
      if(_bc_msg_in.type == WALTER_MODEM_BLUECHERRY_COAP_SEND_TYPE_ACK &&
         _bc_msg_in.message_id == _blueCherry.lastTransmittedMsgID &&
         _bc_msg_in.message_id > _blueCherry.lastReceivedMsgID) {
        _blueCherry.lastReceivedMsgID = _bc_msg_in.message_id;
        _bc_msg_in.should_process = true;
        xQueueSend(_bluecherryIOMessageQueue.handle, &_bc_msg_in, 0);
        _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_RESPONSE_READY;
      } else if(_bc_msg_in.type == WALTER_MODEM_BLUECHERRY_COAP_SEND_TYPE_CON) {
        _bc_msg_in.type = WALTER_MODEM_BLUECHERRY_COAP_SEND_TYPE_ACK;
        _bc_msg_in.should_process = true;
        _blueCherryCoapTransmit(&_bc_msg_in);
        xQueueSend(_bluecherryIOMessageQueue.handle, &_bc_msg_in, 0);
      } else {
        ESP_LOGD("WalterModem", "Dropping invalid (or duplicate) BlueCherry CoAP message");
        return;
      }

      break;

    case WALTER_MODEM_SOCKET_EVENT_DISCONNECTED:
      _blueCherry.bcProfileId = 0;
      break;

    case WALTER_MODEM_SOCKET_EVENT_CONNECTED:
    default:
      break;
    }
  }
}

bool WalterModem::_blueCherryCoapProcessReceived(walter_modem_bluecherry_coap_message_t* msg,
                                                 walter_modem_rsp_t* rsp)
{
  uint16_t payloadOffset = _blueCherryGetCoapHeaderLength(msg) + 1; // +1 for payload marker
  while(payloadOffset < msg->data_len) {
    uint8_t topic = msg->data[payloadOffset++];
    uint8_t dataLen = msg->data[payloadOffset++];

    /*
     * Topic 0 is reserved for BlueCherry events, which are also visible to walter as mqtt
     * messages on topic id 0
     */
    if(topic == 0) {
      _blueCherry.moreDataAvailable = true;

      if(_blueCherryProcessEvent(msg->data + payloadOffset, dataLen)) {
        _blueCherry.emitErrorEvent = true;
        _blueCherry.otaSize = 0;
      }
    } else {
      if(_BCMessageHandler != NULL) {
        _BCMessageHandler(topic, msg->data + payloadOffset, dataLen, _BCMessageHandlerArgs);
      }
    }

    WalterModemBlueCherryMessage* message =
        rsp->data.blueCherry.messages + rsp->data.blueCherry.messageCount;
    message->topic = topic;
    message->dataSize = dataLen;
    message->data = msg->data + payloadOffset;

    rsp->data.blueCherry.messageCount++;

    payloadOffset += dataLen;
  }

  if(_blueCherry.emitErrorEvent) {
    uint8_t blueCherryErrorEventCode = WALTER_MODEM_BLUECHERRY_EVENT_TYPE_OTA_ERROR;
    blueCherryPublish(0, 1, &blueCherryErrorEventCode);
  }

  if(_blueCherry.status == WALTER_MODEM_BLUECHERRY_STATUS_TIMED_OUT) {
    rsp->data.blueCherry.state = WALTER_MODEM_BLUECHERRY_STATUS_TIMED_OUT;
    return false;
  }

  if(_blueCherry.status == WALTER_MODEM_BLUECHERRY_STATUS_NOT_CONNECTED) {
    rsp->data.blueCherry.state = WALTER_MODEM_BLUECHERRY_STATUS_NOT_CONNECTED;
    return false;
  }

  if(_blueCherry.moreDataAvailable) {
    _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_PENDING_MESSAGES;
    rsp->data.blueCherry.state = WALTER_MODEM_BLUECHERRY_STATUS_PENDING_MESSAGES;
    rsp->data.blueCherry.syncFinished = false;
  } else {
    _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_IDLE;
    rsp->data.blueCherry.state = WALTER_MODEM_BLUECHERRY_STATUS_IDLE;
    rsp->data.blueCherry.syncFinished = true;
  }

  return true;
}

bool WalterModem::_blueCherrySynchronize(walter_modem_rsp_t* rsp, bool force_sync)
{
  // Reject when modem is in a different BC operation
  if(_blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_IDLE &&
     _blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_PENDING_MESSAGES &&
     _blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_TIMED_OUT &&
     _blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_NOT_CONNECTED) {
    return false;
  }

  // Prepare response if requested
  rsp->type = WALTER_MODEM_RSP_DATA_TYPE_BLUECHERRY;
  rsp->data.blueCherry.messageCount = 0;
  rsp->data.blueCherry.syncFinished = true;

  bool ack_received = false;

reconnect:
  if(!_blueCherry.connected) {
    if(!_blueCherrySocketConnect()) {
      _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_NOT_CONNECTED;
      return false;
    }
    _blueCherry.connected = true;
  }

  // Inject forced message if needed
  if(xQueuePeek(_bluecherryIOMessageQueue.handle, &_bc_msg_processing, 0) != pdPASS) {
    if(!force_sync) {
      _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_IDLE;
      return false;
    }

    // Create an empty CoAP CON message
    _bc_msg_processing.should_process = false;
    _bc_msg_processing.version = 1;
    _bc_msg_processing.type = WALTER_MODEM_BLUECHERRY_COAP_SEND_TYPE_CON;
    _bc_msg_processing.token_len = 0;
    _bc_msg_processing.data_len = _blueCherryGetCoapHeaderLength(&_bc_msg_processing);

    xQueueSend(_bluecherryIOMessageQueue.handle, &_bc_msg_processing, 0);
    xQueuePeek(_bluecherryIOMessageQueue.handle, &_bc_msg_processing, 0);
  }

  // RETRANSMISSION LOOP — retry failed transmits
  for(int attempt = 0; attempt < 4; ++attempt) {
    // Prepare message IDs
    if(_blueCherry.lastTransmittedMsgID == 0) {
      _blueCherry.lastTransmittedMsgID = 1;
    }

    _bc_msg_processing.message_id = _blueCherry.lastTransmittedMsgID;
    _bc_msg_processing.code = _blueCherry.lastTransmittedMsgID - _blueCherry.lastReceivedMsgID - 1;

    // Process incoming received message
    if(_bc_msg_processing.should_process) {
      _blueCherryCoapProcessReceived(&_bc_msg_processing, rsp);
      xQueueReceive(_bluecherryIOMessageQueue.handle, &_bc_msg_processing, 0);

      if(xQueuePeek(_bluecherryIOMessageQueue.handle, &_bc_msg_processing, 0) != pdPASS) {
        _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_IDLE;
        return true;
      }

      // Continue to next message without consuming a retry
      attempt--;
      continue;
    }

    // Transmit
    _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_AWAITING_RESPONSE;

    if(!_blueCherryCoapTransmit(&_bc_msg_processing)) {
      ESP_LOGW("WalterModem", "BC transmit failed, reconnecting.");
      _blueCherry.connected = false;
      goto reconnect;
    }

    // Wait for ACK with exponential backoff
    TickType_t start = xTaskGetTickCount();
    TickType_t timeout = pdMS_TO_TICKS(1000 * (1 << attempt));

    while(_blueCherry.status == WALTER_MODEM_BLUECHERRY_STATUS_AWAITING_RESPONSE &&
          (xTaskGetTickCount() - start) < timeout) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }

    if(_blueCherry.status == WALTER_MODEM_BLUECHERRY_STATUS_RESPONSE_READY) {
      ack_received = true;
      _blueCherry.lastTransmittedMsgID++;
      _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_IDLE;

      // Consume message
      xQueueReceive(_bluecherryIOMessageQueue.handle, &_bc_msg_processing, 0);

      // Done?
      if(xQueuePeek(_bluecherryIOMessageQueue.handle, &_bc_msg_processing, 0) != pdPASS) {
        return true;
      }

      // More messages → continue send loop with attempt=0
      attempt = -1;
      continue;
    }

    // No ACK → retry (normal retransmission behavior)
  }

  // ---- ALL RETRIES FAILED ----
  _blueCherry.lastTransmittedMsgID++;
  _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_TIMED_OUT;
  return false;
}

void WalterModem::_blueCherrySyncTask(void* args)
{
  if(_watchdogTimeout) {
    esp_task_wdt_add(NULL);
  }

  bool force_sync = true;

  walter_modem_rsp_t rsp;

  while(true) {
    tickleWatchdog();
    vTaskDelay(pdMS_TO_TICKS(10));

    // Force a sync even if the queue is empty if the auto sync interval has elapsed
    TickType_t now = xTaskGetTickCount();
    if((now - _blueCherry.lastTransmissionTick) * portTICK_PERIOD_MS >=
       _blueCherry.auto_sync_interval_ms) {
      _blueCherry.lastTransmissionTick = now;
      force_sync = true;
    }

    if(uxQueueMessagesWaiting(_bluecherryIOMessageQueue.handle) > 0) {
      force_sync = true;
    }

    if(force_sync) {
      _blueCherrySynchronize(&rsp, force_sync);
      force_sync = !rsp.data.blueCherry.syncFinished;
    }
  }
}

#pragma endregion
#pragma region PUBLIC_METHODS

bool WalterModem::blueCherryProvision(const char* cert_pem, const char* priv_key_pem,
                                      const char* ca_cert, walter_modem_rsp_t* rsp,
                                      walter_modem_cb_t cb, void* args)
{
  WalterModemState result = WALTER_MODEM_STATE_OK;

  if(cert_pem) {
    if(!tlsWriteCredential(false, 5, cert_pem)) {
      result = WALTER_MODEM_STATE_ERROR;
    }
  }

  if(priv_key_pem) {
    if(!tlsWriteCredential(true, 0, priv_key_pem)) {
      result = WALTER_MODEM_STATE_ERROR;
    }
  }

  if(ca_cert) {
    if(!tlsWriteCredential(false, 6, ca_cert)) {
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

bool WalterModem::blueCherryInit(uint8_t tls_profile_id, uint8_t* ota_buffer,
                                 walter_modem_rsp_t* rsp, uint16_t ack_timeout_s)
{
  ESP_LOGW("DEPRECATION", "Use blueCherryInit(msg_handler, msg_handler_args, auto_sync_enable, "
                          "tls_profile_id, socket_profile_id) instead.");

  WalterModemBlueCherryInitStatus status;
  status = blueCherryInit(NULL, NULL, false, tls_profile_id, 1);

  if(status == WALTER_MODEM_BLUECHERRY_INIT_STATUS_NOT_PROVISIONED) {
    rsp->type = WALTER_MODEM_RSP_DATA_TYPE_BLUECHERRY;
    rsp->data.blueCherry.state = WALTER_MODEM_BLUECHERRY_STATUS_NOT_PROVISIONED;
    return false;
  }

  if(status == WALTER_MODEM_BLUECHERRY_INIT_STATUS_INITIALIZED) {
    return true;
  }

  return false;
}

WalterModemBlueCherryInitStatus
WalterModem::blueCherryInit(WalterModemBluecherryMessageHandlerCB msg_handler,
                            void* msg_handler_args, bool auto_sync_enable, int tls_profile_id,
                            int socket_profile_id)
{
  if(_blueCherry.initialized) {
    return WALTER_MODEM_BLUECHERRY_INIT_STATUS_INITIALIZED;
  }

  if((!blueCherryIsProvisioned() ||
      !tlsConfigProfile(tls_profile_id, WALTER_MODEM_TLS_VALIDATION_URL_AND_CA,
                        WALTER_MODEM_TLS_VERSION_12, 6, 5, 0))) {
    _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_NOT_PROVISIONED;
    return WALTER_MODEM_BLUECHERRY_INIT_STATUS_NOT_PROVISIONED;
  }

  _bluecherryIOMessageQueue.handle = xQueueCreateStatic(
      WALTER_MODEM_MAX_QUEUED_BLUECHERRY_MESSAGES, sizeof(walter_modem_bluecherry_coap_message_t),
      _bluecherryIOMessageQueue.mem, &(_bluecherryIOMessageQueue.memHandle));

  _blueCherry.tls_profile_id = tls_profile_id;
  _blueCherry.bcProfileId = socket_profile_id;
  _blueCherry.auto_sync_enabled = auto_sync_enable;
  _BCMessageHandler = msg_handler;
  _BCMessageHandlerArgs = msg_handler_args;

  if(auto_sync_enable) {
    _bc_sync_task_handle = xTaskCreateStaticPinnedToCore(
        _blueCherrySyncTask, "bc_sync", 4096, NULL, 1, _bc_sync_task_stack, &_bc_sync_task_buf, 0);
    if(!_bc_sync_task_handle) {
      ESP_LOGW("WalterModem", "Failed to create BlueCherry sync task (Not enough memory?)");
      return WALTER_MODEM_BLUECHERRY_INIT_STATUS_ERROR;
    }
  }

  /* Get OTA partition information */
  const esp_partition_t* part = esp_ota_get_running_partition();
  uint8_t sha_256[33];
  sha_256[0] = WALTER_MODEM_BLUECHERRY_EVENT_TYPE_PARTITION_HASH;
  esp_partition_get_sha256(part, sha_256 + 1);

  /* Publish partition hash to BlueCherry cloud to verify the running firmware version */
  blueCherryPublish(0, 33, sha_256);
  _blueCherry.initialized = true;

  /**
   * Wait for initial bluecherry connection cycle to complete. (when auto sync is enabled)
   * This verifies the application was able to connect to bluecherry successfully and has
   * transmitted the OTA partition hash.
   *
   * Common practice states that a watchdog should trigger a reboot and roll back to a previous
   * firmware partition if no successful connection was made after an OTA update.
   * If this bluecherry init function returns, the application can assume the firmware is valid.
   */
  if(auto_sync_enable) {
    while(_blueCherry.lastReceivedMsgID == 0x0000) {
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }

  return WALTER_MODEM_BLUECHERRY_INIT_STATUS_INITIALIZED;
}

bool WalterModem::blueCherryPublish(uint8_t topic, uint8_t len, uint8_t* data, bool urgent)
{
  _bc_msg_out.should_process = false;
  _bc_msg_out.version = 1;
  _bc_msg_out.type = WALTER_MODEM_BLUECHERRY_COAP_SEND_TYPE_CON;
  _bc_msg_out.token_len = 0;

  if(_bc_msg_out.data_len + len + _blueCherryGetCoapHeaderLength(&_bc_msg_out) + 1 >=
     WALTER_MODEM_MAX_OUTGOING_MESSAGE_LEN) {
    if(xQueueSend(_bluecherryIOMessageQueue.handle, &_bc_msg_out, 0) != pdPASS) {
      ESP_LOGW("WalterModem", "BlueCherry IO queue full, cannot enqueue message.");
      return false;
    }
    _bc_msg_out.data_len = _blueCherryGetCoapHeaderLength(&_bc_msg_out) + 1;
  }

  _bc_msg_out.data[_bc_msg_out.data_len] = topic;
  _bc_msg_out.data[_bc_msg_out.data_len + 1] = len;
  memcpy(_bc_msg_out.data + _bc_msg_out.data_len + 2, data, len);
  _bc_msg_out.data_len += len + 2;

  if(urgent) {
    if(xQueueSend(_bluecherryIOMessageQueue.handle, &_bc_msg_out, 0) != pdPASS) {
      ESP_LOGW("WalterModem", "BlueCherry IO queue full, cannot enqueue message.");
      return false;
    }
    _bc_msg_out.data_len = _blueCherryGetCoapHeaderLength(&_bc_msg_out) + 1;
  }
  return true;
}

bool WalterModem::blueCherrySync(walter_modem_rsp_t* rsp, bool force_sync)
{
  if(_blueCherry.auto_sync_enabled) {
    ESP_LOGW("WalterModem",
             "blueCherrySync called while auto sync is enabled. Ignoring manual sync request.");
    return false;
  }

  if(_blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_IDLE &&
     _blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_PENDING_MESSAGES &&
     _blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_TIMED_OUT &&
     _blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_NOT_CONNECTED) {
    return false;
  }

  return _blueCherrySynchronize(rsp, force_sync);
}

bool WalterModem::blueCherryClose(walter_modem_rsp_t* rsp, walter_modem_cb_t cb, void* args)
{
  _runCmd({ "AT+SQNCOAPCLOSE=0" }, "+SQNCOAPCLOSED: ", rsp, cb, args);
  _returnAfterReply();
}

size_t WalterModem::blueCherryGetOtaProgressPercentage()
{
  if(_blueCherry.otaSize == 0) {
    return 0; /* NO dividing by ZERO */
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