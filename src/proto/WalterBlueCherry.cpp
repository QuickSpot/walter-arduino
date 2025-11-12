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

#include <WalterDefines.h>
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

/**
 * @brief Parse a BlueCherry CoAP message from a buffer.
 *
 * This method will attempt to parse a BlueCherry CoAP message from the provided buffer. It extracts
 * the header from the binary data and populates the provided message structure.
 *
 * @param[out] msg Pointer to the BlueCherry CoAP message structure to populate.
 * @param[in] buf Pointer to the buffer containing the raw CoAP message data.
 * @param[in] buf_size Size of the buffer in bytes.
 *
 * @return true if the message was successfully parsed, false otherwise.
 */
bool WalterModem::_blueCherryCoapParse(walter_modem_bluecherry_coap_message_t* msg, uint8_t* buf,
                                       size_t buf_size)
{
  size_t byteOffset = 0;

  // Ensure header has enough bytes for token and message ID
  if(buf_size < WALTER_MODEM_BLUECHERRY_COAP_HEADER_SIZE + msg->token_len) {
    return false;
  }

  uint8_t recvHeader = buf[byteOffset++];
  msg->version = (recvHeader >> 6) & 0x03;
  msg->type = (recvHeader >> 4) & 0x03;
  msg->token_len = recvHeader & 0x0F;

  // Ensure header has enough bytes for token and message ID
  if(buf_size < WALTER_MODEM_BLUECHERRY_COAP_HEADER_SIZE + msg->token_len) {
    return false;
  }

  msg->code = buf[byteOffset++];
  uint8_t highByteRecvMsgId = buf[byteOffset++];
  uint8_t lowByteRecvMsgId = buf[byteOffset++];
  msg->message_id = ((uint16_t) highByteRecvMsgId << 8) | lowByteRecvMsgId;

  if(msg->token_len > sizeof(msg->token)) {
    return false;
  }
  memcpy(msg->token, &buf[byteOffset], msg->token_len);
  byteOffset += msg->token_len;

  // Check for optional payload marker (0xFF)
  msg->data_size = 0;
  if(byteOffset < buf_size && buf[byteOffset] == 0xFF) {
    // There *is* a payload marker, so skip it and read data
    byteOffset++;
    msg->data_size = buf_size - byteOffset;

    if(msg->data_size > sizeof(msg->data)) {
      return false;
    }

    memcpy(msg->data, &buf[byteOffset], msg->data_size);
  }

  return true;
}

/**
 * @brief Transmit a BlueCherry CoAP message over the socket.
 *
 * This method will transmit the provided BlueCherry CoAP message over the established socket. It
 * will set the coap headers from the message structure and send the message data (offset by header
 * size).
 *
 * @param[in] msg Pointer to the BlueCherry CoAP message structure to transmit.
 *
 * @return true if the message was successfully transmitted, false otherwise.
 */
bool WalterModem::_blueCherryCoapTransmit(walter_modem_bluecherry_coap_message_t* msg)
{
  if(_blueCherry.bcProfileId == 0) {
    return false;
  }

  size_t offset = 0;
  buf[offset++] = (msg->version << 6) | (msg->type << 4) | msg->token_len;
  buf[offset++] = msg->code;
  buf[offset++] = (msg->message_id >> 8) & 0xFF;
  buf[offset++] = msg->message_id & 0xFF;

  memcpy(&buf[offset], msg->token, msg->token_len);
  offset += msg->token_len;

  buf[offset++] = 0xFF;

  memcpy(&buf[offset], msg->data, msg->data_size);
  offset += msg->data_size;

  return socketSend(_blueCherry.bcProfileId, buf, offset);
}

/**
 * @brief Handle BlueCherry socket events.
 *
 * This method processes events received on the BlueCherry socket, such as incoming data or
 * connection events.
 *
 * @param event The type of socket event.
 * @param dataReceived The amount of data received (if applicable).
 * @param dataBuffer Pointer to the buffer containing the received data (if applicable).
 *
 * @return void
 */
void WalterModem::_blueCherrySocketEventHandler(WalterModemSocketEvent event, uint16_t dataReceived,
                                                uint8_t* dataBuffer)
{
  switch(event) {
  case WALTER_MODEM_SOCKET_EVENT_RING:
    if(_blueCherryCoapParse(&_blueCherry.message_in, dataBuffer, dataReceived)) {
      if(_blueCherry.message_in.type == WALTER_MODEM_BLUECHERRY_COAP_SEND_TYPE_ACK &&
         _blueCherry.message_in.message_id == _blueCherry.lastTransmittedMsgID &&
         _blueCherry.message_in.message_id > _blueCherry.lastReceivedMsgID) {
        _blueCherry.lastReceivedMsgID = _blueCherry.message_in.message_id;
        _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_RESPONSE_READY;
      } else if(_blueCherry.message_in.type == WALTER_MODEM_BLUECHERRY_COAP_SEND_TYPE_CON) {
        _blueCherry.message_in.type = WALTER_MODEM_BLUECHERRY_COAP_SEND_TYPE_ACK;
        _blueCherryCoapTransmit(&_blueCherry.message_in);
      } else {
        ESP_LOGD("WalterModem", "Invalid or duplicate CoAP message received from cloud server");
        return;
      }

      xQueueSend(_blueCherry.bcMessageQueue.handle, &_blueCherry.message_in, 0);
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

bool WalterModem::_blueCherryCoapSend()
{
  const uint8_t MAX_RETRANSMIT = 4;
  const double ACK_TIMEOUT = 2.0;
  const double ACK_RANDOM_FACTOR = 1.5;

  // Calculate missed messages
  int32_t lastAcked = _blueCherry.lastReceivedMsgID;
  if(lastAcked > _blueCherry.lastTransmittedMsgID) {
    lastAcked -= 0xffff;
  }
  uint8_t nrMissed = _blueCherry.lastTransmittedMsgID - lastAcked - 1;

  _blueCherrySetCoapHeaders(nrMissed, 0, _blueCherry.lastTransmittedMsgID);

  double timeout = ACK_TIMEOUT * (1 + (rand() / (RAND_MAX + 1.0)) * (ACK_RANDOM_FACTOR - 1));
  _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_AWAITING_RESPONSE;

  for(uint8_t attempt = 1; attempt <= MAX_RETRANSMIT; ++attempt) {
    _blueCherry.lastTransmissionTime = time(NULL);

    socketSend(_blueCherry.bcProfileId, _blueCherry.messageOut, _blueCherry.messageOutLen);

    while(true) {
      if(_blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_AWAITING_RESPONSE) {
        return true;
      }

      vTaskDelay(pdMS_TO_TICKS(10));

      if(difftime(time(NULL), _blueCherry.lastTransmissionTime) >= timeout) {
        break;
      }
    }

    timeout *= 2;
  }

  _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_TIMED_OUT;
  return false;
}

bool WalterModem::_blueCherryCoapProcessReceived(uint8_t* dataBuffer, uint16_t dataReceived)
{
  uint16_t byteOffset = 0;

  if(dataReceived < WALTER_MODEM_BLUECHERRY_COAP_HEADER_SIZE) {
    // No CoAP Headers
    return false;
  }

  uint8_t recvHeader = dataBuffer[byteOffset++];
  uint8_t recvVer = (recvHeader >> 6) & 0x03;

  if(recvVer != 1) {
    // Possible packet malformation
    return false;
  }

  uint8_t recvType = (recvHeader >> 4) & 0x03;
  uint8_t recvTokenLen = recvHeader & 0x0F;
  byteOffset += recvTokenLen;
  uint8_t recvCode = dataBuffer[byteOffset++];
  uint8_t highByteRecvMsgId = dataBuffer[byteOffset++];
  uint8_t lowByteRecvMsgId = dataBuffer[byteOffset++];
  uint16_t recvMsgId = ((uint16_t) highByteRecvMsgId << 8) | lowByteRecvMsgId;
  byteOffset++; // Skip the 0xff payload marker

  if(recvType == WALTER_MODEM_BLUECHERRY_COAP_SEND_TYPE_ACK) {
    if(recvMsgId != _blueCherry.lastTransmittedMsgID) {
      // Acknowledgement was not expected
      return false;
    }
    _blueCherry.lastReceivedMsgID = recvMsgId;
    /* BlueCherry cloud ack means our last error line can be cleared */
    _blueCherry.emitErrorEvent = false;
  }

  switch(recvCode) {
  case WALTER_MODEM_BLUECHERRY_COAP_RSP_VALID:
    _blueCherry.moreDataAvailable = false;
    break;
  case WALTER_MODEM_BLUECHERRY_COAP_RSP_CONTINUE:
    _blueCherry.moreDataAvailable = true;
    break;
  }

  return true;
}

void WalterModem::_blueCherrySyncTask(void* args)
{
  if(_watchdogTimeout) {
    esp_task_wdt_add(NULL);
  }

  bool force_sync = true;

  while(true) {
    tickleWatchdog();
    vTaskDelay(pdMS_TO_TICKS(10));

    // Force a sync even if the queue is empty if the auto sync interval has elapsed
    if(esp_get_time() - _blueCherry.lastTransmissionTime >=
       _blueCherry.auto_sync_interval_ms * 1000ULL) {
      force_sync = true;
    }

    if(xQueueIsEmpty(_bluecherryIOMessageQueue.handle) == pdFALSE) {
      force_sync = true;
    }

    if(force_sync) {
      _blueCherrySynchronize(force_sync);
    }

    force_sync = false;
  }
}

bool WalterModem::_blueCherrySynchronize(bool force_sync, walter_modem_rsp_t* rsp)
{
  if(_blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_IDLE &&
     _blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_PENDING_MESSAGES &&
     _blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_TIMED_OUT &&
     _blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_NOT_CONNECTED) {
    _returnState(WALTER_MODEM_STATE_BUSY)
  }

  if(rsp) {
    rsp->type = WALTER_MODEM_RSP_DATA_TYPE_BLUECHERRY;
    rsp->data.blueCherry.messageCount = 0;
  }

  _blueCherry.lastTransmittedMsgID++;

  bool connected = true;
  bool ack_received = false;
  bool had_messages = false;

  for(int attempt = 0; attempt < 4; ++attempt) {

    if(!connected) {
      // Try to reconnect if not connected
      if(_blueCherrySocketConnect()) {
        attempt = 0;
        connected = true;
      }

      vTaskDelay(pdMS_TO_TICKS(100 * (1 << attempt)));
      continue;
    }

    // Prepare default CoAP headers for forced sync
    _blueCherry.message_processing.data_len = 0;
    _blueCherry.message_processing.type = WALTER_MODEM_BLUECHERRY_COAP_SEND_TYPE_CON;

    bool queue = (xQueuePeek(_bluecherryIOMessageQueue.handle, &_blueCherry.message_processing,
                             0) == pdPASS);

    if(!queue && !force_sync) {
      // No messages and not forcing sync — success if we had sent anything or got an ACK
      if(ack_received || had_messages) {
        _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_IDLE;
        return true;
      }
      return true; // nothing to do, idle
    }

    if(queue) {
      had_messages = true;
    }

    // Transmit or receive the CoAP data
    while(queue || force_sync) {
      _blueCherry.message_processing.message_id = _blueCherry.lastTransmittedMsgID;

      // Attempt to transmit
      if(!_blueCherryCoapTransmit(&_blueCherry.message_processing)) {
        ESP_LOGW("WalterModem", "Failed to transmit bluecherry message.");
        connected = false;
        break;
      }

      if(_blueCherry.message_processing.type == WALTER_MODEM_BLUECHERRY_COAP_SEND_TYPE_CON) {
        _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_AWAITING_RESPONSE;

        // Wait for ACK or timeout
        TickType_t start = xTaskGetTickCount();
        TickType_t timeout = pdMS_TO_TICKS(100 * (1 << attempt));
        while(_blueCherry.status == WALTER_MODEM_BLUECHERRY_STATUS_AWAITING_RESPONSE &&
              (xTaskGetTickCount() - start) < timeout) {
          vTaskDelay(pdMS_TO_TICKS(10));
        }

        if(_blueCherry.status == WALTER_MODEM_BLUECHERRY_STATUS_RESPONSE_READY) {
          ack_received = true;
          _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_IDLE;
        } else {
          continue; // no ACK, retry next attempt
        }
      }

      // Message was handled, remove it
      if(queue) {
        xQueueReceive(_bluecherryIOMessageQueue.handle, &_blueCherry.message_processing, 0);
      }

      // Check if there are more messages
      queue = (xQueuePeek(_bluecherryIOMessageQueue.handle, &_blueCherry.message_processing, 0) ==
               pdPASS);
      force_sync = false;

      // If queue is now empty and we got at least one ACK → done
      if(!queue && ack_received) {
        _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_IDLE;
        return true;
      }
    }
  }

  _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_TIMED_OUT;
  return false;
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
  if(_blueCherry.initialized) {
    // Already initialized
    return true;
  }

  if((!blueCherryIsProvisioned() ||
      !tlsConfigProfile(tls_profile_id, WALTER_MODEM_TLS_VALIDATION_URL_AND_CA,
                        WALTER_MODEM_TLS_VERSION_12, 6, 5, 0))) {
    _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_NOT_PROVISIONED;

    if(rsp) {
      rsp->type = WALTER_MODEM_RSP_DATA_TYPE_BLUECHERRY;
      rsp->data.blueCherry.state = WALTER_MODEM_BLUECHERRY_STATUS_NOT_PROVISIONED;
      rsp->data.blueCherry.messageCount = 0;
    }

    return false;
  }

  _blueCherry.tls_profile_id = tls_profile_id;
  _blueCherry.ack_timeout_s = ack_timeout_s;
  _blueCherry.auto_sync_enabled = false;
  _blueCherry.initialized = true;

  /* Get OTA partition information */
  esp_app_desc_t app_desc;
  esp_ota_handle_t handle;
  const esp_partition_t* part = esp_ota_get_running_partition();
  esp_err_t err;
  uint8_t sha_256[33];
  sha_256[0] = WALTER_MODEM_BLUECHERRY_EVENT_TYPE_PARTITION_HASH;
  esp_partition_get_sha256(part, sha_256 + 1);

  /* Publish partition hash to BlueCherry cloud to verify the running firmware version */
  blueCherryPublish(0, 33, sha_256);

  if(_blueCherrySocketConnect()) {
    _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_IDLE;
    return true;
  } else {
    _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_NOT_CONNECTED;
    return false;
  }
}

WalterModemBlueCherryInitStatus
WalterModem::blueCherryInit(bluecherry_msg_handler_t msg_handler, void* msg_handler_args,
                            bool auto_sync_enable, int tls_profile_id, int socket_profile_id)
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

  if(auto_sync_enable) {
    _bc_sync_task_handle = xTaskCreateStaticPinnedToCore(
        _blueCherrySyncTask, "bc_sync", 4096, NULL, 10, _bc_sync_task_stack, &_bc_sync_task_buf, 0);
    if(!_bc_sync_task_handle) {
      ESP_LOGW("WalterModem", "Failed to create BlueCherry sync task (Not enough memory?)");
      return WALTER_MODEM_BLUECHERRY_INIT_STATUS_ERROR;
    }
  }

  /* Get OTA partition information */
  const esp_partition_t* part = esp_ota_get_running_partition();
  uint8_t sha_256[33] = WALTER_MODEM_BLUECHERRY_EVENT_TYPE_PARTITION_HASH;
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

uint8_t
WalterModem::_blueCherryGetCoapHeaderLength(const walter_modem_bluecherry_coap_message_t& message)
{
  // TODO: Extract header length of coap structure message
  return WALTER_MODEM_BLUECHERRY_COAP_HEADER_SIZE;
}

bool WalterModem::blueCherryPublish(uint8_t topic, uint8_t len, uint8_t* data, bool urgent)
{
  if(_blueCherry.message_out.data_len + len +
         _blueCherryGetCoapHeaderLength(_blueCherry.message_out) >=
     WALTER_MODEM_MAX_OUTGOING_MESSAGE_LEN) {
    if(xQueueSend(_bluecherryIOMessageQueue.handle, &_blueCherry.message_out, 0) != pdPASS) {
      ESP_LOGW("WalterModem", "BlueCherry IO queue full, cannot enqueue message.");
      return false;
    }
    _blueCherry.message_out.data_len = _blueCherryGetCoapHeaderLength(_blueCherry.message_out);
  }

  _blueCherry.message_out.data[_blueCherry.message_out.data_len] = topic;
  _blueCherry.message_out.data[_blueCherry.message_out.data_len + 1] = len;
  memcpy(_blueCherry.message_out.data + _blueCherry.message_out.data_len + 2, data, len);
  _blueCherry.message_out.data_len += len + 2;

  if(urgent) {
    if(xQueueSend(_bluecherryIOMessageQueue.handle, &_blueCherry.message_out, 0) != pdPASS) {
      ESP_LOGW("WalterModem", "BlueCherry IO queue full, cannot enqueue message.");
      return false;
    }
    _blueCherry.message_out.data_len = _blueCherryGetCoapHeaderLength(_blueCherry.message_out);
  }
  return true;
}

bool WalterModem::blueCherrySync(walter_modem_rsp_t* rsp)
{
  ESP_LOGW("DEPRECATION",
           "Please use blueCherrySync(bool force_sync, walter_modem_rsp_t* rsp) instead.");

  if(_blueCherry.auto_sync_enabled) {
    ESP_LOGW("WalterModem",
             "blueCherrySync called while auto sync is enabled. Ignoring manual sync request.");
    return false;
  }

  if(_blueCherrySocketConnect()) {
    if(_blueCherryCoapSend()) {
      // Reset Publish Buffer if successfull
      _blueCherry.message_out.data_len = _blueCherryGetCoapHeaderLength(_blueCherry.message_out);
    }
  } else {
    _blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_NOT_CONNECTED;
  }

  uint16_t payloadOffset = WALTER_MODEM_BLUECHERRY_COAP_HEADER_SIZE;
  while(payloadOffset < _blueCherry.messageInLen) {
    uint8_t topic = _blueCherry.messageIn[payloadOffset++];
    uint8_t dataLen = _blueCherry.messageIn[payloadOffset++];

    /*
     * Topic 0 is reserved for BlueCherry events, which are also visible to walter as mqtt
     * messages on topic id 0
     */
    if(topic == 0) {
      _blueCherry.moreDataAvailable = true;

      if(_blueCherryProcessEvent(_blueCherry.messageIn + payloadOffset, dataLen)) {
        _blueCherry.emitErrorEvent = true;
        _blueCherry.otaSize = 0;
      }
    }

    WalterModemBlueCherryMessage* msg =
        rsp->data.blueCherry.messages + rsp->data.blueCherry.messageCount;
    msg->topic = topic;
    msg->dataSize = dataLen;
    msg->data = _blueCherry.messageIn + payloadOffset;

    rsp->data.blueCherry.messageCount++;

    payloadOffset += dataLen;
  }

  // Mark the sync as finished whether we were successfull or not. This prevents an endless
  // loop and offloads any connectivity issues to the application.
  rsp->data.blueCherry.syncFinished = true;

  _blueCherry.lastTransmittedMsgID++;
  if(_blueCherry.lastTransmittedMsgID == 0) {
    /* on wrap around, skip msg id 0 which we use as a special/error value */
    _blueCherry.lastTransmittedMsgID++;
  }

  if(_blueCherry.emitErrorEvent) {
    uint8_t blueCherryErrorEventCode = WALTER_MODEM_BLUECHERRY_EVENT_TYPE_OTA_ERROR;
    blueCherryPublish(0, 1, &blueCherryErrorEventCode);
  }

  if(_blueCherry.status == WALTER_MODEM_BLUECHERRY_STATUS_TIMED_OUT) {
    rsp->data.blueCherry.state = WALTER_MODEM_BLUECHERRY_STATUS_TIMED_OUT;
    _returnState(WALTER_MODEM_STATE_ERROR)
  }

  if(_blueCherry.status == WALTER_MODEM_BLUECHERRY_STATUS_NOT_CONNECTED) {
    rsp->data.blueCherry.state = WALTER_MODEM_BLUECHERRY_STATUS_NOT_CONNECTED;
    _returnState(WALTER_MODEM_STATE_ERROR)
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

  _returnState(WALTER_MODEM_STATE_OK);
}

bool WalterModem::blueCherrySync(bool force_sync, walter_modem_rsp_t* rsp)
{
  if(_blueCherry.auto_sync_enabled) {
    ESP_LOGW("WalterModem",
             "blueCherrySync called while auto sync is enabled. Ignoring manual sync request.");
    _returnState(WALTER_MODEM_STATE_ERROR)
  }

  if(_blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_IDLE &&
     _blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_PENDING_MESSAGES &&
     _blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_TIMED_OUT &&
     _blueCherry.status != WALTER_MODEM_BLUECHERRY_STATUS_NOT_CONNECTED) {
    _returnState(WALTER_MODEM_STATE_BUSY)
  }

  rsp->type = WALTER_MODEM_RSP_DATA_TYPE_BLUECHERRY;
  rsp->data.blueCherry.messageCount = 0;

  return _bluecherrySync(force_sync);
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