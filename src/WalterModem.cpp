/**
 * @file WalterModem.cpp
 * @author Daan Pape <daan@dptechnics.com>
 * @author Arnoud Devoogdt <arnoud@dptechnics.com>
 * @date 21 September 2026
 * @version 1.5.1
 * @copyright DPTechnics bv <info@dptechnics.com>
 * @brief Walter Modem library
 *
 * @section LICENSE
 *
 * Copyright (C) 2026, DPTechnics bv
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
 * This file contains Walter's modem library implementation.
 */

#include <WalterDefines.h>
#if CONFIG_BLUECHERRY_ENABLE
#include <WalterBlueCherry.h>
#endif

#ifdef ARDUINO

#include <Arduino.h>

#endif

#include <cstdarg>
#include <esp_log.h>
#include <esp_sleep.h>
#include <esp_system.h>
#include <esp_timer.h>

#if CONFIG_WALTER_MODEM_ENABLE_MOTA || CONFIG_BLUECHERRY_ENABLE

#include <esp_ota_ops.h>

#endif

#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_task_wdt.h>

#if CONFIG_WALTER_MODEM_ENABLE_MOTA || CONFIG_BLUECHERRY_ENABLE

#include <esp_image_format.h>
#include <esp_partition.h>

#endif
#pragma region CONFIG

/**
 * @brief The RX pin on which modem data is received.
 */
CONFIG_INT(WALTER_MODEM_PIN_RX, 14)

/**
 * @brief The TX to which modem data must be transmitted.
 */
CONFIG_INT(WALTER_MODEM_PIN_TX, 48)

/**
 * @brief The RTS pin on the ESP32 side.
 */
CONFIG_INT(WALTER_MODEM_PIN_RTS, 21)

/**
 * @brief The CTS pin on the ESP32 size.
 */
CONFIG_INT(WALTER_MODEM_PIN_CTS, 47)

/**
 * @brief The active low modem reset pin.
 */
CONFIG_INT(WALTER_MODEM_PIN_RESET, 45)

/**
 * @brief The baud rate used to talk to the modem.
 */
CONFIG_INT(WALTER_MODEM_BAUD, 115200)

/**
 * @brief The maximum duration of an event in milliseconds.
 */
CONFIG_INT(WALTER_MODEM_MAX_EVENT_DURATION_MS, 1500)

/**
 * @brief UART buffer size.
 * The size of the UART hardware FIFO buffer.
 */
CONFIG_INT(UART_BUF_SIZE, 128)

/**
 * @brief UART buffer threshold.
 * When the hardware UART buffer reaches a certain threshold,
 * an interrupt will get triggered.
 * We had issues with a lower treshold, which would seem to indicate that we're not processing
 * something fast enough, in order to deal with the more frequent interrupts.
 * We need to test this hypothesis.
 */
CONFIG_INT(UART_BUF_THRESHOLD, 122)

#pragma endregion

/**
 * @brief Any modem time below 1 Jan 2023 00:00:00 UTC is considered an invalid time.
 */
#define WALTER_MODEM_MIN_VALID_TIMESTAMP 1672531200

/**
 * @brief The command timeout expressed in system ticks.
 */
#define WALTER_MODEM_CMD_TIMEOUT_TICKS pdMS_TO_TICKS(WALTER_MODEM_CMD_TIMEOUT_MS)

#pragma region RTC_MEMORY
#if CONFIG_WALTER_MODEM_ENABLE_MOTA

struct WalterModemStpRequest stpRequest;
struct WalterModemStpResponseSessionOpen stpResponseSessionOpen;
struct WalterModemStpRequestTransferBlockCmd stpRequestTransferBlockCmd;
struct WalterModemStpResponseTransferBlock stpResponseTransferBlock;

#endif

RTC_DATA_ATTR walter_modem_pdp_context_t _pdpCtxSetRTC[WALTER_MODEM_MAX_PDP_CTXTS] = {};

#if CONFIG_WALTER_MODEM_ENABLE_COAP

RTC_DATA_ATTR WalterModemCoapContext _coapCtxSetRTC[WALTER_MODEM_MAX_COAP_PROFILES] = {};

#endif
#if CONFIG_WALTER_MODEM_ENABLE_MQTT

RTC_DATA_ATTR WalterModemMqttTopic _mqttTopicSetRTC[WALTER_MODEM_MQTT_MAX_TOPICS] = {};

#endif
#if CONFIG_WALTER_MODEM_ENABLE_SOCKETS

RTC_DATA_ATTR WalterModemSocket _socketCtxSetRTC[WALTER_MODEM_MAX_SOCKETS] = {};

#endif
#pragma endregion
#if CONFIG_LOG_MAXIMUM_LEVEL >= ESP_LOG_DEBUG

/**
 * @brief Static buffer for escaping data for logging purposes.
 * Size is twice WALTER_MODEM_AT_BUFFER_SIZE to handle worst-case escaping.
 */
static uint8_t _logEscapeBuffer[WALTER_MODEM_AT_BUFFER_SIZE * 2];

#endif
#pragma region HELPER_FUNCTIONS

/**
 * @brief Convert a digit to a string literal.
 *
 * This function converts a digit [0,9] into a string literal so that it can be passed as an AT
 * command element.
 *
 * @param val The value to convert.
 *
 * @return The resulting string literal or an empty string when the value is not in the [0,9] range.
 */
const char* _digitStr(int val)
{
  switch(val) {
  case 0:
    return "0";

  case 1:
    return "1";

  case 2:
    return "2";

  case 3:
    return "3";

  case 4:
    return "4";

  case 5:
    return "5";

  case 6:
    return "6";

  case 7:
    return "7";

  case 8:
    return "8";

  case 9:
    return "9";
  }

  return "";
}

/**
 * @brief Convert a digit at a certain position in a number to a string literal.
 *
 * This function will convert the digit from a certain integer into a string literal. The position
 * starts at 0 which is the leftmost digit of the number. When position is bigger than the number of
 * digits in the number an empty string literal is returned.
 *
 * @param num The number to get the digit from.
 * @param pos The position of the digit, 0 is the leftmost digit.
 *
 * @return The string literal which represents the digit.
 */
const char* _intToStrDigit(int num, int pos)
{
  pos += 1;
  int pow = 1;
  int tmp = num;
  int digitCount = 1;

  while(tmp >= 10) {
    pow *= 10;
    tmp /= 10;
    digitCount += 1;
  }

  if(pos > digitCount) {
    return "";
  }

  tmp = num;
  int digit = 0;

  for(int i = 1; i <= pos; ++i) {
    digit = tmp / pow;
    tmp %= pow;
    pow /= 10;
  }

  return _digitStr(digit);
}

/**
 * @brief Convert a PDP type to a string
 *
 * @param type The PDP type to convert.
 *
 * @return The resulting string literal.
 */
const char* _pdpTypeStr(WalterModemPDPType type)
{
  switch(type) {
  case WALTER_MODEM_PDP_TYPE_X25:
    return "X.25";

  case WALTER_MODEM_PDP_TYPE_IP:
    return "IP";

  case WALTER_MODEM_PDP_TYPE_IPV6:
    return "IPV6";

  case WALTER_MODEM_PDP_TYPE_IPV4V6:
    return "IPV4V6";

  case WALTER_MODEM_PDP_TYPE_OSPIH:
    return "OSPIH";

  case WALTER_MODEM_PDP_TYPE_PPP:
    return "PPP";

  case WALTER_MODEM_PDP_TYPE_NON_IP:
    return "Non-IP";
  }

  return "";
}

/**
 * @brief Convert a broken-down UTC time to a unix timestamp.
 *
 * Portable replacement for the non-standard timegm(). Unlike mktime(), this is unaffected by the
 * TZ environment variable: the struct tm fields are always interpreted as UTC.
 *
 * Uses Howard Hinnant's days_from_civil algorithm; tm_wday, tm_yday and tm_isdst are ignored.
 *
 * @param tm The broken-down time, interpreted as UTC.
 *
 * @return The unix timestamp.
 */
static int64_t _timegm(const struct tm* tm)
{
  int y = tm->tm_year + 1900;
  int m = tm->tm_mon + 1;
  int d = tm->tm_mday;
  if(m <= 2) {
    y -= 1;
  }
  const int era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = (unsigned) (y - era * 400);
  const unsigned doy = (153u * (m + (m > 2 ? -3 : 9)) + 2u) / 5u + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  const int64_t days = (int64_t) era * 146097 + (int64_t) doe - 719468;
  return days * 86400 + tm->tm_hour * 3600 + tm->tm_min * 60 + tm->tm_sec;
}

/**
 * @brief Convert a time string to a unix timestamp.
 *
 * No time zone is taken into consideration. If the time string is a local time, the time zone
 * offset must be compensated afterwards.
 *
 * @param timeStr The string to convert.
 * @param format The time format to parse.
 *
 * @return The unix timestamp or -1 on error.
 */
int64_t strTotime(const char* timeStr, const char* format)
{
  struct tm tm {};
  if(strptime(timeStr, format, &tm) == NULL) {
    return -1;
  }

  return _timegm(&tm);
}

/**
 * @brief Convert a unix timestamp to a formatted time string.
 *
 * The time string is in UTC (no time zone offset applied).
 *
 * @param timestamp The unix timestamp to convert.
 * @param buffer The output buffer to hold the formatted string.
 * @param buffer_len The length of the output buffer.
 * @param format The time format to use (default: "%Y-%m-%dT%H:%M:%S").
 *
 * @return true on success, false on error.
 */
bool timeToStr(uint64_t timestamp, char* buffer, size_t buffer_len, const char* format)
{
  if(buffer == NULL || buffer_len == 0) {
    return false;
  }

  // Use gmtime_r to convert timestamp to UTC broken-down time
  time_t time_val = (time_t) timestamp;
  struct tm tm_utc;

  if(gmtime_r(&time_val, &tm_utc) == NULL) {
    return false;
  }

  // Format time according to format string
  size_t written = strftime(buffer, buffer_len, format, &tm_utc);
  if(written == 0) {
    // Buffer too small or formatting error
    return false;
  }

  return true;
}

/**
 * @brief Convert a string into an unsigned 32-bit integer.
 *
 * This function will convert a string into a 32-bit unsigned integer.
 *
 * @param str The 0-terminated string to convert.
 * @param len The length of the string or -1 to use strlen internally.
 * @param result Pointer to save the result in.
 * @param radix The radix to convert with, 10 by default.
 * @param max The function will fail if the number is bigger than max, UINT32_MAX by default.
 *
 * @return True when the conversion was successful, false if the conversion failed.
 */
bool strToUint32(const char* str, int len, uint32_t* result, int radix, uint32_t max)
{
  size_t l = len == -1 ? strlen(str) : (size_t) len;

  /* Create a temp stack buffer to make the string 0-terminated */
  char buff[l + 1];
  memcpy(buff, str, l);
  buff[l] = '\0';

  char* end;
  errno = 0;
  long long int sl = strtol(buff, &end, radix);

  if(end == buff) {
    return false;
  } else if('\0' != *end) {
    return false;
  } else if(errno == ERANGE) {
    return false;
  } else if(sl > max) {
    return false;
  } else if(sl < 0) {
    return false;
  } else {
    *result = (uint32_t) sl;
    return true;
  }

  return false;
}

/**
 * @brief Convert a string into an unsigned 16-bit integer.
 *
 * This function will convert a string into a 16-bit unsigned integer.
 *
 * @param str The 0-terminated string to convert.
 * @param len The length of the string or -1 to use strlen internally.
 * @param result Pointer to save the result in.
 * @param radix The radix to convert with, 10 by default.
 *
 * @return True when the conversion was successful, false if the conversion failed.
 */
bool strToUint16(const char* str, int len, uint16_t* result, int radix)
{
  uint32_t tmpResult = 0;

  if(!strToUint32(str, len, &tmpResult, radix, UINT16_MAX)) {
    return false;
  }

  *result = (uint16_t) tmpResult;
  return true;
}

/**
 * @brief Convert a string into an unsigned 8-bit integer.
 *
 * This function will convert a string into a 8-bit unsigned integer.
 *
 * @param str The 0-terminated string to convert.
 * @param len The length of the string or -1 to use strlen internally.
 * @param result Pointer to save the result in.
 * @param radix The radix to convert with, 10 by default.
 *
 * @return True when the conversion was successful, false if the conversion failed.
 */
bool strToUint8(const char* str, int len, uint8_t* result, int radix)
{
  uint32_t tmpResult = 0;

  if(!strToUint32(str, len, &tmpResult, radix, UINT8_MAX)) {
    return false;
  }

  *result = (uint8_t) tmpResult;
  return true;
}

/**
 * @brief Convert a string into an IEEE754 float.
 *
 * This function will convert a string into an IEEE754 float.
 *
 * @param str The 0-terminated string to convert.
 * @param len The length of the string or -1 to use strlen internally.
 * @param result Pointer to save the result in.
 *
 * @return True when the conversion was successful, false if the conversion failed.
 */
bool strToFloat(const char* str, int len, float* result)
{
  size_t l = len == -1 ? strlen(str) : (size_t) len;

  /* Create a temp buffer to make the string 0-terminated */
  char buff[l + 1];
  memcpy(buff, str, l);
  buff[l] = '\0';

  char* end;
  errno = 0;
  *result = strtof(buff, &end);

  if(end == buff) {
    return false;
  } else if('\0' != *end) {
    return false;
  } else if(errno == ERANGE) {
    return false;
  }

  return true;
}

/**
 * @brief Escape a binary buffer into a printable format.
 *
 * This function will escape non-printable characters in the input buffer
 * using C-style escape sequences.
 *
 * @param input The input binary buffer.
 * @param input_len The length of the input buffer.
 * @param max_len The maximum length of the output buffer. (Output buffer can be up to twice this
 * size)
 * @param static_out Optional pre-allocated output buffer to use instead of malloc.
 * @param static_out_size Size of the static output buffer if provided.
 *
 * @return A pointer to a Buffer struct containing the escaped data and its size, or NULL on error.
 */
Buffer* escapeBuffer(const uint8_t* input, size_t input_len, size_t max_len,
                     uint8_t* static_out = nullptr, size_t static_out_size = 0)
{
  if(!input || input_len == 0)
    return NULL;

  // Use static buffer if provided, otherwise allocate
  uint8_t* out = nullptr;
  bool use_static = (static_out != nullptr && static_out_size > 0);

  if(use_static) {
    out = static_out;
  } else {
    // Worst-case: every byte becomes two chars
    out = static_cast<uint8_t*>(malloc(max_len * 2));
    if(!out)
      return nullptr;
  }

  size_t j = 0;
  size_t max_out = use_static ? static_out_size : (max_len * 2);

  for(size_t i = 0; i < input_len && j < max_out - 5; i++) {
    uint8_t c = input[i];
    switch(c) {
    case '\n':
      out[j++] = '\\';
      out[j++] = 'n';
      break;
    case '\r':
      out[j++] = '\\';
      out[j++] = 'r';
      break;
    case '\t':
      out[j++] = '\\';
      out[j++] = 't';
      break;
    case '\0':
      out[j++] = '\\';
      out[j++] = '0';
      break;
    default:
      if(isprint(c)) {
        out[j++] = c;
      } else {
        // Fallback: hex escape for other non-printable chars
        int n = snprintf((char*) &out[j], 5, "\\x%02X", c);
        j += n;
      }
      break;
    }
  }

  if(use_static) {
    // For static buffer, use a static Buffer struct
    static Buffer buff;
    buff.data = out;
    buff.size = j;
    return &buff;
  } else {
    // For dynamic allocation, allocate Buffer struct too
    Buffer* buff = static_cast<Buffer*>(malloc(sizeof(Buffer)));
    if(!buff) {
      free(out);
      return NULL;
    }
    buff->data = out;
    buff->size = j;
    return buff;
  }
}

/**
 * @brief Check whether a command starts with the given AT command prefix.
 *
 * @param cmd The command, may be NULL.
 * @param prefix The AT command prefix, e.g. "AT+SQNSRECV=".
 *
 * @return True when cmd is not NULL and starts with prefix.
 */
static bool cmdHasPrefix(const WalterModemCmd* cmd, const char* prefix)
{
  return cmd != NULL && cmd->atCmd[0] != NULL &&
         strncmp(cmd->atCmd[0], prefix, strlen(prefix)) == 0;
}

#if CONFIG_LOG_MAXIMUM_LEVEL >= ESP_LOG_DEBUG

/**
 * @brief Check whether payloads are logged in full (VERBOSE) instead of as a byte count (DEBUG).
 *
 * @return True when the WalterModem log level is VERBOSE.
 */
static bool logPayloadsVerbose()
{
#ifdef ARDUINO
  return ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_VERBOSE;
#else
  return esp_log_level_get("WalterModem") >= ESP_LOG_VERBOSE;
#endif
}

/**
 * @brief Escape a binary buffer into a caller provided output buffer.
 *
 * @return The number of characters written.
 */
static size_t escapeInto(uint8_t* out, size_t outSize, const uint8_t* in, size_t inLen)
{
  /* escapeBuffer needs room for at least one escape sequence */
  if(outSize <= 5) {
    return 0;
  }

  Buffer* escaped = escapeBuffer(in, inLen, 0, out, outSize);
  return escaped ? escaped->size : 0;
}

/**
 * @brief Log AT traffic that carried a payload.
 *
 * The payload is spliced into the escaped line at offset at: in full when the log level is
 * VERBOSE, as a byte count otherwise.
 *
 * @param rx True for received data, false for transmitted data.
 * @param line The AT bytes around the payload, may be NULL.
 * @param lineLen The number of AT bytes.
 * @param at The offset in line at which the payload sits.
 * @param payload The stored payload bytes, may be NULL.
 * @param payloadLen The number of stored payload bytes.
 * @param dropped The number of payload bytes that were not stored.
 */
static void logWithPayload(bool rx, const uint8_t* line, size_t lineLen, size_t at,
                           const uint8_t* payload, size_t payloadLen, size_t dropped)
{
  uint8_t* out = _logEscapeBuffer;
  size_t cap = sizeof(_logEscapeBuffer);
  bool verbose = payload != NULL && payloadLen > 0 && logPayloadsVerbose();

  if(at > lineLen) {
    at = lineLen;
  }

  if(verbose) {
    /* The payload can be far larger than the static log buffer, a byte escapes to 4 chars max */
    size_t heapCap = (lineLen + payloadLen) * 4 + 48;
    uint8_t* heap = (uint8_t*) malloc(heapCap);
    if(heap != NULL) {
      out = heap;
      cap = heapCap;
    } else {
      verbose = false;
    }
  }

  size_t n = escapeInto(out, cap, line, at);

  int m = 0;
  if(verbose) {
    n += escapeInto(out + n, cap - n, payload, payloadLen);
    if(dropped > 0) {
      m = snprintf((char*) out + n, cap - n, "...<%u bytes dropped>...", (unsigned) dropped);
    }
  } else if(dropped > 0) {
    m = snprintf((char*) out + n, cap - n, "...<%u bytes %s, %u dropped>...",
                 (unsigned) (payloadLen + dropped), rx ? "received" : "sent", (unsigned) dropped);
  } else {
    m = snprintf((char*) out + n, cap - n, "...<%u bytes %s>...", (unsigned) payloadLen,
                 rx ? "received" : "sent");
  }

  if(m > 0 && n < cap) {
    n += (size_t) m < cap - n ? (size_t) m : cap - n - 1;
  }

  if(lineLen > at) {
    n += escapeInto(out + n, cap - n, line + at, lineLen - at);
  }

  ESP_LOGD("WalterModem", "%s: %.*s", rx ? "RX" : "TX", (int) n, out);

  if(out != _logEscapeBuffer) {
    free(out);
  }
}

#endif
#pragma endregion
#pragma region PRIVATE_METHODS
#pragma region MODEM_UPGRADE
#if CONFIG_WALTER_MODEM_ENABLE_MOTA

uint16_t WalterModem::_modemFirmwareUpgradeStart(void)
{
  char* atCmd[WALTER_MODEM_COMMAND_MAX_ELEMS + 1] = { NULL };
  int len;

  /* Scratch for the AT replies below. The largest read is 64 bytes plus the NUL. */
  uint8_t rsp[65];

  _rxHandlerInterrupted = true;

  /* reboot to recovery */
  vTaskDelay(pdMS_TO_TICKS(5000));
  tickleWatchdog();
  atCmd[0] = (char*) "AT+SMSWBOOT=3,1";
  atCmd[1] = NULL;
  _transmitCmd(WALTER_MODEM_CMD_TYPE_TX, atCmd);
  ESP_LOGD("WalterModem", "sent reboot to recovery command, waiting 10 seconds");
  vTaskDelay(pdMS_TO_TICKS(10000));
  tickleWatchdog();

  /* check if booted in recovery mode */
  atCmd[0] = (char*) "AT";
  atCmd[1] = NULL;
  _transmitCmd(WALTER_MODEM_CMD_TYPE_TX, atCmd);

  len = _uartRead(rsp, 6);
  rsp[len] = 0;
  ESP_LOGD("WalterModem", "sent AT, got %d:%s", len, rsp);

  atCmd[0] = (char*) "AT+SMLOG?";
  atCmd[1] = NULL;
  _transmitCmd(WALTER_MODEM_CMD_TYPE_TX, atCmd);

  len = _uartRead(rsp, 25);
  rsp[len] = 0;
  ESP_LOGD("WalterModem", "sent AT+SMLOG?, got %d:%s", len, rsp);

  atCmd[0] = (char*) "AT+SMOD?";
  atCmd[1] = NULL;
  _transmitCmd(WALTER_MODEM_CMD_TYPE_TX, atCmd);

  len = _uartRead(rsp, 7);
  rsp[len] = 0;
  ESP_LOGD("WalterModem", "sent AT+SMOD?, got %d:%s", len, rsp);

  /* prepare modem firmware data transfer - must wait for OK still!! */
  atCmd[0] = (char*) "AT+SMSTPU=\"ON_THE_FLY\"";
  atCmd[1] = NULL;
  _transmitCmd(WALTER_MODEM_CMD_TYPE_TX, atCmd);

  vTaskDelay(pdMS_TO_TICKS(2000));
  len = _uartRead(rsp, 64);
  rsp[len] = 0;
  ESP_LOGD("WalterModem", "started STP mode, got %d:%s", len, rsp);

  size_t bytesSent, bytesReceived;

  stpRequest.signature = _switchEndian32(WALTER_MODEM_STP_SIGNATURE_REQUEST);
  stpRequest.operation = WALTER_MODEM_STP_OPERATION_RESET;
  stpRequest.sessionId = 0;
  stpRequest.payloadLength = 0;
  stpRequest.transactionId = 0;
  stpRequest.headerCrc16 = 0;
  stpRequest.payloadCrc16 = 0;
  stpRequest.headerCrc16 = _calculateStpCrc16(&stpRequest, sizeof(stpRequest));

  bytesSent = _uartWrite((uint8_t*) &stpRequest, sizeof(stpRequest));

  ESP_LOGD("WalterModem",
           "sent STP reset: tx=%d header: 0x%" PRIx32 " 0x%x 0x%x %d %" PRIu32 " 0x%x 0x%x",
           bytesSent, _switchEndian32(stpRequest.signature), stpRequest.operation,
           stpRequest.sessionId, _switchEndian16(stpRequest.payloadLength),
           _switchEndian32(stpRequest.transactionId), _switchEndian16(stpRequest.headerCrc16),
           _switchEndian16(stpRequest.payloadCrc16));

  bytesReceived = _uartRead((uint8_t*) &stpRequest, sizeof(stpRequest), true);

  ESP_LOGD("WalterModem",
           "received STP reset ack: rx=%d header: 0x%" PRIx32 " 0x%x 0x%x %d %" PRIu32 " 0x%x 0x%x",
           bytesReceived, _switchEndian32(stpRequest.signature), stpRequest.operation,
           stpRequest.sessionId, _switchEndian16(stpRequest.payloadLength),
           _switchEndian32(stpRequest.transactionId), _switchEndian16(stpRequest.headerCrc16),
           _switchEndian16(stpRequest.payloadCrc16));

  stpRequest.signature = _switchEndian32(WALTER_MODEM_STP_SIGNATURE_REQUEST);
  stpRequest.operation = WALTER_MODEM_STP_OPERATION_OPEN_SESSION;
  stpRequest.sessionId = 1;
  stpRequest.payloadLength = 0;
  stpRequest.transactionId = _switchEndian32(1);
  stpRequest.headerCrc16 = 0;
  stpRequest.payloadCrc16 = 0;
  stpRequest.headerCrc16 = _calculateStpCrc16(&stpRequest, sizeof(stpRequest));

  bytesSent = _uartWrite((uint8_t*) &stpRequest, sizeof(stpRequest));

  ESP_LOGD("WalterModem",
           "sent STP open session: tx=%d header: 0x%" PRIx32 " 0x%x 0x%x %d %" PRIu32 " 0x%x 0x%x",
           bytesSent, _switchEndian32(stpRequest.signature), stpRequest.operation,
           stpRequest.sessionId, _switchEndian16(stpRequest.payloadLength),
           _switchEndian32(stpRequest.transactionId), _switchEndian16(stpRequest.headerCrc16),
           _switchEndian16(stpRequest.payloadCrc16));

  bytesReceived = _uartRead((uint8_t*) &stpRequest, sizeof(stpRequest), true);

  ESP_LOGD("WalterModem",
           "received STP open session ack: rx=%d header: 0x%" PRIx32 " 0x%x 0x%x %d %" PRIu32
           " 0x%x 0x%x",
           bytesReceived, _switchEndian32(stpRequest.signature), stpRequest.operation,
           stpRequest.sessionId, _switchEndian16(stpRequest.payloadLength),
           _switchEndian32(stpRequest.transactionId), _switchEndian16(stpRequest.headerCrc16),
           _switchEndian16(stpRequest.payloadCrc16));

  bytesReceived =
      _uartRead((uint8_t*) &stpResponseSessionOpen, sizeof(stpResponseSessionOpen), true);

  ESP_LOGD("WalterModem", "received STP open session ack data: rx=%d payload: %d %d %d",
           bytesReceived, stpResponseSessionOpen.success, stpResponseSessionOpen.version,
           _switchEndian16(stpResponseSessionOpen.maxTransferSize));

  tickleWatchdog();

  /* A session that never opened leaves maxTransferSize at 0, and 0 - sizeof(stpRequest) wraps to
   * 65520 rather than to something obviously wrong. Report 0 so the caller can tell the two apart.
   */
  uint16_t maxTransferSize = _switchEndian16(stpResponseSessionOpen.maxTransferSize);
  if(maxTransferSize <= sizeof(stpRequest)) {
    return 0;
  }

  return maxTransferSize - sizeof(stpRequest);
}

void WalterModem::_modemFirmwareUpgradeFinish(bool success)
{
  char* atCmd[WALTER_MODEM_COMMAND_MAX_ELEMS + 1] = { NULL };
  int len;

  /* Scratch for the AT replies below. The largest read is 64 bytes plus the NUL. */
  uint8_t rsp[65];

  if(!success) {
    _rxHandlerInterrupted = false;
    reset();
    return;
  }

  /* send final STP reset command after transfer */
  size_t bytesSent, bytesReceived;

  stpRequest.signature = _switchEndian32(WALTER_MODEM_STP_SIGNATURE_REQUEST);
  stpRequest.operation = WALTER_MODEM_STP_OPERATION_RESET;
  stpRequest.sessionId = 0;
  stpRequest.payloadLength = 0;
  stpRequest.transactionId = 0;
  stpRequest.headerCrc16 = 0;
  stpRequest.payloadCrc16 = 0;
  stpRequest.headerCrc16 = _calculateStpCrc16(&stpRequest, sizeof(stpRequest));

  bytesSent = _uartWrite((uint8_t*) &stpRequest, sizeof(stpRequest));

  ESP_LOGD("WalterModem",
           "sent STP reset: tx=%d header: 0x%" PRIx32 " 0x%x 0x%x %d %" PRIu32 " 0x%x 0x%x",
           bytesSent, _switchEndian32(stpRequest.signature), stpRequest.operation,
           stpRequest.sessionId, _switchEndian16(stpRequest.payloadLength),
           _switchEndian32(stpRequest.transactionId), _switchEndian16(stpRequest.headerCrc16),
           _switchEndian16(stpRequest.payloadCrc16));

  bytesReceived = _uartRead((uint8_t*) &stpRequest, sizeof(stpRequest), true);

  ESP_LOGD("WalterModem",
           "received STP reset ack: rx=%d header: 0x%" PRIx32 " 0x%x 0x%x %d %" PRIu32 " 0x%x 0x%x",
           bytesReceived, _switchEndian32(stpRequest.signature), stpRequest.operation,
           stpRequest.sessionId, _switchEndian16(stpRequest.payloadLength),
           _switchEndian32(stpRequest.transactionId), _switchEndian16(stpRequest.headerCrc16),
           _switchEndian16(stpRequest.payloadCrc16));

  /* send AT and retry until we get OK */
  for(;;) {
    vTaskDelay(pdMS_TO_TICKS(5000));

    atCmd[0] = (char*) "AT";
    atCmd[1] = NULL;
    _transmitCmd(WALTER_MODEM_CMD_TYPE_TX, atCmd);

    len = _uartRead(rsp, 32);
    rsp[len] = 0;
    ESP_LOGD("WalterModem", "sent AT, got %d:%s", len, rsp);

    if(!strcmp((char*) rsp, "\r\nOK\r\n")) {
      break;
    }
  }

  /* we got OK so ready to boot into new firmware; switch back to FFF mode */
  atCmd[0] = (char*) "AT+SMSWBOOT=1,0";
  atCmd[1] = NULL;
  _transmitCmd(WALTER_MODEM_CMD_TYPE_TX, atCmd);

  len = _uartRead(rsp, 16);
  rsp[len] = 0;
  ESP_LOGD("WalterModem", "switched modem to FFF mode, got %d:%s", len, rsp);

  /* now reboot into new firmware */
  atCmd[0] = (char*) "AT^RESET";
  atCmd[1] = NULL;
  _transmitCmd(WALTER_MODEM_CMD_TYPE_TX, atCmd);
  ESP_LOGD("WalterModem", "sent reset command, waiting 10 seconds");
  vTaskDelay(pdMS_TO_TICKS(10000));

  len = _uartRead(rsp, 64);
  rsp[len] = 0;
  ESP_LOGD("WalterModem", "assuming modem boot complete; got %d:%s", len, rsp);

  /* check if we are back in fff mode and check update status */
  atCmd[0] = (char*) "AT+SMLOG?";
  atCmd[1] = NULL;
  _transmitCmd(WALTER_MODEM_CMD_TYPE_TX, atCmd);

  len = _uartRead(rsp, 64);
  rsp[len] = 0;
  ESP_LOGD("WalterModem", "AT+SMLOG? got %d:%s", len, rsp);

  atCmd[0] = (char*) "AT+SMOD?";
  atCmd[1] = NULL;
  _transmitCmd(WALTER_MODEM_CMD_TYPE_TX, atCmd);

  len = _uartRead(rsp, 64);
  rsp[len] = 0;
  ESP_LOGD("WalterModem", "AT+SMOD? got %d:%s", len, rsp);

  atCmd[0] = (char*) "AT+SMUPGRADE?";
  atCmd[1] = NULL;
  _transmitCmd(WALTER_MODEM_CMD_TYPE_TX, atCmd);

  len = _uartRead(rsp, 64);
  rsp[len] = 0;
  ESP_LOGD("WalterModem", "AT+SMUPGRADE? got %d:%s", len, rsp);

  _rxHandlerInterrupted = false;
}

void WalterModem::_modemFirmwareUpgradeBlock(uint8_t* block, size_t blockSize,
                                             uint32_t transactionId)
{
  size_t bytesSent, bytesReceived;

  /* STP transfer block command: specify block size */
  stpRequestTransferBlockCmd.blockSize = _switchEndian16(blockSize);
  stpRequest.signature = _switchEndian32(WALTER_MODEM_STP_SIGNATURE_REQUEST);
  stpRequest.operation = WALTER_MODEM_STP_OPERATION_TRANSFER_BLOCK_COMMAND;
  stpRequest.sessionId = 1;
  stpRequest.payloadLength = _switchEndian16(sizeof(stpRequestTransferBlockCmd));
  stpRequest.transactionId = _switchEndian32(transactionId);
  stpRequest.headerCrc16 = 0;
  stpRequest.payloadCrc16 =
      _calculateStpCrc16(&stpRequestTransferBlockCmd, sizeof(stpRequestTransferBlockCmd));
  stpRequest.headerCrc16 = _calculateStpCrc16(&stpRequest, sizeof(stpRequest));

  bytesSent = _uartWrite((uint8_t*) &stpRequest, sizeof(stpRequest));

  ESP_LOGD("WalterModem",
           "sent STP transfer block command: tx=%d header: 0x%" PRIx32 " 0x%x 0x%x %d %" PRIu32
           " 0x%x 0x%x",
           bytesSent, _switchEndian32(stpRequest.signature), stpRequest.operation,
           stpRequest.sessionId, _switchEndian16(stpRequest.payloadLength),
           _switchEndian32(stpRequest.transactionId), _switchEndian16(stpRequest.headerCrc16),
           _switchEndian16(stpRequest.payloadCrc16));

  bytesSent =
      _uartWrite((uint8_t*) &stpRequestTransferBlockCmd, sizeof(stpRequestTransferBlockCmd));

  ESP_LOGD("WalterModem", "sent STP transfer block command data: sent=%d payload: %d", bytesSent,
           _switchEndian16(stpRequestTransferBlockCmd.blockSize));

  bytesReceived = _uartRead((uint8_t*) &stpRequest, sizeof(stpRequest), true);

  ESP_LOGD("WalterModem",
           "received STP transfer block command ack: rx=%d header: "
           "0x%" PRIx32 " 0x%x 0x%x %d %" PRIu32 " 0x%x 0x%x",
           bytesReceived, _switchEndian32(stpRequest.signature), stpRequest.operation,
           stpRequest.sessionId, _switchEndian16(stpRequest.payloadLength),
           _switchEndian32(stpRequest.transactionId), _switchEndian16(stpRequest.headerCrc16),
           _switchEndian16(stpRequest.payloadCrc16));

  /* transfer block: actual data transfer */
  stpRequest.signature = _switchEndian32(WALTER_MODEM_STP_SIGNATURE_REQUEST);
  stpRequest.operation = WALTER_MODEM_STP_OPERATION_TRANSFER_BLOCK;
  stpRequest.sessionId = 1;
  stpRequest.payloadLength = _switchEndian16(blockSize);
  stpRequest.transactionId = _switchEndian32(transactionId + 1);
  stpRequest.headerCrc16 = 0;
  stpRequest.payloadCrc16 = _calculateStpCrc16(block, blockSize);
  stpRequest.headerCrc16 = _calculateStpCrc16(&stpRequest, sizeof(stpRequest));

  bytesSent = _uartWrite((uint8_t*) &stpRequest, sizeof(stpRequest));

  ESP_LOGD(
      "WalterModem",
      "sent STP transfer block: tx=%d header: 0x%" PRIx32 " 0x%x 0x%x %d %" PRIu32 " 0x%x 0x%x",
      bytesSent, _switchEndian32(stpRequest.signature), stpRequest.operation, stpRequest.sessionId,
      _switchEndian16(stpRequest.payloadLength), _switchEndian32(stpRequest.transactionId),
      _switchEndian16(stpRequest.headerCrc16), _switchEndian16(stpRequest.payloadCrc16));

  bytesSent = _uartWrite(block, blockSize);

  ESP_LOGD("WalterModem",
           "sent STP transfer block data: tx=%d payload: %d bytes data from flash dup file",
           bytesSent, blockSize);

  bytesReceived = _uartRead((uint8_t*) &stpRequest, sizeof(stpRequest), true);

  ESP_LOGD("WalterModem",
           "received STP transfer block ack: rx=%d header: "
           "0x%" PRIx32 " 0x%x 0x%x %d %" PRIu32 " 0x%x 0x%x",
           bytesReceived, _switchEndian32(stpRequest.signature), stpRequest.operation,
           stpRequest.sessionId, _switchEndian16(stpRequest.payloadLength),
           _switchEndian32(stpRequest.transactionId), _switchEndian16(stpRequest.headerCrc16),
           _switchEndian16(stpRequest.payloadCrc16));

  bytesReceived =
      _uartRead((uint8_t*) &stpResponseTransferBlock, sizeof(stpResponseTransferBlock), true);

  ESP_LOGD("WalterModem", "received STP transfer block ack data: received=%d payload: %d",
           bytesReceived, _switchEndian16(stpResponseTransferBlock.residue));
}

#endif
#pragma endregion
#pragma region UART

size_t WalterModem::_uartRead(uint8_t* buf, int readSize, bool tryHard)
{
  size_t totalBytesRead = 0;

#ifdef ARDUINO
  do {
    totalBytesRead += _uart->readBytes(buf, readSize - totalBytesRead);
  } while(tryHard && totalBytesRead < readSize);
#else
  do {
    int bytesRead =
        uart_read_bytes(_uartNo, buf, readSize - totalBytesRead, WALTER_MODEM_CMD_TIMEOUT_TICKS);
    if(bytesRead < 0) {
      break;
    }
    totalBytesRead += bytesRead;
  } while(tryHard && totalBytesRead < readSize);
#endif

  return totalBytesRead;
}

size_t WalterModem::_uartWrite(uint8_t* buf, int writeSize)
{
#ifdef ARDUINO
  writeSize = _uart->write(buf, writeSize);
  _uart->flush();
#else
  writeSize = uart_write_bytes(_uartNo, buf, writeSize);
  uart_wait_tx_done(_uartNo, pdMS_TO_TICKS(10));
#endif

  return writeSize;
}

#pragma endregion
#pragma region STATE_CMD_POOL

WalterModemCmd* WalterModem::_cmdPoolGet()
{
  for(size_t i = 0; i < WALTER_MODEM_CMD_QUEUE_MAX_ITEMS; ++i) {
    WalterModemCmd* cmd = _cmdPool + i;
    if(cmd->state == WALTER_MODEM_CMD_STATE_FREE) {
      cmd->state = WALTER_MODEM_CMD_STATE_POOLED;
      return cmd;
    }
  }

  return NULL;
}

WalterModemCmd* WalterModem::_cmdQueuePop()
{
  if(_cmdQueue.inIdx == _cmdQueue.outIdx && _cmdQueue.queue[_cmdQueue.outIdx] == NULL) {
    /* The queue is empty */
    return NULL;
  }

  WalterModemCmd* cmd = _cmdQueue.queue[_cmdQueue.outIdx];
  _cmdQueue.queue[_cmdQueue.outIdx] = NULL;
  _cmdQueue.outIdx += 1;
  if(_cmdQueue.outIdx == WALTER_MODEM_CMD_QUEUE_MAX_ITEMS) {
    _cmdQueue.outIdx = 0;
  }
  return cmd;
}

bool WalterModem::_cmdQueuePut(WalterModemCmd* cmd)
{
  if(_cmdQueue.inIdx == _cmdQueue.outIdx && _cmdQueue.queue[_cmdQueue.outIdx] != NULL) {
    /* The queue is full */
    return false;
  }

  _cmdQueue.queue[_cmdQueue.inIdx] = cmd;

  _cmdQueue.inIdx += 1;
  if(_cmdQueue.inIdx == WALTER_MODEM_CMD_QUEUE_MAX_ITEMS) {
    _cmdQueue.inIdx = 0;
  }
  return true;
}

#pragma endregion // STATE_CMD_POOL
#pragma region STATE_PDP_CONTEXT

walter_modem_pdp_context_t* WalterModem::_pdpContextGet(int id)
{
  if(id < 0) {
    return _pdpCtx;
  }

  if(id >= WALTER_MODEM_MAX_PDP_CTXTS) {
    return NULL;
  }
  _pdpCtx = &_pdpCtxSet[id];
  return _pdpCtx;
}

void WalterModem::_saveRTCPdpContextSet(walter_modem_pdp_context_t* _pdpCtxSetRTC)
{
  for(int i = 0; i < WALTER_MODEM_MAX_PDP_CTXTS; ++i) {
    _pdpCtxSetRTC[i] = _pdpCtxSet[i];
  }
}

void WalterModem::_loadRTCPdpContextSet(walter_modem_pdp_context_t* _pdpCtxSetRTC)
{
  if(_pdpCtxSetRTC == NULL) {
    return;
  }

  for(int i = 0; i < WALTER_MODEM_MAX_PDP_CTXTS; ++i) {
    _pdpCtxSet[i] = _pdpCtxSetRTC[i];
  }

  _pdpCtx = _pdpCtxSet;
}

#pragma endregion // STATE_PDP_CONTEXT
#pragma region UART_PROC_TASK

WalterModemBuffer* WalterModem::_getFreeBuffer(void)
{
  WalterModemBuffer* chosenBuf = NULL;

  for(int i = 0; i < WALTER_MODEM_BUFFER_POOL_SIZE; ++i) {
    if(_bufferPool[i].free) {
      chosenBuf = _bufferPool + i;
      chosenBuf->free = false;
      chosenBuf->size = 0;

      break;
    }
  }

  if(chosenBuf == NULL) {
    ESP_LOGE("WalterModem", "No free buffers");
  }

  return chosenBuf;
}

void WalterModem::_bufPrintf(WalterModemBuffer* buf, const char* fmt, ...)
{
  const size_t space = sizeof(buf->data) - buf->size;

  /* Already truncated, keep the first error only */
  if(space <= 1) {
    return;
  }

  va_list args;
  va_start(args, fmt);
  int n = vsnprintf((char*) buf->data + buf->size, space, fmt, args);
  va_end(args);

  if(n < 0) {
    return;
  }

  if((size_t) n >= space) {
    buf->size = sizeof(buf->data) - 1;
    ESP_LOGE("WalterModem", "AT command does not fit in %u bytes and was truncated",
             (unsigned) (sizeof(buf->data) - 1));
  } else {
    buf->size += n;
  }
}

void WalterModem::_addATByteToBuffer(char data, bool raw)
{
  // TODO: in the future we must be aware of length, or at least check if the ending \r\n is
  // escaped when data is transmitted.
  if(!raw && data == '\r') {
    _parserData.state = WALTER_MODEM_RSP_PARSER_END_LF;
    return;
  }

  /* Try to get a free response buffer from the pool */
  if(_parserData.buf == NULL) {
    _parserData.buf = _getFreeBuffer();
  }

  /*
   * The buffer could be NULL when all buffers in the pool are in use. When this happens we will
   * continue parsing but drop the fully parsed command.
   */
  if(_parserData.buf != NULL) {
    _parserData.buf->data[_parserData.buf->size++] = data;
  }
}

void WalterModem::_addATBytesToBuffer(const char* data, size_t length)
{
  /* Try to get a free buffer if not already set */
  if(_parserData.buf == NULL) {
    _parserData.buf = _getFreeBuffer();
  }

  /* If still NULL, drop data silently */
  if(_parserData.buf == NULL) {
    return;
  }

  WalterModemBuffer* buf = _parserData.buf;
  const size_t cap = sizeof(buf->data) - 1; /* room for the terminator _buffStr writes */

  if(buf->size + length > cap) {
    /*
     * No AT line is this long. Keep its head and roll the newest bytes through the tail, so the
     * CRLF or end-of-payload marker that ends it is still detected.
     */
    const size_t tail = 32;
    const size_t head = cap - tail;

    if(buf->size <= head) {
      ESP_LOGW("WalterParser", "Line longer than %u bytes, truncating it", (unsigned) cap);
      size_t n = head - buf->size;
      memcpy(buf->data + buf->size, data, n);
      buf->size = head;
      data += n;
      length -= n;
    }

    size_t kept = buf->size - head;
    if(length >= tail) {
      memcpy(buf->data + head, data + length - tail, tail);
      buf->size = cap;
    } else {
      size_t drop = kept + length > tail ? kept + length - tail : 0;
      memmove(buf->data + head, buf->data + head + drop, kept - drop);
      memcpy(buf->data + head + kept - drop, data, length);
      buf->size = head + kept - drop + length;
    }
  } else {
    memcpy(buf->data + buf->size, data, length);
    buf->size += length;
  }

  buf->data[buf->size] = '\0';
}

void WalterModem::_addPayloadBytes(const char* data, size_t length)
{
  WalterModemPayloadSink* sink = &_parserData.sink;

  portENTER_CRITICAL(&_sinkLock);
  size_t space = sink->buf != NULL ? sink->size - sink->written : 0;
  size_t n = length < space ? length : space;
  if(n > 0) {
    memcpy(sink->buf + sink->written, data, n);
    sink->written += n;
  }
  bool firstDrop = sink->buf != NULL && sink->dropped == 0 && n < length;
  sink->dropped += length - n;
  portEXIT_CRITICAL(&_sinkLock);

  if(firstDrop) {
    ESP_LOGW("WalterParser", "Payload does not fit the receive buffer, dropping the excess");
  }
}

size_t WalterModem::_armPayloadSink(WalterModemCmd* cmd, uint16_t hdrAt, bool headerless)
{
  WalterModemPayloadSink* sink = &_parserData.sink;
  size_t expected = 0;

  portENTER_CRITICAL(&_sinkLock);
  bool usable =
      cmd != NULL && cmd == _curCmd && sink->owner != cmd && (!headerless || cmd->payloadSize > 0);

  if(usable || !headerless) {
    sink->buf = usable ? cmd->payload : NULL;
    sink->size = usable ? cmd->payloadSize : 0;
    sink->written = 0;
    sink->dropped = 0;
    sink->hdrBuf = _parserData.buf;
    sink->hdrAt = hdrAt;

    if(usable) {
      sink->owner = cmd;
      expected = cmd->payloadSize;
    }
  }
  portEXIT_CRITICAL(&_sinkLock);

  return expected;
}

void WalterModem::_disarmPayloadSink(WalterModemCmd* owner)
{
  portENTER_CRITICAL(&_sinkLock);
  _parserData.sink.buf = NULL;
  _parserData.sink.owner = owner;
  _parserData.sink.hdrBuf = NULL;
  portEXIT_CRITICAL(&_sinkLock);
}

void WalterModem::_queueRxBuffer()
{
  if(_parserData.buf != NULL) {
    if(_parserData.buf->size > 0) {
      walter_modem_cmd_queue_t qItem = {};
      qItem.rsp = _parserData.buf;
      // ESP_LOGV("WalterParser", "Queued the buffer (size: %u bytes)", _parserData.buf->size);
      if(xQueueSend(_taskQueue.handle, &qItem, 0) != pdTRUE) {
        /*
         * When we can not send the buffer to the queue we release it immediately and thus
         * drop the packet. In the other case the buffer will be released by the queue
         * consumer.
         */
        _parserData.buf->free = true;
        ESP_LOGW("WalterParser", "unable to queue the buffer");
      }
    } else {
      /* Release empty buffers immediately */
      _parserData.buf->free = true;
    }

    _parserData.buf = NULL;
  }
}

bool WalterModem::_getCRLFPosition(const char* rx_data, size_t len, bool findWhole, size_t* pos)
{
  if(!findWhole) {
    const char* crPtr = (const char*) memchr(rx_data, '\r', len);
    const char* lfPtr = (const char*) memchr(rx_data, '\n', len);

    if(pos != nullptr) {
      if(crPtr) {
        *pos = (size_t) (crPtr - rx_data);
      } else if(lfPtr) {
        *pos = (size_t) (lfPtr - rx_data);
      } else {
        *pos = SIZE_MAX;
      }
    }

    if(crPtr && (size_t) (crPtr - rx_data) + 1 < len && rx_data[(crPtr - rx_data) + 1] == '\n') {
      return true;
    }
    return false;
  } else {
    for(size_t i = 0; i + 1 < len; ++i) {
      if(rx_data[i] == '\r' && rx_data[i + 1] == '\n') {
        if(pos)
          *pos = i;
        return true;
      }
    }
    if(pos)
      *pos = SIZE_MAX;
    return false;
  }
}

bool WalterModem::_checkPayloadComplete()
{
  char* data = reinterpret_cast<char*>(_parserData.buf->data);
  size_t size = _parserData.buf->size;

  auto handleMatch = [&](size_t offset) -> bool {
    size_t remaining = size - offset;
    const uint8_t* chunk = _parserData.buf->data + offset;

    /* Cut the end-payload marker chunk off from this buffer */
    _parserData.buf->size -= remaining;
    /* Optionally also strip the ending CRLF \r\n because the rspProcessor doesn't use it */
    _parserData.buf->size -= 2;
    _queueRxBuffer();

    /* Strip the CR/LF from the beginning of the end-payload marker chunk */
    while(remaining && (*chunk == '\r' || *chunk == '\n')) {
      ++chunk;
      --remaining;
    }

    /* Add the end-payload marker chunk to a fresh buffer */
    if(remaining > 0) {
      _addATBytesToBuffer(reinterpret_cast<const char*>(chunk), remaining);
    }

    return true;
  };

  if(void* p = memmem(data, size, "\r\nOK\r\n", 6)) {
    return handleMatch(static_cast<size_t>(static_cast<char*>(p) - data));
  }

  if(void* p = memmem(data, size, "\r\nERROR\r\n", 9)) {
    return handleMatch(static_cast<size_t>(static_cast<char*>(p) - data));
  }

  if(void* p = memmem(data, size, "\r\n+CME ERROR: ", 12)) {
    return handleMatch(static_cast<size_t>(static_cast<char*>(p) - data));
  }

  return false;
}

bool WalterModem::_expectingPayload()
{
  _receivedPayloadSize = SIZE_MAX;

  const char* data = (const char*) _parserData.buf->data;
  size_t dataSize = _parserData.buf->size;

  if(dataSize <= 2) {
    return false;
  }

  // Skip leading \r\n if present
  if(data[0] == '\r' && data[1] == '\n') {
    data += 2;
    dataSize -= 2;
  }

  /**
   * Responses without a payload header, the transmitted command states how many bytes to read.
   * Checked first because such a payload may start with any of the headers below.
   */
  WalterModemCmd* cmd = NULL;
  const char* marker = _headerlessMarker(&cmd);
  if(marker != NULL) {
    return _expectingHeaderlessPayload(cmd, marker, _parserData.buf->size - dataSize);
  }

  /**
   * Payload headers with defined payload length.
   * This is very reliable as the length is explicitly stated.
   */

  if(strncmp(data, "+SQNSRECV: ", strlen("+SQNSRECV: ")) == 0) {
    // Find the end of the header line (\r\n)
    const char* headerEnd = strstr(data, "\r\n");
    if(headerEnd != nullptr) {
      size_t headerLength = (headerEnd - data) + 2;
      // If buffer contains more than just the header, payload is already present in buffer.
      if(dataSize > headerLength) {
        _receivedPayloadSize = 0;
        return false;
      }
    }
    sscanf(data, "+SQNSRECV: %*d,%d", &_receivedPayloadSize);
    if(_receivedPayloadSize > 0) {
      cmd = _curCmd;
      _armPayloadSink(cmdHasPrefix(cmd, "AT+SQNSRECV=") ? cmd : NULL, _parserData.buf->size, false);
      return true;
    } else {
      return false;
    }
  }

  if(strncmp(data, "+SQNCOAPRCV: ", strlen("+SQNCOAPRCV: ")) == 0) {
    // Find the end of the header line (\r\n)
    const char* headerEnd = strstr(data, "\r\n");
    if(headerEnd != nullptr) {
      size_t headerLength = (headerEnd - data) + 2;
      // If buffer contains more than just the header, payload is already present in buffer.
      if(dataSize > headerLength) {
        _receivedPayloadSize = 0;
        return false;
      }
    }
    // There are two possible formats for this response, so we try to parse both.
    int matched = sscanf(data, "+SQNCOAPRCV: %*d,%*d,,%*d,%*d,%*d,%d", &_receivedPayloadSize);
    if(matched != 1) {
      sscanf(data, "+SQNCOAPRCV: %*d,%*d,%*[^,],%*d,%*d,%*d,%d", &_receivedPayloadSize);
    }
    if(_receivedPayloadSize > 0) {
      cmd = _curCmd;
      _armPayloadSink(cmdHasPrefix(cmd, "AT+SQNCOAPRCV=") ? cmd : NULL, _parserData.buf->size,
                      false);
      return true;
    } else {
      return false;
    }
  }

  /**
   * Payload headers are present, but do not contain payload length. This is less reliable but still
   * very unlikely to fail.
   *
   * @warning The payload must NOT contain:
   * - "\r\nOK\r\n"
   * - "\r\nERROR\r\n"
   * - "\r\n+CME ERROR: "
   */

  // Check for "+SQNSNVR: "
  if(strncmp(data, "+SQNSNVR: ", strlen("+SQNSNVR: ")) == 0) {
    return true;
  }

  return false;
}

const char* WalterModem::_headerlessMarker(WalterModemCmd** cmd)
{
  WalterModemCmd* cur = _curCmd;
  const char* marker = NULL;

  if(cmdHasPrefix(cur, "AT+SQNHTTPRCV=")) {
    marker = "<<<";
  } else if(cmdHasPrefix(cur, "AT+SQNSMQTTRCVMESSAGE=")) {
    marker = "";
  }

  if(marker != NULL && cmd != NULL) {
    *cmd = cur;
  }

  return marker;
}

bool WalterModem::_expectingHeaderlessPayload(WalterModemCmd* cmd, const char* marker, size_t lead)
{
  WalterModemBuffer* buf = _parserData.buf;
  const char* data = (const char*) buf->data + lead;
  size_t dataSize = buf->size - lead;
  size_t markerLen = strlen(marker);

  /**
   * This might fail due to race conditions (a URC arriving before the payload) but is still very
   * unlikely.
   *
   * @note This is an unfortunate quirk of the Sequans modem AT command set.
   *
   * @warning The payload must NOT start with:
   * - "> " or ">>>"
   * - "OK\r\n"
   * - "ERROR\r\n"
   * - "+CME ERROR: "
   */
  if(strncmp(data, "OK\r\n", 4) == 0 || strncmp(data, "ERROR\r\n", 7) == 0 ||
     strncmp(data, "+CME ERROR: ", 12) == 0) {
    return false;
  }

  if(dataSize < markerLen || memcmp(data, marker, markerLen) != 0) {
    return false;
  }

  uint16_t hdrAt = lead + markerLen;
  size_t expected = _armPayloadSink(cmd, hdrAt, true);
  if(expected == 0) {
    return false;
  }

  /* Move what is already buffered into the sink, bytes left after the payload are framing */
  size_t buffered = buf->size - hdrAt;
  size_t n = buffered < expected ? buffered : expected;
  _addPayloadBytes((const char*) buf->data + hdrAt, n);
  memmove(buf->data + hdrAt, buf->data + hdrAt + n, buffered - n);
  buf->size -= n;
  buf->data[buf->size] = '\0';

  if(n == expected) {
    _receivedPayloadSize = 0;
    return false;
  }

  _receivedPayloadSize = expected - n;
  return true;
}

void WalterModem::_parseRxData(char* rx_data, size_t rx_len)
{
  if(rx_len == 0 || _hardwareReset || rx_len > UART_BUF_SIZE)
    return;

  /* Main processing loop: parse until all bytes consumed */
  for(size_t offset = 0; offset < rx_len;) {
    size_t rx_remaining = rx_len - offset;
    const char* message = rx_data + offset;
    size_t message_len = 0;
    bool payloadSizeKnown = (_receivedPayloadSize != SIZE_MAX);

    /* Receiving binary payload with a known total size */
    /* Here we count the amount of bytes to read, and stream them into the payload sink. The header
     * stays in the buffer, which is queued once the trailing CRLF arrives */
    if(_receivingPayload && payloadSizeKnown) {
      message_len = (_receivedPayloadSize > rx_remaining) ? rx_remaining : _receivedPayloadSize;
      _addPayloadBytes(message, message_len);
      _receivedPayloadSize -= message_len;
      offset += message_len;
      if(_receivedPayloadSize == 0) {
        _receivingPayload = false;
      }
      continue;
    }

    /* Receiving messages with undefined size (AT-commands, URCs, payloads with unknown size) */
    /* We keep appending the message to the buffer until the CRLF is found. Queue when complete */
    size_t crlfPos;
    if(_parserData.buf && _parserData.buf->size > 0 &&
       _parserData.buf->data[_parserData.buf->size - 1] == '\r' && message[0] == '\n') {
      /* Complete a CRLF that was split over two UART reads, as if it arrived in one */
      message_len = 1;
    } else if(_getCRLFPosition(message, rx_remaining, false, &crlfPos)) {
      /* If we found a full CRLF, we can read until the end of it */
      message_len += crlfPos + 2;
    } else {
      if(crlfPos != SIZE_MAX) {
        /* If we found a CR or LF, but not the full CRLF, we read until the found position */
        message_len += crlfPos + 1;
      } else {
        /* If no CR or LF found, we read all remaining bytes */
        message_len = rx_remaining;
      }
    }

    /* A line longer than a buffer can only be the payload of a response without a length header,
     * start streaming it before its first CRLF arrives */
    if(!_receivingPayload && _parserData.buf &&
       _parserData.buf->size + message_len >= sizeof(_parserData.buf->data) &&
       _headerlessMarker(NULL) != NULL && _expectingPayload()) {
      _receivingPayload = true;
      continue;
    }

    _addATBytesToBuffer(message, message_len);
    offset += message_len;

    /* We need to check if the CRLF symbol is part of the payload, or if it marks the end */
    if(_receivingPayload && !payloadSizeKnown) {

      /* If the payload is not yet complete, we continue adding to the buffer without queueing */
      if(!_checkPayloadComplete()) {
        continue;
      }
      _receivingPayload = false;
    }

    /* Check if the full message or payload is received by looking for a trailing \r\n CRLF in the
     * buffer. If not present, we assume the message was split by the UART buffer, so we continue
     * buffering, unless we are receiving a special prompt.
     */
    if(_parserData.buf && _parserData.buf->size > 2 &&
       _getCRLFPosition((const char*) _parserData.buf->data + 2, _parserData.buf->size - 2, true)) {

      /* Set a flag and continue buffering if the current message expects a binary payload, or the
       * current message is payload and we expect more data */
      if(_expectingPayload()) {
        _receivingPayload = true;
        continue;
      } else {
        _receivingPayload = false;
        /* Finally, queue the buffer if we are sure the message is complete */
        _queueRxBuffer();
      }

      continue;
    }

    /* Check for special prompts (e.g. "\r\n> " or ">>>" or "\0") which don't use trailing CRLF's
     * but still mark the end of a message */
    bool prompt1 = (_parserData.buf && _parserData.buf->size >= 4 &&
                    _parserData.buf->data[2] == '>' && _parserData.buf->data[3] == ' ');
    bool prompt2 =
        (_parserData.buf && _parserData.buf->size >= 3 && _parserData.buf->data[0] == '>' &&
         _parserData.buf->data[1] == '>' && _parserData.buf->data[2] == '>');
    bool nullbyte =
        (_parserData.buf && _parserData.buf->size == 1 && _parserData.buf->data[0] == '\0');

    if(prompt1 || prompt2 || nullbyte) {
      _queueRxBuffer();
    }
  }
}

#ifdef ARDUINO

void WalterModem::_handleRxData(void)
{
  size_t uartBufLen;
  static char incomingBuf[UART_BUF_SIZE];

#if CONFIG_WALTER_MODEM_ENABLE_MOTA

  if(_rxHandlerInterrupted) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    return;
  }

#endif

  uartBufLen = _uart->available();

  if(uartBufLen == 0) {
    return;
  }

  if(uartBufLen > UART_BUF_SIZE) {
    uartBufLen = UART_BUF_SIZE;
  }

  uartBufLen = _uartRead((uint8_t*) incomingBuf, uartBufLen);
  _parseRxData(incomingBuf, uartBufLen);
}

#else

void WalterModem::_handleRxData(void* params)
{
  size_t uartBufLen;
  static char incomingBuf[UART_BUF_SIZE];

  /* init watchdog for uart rx task on ESP-IDF */
  if(_watchdogTimeout != 0) {
    esp_task_wdt_add(NULL);
  }

  for(;;) {
    tickleWatchdog();

#if CONFIG_WALTER_MODEM_ENABLE_MOTA

    if(_rxHandlerInterrupted) {
      vTaskDelay(pdMS_TO_TICKS(1000));
      continue;
    }

#endif

    uart_get_buffered_data_len(_uartNo, &uartBufLen);

    if(uartBufLen == 0) {
      /* if nothing available, yield by waiting for 1 new byte */
      uartBufLen = 1;
    }

    if(uartBufLen > UART_BUF_SIZE) {
      uartBufLen = UART_BUF_SIZE;
    }

    uartBufLen = _uartRead((uint8_t*) incomingBuf, uartBufLen);
    _parseRxData(incomingBuf, uartBufLen);
  }
}

#endif

void WalterModem::_cmdProcessingTask(void* args)
{
  /* init watchdog for internal command processing task */
  if(_watchdogTimeout) {
    esp_task_wdt_add(NULL);
  }

  walter_modem_cmd_queue_t qItem = {};
  TickType_t blockTime = WALTER_MODEM_CMD_TIMEOUT_TICKS;

  while(true) {
    tickleWatchdog();

    if(xQueueReceive(_taskQueue.handle, &qItem, blockTime) == pdTRUE) {
      if(qItem.cmd != NULL) {
        if(_curCmd == NULL) {
          _curCmd = qItem.cmd;
        } else {
          if(!_cmdQueuePut(qItem.cmd)) {
            _processModemCMD(qItem.cmd, true);
          }
        }
      } else if(qItem.rsp != NULL) {
        _processModemRSP(_curCmd, qItem.rsp);
      }
    }

    blockTime = WALTER_MODEM_CMD_TIMEOUT_TICKS;

    if(_curCmd != NULL) {
      switch(_curCmd->state) {
      case WALTER_MODEM_CMD_STATE_FREE:
      case WALTER_MODEM_CMD_STATE_POOLED:
        /*
         * There was a programming issue, commands with these states should never be
         * sent to the queue. We are going to ignore them.
         */
        _curCmd = NULL;
        break;

      case WALTER_MODEM_CMD_STATE_RETRY_AFTER_ERROR:
        blockTime = _processModemCMD(_curCmd);
        break;

      case WALTER_MODEM_CMD_STATE_NEW:
      case WALTER_MODEM_CMD_STATE_PENDING:
        blockTime = _processModemCMD(_curCmd);
        if(_curCmd->state == WALTER_MODEM_CMD_STATE_COMPLETE) {
          _curCmd->state = WALTER_MODEM_CMD_STATE_FREE;
          _curCmd = NULL;
        }
        break;

      case WALTER_MODEM_CMD_STATE_SYNC_LOCK_NOTIFIED:
        /* We need to wait until the other thread is ready. Nothing wakes this task when it is, so
         * poll: blocking the full timeout stalls any command already queued behind this one. */
        blockTime = 1;
        break;

      case WALTER_MODEM_CMD_STATE_COMPLETE:
        _curCmd->state = WALTER_MODEM_CMD_STATE_FREE;
        _curCmd = NULL;
        break;
      }
    }

    if(_curCmd == NULL) {
      _curCmd = _cmdQueuePop();
      if(_curCmd != NULL) {
        blockTime = 0;
      }
    }
  }
}

#pragma endregion // UART_PROC_TASK
#pragma region URC_PROC_TASK

void WalterModem::_eventProcessingTask(void* args)
{
  WalterModemEvent qItem;
  while(true) {
    vTaskDelay(pdMS_TO_TICKS(10));
    if(xQueueReceive(_eventQueue.handle, &qItem, 0) == pdTRUE) {

      /* Small delay to allow rsp processor to complete */
      vTaskDelay(pdMS_TO_TICKS(10));

#if CONFIG_BLUECHERRY_ENABLE

      if(qItem.type == WALTER_MODEM_EVENT_TYPE_SOCKET &&
         qItem.socket.data.conn_id == WalterBlueCherry::_socketId()) {
        WalterBlueCherry::_handleSocketEvent(qItem.socket.event, qItem.socket.data.data_len);
        continue;
      }

#endif
      _dispatchEvent(&qItem);
    }
  }
}

#pragma endregion // URC_PROC_TASK
#pragma region CMD_PROCESSING

WalterModemCmd* WalterModem::_queueModemCMD(
    const char* atCmd[WALTER_MODEM_COMMAND_MAX_ELEMS + 1], const char* atRsp, WalterModemRsp* rsp,
    walterModemCb userCb, void* userCbArgs,
    void (*completeHandler)(struct sWalterModemCmd* cmd, WalterModemState result),
    void* completeHandlerArg, WalterModemCmdType type, uint8_t* payload, uint16_t payloadSize,
    WalterModemBuffer* stringsBuffer, uint8_t maxAttempts)
{
  WalterModemCmd* cmd = _cmdPoolGet();
  if(cmd == NULL) {
    return NULL;
  }

  for(size_t i = 0; i < WALTER_MODEM_COMMAND_MAX_ELEMS; ++i) {
    if(atCmd[i] == NULL) {
      cmd->atCmd[i] = NULL;
      break;
    }
    cmd->atCmd[i] = atCmd[i];
  }
  cmd->atCmd[WALTER_MODEM_COMMAND_MAX_ELEMS] = NULL;
  cmd->atRsp = atRsp;
  cmd->rsp = rsp == NULL ? &(cmd->rspMem) : rsp;
  cmd->userCb = userCb;
  cmd->userCbArgs = userCbArgs;
  cmd->type = type;
  cmd->completeHandler = completeHandler;
  cmd->completeHandlerArg = completeHandlerArg;
  cmd->payload = payload;
  cmd->payloadSize = payloadSize;
  cmd->maxAttempts = maxAttempts;
  cmd->atRspLen = atRsp == NULL ? 0 : strlen(atRsp);
  cmd->state = WALTER_MODEM_CMD_STATE_NEW;
  cmd->attempt = 0;
  cmd->attemptStart = 0;
  cmd->stringsBuffer = stringsBuffer;
  *(cmd->rsp) = {};

  walter_modem_cmd_queue_t qItem = {};
  qItem.cmd = cmd;

  if(xQueueSend(_taskQueue.handle, &qItem, 0) != pdTRUE) {
    cmd->state = WALTER_MODEM_CMD_STATE_FREE;
    if(stringsBuffer) {
      stringsBuffer->free = true;
    }

    return NULL;
  }

  return cmd;
}

void WalterModem::_finishModemCMD(WalterModemCmd* cmd, WalterModemState result)
{
  /* A blocking caller's buffer goes out of scope once it is notified */
  if(cmd == _curCmd) {
    _disarmPayloadSink(cmd);
  }

  cmd->rsp->result = result;

  if(cmd->stringsBuffer) {
    cmd->stringsBuffer->free = true;
  }

  if(cmd->completeHandler != NULL) {
    cmd->completeHandler(cmd, result);
  }

  if(cmd->userCb) {
    cmd->userCb(cmd->rsp, cmd->userCbArgs);
    cmd->state = WALTER_MODEM_CMD_STATE_COMPLETE;
  } else {
    cmd->state = WALTER_MODEM_CMD_STATE_SYNC_LOCK_NOTIFIED;
    std::unique_lock<std::mutex> lock { cmd->cmdLock.mutex };
    cmd->cmdLock.cond.notify_one();
    lock.unlock();
  }
}

TickType_t WalterModem::_processModemCMD(WalterModemCmd* cmd, bool queueError)
{
  if(queueError) {
    _finishModemCMD(cmd, WALTER_MODEM_STATE_NO_MEMORY);
    return WALTER_MODEM_CMD_TIMEOUT_TICKS;
  }

  switch(cmd->type) {
  case WALTER_MODEM_CMD_TYPE_TX:
    _disarmPayloadSink(NULL);
    _transmitCmd(cmd->type, cmd->atCmd);
    cmd->state = WALTER_MODEM_CMD_STATE_PENDING;
    _finishModemCMD(cmd);
    break;

  case WALTER_MODEM_CMD_TYPE_TX_WAIT:
  case WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT:
    if(cmd->state == WALTER_MODEM_CMD_STATE_NEW) {
      _disarmPayloadSink(NULL);
      _transmitCmd(cmd->type, cmd->atCmd);
      cmd->attempt = 1;
      cmd->attemptStart = xTaskGetTickCount();
      cmd->state = WALTER_MODEM_CMD_STATE_PENDING;
      return WALTER_MODEM_CMD_TIMEOUT_TICKS;
    } else {
      TickType_t diff = xTaskGetTickCount() - cmd->attemptStart;
      bool timedOut = diff >= WALTER_MODEM_CMD_TIMEOUT_TICKS;
      if(timedOut || cmd->state == WALTER_MODEM_CMD_STATE_RETRY_AFTER_ERROR) {
        if(timedOut) {
          ESP_LOGW("WalterModem", "Command time-out (TX) Attempt %u of %u", cmd->attempt,
                   cmd->maxAttempts);
        } else {
          ESP_LOGD("WalterModem", "Command ERROR (TX) Attempt %u of %u", cmd->attempt,
                   cmd->maxAttempts);
        }
        _receivingPayload = false;
        if(cmd->attempt >= cmd->maxAttempts) {
          _finishModemCMD(cmd, timedOut ? WALTER_MODEM_STATE_TIMEOUT : WALTER_MODEM_STATE_ERROR);
        } else {
          _disarmPayloadSink(NULL);
          _transmitCmd(cmd->type, cmd->atCmd);
          cmd->attempt += 1;
          cmd->attemptStart = xTaskGetTickCount();
          cmd->state = WALTER_MODEM_CMD_STATE_PENDING;
          return WALTER_MODEM_CMD_TIMEOUT_TICKS;
        }
      } else {
        return WALTER_MODEM_CMD_TIMEOUT_TICKS - diff;
      }
    }
    break;

  case WALTER_MODEM_CMD_TYPE_WAIT:
    if(cmd->state == WALTER_MODEM_CMD_STATE_NEW) {
      cmd->attemptStart = xTaskGetTickCount();
      cmd->state = WALTER_MODEM_CMD_STATE_PENDING;
      return WALTER_MODEM_CMD_TIMEOUT_TICKS;
    } else {
      TickType_t diff = xTaskGetTickCount() - cmd->attemptStart;
      if(diff >= WALTER_MODEM_CMD_TIMEOUT_TICKS) {
        ESP_LOGW("WalterModem", "Command time-out (WAIT)");
        _receivingPayload = false;
        _finishModemCMD(cmd, WALTER_MODEM_STATE_TIMEOUT);
      } else {
        return diff;
      }
    }
    break;
  }

  return WALTER_MODEM_CMD_TIMEOUT_TICKS;
}

#pragma endregion // CMD_PROCESSING
#pragma region RSP_PROCESSING

void WalterModem::_processModemRSP(WalterModemCmd* cmd, WalterModemBuffer* buff)
{

#if CONFIG_LOG_MAXIMUM_LEVEL >= ESP_LOG_DEBUG
  if(buff == _parserData.sink.hdrBuf) {
    WalterModemPayloadSink* sink = &_parserData.sink;
    logWithPayload(true, buff->data, buff->size, sink->hdrAt, sink->buf, sink->written,
                   sink->dropped);
    sink->hdrBuf = NULL;
  } else {
    Buffer* escaped =
        escapeBuffer((const uint8_t*) buff->data, buff->size, WALTER_MODEM_AT_BUFFER_SIZE,
                     _logEscapeBuffer, sizeof(_logEscapeBuffer));
    if(escaped) {
      ESP_LOGD("WalterModem", "RX: %.*s", escaped->size, escaped->data);
    }
  }
#endif

  // Skip leading \r\n if present
  if(buff->size >= 2 && buff->data[0] == '\r' && buff->data[1] == '\n') {
    memmove(buff->data, buff->data + 2, buff->size - 2);
    buff->size -= 2;
  }

  // Skip trailing \r\n if present
  if(buff->size >= 2 && buff->data[buff->size - 2] == '\r' && buff->data[buff->size - 1] == '\n') {
    buff->size -= 2;
  }

  WalterModemState result = WALTER_MODEM_STATE_OK;

#pragma region RSP_PROC_GENERAL

  /* CEREG Response & CEREG event (URC) */
  if(_buffStartsWith(buff, "+CEREG: ")) {
    const char* rspStr = _buffStr(buff);
    int mode = 0;
    int ceReg = 0;
    char lac[16] = { 0 };
    char ci[16] = { 0 };
    int act = 0;
    char activeTime[16] = { 0 };
    char periodicTau[16] = { 0 };
    bool hasPsmInfo = false;
    bool isResponse = false;
    bool isUrc = false;

    // Check for comma after first digit to distinguish response from URC
    const char* firstComma = strchr(rspStr + _strLitLen("+CEREG: "), ',');
    if(firstComma && (firstComma - rspStr) == _strLitLen("+CEREG: ") + 1) {
      // Format: +CEREG: x,... (second char is comma, so first is mode)
      if(*(firstComma + 1) == '"') {
        // Extended URC: +CEREG: x,"LAC","CI",act,,,"activeTime","periodicTau"
        int result = sscanf(rspStr, "+CEREG: %d,\"%[^\"]\",\"%[^\"]\",%d,,,\"%[^\"]\",\"%[^\"]\"",
                            &ceReg, lac, ci, &act, activeTime, periodicTau);
        if(result >= 4) {
          hasPsmInfo = (result == 6);
          isUrc = true;
        }
      } else if(*(firstComma + 1) >= '0' && *(firstComma + 1) <= '9') {
        // Check if this is extended response or simple response
        const char* secondComma = strchr(firstComma + 1, ',');
        if(secondComma && *(secondComma + 1) == '"') {
          // Extended response: +CEREG: x,x,"LAC","CI",act,,,"activeTime","periodicTau"
          int result =
              sscanf(rspStr, "+CEREG: %d,%d,\"%[^\"]\",\"%[^\"]\",%d,,,\"%[^\"]\",\"%[^\"]\"",
                     &mode, &ceReg, lac, ci, &act, activeTime, periodicTau);
          if(result >= 5) {
            hasPsmInfo = (result == 7);
            isResponse = true;
          }
        } else {
          // Simple response: +CEREG: x,x
          int result = sscanf(rspStr, "+CEREG: %d,%d", &mode, &ceReg);
          if(result == 2) {
            isResponse = true;
          }
        }
      }
    } else {
      // Simple URC: +CEREG: x
      int result = sscanf(rspStr, "+CEREG: %d", &ceReg);
      if(result == 1) {
        isUrc = true;
      }
    }

    // Process based on format type
    if(isResponse) {
      if(mode > 0) {
        bool attached = ceReg == 5 || ceReg == 1;
        _networkAttached = attached;
        for(size_t i = 0; i < WALTER_MODEM_MAX_PDP_CTXTS; i++) {
          if(_pdpCtxSet[i].state != WALTER_MODEM_PDP_CONTEXT_STATE_INACTIVE) {
            _pdpCtxSet[i].state = attached ? WALTER_MODEM_PDP_CONTEXT_STATE_ATTACHED
                                           : WALTER_MODEM_PDP_CONTEXT_STATE_NOT_ATTACHED;
          }
        }
        _regState = (WalterModemNetworkRegState) ceReg;
      }
    } else if(isUrc) {
      bool attached = ceReg == 5 || ceReg == 1;
      _networkAttached = attached;
      for(size_t i = 0; i < WALTER_MODEM_MAX_PDP_CTXTS; i++) {
        if(_pdpCtxSet[i].state != WALTER_MODEM_PDP_CONTEXT_STATE_INACTIVE) {
          _pdpCtxSet[i].state = attached ? WALTER_MODEM_PDP_CONTEXT_STATE_ATTACHED
                                         : WALTER_MODEM_PDP_CONTEXT_STATE_NOT_ATTACHED;
        }
      }
      _regState = (WalterModemNetworkRegState) ceReg;

      WalterModemEvent newEvent = {};
      newEvent.type = WALTER_MODEM_EVENT_TYPE_NETWORK;
      newEvent.network.event = WALTER_MODEM_NETWORK_EVENT_REG_STATE_CHANGE;
      newEvent.network.data.cereg.state = _regState;
      newEvent.network.data.cereg.hasPsmInfo = hasPsmInfo;

      if(hasPsmInfo) {
        _strncpy_s(newEvent.network.data.cereg.lac, lac, sizeof(newEvent.network.data.cereg.lac));
        _strncpy_s(newEvent.network.data.cereg.ci, ci, sizeof(newEvent.network.data.cereg.ci));
        newEvent.network.data.cereg.act = act;
        _strncpy_s(newEvent.network.data.cereg.activeTime, activeTime,
                   sizeof(newEvent.network.data.cereg.activeTime));
        _strncpy_s(newEvent.network.data.cereg.periodicTau, periodicTau,
                   sizeof(newEvent.network.data.cereg.periodicTau));
      } else {
        newEvent.network.data.cereg.lac[0] = '\0';
        newEvent.network.data.cereg.ci[0] = '\0';
        newEvent.network.data.cereg.act = 0;
        newEvent.network.data.cereg.activeTime[0] = '\0';
        newEvent.network.data.cereg.periodicTau[0] = '\0';
      }

      xQueueSend(_eventQueue.handle, &newEvent, 0);
    }

    goto after_processing_logic;
  }

  /* CEDRXP eDRX parameters event (URC) */
  if(_buffStartsWith(buff, "+CEDRXP: ")) {
    const char* rspStr = _buffStr(buff);
    int actType = 0;
    char requestedEdrx[16] = { 0 };
    char nwProvidedEdrx[16] = { 0 };
    char pagingTimeWindow[16] = { 0 };

    int result = sscanf(rspStr, "+CEDRXP: %d,\"%[^\"]\",\"%[^\"]\",\"%[^\"]\"", &actType,
                        requestedEdrx, nwProvidedEdrx, pagingTimeWindow);
    if(result == 4) {
      WalterModemEvent newEvent = {};
      newEvent.type = WALTER_MODEM_EVENT_TYPE_NETWORK;
      newEvent.network.event = WALTER_MODEM_NETWORK_EVENT_EDRX_RECEIVED;
      newEvent.network.data.edrx.actType = actType;
      _strncpy_s(newEvent.network.data.edrx.requestedEdrx, requestedEdrx,
                 sizeof(newEvent.network.data.edrx.requestedEdrx));
      _strncpy_s(newEvent.network.data.edrx.nwProvidedEdrx, nwProvidedEdrx,
                 sizeof(newEvent.network.data.edrx.nwProvidedEdrx));
      _strncpy_s(newEvent.network.data.edrx.pagingTimeWindow, pagingTimeWindow,
                 sizeof(newEvent.network.data.edrx.pagingTimeWindow));
      xQueueSend(_eventQueue.handle, &newEvent, 0);
    }

    goto after_processing_logic;
  }

  /* SQNTMON Response */
  if(_buffStartsWith(buff, "+SQNTMON: ")) {
    const char* rspStr = _buffStr(buff);
    int mode = 0;
    int extreme_low = 0;
    int warning_low = 0;
    int warning_high = 0;
    int extreme_high = 0;
    int temperature = 0;

    // Query response: +SQNTMON:
    // <mode>,<extreme_low>,<warning_low>,<warning_high>,<extreme_high>,<temperature>
    int parsed = sscanf(rspStr, "+SQNTMON: %d,%d,%d,%d,%d,%d", &mode, &extreme_low, &warning_low,
                        &warning_high, &extreme_high, &temperature);
    if(parsed == 6 && cmd != NULL && cmd->rsp != NULL) {
      cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_TEMPERATURE;
      cmd->rsp->data.temperature.mode = (WalterModemTempMonitorMode) mode;
      cmd->rsp->data.temperature.temperature = (int8_t) temperature;
      // Determine status based on temperature vs thresholds
      if(temperature <= extreme_low) {
        cmd->rsp->data.temperature.status = WALTER_MODEM_TEMP_STATUS_EXTREME_LOW;
      } else if(temperature <= warning_low) {
        cmd->rsp->data.temperature.status = WALTER_MODEM_TEMP_STATUS_WARNING_LOW;
      } else if(temperature >= extreme_high) {
        cmd->rsp->data.temperature.status = WALTER_MODEM_TEMP_STATUS_EXTREME_HIGH;
      } else if(temperature >= warning_high) {
        cmd->rsp->data.temperature.status = WALTER_MODEM_TEMP_STATUS_WARNING_HIGH;
      } else {
        cmd->rsp->data.temperature.status = WALTER_MODEM_TEMP_STATUS_NORMAL;
      }
    }

    goto after_processing_logic;
  }

  /* SQNTMONS URC (unsolicited result code) */
  if(_buffStartsWith(buff, "+SQNTMONS: ")) {
    const char* rspStr = _buffStr(buff);
    int mode = 0;
    int status = 0;
    int temperature = 0;

    int parsed = sscanf(rspStr, "+SQNTMONS: %d,%d,%d", &mode, &status, &temperature);
    if(parsed == 3) {
      WalterModemEvent newEvent = {};
      newEvent.type = WALTER_MODEM_EVENT_TYPE_TEMPERATURE;
      newEvent.temperature.mode = (WalterModemTempMonitorMode) mode;
      newEvent.temperature.status = (WalterModemTempStatus) status;
      newEvent.temperature.temperature = (int8_t) temperature;
      xQueueSend(_eventQueue.handle, &newEvent, 0);
    }

    goto after_processing_logic;
  }

  /* SQNVMON Response (set command response) */
  if(_buffStartsWith(buff, "+SQNVMON: ") && cmd != NULL && cmd->rsp != NULL) {
    const char* rspStr = _buffStr(buff);
    int mode = 0;
    int statusOrThreshold = 0;
    int voltageOrPeriod = 0;
    int voltage = 0;

    // Try to parse as query response first: +SQNVMON:<mode>,<threshold>,<period>,<voltage>
    int parsed = sscanf(rspStr, "+SQNVMON: %d,%d,%d,%d", &mode, &statusOrThreshold,
                        &voltageOrPeriod, &voltage);
    if(parsed == 4) {
      cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_VOLTAGE;
      cmd->rsp->data.voltage.mode = (WalterModemVoltageMonitorMode) mode;
      cmd->rsp->data.voltage.threshold = (uint16_t) statusOrThreshold;
      cmd->rsp->data.voltage.period = (uint16_t) voltageOrPeriod;
      cmd->rsp->data.voltage.voltage = (uint16_t) voltage;
      cmd->rsp->data.voltage.status =
          WALTER_MODEM_VOLTAGE_STATUS_ABOVE_THRESHOLD; // Default for query
    } else {
      // Parse as set response: +SQNVMON:<mode>,<status>,<voltage>
      parsed = sscanf(rspStr, "+SQNVMON: %d,%d,%d", &mode, &statusOrThreshold, &voltageOrPeriod);
      if(parsed == 3) {
        cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_VOLTAGE;
        cmd->rsp->data.voltage.mode = (WalterModemVoltageMonitorMode) mode;
        cmd->rsp->data.voltage.status = (WalterModemVoltageStatus) statusOrThreshold;
        cmd->rsp->data.voltage.voltage = (uint16_t) voltageOrPeriod;
        cmd->rsp->data.voltage.threshold = 0; // Not provided in set response
        cmd->rsp->data.voltage.period = 0;    // Not provided in set response
      }
    }

    goto after_processing_logic;
  }

  /* SQNSVMONS URC (unsolicited result code) */
  if(_buffStartsWith(buff, "+SQNSVMONS: ")) {
    const char* rspStr = _buffStr(buff);
    int mode = 0;
    int status = 0;
    int voltage = 0;

    int parsed = sscanf(rspStr, "+SQNSVMONS: %d,%d,%d", &mode, &status, &voltage);
    if(parsed == 3) {
      WalterModemEvent newEvent = {};
      newEvent.type = WALTER_MODEM_EVENT_TYPE_VOLTAGE;
      newEvent.voltage.mode = (WalterModemVoltageMonitorMode) mode;
      newEvent.voltage.status = (WalterModemVoltageStatus) status;
      newEvent.voltage.voltage = (uint16_t) voltage;
      newEvent.voltage.threshold = 0; // Not provided in URC
      newEvent.voltage.period = 0;    // Not provided in URC
      xQueueSend(_eventQueue.handle, &newEvent, 0);
    }

    goto after_processing_logic;
  }

  /* Data prompt for data transmission */
  if(_buffStartsWith(buff, "> ") || _buffStartsWith(buff, ">>>")) {
    if(cmd != NULL && cmd->type == WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT && cmd->payload != NULL) {

#if CONFIG_LOG_MAXIMUM_LEVEL >= ESP_LOG_DEBUG
      logWithPayload(false, NULL, 0, 0, cmd->payload, cmd->payloadSize, 0);
#endif

#ifdef ARDUINO

      _uart->write(cmd->payload, cmd->payloadSize);

#else

      uart_write_bytes(_uartNo, cmd->payload, cmd->payloadSize);

#endif
    }

    goto after_processing_logic;
  }

  /* ERROR response */
  if(_buffStartsWith(buff, "ERROR")) {
    if(cmd != NULL) {
      cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_NO_DATA;
      cmd->state = WALTER_MODEM_CMD_STATE_RETRY_AFTER_ERROR;
    }

    result = WALTER_MODEM_STATE_ERROR;
    goto after_processing_logic;
  }

  /* CME ERROR response */
  if(_buffStartsWith(buff, "+CME ERROR: ")) {
    if(cmd != NULL) {
      const char* rspStr = _buffStr(buff);
      int cmeError = atoi(rspStr + _strLitLen("+CME ERROR: "));
      cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_CME_ERROR;
      cmd->rsp->data.cmeError = (WalterModemCMEError) cmeError;
      cmd->state = WALTER_MODEM_CMD_STATE_RETRY_AFTER_ERROR;
    }

    result = WALTER_MODEM_STATE_ERROR;
    goto after_processing_logic;
  }

  /* NO CARRIER response */
  if(_buffStartsWith(buff, "NO CARRIER")) {
    if(cmd != NULL) {
      cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_NO_DATA;
      cmd->state = WALTER_MODEM_CMD_STATE_SYNC_LOCK_NOTIFIED;
    }

    result = WALTER_MODEM_STATE_ERROR;
    goto after_processing_logic;
  }

  /* Operational state response */
  if(_buffStartsWith(buff, "+CFUN: ")) {
    const char* rspStr = _buffStr(buff);
    int op_state = atoi(rspStr + _strLitLen("+CFUN: "));
    _opState = (WalterModemOpState) op_state;

    if(cmd == NULL) {
      buff->free = true;
      return;
    }

    cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_OPSTATE;
    cmd->rsp->data.opState = (WalterModemOpState) op_state;
    goto after_processing_logic;
  }

  /* SIM state response */
  if(_buffStartsWith(buff, "+CPIN: ") && cmd != NULL) {
    cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_SIM_STATE;
    if(_dataStrIs(buff, "+CPIN: ", "READY")) {
      cmd->rsp->data.simState = WALTER_MODEM_SIM_STATE_READY;
    } else if(_dataStrIs(buff, "+CPIN: ", "SIM PIN")) {
      cmd->rsp->data.simState = WALTER_MODEM_SIM_STATE_PIN_REQUIRED;
    } else if(_dataStrIs(buff, "+CPIN: ", "SIM PUK")) {
      cmd->rsp->data.simState = WALTER_MODEM_SIM_STATE_PUK_REQUIRED;
    } else if(_dataStrIs(buff, "+CPIN: ", "PH-SIM PIN")) {
      cmd->rsp->data.simState = WALTER_MODEM_SIM_STATE_PHONE_TO_SIM_PIN_REQUIRED;
    } else if(_dataStrIs(buff, "+CPIN: ", "PH-FSIM PIN")) {
      cmd->rsp->data.simState = WALTER_MODEM_SIM_STATE_PHONE_TO_FIRST_SIM_PIN_REQUIRED;
    } else if(_dataStrIs(buff, "+CPIN: ", "PH-FSIM PUK")) {
      cmd->rsp->data.simState = WALTER_MODEM_SIM_STATE_PHONE_TO_FIRST_SIM_PUK_REQUIRED;
    } else if(_dataStrIs(buff, "+CPIN: ", "SIM PIN2")) {
      cmd->rsp->data.simState = WALTER_MODEM_SIM_STATE_PIN2_REQUIRED;
    } else if(_dataStrIs(buff, "+CPIN: ", "SIM PUK2")) {
      cmd->rsp->data.simState = WALTER_MODEM_SIM_STATE_PUK2_REQUIRED;
    } else if(_dataStrIs(buff, "+CPIN: ", "PH-NET PIN")) {
      cmd->rsp->data.simState = WALTER_MODEM_SIM_STATE_NETWORK_PIN_REQUIRED;
    } else if(_dataStrIs(buff, "+CPIN: ", "PH-NET PUK")) {
      cmd->rsp->data.simState = WALTER_MODEM_SIM_STATE_NETWORK_PUK_REQUIRED;
    } else if(_dataStrIs(buff, "+CPIN: ", "PH-NETSUB PIN")) {
      cmd->rsp->data.simState = WALTER_MODEM_SIM_STATE_NETWORK_SUBSET_PIN_REQUIRED;
    } else if(_dataStrIs(buff, "+CPIN: ", "PH-NETSUB PUK")) {
      cmd->rsp->data.simState = WALTER_MODEM_SIM_STATE_NETWORK_SUBSET_PUK_REQUIRED;
    } else if(_dataStrIs(buff, "+CPIN: ", "PH-SP PIN")) {
      cmd->rsp->data.simState = WALTER_MODEM_SIM_STATE_SERVICE_PROVIDER_PIN_REQUIRED;
    } else if(_dataStrIs(buff, "+CPIN: ", "PH-SP PUK")) {
      cmd->rsp->data.simState = WALTER_MODEM_SIM_STATE_SERVICE_PROVIDER_PUK_REQUIRED;
    } else if(_dataStrIs(buff, "+CPIN: ", "PH-CORP PIN")) {
      cmd->rsp->data.simState = WALTER_MODEM_SIM_STATE_CORPORATE_SIM_REQUIRED;
    } else if(_dataStrIs(buff, "+CPIN: ", "PH-CORP PUK")) {
      cmd->rsp->data.simState = WALTER_MODEM_SIM_STATE_CORPORATE_PUK_REQUIRED;
    }

    goto after_processing_logic;
  }

  /* Packet domain service attachment state response */
  if(_buffStartsWith(buff, "+CGATT: ")) {
    const char* rspStr = _buffStr(buff);
    int attached = atoi(rspStr + _strLitLen("+CGATT: "));
    for(size_t i = 0; i < WALTER_MODEM_MAX_PDP_CTXTS; i++) {
      if(_pdpCtxSet[i].state != WALTER_MODEM_PDP_CONTEXT_STATE_INACTIVE) {
        _pdpCtxSet[i].state = attached ? WALTER_MODEM_PDP_CONTEXT_STATE_ATTACHED
                                       : WALTER_MODEM_PDP_CONTEXT_STATE_NOT_ATTACHED;
      }
    }

    goto after_processing_logic;
  }

  /* Card Identification Number read response */
  if(_buffStartsWith(buff, "+SQNCCID: ") && cmd != NULL) {
    if(cmd == NULL) {
      buff->free = true;
      return;
    }
    cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_SIM_CARD_ID;

    bool inICCID = true;
    size_t offset = 0;

    for(int i = _strLitLen("+SQNCCID: \""); i < buff->size; ++i) {
      if(inICCID) {
        if(buff->data[i] == '"' || offset >= 22) {
          cmd->rsp->data.simCardID.iccid[offset] = '\0';
          inICCID = false;
          offset = 0;
          i += 2;
          i += 2;
          continue;
        }

        cmd->rsp->data.simCardID.iccid[offset++] = buff->data[i];
      } else {
        if(buff->data[i] == '"' || offset >= 32) {
          cmd->rsp->data.simCardID.euiccid[offset] = '\0';
          break;
        }

        cmd->rsp->data.simCardID.euiccid[offset++] = buff->data[i];
      }
    }

    goto after_processing_logic;
  }

  /* Return PDP addresses response */
  if(_buffStartsWith(buff, "+CGPADDR: ")) {
    uint16_t dataSize = buff->size - _strLitLen("+CGPADDR: ");
    uint8_t* data = buff->data + _strLitLen("+CGPADDR: ");

    if(dataSize < 4) {
      cmd->state = WALTER_MODEM_CMD_STATE_RETRY_AFTER_ERROR;
      buff->free = true;
      return;
    }

    uint16_t addr1Offset = 1;
    uint16_t addr2Offset = 1;
    for(uint16_t i = 0; i < dataSize; ++i) {
      if(data[i] == ',') {
        data[i] = '\0';
        if(addr1Offset <= 1) {
          addr1Offset = i + 2;
        } else {
          addr2Offset = i + 2;
        }
      } else if(data[i] == '"') {
        data[i] = '\0';
      }
    }

    if(addr1Offset >= dataSize) {
      addr1Offset = 0;
    }

    if(addr2Offset >= dataSize) {
      addr2Offset = 0;
    }

    int pdp_ctx_id = atoi((const char*) data);
    walter_modem_pdp_context_t* ctx = _pdpContextGet(pdp_ctx_id);
    if(ctx != NULL) {
      _strncpy_s(ctx->pdpAddress, (const char*) data + addr1Offset, WALTER_MODEM_PDP_ADDR_MAX_SIZE);

      if(addr2Offset != 0) {
        _strncpy_s(ctx->pdpAddress2, (const char*) data + addr2Offset,
                   WALTER_MODEM_PDP_ADDR_MAX_SIZE);
      } else {
        ctx->pdpAddress2[0] = '\0';
      }

      if(cmd != NULL) {
        cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_PDP_ADDR;
        cmd->rsp->data.pdpAddressList.pdpAddress = ctx->pdpAddress;
        cmd->rsp->data.pdpAddressList.pdpAddress2 = ctx->pdpAddress2;
      }
    }

    goto after_processing_logic;
  }

  /* Signal quality response */
  if(_buffStartsWith(buff, "+CSQ: ")) {
    const char* rspStr = _buffStr(buff);
    char* data = (char*) rspStr + _strLitLen("+CSQ: ");

    for(size_t i = 0; i < buff->size - _strLitLen("+CSQ: "); ++i) {
      if(data[i] == ',') {
        data[i] = '\0';
        break;
      }
    }

    int rawRSSI = atoi(data);

    if(cmd == NULL) {
      buff->free = true;
      return;
    }

    cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_RSSI;
    cmd->rsp->data.rssi = -113 + (rawRSSI * 2);
    goto after_processing_logic;
  }

  /* Extended signal quality response */
  if(_buffStartsWith(buff, "+CESQ: ")) {
    cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_SIGNAL_QUALITY;

    uint16_t dataSize = buff->size - _strLitLen("+CESQ: ");
    const char* rspStr = _buffStr(buff);
    char* data = (char*) rspStr + _strLitLen("+CESQ: ");

    uint16_t offset = 0;
    uint8_t param = 0;

    for(uint16_t i = 0; i <= dataSize; ++i) {
      if(data[i] != ',' && data[i] != '\0') {
        continue;
      }

      if(param > 3) {
        data[i] = 0;

        int quality = atoi((const char*) data + offset);

        if(param == 4) {
          cmd->rsp->data.signalQuality.rsrq = -195 + (quality * 5);
        } else if(param == 5) {
          cmd->rsp->data.signalQuality.rsrp = -140 + quality;
        }
      }

      param += 1;
      offset = i + 1;
    }

    goto after_processing_logic;
  }

  /* Cellular information response */
  if(_buffStartsWith(buff, "+SQNMONI: ")) {
    cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_CELL_INFO;

    const char* rspStr = _buffStr(buff);
    char* data = (char*) rspStr;
    uint16_t dataSize = buff->size;
    uint16_t offset = _strLitLen("+SQNMONI: ") - 1;
    int8_t lastColon = -1;
    bool firstKeyParsed = false;

    for(int i = offset + 1; i < dataSize; ++i) {
      if(data[i] == ':') {
        lastColon = i;
      } else if(data[i] == ' ' || data[i] == '\r') {
        if(lastColon > 0) {
          const char* key = data + offset + 1;
          int key_len = lastColon - offset - 1;
          const char* value = data + lastColon + 1;
          int value_len = i - lastColon - 1;
          if(!firstKeyParsed) {
            if(key_len > 2) {
              /* The operator name is present */
              memcpy(cmd->rsp->data.cellInformation.netName, key,
                     key_len - 3 > (WALTER_MODEM_OPERATOR_MAX_SIZE - 1)
                         ? WALTER_MODEM_OPERATOR_MAX_SIZE - 1
                         : key_len - 3);
              key += key_len - 2;
              key_len = 2;
            }
            firstKeyParsed = true;
          }
          if(strncmp("Cc", key, key_len) == 0) {
            strToUint16(value, value_len, &(cmd->rsp->data.cellInformation.cc));
          } else if(strncmp("Nc", key, key_len) == 0) {
            strToUint16(value, value_len, &(cmd->rsp->data.cellInformation.nc));
          } else if(strncmp("RSRP", key, key_len) == 0) {
            strToFloat(value, value_len, &(cmd->rsp->data.cellInformation.rsrp));
          } else if(strncmp("CINR", key, key_len) == 0) {
            strToFloat(value, value_len, &(cmd->rsp->data.cellInformation.cinr));
          } else if(strncmp("RSRQ", key, key_len) == 0) {
            strToFloat(value, value_len, &(cmd->rsp->data.cellInformation.rsrq));
          } else if(strncmp("TAC", key, key_len) == 0) {
            strToUint16(value, value_len, &(cmd->rsp->data.cellInformation.tac));
          } else if(strncmp("Id", key, key_len) == 0) {
            strToUint16(value, value_len, &(cmd->rsp->data.cellInformation.pci));
          } else if(strncmp("EARFCN", key, key_len) == 0) {
            strToUint16(value, value_len, &(cmd->rsp->data.cellInformation.earfcn));
          } else if(strncmp("PWR", key, key_len) == 0) {
            strToFloat(value, value_len, &(cmd->rsp->data.cellInformation.rssi));
          } else if(strncmp("PAGING", key, key_len) == 0) {
            strToUint16(value, value_len, &(cmd->rsp->data.cellInformation.paging));
          } else if(strncmp("CID", key, key_len) == 0) {
            strToUint32(value, value_len, &(cmd->rsp->data.cellInformation.cid), 16);
          } else if(strncmp("BAND", key, key_len) == 0) {
            strToUint8(value, value_len, &(cmd->rsp->data.cellInformation.band));
          } else if(strncmp("BW", key, key_len) == 0) {
            strToUint16(value, value_len, &(cmd->rsp->data.cellInformation.bw));
          } else if(strncmp("CE", key, key_len) == 0) {
            strToUint8(value, value_len, &(cmd->rsp->data.cellInformation.ceLevel));
          }

          offset = i;
        }
      }
    }

    goto after_processing_logic;
  }

  /* Request product serial number identification response */
  if(_buffStartsWith(buff, "+CGSN: ")) {
    if(cmd == NULL || buff->size < _strLitLen("+CGSN: \"xxxxxxxxxxxxxxxx\"")) {
      buff->free = true;
      return;
    }

    cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_IDENTITY;

    /* Copy IMEISV number from the response */
    memcpy(cmd->rsp->data.identity.imeisv, buff->data + 8, 16);
    cmd->rsp->data.identity.imeisv[16] = '\0';

    /* The last two digits from the IMEISV number are the SVN number */
    cmd->rsp->data.identity.svn[0] = cmd->rsp->data.identity.imeisv[14];
    cmd->rsp->data.identity.svn[1] = cmd->rsp->data.identity.imeisv[15];
    cmd->rsp->data.identity.svn[2] = '\0';

    /* Copy the 14-digit long IMEI number, and add the checksum */
    memcpy(cmd->rsp->data.identity.imei, cmd->rsp->data.identity.imeisv, 14);
    cmd->rsp->data.identity.imei[14] = _getLuhnChecksum((const char*) cmd->rsp->data.identity.imei);
    cmd->rsp->data.identity.imei[15] = '\0';
    goto after_processing_logic;
  }

  /* IoT Mode activation response */
  if(_buffStartsWith(buff, "+SQNMODEACTIVE: ")) {
    const char* rspStr = _buffStr(buff);
    int rat = atoi(rspStr + _strLitLen("+SQNMODEACTIVE: "));
    _ratType = (WalterModemRAT) (rat - 1);

    if(cmd == NULL) {
      buff->free = true;
      return;
    }

    cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_RAT;
    cmd->rsp->data.rat = (WalterModemRAT) (rat - 1);
    goto after_processing_logic;
  }

  /* Network band selection response */
  if(_buffStartsWith(buff, "+SQNBANDSEL: ")) {
    if(buff->size <= _strLitLen("+SQNBANDSEL: 1,,\"\"")) {
      buff->free = true;
      return;
    }

    uint16_t dataSize = buff->size - _strLitLen("+SQNBANDSEL: ");
    uint8_t* data = buff->data + _strLitLen("+SQNBANDSEL: ");

    cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_BANDSET_CFG_SET;
    WalterModemBandSelection* bSel =
        cmd->rsp->data.bandSelCfgSet.config + cmd->rsp->data.bandSelCfgSet.count;
    cmd->rsp->data.bandSelCfgSet.count += 1;

    /* Parse RAT */
    bSel->rat = data[0] == '0' ? WALTER_MODEM_RAT_LTEM : WALTER_MODEM_RAT_NBIOT;
    data += 2;
    dataSize -= 2;

    /* Parse operator name */
    bSel->netOperator.format = WALTER_MODEM_OPERATOR_FORMAT_LONG_ALPHANUMERIC;
    for(uint16_t i = 0; i < dataSize; ++i) {
      if(data[i] == ',') {
        data += i + 1;
        dataSize -= i + 1;

        if(i < WALTER_MODEM_OPERATOR_MAX_SIZE) {
          bSel->netOperator.name[i] = '\0';
        } else {
          bSel->netOperator.name[WALTER_MODEM_OPERATOR_MAX_SIZE] = '\0';
        }
        break;
      }

      if(i < WALTER_MODEM_OPERATOR_MAX_SIZE) {
        bSel->netOperator.name[i] = data[i];
      }
    }

    /* Parse configured bands */
    uint16_t start = 1;
    for(uint16_t i = 1; i < dataSize; ++i) {
      if(!(data[i] == ',' || data[i] == '"') || start == i) {
        continue;
      }

      int band = atoi((const char*) data + start);
      start = i + 1;

      switch(band) {
      case 1:
        bSel->bands |= WALTER_MODEM_BAND_B1;
        break;

      case 2:
        bSel->bands |= WALTER_MODEM_BAND_B2;
        break;

      case 3:
        bSel->bands |= WALTER_MODEM_BAND_B3;
        break;

      case 4:
        bSel->bands |= WALTER_MODEM_BAND_B4;
        break;

      case 5:
        bSel->bands |= WALTER_MODEM_BAND_B5;
        break;

      case 8:
        bSel->bands |= WALTER_MODEM_BAND_B8;
        break;

      case 12:
        bSel->bands |= WALTER_MODEM_BAND_B12;
        break;

      case 13:
        bSel->bands |= WALTER_MODEM_BAND_B13;
        break;

      case 14:
        bSel->bands |= WALTER_MODEM_BAND_B14;
        break;

      case 17:
        bSel->bands |= WALTER_MODEM_BAND_B17;
        break;

      case 18:
        bSel->bands |= WALTER_MODEM_BAND_B18;
        break;

      case 19:
        bSel->bands |= WALTER_MODEM_BAND_B19;
        break;

      case 20:
        bSel->bands |= WALTER_MODEM_BAND_B20;
        break;

      case 25:
        bSel->bands |= WALTER_MODEM_BAND_B25;
        break;

      case 26:
        bSel->bands |= WALTER_MODEM_BAND_B26;
        break;

      case 28:
        bSel->bands |= WALTER_MODEM_BAND_B28;
        break;

      case 66:
        bSel->bands |= WALTER_MODEM_BAND_B66;
        break;

      case 71:
        bSel->bands |= WALTER_MODEM_BAND_B71;
        break;

      case 85:
        bSel->bands |= WALTER_MODEM_BAND_B85;
        break;
      }
    }

    goto after_processing_logic;
  }

  /* Clock read response */
  if(_buffStartsWith(buff, "+CCLK: \"")) {
    buff->data[buff->size - 1] = '\0';
    char* data = (char*) buff->data + _strLitLen("+CCLK: \"");
    cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_CLOCK;
    struct tm tm = {};
    int tz_offset = 0;
    char tz_sign = '+';

    // Parse time and timezone
    if(sscanf(data, "%2d/%2d/%2d,%2d:%2d:%2d%c%2d", &tm.tm_year, &tm.tm_mon, &tm.tm_mday,
              &tm.tm_hour, &tm.tm_min, &tm.tm_sec, &tz_sign, &tz_offset) != 8) {
    }
    tm.tm_year += 2000 - 1900; // years since 1900
    tm.tm_mon -= 1;            // months since January

    // Interpret the broken-down time as UTC, then apply the modem's reported offset.
    int64_t local_time = _timegm(&tm);

    // Convert quarter-hour offset (e.g. +08 = 8 * 900)
    int offset_seconds = tz_offset * 15 * 60;
    if(tz_sign == '+') {
      cmd->rsp->data.clock.epochTime = local_time - offset_seconds;
      cmd->rsp->data.clock.timeZoneOffset = tz_offset;

    } else {
      cmd->rsp->data.clock.epochTime = local_time + offset_seconds;
      cmd->rsp->data.clock.timeZoneOffset = -tz_offset;
    }

    if(cmd->rsp->data.clock.epochTime < WALTER_MODEM_MIN_VALID_TIMESTAMP) {
      cmd->rsp->data.clock.epochTime = -1;
      cmd->rsp->data.clock.timeZoneOffset = 0;
    }

    goto after_processing_logic;
  }

#pragma endregion // RSP_PROC_GENERAL
#pragma region RSP_PROC_GNSS
#if CONFIG_WALTER_MODEM_ENABLE_GNSS

  /* GNSS fix data event (URC) */
  if(_buffStartsWith(buff, "+LPGNSSFIXREADY: ")) {
    uint16_t dataSize = buff->size - _strLitLen("+LPGNSSFIXREADY: ");
    uint8_t* data = buff->data + _strLitLen("+LPGNSSFIXREADY: ");

    WMGNSSFixEvent fix = {};
    fix.satCount = 0;

    char* start = (char*) data;
    char* lastCharacter = NULL;
    uint8_t partNo = 0;
    bool parenthesisOpen = false;

    for(uint16_t i = 0; i < dataSize; ++i) {
      bool partComplete = false;

      if(data[i] == ',' && !parenthesisOpen) {
        data[i] = '\0';
        lastCharacter = (char*) data + i - 1;
        partComplete = true;
      } else if(i + 1 == dataSize) {
        data[i + 1] = '\0';
        lastCharacter = (char*) data + i;
        partComplete = true;
      } else if(data[i] == '(') {
        parenthesisOpen = true;
      } else if(data[i] == ')') {
        parenthesisOpen = false;
      }

      if(partComplete) {
        /* sometimes we need to skip surrounding " " */
        switch(partNo) {
        case 0:
          fix.fixId = atoi(start);
          break;

        case 1:
          *lastCharacter = '\0';
          fix.timestamp = strTotime(start + 1);
          break;

        case 2:
          fix.timeToFix = atoi(start);
          break;

        case 3:
          *lastCharacter = '\0';
          fix.estimatedConfidence = strtod(start + 1, NULL);
          break;

        case 4:
          *lastCharacter = '\0';
          fix.latitude = atof(start + 1);
          break;

        case 5:
          *lastCharacter = '\0';
          fix.longitude = atof(start + 1);
          break;

        case 6:
          *lastCharacter = '\0';
          fix.height = atof(start + 1);
          break;

        case 7:
          *lastCharacter = '\0';
          fix.northSpeed = atof(start + 1);
          break;

        case 8:
          *lastCharacter = '\0';
          fix.eastSpeed = atof(start + 1);
          break;

        case 9:
          *lastCharacter = '\0';
          fix.downSpeed = atof(start + 1);
          break;

        case 10:
          /* Raw satellite signal sample ignored — reset satellite count */
          fix.satCount = 0;
          break;

        default: {
          if(fix.satCount >= WALTER_MODEM_GNSS_MAX_SATS) {
            break;
          }

          const char* satNoStr = start + 1; /* skip '(' */
          const char* satSigStr = start;

          for(int j = 0; start[j] != '\0'; ++j) {
            if(start[j] == ',') {
              start[j] = '\0';
              satSigStr = start + j + 1;
              break;
            }
          }

          fix.sats[fix.satCount].satNo = atoi(satNoStr);
          fix.sats[fix.satCount].signalStrength = atoi(satSigStr);
          fix.satCount += 1;
          break;
        }
        }

        /* +1 for the comma */
        start = (char*) data + i + 1;
        partNo += 1;
      }
    }

    WalterModemEvent newEvent = {};
    newEvent.type = WalterModemEventType::WALTER_MODEM_EVENT_TYPE_GNSS;
    newEvent.timestamp = esp_timer_get_time();
    newEvent.gnss.event = WALTER_MODEM_GNSS_EVENT_FIX;
    newEvent.gnss.data.gnssfix = fix;
    xQueueSend(_eventQueue.handle, &newEvent, 0);
    goto after_processing_logic;
  }

  /* GNSS assistance response or URC */
  if(_buffStartsWith(buff, "+LPGNSSASSISTANCE: ")) {

    uint16_t dataSize = buff->size - _strLitLen("+LPGNSSASSISTANCE: ");
    uint8_t* data = buff->data + _strLitLen("+LPGNSSASSISTANCE: ");

    char* start = (char*) data;
    uint8_t partNo = 0;

    bool hasMultipleFields = false;

    WMGNSSAssistance assistance = {};
    assistance.available = false;
    assistance.lastUpdate = 0;
    assistance.timeToUpdate = 0;
    assistance.timeToExpire = 0;

    for(uint16_t i = 0; i < dataSize; ++i) {
      bool partComplete = false;

      if(data[i] == ',') {
        data[i] = '\0';
        hasMultipleFields = true;
        partComplete = true;
      } else if(i + 1 == dataSize) {
        data[i + 1] = '\0';
        partComplete = true;
      }

      if(partComplete) {
        switch(partNo) {
        case 0:
          assistance.type = (WMGNSSAssistanceType) atoi(start);
          break;

        case 1:
          assistance.available = atoi(start) == 1;
          break;

        case 2:
          assistance.lastUpdate = atoi(start);
          break;

        case 3:
          assistance.timeToUpdate = atoi(start);
          break;

        case 4:
          assistance.timeToExpire = atoi(start);
          break;

        default:
          break;
        }

        start = (char*) data + i + 1;
        partNo++;
      }
    }

    /* ---- URC: only assistance type ---- */
    if(!hasMultipleFields) {
      WalterModemEvent newEvent = {};
      newEvent.type = WALTER_MODEM_EVENT_TYPE_GNSS;
      newEvent.timestamp = esp_timer_get_time();
      newEvent.gnss.event = WALTER_MODEM_GNSS_EVENT_ASSISTANCE;
      newEvent.gnss.data.assistance = assistance.type;
      xQueueSend(_eventQueue.handle, &newEvent, 0);
      goto after_processing_logic;
    }

    /* ---- Command response: store by enum index ---- */
    if(assistance.type < WALTER_MODEM_GNSS_ASSISTANCE_TYPE_COUNT) {
      cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_GNSS_ASSISTANCE_DATA;
      cmd->rsp->data.gnssAssistance[assistance.type] = assistance;
    }

    goto after_processing_logic;
  }

  /* GNSS UTC time read response */
  if(_buffStartsWith(buff, "+LPGNSSUTCTIME: ")) {
    uint8_t* data = buff->data + _strLitLen("+LPGNSSUTCTIME: ");

    char* start = (char*) data;
    if(strstr(start + 1, "NO_CLOCK_DEFINED") != nullptr) {
      cmd->rsp->data.clock.epochTime = 0;
      result = WALTER_MODEM_STATE_NO_DATA;
    } else {
      cmd->rsp->data.clock.epochTime = strTotime(start + 1);
    }

    goto after_processing_logic;
  }

#endif // RSP_PROC_GNSS
#pragma endregion
#pragma region RSP_PROC_HTTP
#if CONFIG_WALTER_MODEM_ENABLE_HTTP

  /* HTTP data receive response */
  /* <<< is start of SQNHTTPRCV answer, there is no other header information available */
  if(_buffStartsWith(buff, "<<<")) {

    cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_HTTP;

    /*
     * The body was streamed into cmd->payload. Bytes left behind the marker, or dropped by the
     * sink, did not fit the requested size.
     */
    if(buff->size > 3 || (_parserData.sink.owner == cmd && _parserData.sink.dropped > 0)) {
      ESP_LOGW("WalterModem", "Unable to store HTTP payload (buffer to small)");
      result = WALTER_MODEM_STATE_NO_MEMORY;
    }

    goto after_processing_logic;
  }

  /* HTTP ring event (URC) */
  if(_buffStartsWith(buff, "+SQNHTTPRING: ")) {
    const char* rspStr = _buffStr(buff);
    char* commaPos = strchr(rspStr, ',');
    char* start = (char*) rspStr + _strLitLen("+SQNHTTPRING: ");

    uint8_t profileId = 0;
    uint16_t httpStatus = 0;
    char* contentType = NULL;
    uint16_t contentLength = 0;
    uint16_t contentTypeLen = 0;

    if(commaPos) {
      *commaPos = '\0';
      profileId = atoi(start);
      start = ++commaPos;
      commaPos = strchr(commaPos, ',');
    }

    if(commaPos) {
      *commaPos = '\0';
      httpStatus = atoi(start);
      start = ++commaPos;
      commaPos = strchr(commaPos, ',');
    }

    if(commaPos) {
      contentType = start;
      contentTypeLen = (size_t) (commaPos - start);
      *commaPos = '\0';
      contentLength = atoi(commaPos + 1);
    }

    WalterModemEvent newEvent = {};
    newEvent.type = WalterModemEventType::WALTER_MODEM_EVENT_TYPE_HTTP;
    newEvent.timestamp = esp_timer_get_time();
    newEvent.http.event = WALTER_MODEM_HTTP_EVENT_RING;
    newEvent.http.data.profile_id = profileId;
    newEvent.http.data.status = httpStatus;
    _strncpy_s(newEvent.http.data.content_type, contentType, contentTypeLen);
    newEvent.http.data.data_len = contentLength;
    xQueueSend(_eventQueue.handle, &newEvent, 0);
    goto after_processing_logic;
  }

  /* HTTP connect list response & HTTP connect event (URC) */
  if(_buffStartsWith(buff, "+SQNHTTPCONNECT: ")) {
    const char* rspStr = _buffStr(buff);
    char* commaPos = strchr(rspStr, ',');
    uint8_t profileId, resultCode;

    if(commaPos) {
      *commaPos = '\0';
      resultCode = atoi(commaPos + 1);
    } else {
      resultCode = 0;
    }

    profileId = atoi(rspStr + _strLitLen("+SQNHTTPCONNECT: "));

    if(profileId < WALTER_MODEM_MAX_HTTP_PROFILES) {
      if(resultCode == 0) {
        _httpContextSet[profileId].connected = true;
      } else {
        _httpContextSet[profileId].connected = false;
      }
    }

    WalterModemEvent newEvent = {};
    newEvent.type = WalterModemEventType::WALTER_MODEM_EVENT_TYPE_HTTP;
    newEvent.timestamp = esp_timer_get_time();
    newEvent.http.event = WALTER_MODEM_HTTP_EVENT_CONNECTED;
    newEvent.http.data.profile_id = profileId;
    newEvent.http.data.rc = resultCode;
    xQueueSend(_eventQueue.handle, &newEvent, 0);
    goto after_processing_logic;
  }

  /* HTTP disconnect list response & HTTP disconnect event (URC) */
  if(_buffStartsWith(buff, "+SQNHTTPDISCONNECT: ")) {
    const char* rspStr = _buffStr(buff);
    uint8_t profileId = atoi(rspStr + _strLitLen("+SQNHTTPDISCONNECT: "));

    if(profileId < WALTER_MODEM_MAX_HTTP_PROFILES) {
      _httpContextSet[profileId].connected = false;
    }

    WalterModemEvent newEvent = {};
    newEvent.type = WalterModemEventType::WALTER_MODEM_EVENT_TYPE_HTTP;
    newEvent.timestamp = esp_timer_get_time();
    newEvent.http.event = WALTER_MODEM_HTTP_EVENT_DISCONNECTED;
    newEvent.http.data.profile_id = profileId;
    xQueueSend(_eventQueue.handle, &newEvent, 0);
    goto after_processing_logic;
  }

  /* HTTP connection closed event (URC) */
  if(_buffStartsWith(buff, "+SQNHTTPSH: ")) {
    const char* rspStr = _buffStr(buff);
    char* commaPos = strchr(rspStr, ',');
    uint8_t profileId, resultCode;

    if(commaPos) {
      *commaPos = '\0';
      resultCode = atoi(commaPos + 1);
    } else {
      resultCode = 0;
    }

    profileId = atoi(rspStr + _strLitLen("+SQNHTTPSH: "));

    if(profileId < WALTER_MODEM_MAX_HTTP_PROFILES) {
      _httpContextSet[profileId].connected = false;
    }

    WalterModemEvent newEvent = {};
    newEvent.type = WalterModemEventType::WALTER_MODEM_EVENT_TYPE_HTTP;
    newEvent.timestamp = esp_timer_get_time();
    newEvent.http.event = WALTER_MODEM_HTTP_EVENT_CONNECTION_CLOSED;
    newEvent.http.data.profile_id = profileId;
    newEvent.http.data.rc = resultCode;
    xQueueSend(_eventQueue.handle, &newEvent, 0);
    goto after_processing_logic;
  }

#endif
#pragma endregion // RSP_PROC_HTTP
#pragma region RSP_PROC_COAP
#if CONFIG_WALTER_MODEM_ENABLE_COAP

  /* CoAP receive response */
  if(_buffStartsWith(buff, "+SQNCOAPRCV: ")) {
    const char* rspStr = _buffStr(buff);
    char* commaPos = strchr(rspStr, ',');
    char* start = (char*) rspStr + _strLitLen("+SQNCOAPRCV: ");
    uint8_t profileId = 0;
    uint16_t messageId = 0;

    WalterModemCoapSendType sendType = WALTER_MODEM_COAP_SEND_TYPE_CON;
    uint8_t reqRspCodeRaw = 0;

    if(commaPos) {
      /* got prof_id */
      *commaPos = '\0';
      profileId = atoi(start);
      start = ++commaPos;
      commaPos = strchr(commaPos, ',');
    }

    if(commaPos) {
      /* got msg_id */
      *commaPos = '\0';
      messageId = atoi(start);
      start = ++commaPos;
      commaPos = strchr(commaPos, ',');
    }

    if(commaPos) {
      /* got token */
      *commaPos = '\0';
      start = ++commaPos;
      commaPos = strchr(commaPos, ',');
    }

    if(commaPos) {
      /* got req_resp */
      *commaPos = '\0';
      start = ++commaPos;
      commaPos = strchr(commaPos, ',');
    }

    if(commaPos) {
      /* got type (con, noncon, ack, rst) */
      *commaPos = '\0';
      sendType = (WalterModemCoapSendType) atoi(start);
      start = ++commaPos;
      commaPos = strchr(commaPos, ',');
    }

    if(commaPos) {
      /* got rsp code (or method if we allow inbound requests) */
      *commaPos = '\0';
      reqRspCodeRaw = atoi(start);
      uint16_t length = atoi(commaPos + 1);

      if(profileId >= WALTER_MODEM_MAX_COAP_PROFILES) {
        result = WALTER_MODEM_STATE_ERROR;
        goto after_processing_logic;
      }

      cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_COAP;
      cmd->rsp->data.coapResponse.profileId = profileId;
      cmd->rsp->data.coapResponse.messageId = messageId;
      cmd->rsp->data.coapResponse.sendType = sendType;
      cmd->rsp->data.coapResponse.methodRsp = (WalterModemCoapSendMethodRsp) reqRspCodeRaw;

      /* The payload itself was streamed into cmd->payload */
      if(length > cmd->payloadSize) {
        cmd->rsp->data.coapResponse.length = cmd->payloadSize;
      } else {
        cmd->rsp->data.coapResponse.length = length;
      }
    }

    goto after_processing_logic;
  }

  /* CoAP ring event (URC) */
  if(_buffStartsWith(buff, "+SQNCOAPRING: ")) {
    const char* rspStr = _buffStr(buff);
    char* commaPos = strchr(rspStr, ',');
    char* start = (char*) rspStr + _strLitLen("+SQNCOAPRING: ");

    char* profileIdStr = NULL;
    char* messageIdStr = NULL;
    char* reqOrRspStr = NULL;
    char* sendTypeStr = NULL;

    if(commaPos) {
      *commaPos = '\0';
      profileIdStr = start;
      start = ++commaPos;
      commaPos = strchr(commaPos, ',');
    }

    if(commaPos) {
      *commaPos = '\0';
      messageIdStr = start;
      start = ++commaPos;
      commaPos = strchr(commaPos, ',');
    }

    if(commaPos) {
      *commaPos = '\0';
      reqOrRspStr = start;
      start = ++commaPos;
      commaPos = strchr(commaPos, ',');
    }

    if(commaPos) {
      *commaPos = '\0';
      sendTypeStr = start;
      start = ++commaPos;
      commaPos = strchr(commaPos, ',');
    }

    if(commaPos) {
      *commaPos = '\0';
    }

    /* convert parameters to int */
    uint8_t profileId = atoi(profileIdStr);
    uint16_t messageId = atoi(messageIdStr);
    uint8_t reqOrRsp = atoi(reqOrRspStr);
    WalterModemCoapSendType sendType = (WalterModemCoapSendType) atoi(sendTypeStr);

    WalterModemEvent newEvent = {};
    newEvent.type = WalterModemEventType::WALTER_MODEM_EVENT_TYPE_COAP;
    newEvent.timestamp = esp_timer_get_time();
    newEvent.coap.event = WALTER_MODEM_COAP_EVENT_RING;
    newEvent.coap.data.profile_id = profileId;
    newEvent.coap.data.msg_id = messageId;
    newEvent.coap.data.req_rsp = reqOrRsp == 1;
    newEvent.coap.data.type = sendType;
    xQueueSend(_eventQueue.handle, &newEvent, 0);
    goto after_processing_logic;
  }

  /* CoAP connected event (URC) */
  else if(_buffStartsWith(buff, "+SQNCOAPCONNECTED: ")) {
    const char* rspStr = _buffStr(buff);
    char* commaPos = strchr(rspStr, ',');
    if(commaPos) {
      *commaPos = '\0';
    }

    uint8_t profileId = atoi(rspStr + _strLitLen("+SQNCOAPCONNECTED: "));

    if(profileId < WALTER_MODEM_MAX_COAP_PROFILES) {
      _coapContextSet[profileId].connected = true;
    }

    WalterModemEvent newEvent = {};
    newEvent.type = WalterModemEventType::WALTER_MODEM_EVENT_TYPE_COAP;
    newEvent.timestamp = esp_timer_get_time();
    newEvent.coap.event = WALTER_MODEM_COAP_EVENT_CONNECTED;
    newEvent.coap.data.profile_id = profileId;
    xQueueSend(_eventQueue.handle, &newEvent, 0);
    goto after_processing_logic;
  }

  /* CoAP closed event (URC) */
  if(_buffStartsWith(buff, "+SQNCOAPCLOSED: ")) {
    const char* rspStr = _buffStr(buff);
    char* commaPos = strchr(rspStr, ',');
    char* start = (char*) rspStr + _strLitLen("+SQNCOAPCLOSED: ");

    uint8_t profileId = 0;
    char* reason = NULL;
    uint16_t reasonLen = 0;

    if(commaPos) {
      *commaPos = '\0';
      profileId = atoi(start);
      start = ++commaPos;
      reason = start;
      reasonLen = strlen(reason);
    }

    if(profileId < WALTER_MODEM_MAX_COAP_PROFILES) {
      _coapContextSet[profileId].connected = false;
    }

    WalterModemEvent newEvent = {};
    newEvent.type = WalterModemEventType::WALTER_MODEM_EVENT_TYPE_COAP;
    newEvent.timestamp = esp_timer_get_time();
    newEvent.coap.event = WALTER_MODEM_COAP_EVENT_CLOSED;
    newEvent.coap.data.profile_id = profileId;
    _strncpy_s(newEvent.coap.data.reason, reason, reasonLen);
    xQueueSend(_eventQueue.handle, &newEvent, 0);
    goto after_processing_logic;
  }

#endif
#pragma endregion // RSP_PROC_COAP
#pragma region RSP_PROC_SOCKETS
#if CONFIG_WALTER_MODEM_ENABLE_SOCKETS

  /* Socket closed event (URC) */
  if(_buffStartsWith(buff, "+SQNSH: ")) {
    const char* rspStr = _buffStr(buff);
    int sockId = atoi(rspStr + _strLitLen("+SQNSH: "));

    _socketGet(sockId)->state = WALTER_MODEM_SOCKET_STATE_FREE;

    WalterModemEvent newEvent = {};
    newEvent.type = WalterModemEventType::WALTER_MODEM_EVENT_TYPE_SOCKET;
    newEvent.timestamp = esp_timer_get_time();
    newEvent.socket.event = WALTER_MODEM_SOCKET_EVENT_DISCONNECTED;
    newEvent.socket.data.conn_id = sockId;
    xQueueSend(_eventQueue.handle, &newEvent, 0);
    goto after_processing_logic;
  }

  /* Socket ring event (URC) */
  if(_buffStartsWith(buff, "+SQNSRING: ")) {
    const char* rspStr = _buffStr(buff);
    char* start = (char*) rspStr + _strLitLen("+SQNSRING: ");
    int sockId = atoi(start);

    uint16_t dataReceived = 0;

    char* commaPos = strchr(start, ',');
    if(commaPos) {
      *commaPos = '\0';
      start = ++commaPos;
      dataReceived = atoi(commaPos);
    }

    WalterModemEvent newEvent = {};
    newEvent.type = WalterModemEventType::WALTER_MODEM_EVENT_TYPE_SOCKET;
    newEvent.timestamp = esp_timer_get_time();
    newEvent.socket.event = WALTER_MODEM_SOCKET_EVENT_RING;
    newEvent.socket.data.conn_id = sockId;
    newEvent.socket.data.data_len = dataReceived;
    xQueueSend(_eventQueue.handle, &newEvent, 0);
    goto after_processing_logic;
  }

  /* Socket receive response */
  if(_buffStartsWith(buff, "+SQNSRECV: ")) {
    const char* rspStr = _buffStr(buff);
    char* start = (char*) rspStr + _strLitLen("+SQNSRECV: ");

    int sockId = atoi(start);
    uint16_t dataReceived = 0;

    char* commaPos = strchr(start, ',');
    if(commaPos) {
      *commaPos = '\0';
      start = ++commaPos;
      dataReceived = atoi(start);
    }

    /* The payload itself was streamed into cmd->payload */
    cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_SOCKET;
    cmd->rsp->data.socketResponse.socketId = sockId;
    cmd->rsp->data.socketResponse.bytesReceived = dataReceived;

    goto after_processing_logic;
  }

  /* Socket status update response */
  if(_buffStartsWith(buff, "+SQNSS: ")) {
    const char* rspStr = _buffStr(buff);

    int sockId = 0;
    int status = 0;

    // Only parse the first two fields (sockId and status)
    int parsed = std::sscanf(rspStr, "+SQNSS: %d,%d", &sockId, &status);

    if(parsed == 2 && sockId < (WALTER_MODEM_MAX_SOCKETS - 1)) {
      WalterModemSocket* sock = _socketGet(sockId);
      if(sock != nullptr) {
        sock->state = (WalterModemSocketState) (status + 3);
      }
    }

    goto after_processing_logic;
  }

#endif
#pragma endregion // RSP_PROC_SOCKETS
#pragma region RSP_PROC_MQTT
#if CONFIG_WALTER_MODEM_ENABLE_MQTT

  /* MQTT connect event (URC) */
  if(_buffStartsWith(buff, "+SQNSMQTTONCONNECT:0,")) {
    const char* rspStr = _buffStr(buff);
    int status = atoi(rspStr + _strLitLen("+SQNSMQTTONCONNECT:0,"));

    _mqttStatus = (WMMQTTConnRC) status;

    WalterModemEvent newEvent = {};
    newEvent.type = WalterModemEventType::WALTER_MODEM_EVENT_TYPE_MQTT;
    newEvent.timestamp = esp_timer_get_time();
    newEvent.mqtt.event = WALTER_MODEM_MQTT_EVENT_CONNECTED;
    newEvent.mqtt.data.rc = _mqttStatus;
    xQueueSend(_eventQueue.handle, &newEvent, 0);
    goto after_processing_logic;
  }

  /* MQTT disconnect event (URC) */
  if(_buffStartsWith(buff, "+SQNSMQTTONDISCONNECT:0,")) {
    const char* rspStr = _buffStr(buff);
    int status = atoi(rspStr + _strLitLen("+SQNSMQTTONDISCONNECT:0,"));

    _mqttStatus = (WMMQTTConnRC) status;

    WalterModemEvent newEvent = {};
    newEvent.type = WalterModemEventType::WALTER_MODEM_EVENT_TYPE_MQTT;
    newEvent.timestamp = esp_timer_get_time();
    newEvent.mqtt.event = WALTER_MODEM_MQTT_EVENT_DISCONNECTED;
    newEvent.mqtt.data.rc = _mqttStatus;
    xQueueSend(_eventQueue.handle, &newEvent, 0);
    goto after_processing_logic;
  }

  /* MQTT publish event (URC) */
  if(_buffStartsWith(buff, "+SQNSMQTTONPUBLISH:0,")) {
    char* rsp = (char*) _buffStr(buff);
    char* p = rsp + _strLitLen("+SQNSMQTTONPUBLISH:0,");

    // message ID
    char* comma = strchr(p, ',');
    if(!comma)
      goto after_processing_logic;
    *comma = '\0';
    uint16_t messageId = atoi(p);
    p = comma + 1;

    // status
    int status = atoi(p);
    _mqttStatus = (WMMQTTConnRC) status;

    // prepare event
    WalterModemEvent newEvent = {};
    newEvent.type = WalterModemEventType::WALTER_MODEM_EVENT_TYPE_MQTT;
    newEvent.timestamp = esp_timer_get_time();
    newEvent.mqtt.event = WALTER_MODEM_MQTT_EVENT_PUBLISHED;
    newEvent.mqtt.data.rc = _mqttStatus;
    newEvent.mqtt.data.mid = messageId;
    xQueueSend(_eventQueue.handle, &newEvent, 0);
    goto after_processing_logic;
  }

  /* MQTT subscribe event (URC) */
  if(_buffStartsWith(buff, "+SQNSMQTTONSUBSCRIBE:0,")) {
    char* rsp = (char*) _buffStr(buff);
    char* p = rsp + _strLitLen("+SQNSMQTTONSUBSCRIBE:0,");

    // topic (quoted)
    if(*p != '"')
      goto after_processing_logic;
    char* topicStart = ++p;
    char* topicEnd = strchr(topicStart, '"');
    if(!topicEnd)
      goto after_processing_logic;
    *topicEnd = '\0';
    p = topicEnd + 1;

    if(*p != ',')
      goto after_processing_logic;
    p++; // skip comma

    // status
    char* statusStr = p;
    int status = atoi(statusStr);
    _mqttStatus = (WMMQTTConnRC) status;

    // prepare event
    WalterModemEvent newEvent = {};
    newEvent.type = WalterModemEventType::WALTER_MODEM_EVENT_TYPE_MQTT;
    newEvent.timestamp = esp_timer_get_time();
    newEvent.mqtt.event = WALTER_MODEM_MQTT_EVENT_SUBSCRIBED;
    _strncpy_s(newEvent.mqtt.data.topic, topicStart, WALTER_MODEM_MQTT_TOPIC_BUF_SIZE);
    newEvent.mqtt.data.rc = _mqttStatus;
    xQueueSend(_eventQueue.handle, &newEvent, 0);
    goto after_processing_logic;
  }

  /* MQTT memory full event (URC) */
  if(_buffStartsWith(buff, "+SQNSMQTTMEMORYFULL:")) {
    WalterModemEvent newEvent = {};
    newEvent.type = WalterModemEventType::WALTER_MODEM_EVENT_TYPE_MQTT;
    newEvent.timestamp = esp_timer_get_time();
    newEvent.mqtt.event = WALTER_MODEM_MQTT_EVENT_MEMORY_FULL;
    xQueueSend(_eventQueue.handle, &newEvent, 0);
    goto after_processing_logic;
  }

  /* MQTT receive message event (URC) */
  if(_buffStartsWith(buff, "+SQNSMQTTONMESSAGE:0,")) {
    char* rsp = (char*) _buffStr(buff);
    char* p = rsp + _strLitLen("+SQNSMQTTONMESSAGE:");

    // profile (must be 0)
    char* comma = strchr(p, ',');
    if(!comma)
      goto after_processing_logic;
    *comma = '\0';
    // atoi(p) == 0
    p = comma + 1;

    // topic (quoted)
    if(*p != '"')
      goto after_processing_logic;
    char* topicStart = ++p;
    char* topicEnd = strchr(topicStart, '"');
    if(!topicEnd)
      goto after_processing_logic;
    *topicEnd = '\0';
    p = topicEnd + 1;

    if(*p != ',')
      goto after_processing_logic;
    p++;

    // length
    char* lenStr = p;
    comma = strchr(p, ',');
    if(!comma)
      goto after_processing_logic;
    *comma = '\0';
    p = comma + 1;

    // qos
    char* qosStr = p;
    comma = strchr(p, ',');
    char* midStr = NULL;

    if(comma) {
      *comma = '\0';
      midStr = comma + 1;
    }

    uint16_t length = atoi(lenStr);
    uint8_t qos = atoi(qosStr);
    uint16_t messageId = midStr ? atoi(midStr) : 0;

    WalterModemEvent newEvent = {};
    newEvent.type = WalterModemEventType::WALTER_MODEM_EVENT_TYPE_MQTT;
    newEvent.timestamp = esp_timer_get_time();
    newEvent.mqtt.event = WALTER_MODEM_MQTT_EVENT_MESSAGE;
    _strncpy_s(newEvent.mqtt.data.topic, topicStart, WALTER_MODEM_MQTT_TOPIC_BUF_SIZE);
    newEvent.mqtt.data.msg_length = length;
    newEvent.mqtt.data.qos = qos;
    newEvent.mqtt.data.mid = messageId;
    xQueueSend(_eventQueue.handle, &newEvent, 0);
    goto after_processing_logic;
  }

  /* MQTT receive message response */
  /**
   * The modem response does not include a header, so this condition is triggered when the last
   * sent command is a MQTT receive message command. The payload itself was streamed into
   * cmd->payload.
   */
  if(cmd && cmd->atCmd[0] && !strcmp(cmd->atCmd[0], "AT+SQNSMQTTRCVMESSAGE=0,") &&
     cmd->rsp->type != WALTER_MODEM_RSP_DATA_TYPE_MQTT) {
    cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_MQTT;
    goto after_processing_logic;
  }

#endif
#pragma endregion // RSP_PROC_MQTT
#pragma region RSP_PROC_SIM_CARD

  /* SIM card IMSI response */
  if(_buffStartsWithDigit(buff)) {
    if(cmd == NULL) {
      buff->free = true;
      return;
    }

    cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_SIM_CARD_IMSI;

    int offset = 0;
    for(int i = 0; i < buff->size && offset < 15; ++i) {
      cmd->rsp->data.imsi[offset++] = buff->data[i];
    }

    cmd->rsp->data.imsi[offset++] = '\0';
    goto after_processing_logic;
  }

#pragma endregion // RSP_PROC_SIM_CARD
#pragma region RSP_PROC_FINISH
after_processing_logic:

  /**
   * If the message contains an expected response, or if an error occurred, finish the command
   * processing.
   */
  if((cmd != NULL && cmd->atRsp != NULL && memcmp(cmd->atRsp, buff->data, cmd->atRspLen) == 0) ||
     result != WALTER_MODEM_STATE_OK) {
    _finishModemCMD(cmd, result);
    buff->free = true;
    return;
  }

  /**
   * If the message doesn't contain an expected response, or if the received message is
   * unsolicited (URC or multi-part response), free the buffer and return.
   */

  buff->free = true;

#pragma endregion // RSP_PROC_FINISH
}

#pragma endregion // RSP_PROCESSING
#pragma region MOTA_BLUECHERRY
#if CONFIG_WALTER_MODEM_ENABLE_MOTA

uint16_t WalterModem::_calculateStpCrc16(const void* input, size_t length)
{
  uint16_t crc = 0;
  const uint8_t* data = (const uint8_t*) input;

  while(length-- > 0) {
    crc = (uint16_t) ((0xff & (crc >> 8)) | (crc << 8));
    crc ^= (uint16_t) *data++;
    crc ^= (uint16_t) ((crc & 0xff) >> 4);
    crc ^= (uint16_t) (crc << 12);
    crc ^= (uint16_t) ((uint8_t) (crc & 0xff) << 5);
  }

  /* ESP32 is little-endian but Monarch chip is big-endian */
  return _switchEndian16(crc);
}

bool WalterModem::_motaFormatAndMount(void)
{
  esp_err_t result;
  if(_wl_handle != WL_INVALID_HANDLE) {
    ESP_LOGD("WalterModem", "Trying to format FAT partition but it was already mounted");
    if(_mota_file_ptr) {
      fclose(_mota_file_ptr);
      _mota_file_ptr = NULL;
    }
    return true;
  }

  _mota_file_ptr = NULL;

  /* Attempt to mount to see if there is already data */
  wl_handle_t temp_handle;
  const esp_partition_t* ffat_partition =
      esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_FAT, "ffat");

  if(!ffat_partition) {
    ESP_LOGD("WalterModem", "No FAT partition found to store new modem firmware in");
    return false;
  }
  result = wl_mount(ffat_partition, &temp_handle);

  if(result == ESP_OK) {
    /* Wipe disk- we just wipe the FAT, not the data */
    wl_erase_range(temp_handle, 0, 16384);
    wl_unmount(temp_handle);
  } else {
    ESP_LOGD("WalterModem", "Erase FAT partition failed");
    return false;
  }

  /* Now do a mount with format_if_fail (which it will) */
  esp_vfs_fat_mount_config_t conf = {};
  conf.format_if_mount_failed = true;
  conf.max_files = 1;
  conf.allocation_unit_size = CONFIG_WL_SECTOR_SIZE;

  result = esp_vfs_fat_spiflash_mount_rw_wl("/ffat", "ffat", &conf, &_wl_handle);
  if(result != ESP_OK) {
    ESP_LOGD("WalterModem", "Mount/format FAT partition failed!");
    _wl_handle = WL_INVALID_HANDLE;
    return false;
  }

  return true;
}

bool WalterModem::_processMotaInitializeEvent(uint8_t* data, uint16_t len)
{
  if(len != sizeof(uint32_t)) {
    return true;
  }

  _motaSize = *((uint32_t*) data);

  if(!_motaFormatAndMount()) {
    ESP_LOGD("WalterModem", "MOTA initialize (format partition) failed");
    return true;
  }

  _mota_file_ptr = fopen("/ffat/mota.dup", "w+");

  /* Initialize bytes written counter */
  _motaProgress = 0;

  ESP_LOGD("WalterModem", "MOTA procedure initialized");

  return false;
}

bool WalterModem::_processMotaChunkEvent(uint8_t* data, uint16_t len)
{
  if(!_motaSize || len == 0 || _motaProgress + len > _motaSize) {
    ESP_LOGD("WalterModem", "MOTA: cancelled because empty chunk or chunk beyond update size");
    return true;
  }

  if(!_mota_file_ptr) {
    ESP_LOGD("WalterModem", "Bug: MOTA file not open!");
    return true;
  }

  if(fwrite(data, 1, len, _mota_file_ptr) != len) {
    fclose(_mota_file_ptr);
    _mota_file_ptr = NULL;
    ESP_LOGD("WalterModem", "Error: short write to MOTA update file");
    return true;
  }

  _motaProgress += len;

  ESP_LOGD("WalterModem",
           "MOTA: appending new chunk of %d bytes to ESP32 flash; so far %" PRIu32 "/%" PRIu32
           " bytes written",
           len, _motaProgress, _motaSize);

  return false;
}

bool WalterModem::_processMotaFinishEvent(void)
{
  if(!_motaSize || _motaProgress != _motaSize || !_mota_file_ptr) {
    ESP_LOGD("WalterModem", "MOTA error: incomplete or missing dup file");
    return true;
  }

  /* prepare modem, disable rx until done so we can talk with modem directly */
  uint16_t blockSize = _modemFirmwareUpgradeStart();

  if(blockSize == 0) {
    ESP_LOGD("WalterModem", "MOTA error: STP session did not open");
    _modemFirmwareUpgradeFinish(false);
    return true;
  }

  /* Borrowed for the duration of the transfer. Safe because the rx handler is muted from
   * _modemFirmwareUpgradeStart until _modemFirmwareUpgradeFinish, so no OTA chunk can arrive and
   * stage into it while we are here. */
  uint8_t* block = WalterBlueCherry::_otaBuffer();

  /* Every block declares its own length, so transferring less than the modem offered only costs
   * round trips. Clamping to the staging buffer is what keeps the fread below in bounds. */
  if(blockSize > BLUECHERRY_OTA_BUFFER_SIZE) {
    blockSize = (uint16_t) BLUECHERRY_OTA_BUFFER_SIZE;
  }

  fseek(_mota_file_ptr, 0L, SEEK_SET);

  uint32_t transactionId = 2;
  long bytesLeft = _motaSize;

  while(bytesLeft > 0) {
    size_t bytesRead = fread(block, 1, blockSize, _mota_file_ptr);
    if(bytesRead <= 0) {
      break;
    }

    /* send this chunk to the modem */
    _modemFirmwareUpgradeBlock(block, bytesRead, transactionId);
    transactionId += 2;

    /* next block */
    bytesLeft -= bytesRead;
    ESP_LOGD("WalterModem",
             "sent chunk of %d bytes from ESP flash to modem; so far %" PRIu32 "/%" PRIu32
             " bytes sent",
             bytesRead, _motaSize - bytesLeft, _motaSize);

    tickleWatchdog();
  }

  /* boot modem into new firmware if valid, reenable rx, resume queue processing */
  fclose(_mota_file_ptr);
  _mota_file_ptr = NULL;
  _modemFirmwareUpgradeFinish(bytesLeft == 0);
  tickleWatchdog();

  if(bytesLeft != 0) {
    ESP_LOGD("WalterModem", "MOTA Error: invalid dup file size or read failed");
    return true;
  } else {
    ESP_LOGD("WalterModem", "MOTA Finished, rebooted into new modem firmware");
    return false;
  }
}

void WalterModem::offlineMotaUpgrade()
{
  if(_wl_handle == WL_INVALID_HANDLE) {
    esp_err_t result;
    esp_vfs_fat_mount_config_t conf = {};
    conf.format_if_mount_failed = false;
    conf.max_files = 1;
    conf.allocation_unit_size = CONFIG_WL_SECTOR_SIZE;

    result = esp_vfs_fat_spiflash_mount_rw_wl("/ffat", "ffat", &conf, &_wl_handle);
    if(result != ESP_OK) {
      ESP_LOGD("WalterModem", "Mount FAT partition failed!");
      _wl_handle = WL_INVALID_HANDLE;
    }
  }
  if(_wl_handle != WL_INVALID_HANDLE) {
    if(!_mota_file_ptr) {
      _mota_file_ptr = fopen("/ffat/mota.dup", "r");
    }

    if(!_mota_file_ptr) {
      ESP_LOGD("WalterModem", "Could not open MOTA update package");
    } else {
      fseek(_mota_file_ptr, 0L, SEEK_END);
      _motaProgress = _motaSize = ftell(_mota_file_ptr);
      fseek(_mota_file_ptr, 0L, SEEK_SET);

      if(_processMotaFinishEvent()) {
        ESP_LOGD("WalterModem", "MOTA offline update failed");
      } else {
        ESP_LOGD("WalterModem", "MOTA offline update succeeded");
      }
    }
  }
}

#endif
#pragma endregion
#pragma region TLS

bool WalterModem::tlsWriteCredential(bool isPrivateKey, uint8_t slotIdx, const char* credential)
{
  WalterModemRsp* rsp = NULL;
  walterModemCb cb = NULL;
  void* args = NULL;

  const char* keyType = isPrivateKey ? "privatekey" : "certificate";

  _runCmd(
      arr("AT+SQNSNVW=", _atStr(keyType), ",", _atNum(slotIdx), ",", _atNum(strlen(credential))),
      "OK", rsp, cb, args, NULL, NULL, WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT, (uint8_t*) credential,
      strlen(credential));

  _returnAfterReply();
}

bool WalterModem::_tlsIsCredentialPresent(bool isPrivateKey, uint8_t slotIdx)
{
  WalterModemRsp* rsp = NULL;
  walterModemCb cb = NULL;
  void* args = NULL;

  const char* keyType = isPrivateKey ? "privatekey" : "certificate";
  _runCmd(arr("AT+SQNSNVR=", _atStr(keyType), ",", _atNum(slotIdx)), "+SQNSNVR: ", rsp, cb, args);
  _returnAfterReply();
}

char WalterModem::_getLuhnChecksum(const char* imei)
{
  int sum = 0;

  for(int i = 13; i >= 0; --i) {
    int digit = imei[i] - '0';

    if((13 - i) % 2 == 0) {
      int double_digit = digit * 2;
      if(double_digit > 9) {
        double_digit -= 9;
      }
      sum += double_digit;
    } else {
      sum += digit;
    }
  }

  return (char) (((10 - (sum % 10)) % 10) + '0');
}

#pragma endregion
#pragma region EVENTS

void WalterModem::_checkEventDuration(
    const std::chrono::time_point<std::chrono::steady_clock>& start)
{
  auto end = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

  if(elapsedTime > WALTER_MODEM_MAX_EVENT_DURATION_MS) {
    ESP_LOGW("WalterModem",
             "The event handler took %lldms to complete. Consider offloading heavy processing to "
             "another task.",
             static_cast<long long>(elapsedTime));
  }
}

void WalterModem::_dispatchEvent(WalterModemEvent* ev)
{
  WalterModemEventHandler* handler = _eventHandlers;
  auto start = std::chrono::steady_clock::now();

  switch(ev->type) {
  case WALTER_MODEM_EVENT_TYPE_NETWORK:
    handler += WALTER_MODEM_EVENT_TYPE_NETWORK;
    if(handler->netHandler != nullptr) {
      handler->netHandler(ev->network.event, &ev->network.data, handler->args);
    }
    break;

#if CONFIG_WALTER_MODEM_ENABLE_GNSS

  case WALTER_MODEM_EVENT_TYPE_GNSS:
    handler += WALTER_MODEM_EVENT_TYPE_GNSS;
    if(handler->gnssHandler != nullptr) {
      handler->gnssHandler(ev->gnss.event, &ev->gnss.data, handler->args);
    }
    break;

#endif
#if CONFIG_WALTER_MODEM_ENABLE_MQTT

  case WALTER_MODEM_EVENT_TYPE_MQTT:
    handler += WALTER_MODEM_EVENT_TYPE_MQTT;
    if(handler->mqttHandler != nullptr) {
      handler->mqttHandler(ev->mqtt.event, &ev->mqtt.data, handler->args);
    }
    break;

#endif
#if CONFIG_WALTER_MODEM_ENABLE_HTTP

  case WALTER_MODEM_EVENT_TYPE_HTTP:
    handler += WALTER_MODEM_EVENT_TYPE_HTTP;
    if(handler->httpHandler != nullptr) {
      handler->httpHandler(ev->http.event, &ev->http.data, handler->args);
    }
    break;

#endif
#if CONFIG_WALTER_MODEM_ENABLE_COAP

  case WALTER_MODEM_EVENT_TYPE_COAP:
    handler += WALTER_MODEM_EVENT_TYPE_COAP;
    if(handler->coapHandler != nullptr) {
      handler->coapHandler(ev->coap.event, &ev->coap.data, handler->args);
    }
    break;

#endif
#if CONFIG_WALTER_MODEM_ENABLE_SOCKETS

  case WALTER_MODEM_EVENT_TYPE_SOCKET:
    handler += WALTER_MODEM_EVENT_TYPE_SOCKET;
    if(handler->socketHandler != nullptr) {
      handler->socketHandler(ev->socket.event, &ev->socket.data, handler->args);
    }
    break;

#endif

  case WALTER_MODEM_EVENT_TYPE_TEMPERATURE:
    handler += WALTER_MODEM_EVENT_TYPE_TEMPERATURE;
    if(handler->tempHandler != nullptr) {
      handler->tempHandler(&ev->temperature, handler->args);
    }
    break;

  case WALTER_MODEM_EVENT_TYPE_VOLTAGE:
    handler += WALTER_MODEM_EVENT_TYPE_VOLTAGE;
    if(handler->voltageHandler != nullptr) {
      handler->voltageHandler(&ev->voltage, handler->args);
    }
    break;

  default:
    break;
  }

  _checkEventDuration(start);
}

#pragma endregion
#pragma region MODEM_SLEEP

void WalterModem::_sleepPrepare()
{
  memcpy(_pdpCtxSetRTC, _pdpCtxSet,
         WALTER_MODEM_MAX_PDP_CTXTS * sizeof(walter_modem_pdp_context_t));

#if CONFIG_WALTER_MODEM_ENABLE_COAP

  memcpy(_coapCtxSetRTC, _coapContextSet,
         WALTER_MODEM_MAX_COAP_PROFILES * sizeof(WalterModemCoapContext));

#endif
#if CONFIG_WALTER_MODEM_ENABLE_MQTT

  memcpy(_mqttTopicSetRTC, _mqttTopics,
         WALTER_MODEM_MQTT_MAX_TOPICS * sizeof(WalterModemMqttTopic));

#endif
#if CONFIG_WALTER_MODEM_ENABLE_SOCKETS

  memcpy(_socketCtxSetRTC, _socketSet, WALTER_MODEM_MAX_SOCKETS * sizeof(WalterModemSocket));

#endif
#if CONFIG_BLUECHERRY_ENABLE

  WalterBlueCherry::_sleepPrepare();

#endif
}

void WalterModem::_sleepWakeup()
{

  WalterModem::checkComm();

  memcpy(_pdpCtxSet, _pdpCtxSetRTC,
         WALTER_MODEM_MAX_PDP_CTXTS * sizeof(walter_modem_pdp_context_t));

#if CONFIG_WALTER_MODEM_ENABLE_COAP

  memcpy(_coapContextSet, _coapCtxSetRTC,
         WALTER_MODEM_MAX_COAP_PROFILES * sizeof(WalterModemCoapContext));

#endif
#if CONFIG_WALTER_MODEM_ENABLE_MQTT

  memcpy(_mqttTopics, _mqttTopicSetRTC,
         WALTER_MODEM_MQTT_MAX_TOPICS * sizeof(WalterModemMqttTopic));

#endif

  for(size_t i = 0; i < WALTER_MODEM_MAX_PDP_CTXTS; i++) {
    if(_pdpCtxSet[i].state == WALTER_MODEM_PDP_CONTEXT_STATE_ACTIVE) {
      _pdpCtx = _pdpCtxSet + i;
    }
  }

  /* Asked for its side effect: the modem stayed registered across the sleep but _regState and
   * _networkAttached did not survive it, and a URC only arrives when something changes. Runs after
   * the contexts are restored, so the answer refreshes their attach state too. */
  WalterModem::getNetworkRegState();

#if CONFIG_WALTER_MODEM_ENABLE_SOCKETS

  memcpy(_socketSet, _socketCtxSetRTC, WALTER_MODEM_MAX_SOCKETS * sizeof(WalterModemSocket));
  WalterModem::socketGetState();

#endif
}

#pragma endregion
#pragma endregion
#pragma region PUBLIC_METHODS
#pragma region BEGIN
#ifdef ARDUINO

bool WalterModem::begin(HardwareSerial* uart, uint16_t watchdogTimeout)

#else

bool WalterModem::begin(uart_port_t uartNo, uint16_t watchdogTimeout)

#endif

{
  if(_initialized) {
    return true;
  }

  _watchdogTimeout = watchdogTimeout;
  if(_watchdogTimeout) {
    /* wdt timeout must be longer than max wait time for a modem response */
    if(_watchdogTimeout * 1000UL < WALTER_MODEM_CMD_TIMEOUT_MS + 5000UL) {
      _watchdogTimeout = (uint16_t) ((WALTER_MODEM_CMD_TIMEOUT_MS / 1000UL) + 5);
    }

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)

    esp_task_wdt_init(_watchdogTimeout, true);

#else

    esp_task_wdt_config_t twdt_config = { .timeout_ms = (uint32_t) (_watchdogTimeout * 1000UL),
                                          .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
                                          .trigger_panic = true };

#if CONFIG_ESP_TASK_WDT_INIT

    esp_task_wdt_reconfigure(&twdt_config);

#else

    esp_task_wdt_init(&twdt_config);

#endif
#endif

    esp_task_wdt_add(NULL);
  }

  _taskQueue.handle =
      xQueueCreateStatic(WALTER_MODEM_CMD_QUEUE_MAX_ITEMS, sizeof(walter_modem_cmd_queue_t),
                         _taskQueue.mem, &(_taskQueue.memHandle));

  _eventQueue.handle =
      xQueueCreateStatic(WALTER_MODEM_EVENT_QUEUE_MAX_ITEMS, sizeof(WalterModemEvent),
                         _eventQueue.mem, &(_eventQueue.memHandle));

  gpio_set_direction((gpio_num_t) WALTER_MODEM_PIN_RESET, GPIO_MODE_OUTPUT);
  gpio_set_pull_mode((gpio_num_t) WALTER_MODEM_PIN_RESET, GPIO_FLOATING);
  gpio_deep_sleep_hold_en();

#ifdef ARDUINO

  _uart = uart;
  _uart->setRxBufferSize(UART_BUF_SIZE * 2);
  _uart->begin(WALTER_MODEM_BAUD, SERIAL_8N1, WALTER_MODEM_PIN_RX, WALTER_MODEM_PIN_TX);

  _uart->setPins(WALTER_MODEM_PIN_RX, WALTER_MODEM_PIN_TX, WALTER_MODEM_PIN_CTS,
                 WALTER_MODEM_PIN_RTS);

  _uart->setHwFlowCtrlMode(UART_HW_FLOWCTRL_CTS_RTS, UART_BUF_THRESHOLD);
  _uart->onReceive(_handleRxData);

#else

  // the initialization is done this way because otherwise we get warnings

  uart_config_t uart_config = {};
  uart_config.baud_rate = WALTER_MODEM_BAUD;
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.parity = UART_PARITY_DISABLE;
  uart_config.stop_bits = UART_STOP_BITS_1;
  uart_config.flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS;
  uart_config.rx_flow_ctrl_thresh = (uint8_t) (UART_BUF_THRESHOLD);
  uart_config.source_clk = UART_SCLK_DEFAULT;

  _uartNo = uartNo;
  uart_driver_install(uartNo, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
  uart_param_config(uartNo, &uart_config);
  uart_set_pin(uartNo, WALTER_MODEM_PIN_TX, WALTER_MODEM_PIN_RX, WALTER_MODEM_PIN_RTS,
               WALTER_MODEM_PIN_CTS);

  xTaskCreateStaticPinnedToCore(_handleRxData, "uart_rx_task", WALTER_MODEM_CMD_TASK_STACK_SIZE,
                                NULL, 3, _rxTaskStack, &_rxTaskBuf, 0);

#endif
#ifdef ARDUINO

  /* the queueProcessingTask cannot be on the same level as the UART task otherwise a modem freeze
   * can occur */
  _queueTask = xTaskCreateStaticPinnedToCore(_cmdProcessingTask, "queueProcessingTask",
                                             WALTER_MODEM_CMD_TASK_STACK_SIZE, NULL, 2,
                                             _queueTaskStack, &_queueTaskBuf, 1);

  _EventTask = xTaskCreateStaticPinnedToCore(_eventProcessingTask, "eventProcessingTask",
                                             WALTER_MODEM_EVENT_TASK_STACK_SIZE, NULL, 4,
                                             _eventTaskStack, &_eventTaskBuf, 1);

#else

  _queueTask = xTaskCreateStaticPinnedToCore(_cmdProcessingTask, "queueProcessingTask",
                                             WALTER_MODEM_CMD_TASK_STACK_SIZE, NULL, 2,
                                             _queueTaskStack, &_queueTaskBuf, 0);

  _EventTask = xTaskCreateStaticPinnedToCore(_eventProcessingTask, "eventProcessingTask",
                                             WALTER_MODEM_EVENT_TASK_STACK_SIZE, NULL, 4,
                                             _eventTaskStack, &_eventTaskBuf, 0);

#endif

  esp_sleep_wakeup_cause_t wakeupReason;
  wakeupReason = esp_sleep_get_wakeup_cause();

  if(wakeupReason == ESP_SLEEP_WAKEUP_UNDEFINED) {
    if(!reset()) {
      return false;
    }
  } else {
    _sleepWakeup();
  }

  /* Configure reports as we expect them in the library */
  if(!configCMEErrorReports()) {
    return false;
  }

  if(!configCEREGReports(WALTER_MODEM_CEREG_REPORTS_ENABLED_UE_PSM_WITH_LOCATION_EMM_CAUSE)) {
    return false;
  }

  _initialized = true;
  return true;
}

#pragma endregion
#pragma region GENERAL

void WalterModem::tickleWatchdog(void)
{
  if(_watchdogTimeout) {
    esp_task_wdt_reset();
  }
}

bool WalterModem::sendCmd(const char* at_cmd, const char* at_cmd_rsp, WalterModemRsp* rsp,
                          walterModemCb cb, void* args)
{
  _runCmd({ at_cmd }, at_cmd_rsp, rsp, cb, args);
  _returnAfterReply();
}

bool WalterModem::softReset(WalterModemRsp* rsp, walterModemCb cb, void* args)
{
  if(_parserData.buf != NULL) {
    _parserData.buf->free = true;
    _parserData.buf = NULL;
  }
  _receivingPayload = false;
  _disarmPayloadSink(NULL);

  _runCmd({ "AT^RESET" }, "+SYSSTART", rsp, cb, args);

  /* Also (re)initialize internal modem related library state */
  _regState = WALTER_MODEM_NETWORK_REG_NOT_SEARCHING;
  _opState = WALTER_MODEM_OPSTATE_MINIMUM;
  _networkSelMode = WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC;

  _pdpCtx = NULL;
  _simPIN = NULL;

  for(int i = 0; i < WALTER_MODEM_MAX_PDP_CTXTS; ++i) {
    _pdpCtxSet[i] = {};
  }

#if CONFIG_WALTER_MODEM_ENABLE_SOCKETS

  for(int i = 0; i < WALTER_MODEM_MAX_SOCKETS; ++i) {
    _socketSet[i] = {};
  }

#endif
#if CONFIG_WALTER_MODEM_ENABLE_COAP

  for(int i = 0; i < WALTER_MODEM_MAX_COAP_PROFILES; ++i) {
    _coapContextSet[i] = {};
  }

#endif

#if CONFIG_WALTER_MODEM_ENABLE_HTTP

  for(int i = 0; i < WALTER_MODEM_MAX_HTTP_PROFILES; ++i) {
    _httpContextSet[i] = {};
  }

#endif

  _operator = {};
  _returnAfterReply();
}

bool WalterModem::reset(WalterModemRsp* rsp, walterModemCb cb, void* args)
{
  _hardwareReset = true;
  gpio_hold_dis((gpio_num_t) WALTER_MODEM_PIN_RESET);
  gpio_set_level((gpio_num_t) WALTER_MODEM_PIN_RESET, 0);
  vTaskDelay(pdMS_TO_TICKS(10));
  gpio_set_level((gpio_num_t) WALTER_MODEM_PIN_RESET, 1);
  gpio_hold_en((gpio_num_t) WALTER_MODEM_PIN_RESET);
  vTaskDelay(pdMS_TO_TICKS(1000));

  if(_parserData.buf != NULL) {
    _parserData.buf->free = true;
    _parserData.buf = NULL;
  }
  _receivingPayload = false;
  _disarmPayloadSink(NULL);

  _runCmd({}, "+SYSSTART", rsp, cb, args, NULL, NULL, WALTER_MODEM_CMD_TYPE_TX_WAIT);
  _hardwareReset = false;

  /* Also (re)initialize internal modem related library state */
  _regState = WALTER_MODEM_NETWORK_REG_NOT_SEARCHING;
  _opState = WALTER_MODEM_OPSTATE_MINIMUM;
  _networkSelMode = WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC;

  _pdpCtx = NULL;
  _simPIN = NULL;

  for(int i = 0; i < WALTER_MODEM_MAX_PDP_CTXTS; ++i) {
    _pdpCtxSet[i] = {};
  }

#if CONFIG_WALTER_MODEM_ENABLE_SOCKETS

  for(int i = 0; i < WALTER_MODEM_MAX_SOCKETS; ++i) {
    _socketSet[i] = {};
  }

#endif
#if CONFIG_WALTER_MODEM_ENABLE_COAP

  for(int i = 0; i < WALTER_MODEM_MAX_COAP_PROFILES; ++i) {
    _coapContextSet[i] = {};
  }

#endif
#if CONFIG_WALTER_MODEM_ENABLE_HTTP

  for(int i = 0; i < WALTER_MODEM_MAX_HTTP_PROFILES; ++i) {
    _httpContextSet[i] = {};
  }

#endif

  _operator = {};
  _returnAfterReply();
}

void WalterModem::sleep(uint32_t sleep_time_s, bool is_light_sleep)
{
  if(is_light_sleep) {
    /* Disable RTS (make it high) so the modem can go to sleep */

#ifdef ARDUINO

    _uart->setHwFlowCtrlMode(UART_HW_FLOWCTRL_DISABLE);
    pinMode((gpio_num_t) WALTER_MODEM_PIN_RTS, OUTPUT);
    digitalWrite((gpio_num_t) WALTER_MODEM_PIN_RTS, HIGH);

#else

    uart_set_hw_flow_ctrl(_uartNo, UART_HW_FLOWCTRL_DISABLE, 0);
    gpio_set_direction((gpio_num_t) WALTER_MODEM_PIN_RTS, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t) WALTER_MODEM_PIN_RTS, 1);

#endif

    esp_sleep_enable_timer_wakeup(sleep_time_s * 1000000);
    esp_light_sleep_start();

    /* Re-enable RTS after waking up */

#ifdef ARDUINO

    _uart->setPins(WALTER_MODEM_PIN_RX, WALTER_MODEM_PIN_TX, WALTER_MODEM_PIN_CTS,
                   WALTER_MODEM_PIN_RTS);
    _uart->setHwFlowCtrlMode(UART_HW_FLOWCTRL_CTS_RTS, UART_BUF_THRESHOLD);

#else

    uart_set_pin(_uartNo, WALTER_MODEM_PIN_TX, WALTER_MODEM_PIN_RX, WALTER_MODEM_PIN_RTS,
                 WALTER_MODEM_PIN_CTS);
    uart_set_hw_flow_ctrl(_uartNo, UART_HW_FLOWCTRL_CTS_RTS, UART_BUF_THRESHOLD);

#endif

  } else {
    _sleepPrepare();
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_deep_sleep(sleep_time_s * 1000000);
  }
}

bool WalterModem::checkComm(WalterModemRsp* rsp, walterModemCb cb, void* args)
{
  _runCmd({ "AT" }, "OK", rsp, cb, args);
  _returnAfterReply();
}

bool WalterModem::configCMEErrorReports(WalterModemCMEErrorReportsType type, WalterModemRsp* rsp,
                                        walterModemCb cb, void* args)
{
  _runCmd(arr("AT+CMEE=", _digitStr(type)), "OK", rsp, cb, args);
  _returnAfterReply();
}

bool WalterModem::configCEREGReports(WalterModemCEREGReportsType type, WalterModemRsp* rsp,
                                     walterModemCb cb, void* args)
{
  _runCmd(arr("AT+CEREG=", _digitStr(type)), "OK", rsp, cb, args);
  _returnAfterReply();
}

bool WalterModem::getRSSI(WalterModemRsp* rsp, walterModemCb cb, void* args)
{
  _runCmd(arr("AT+CSQ"), "OK", rsp, cb, args);
  _returnAfterReply();
}

bool WalterModem::getSignalQuality(WalterModemRsp* rsp, walterModemCb cb, void* args)
{
  _runCmd(arr("AT+CESQ"), "OK", rsp, cb, args);
  _returnAfterReply();
}

bool WalterModem::getCellInformation(WalterModemSQNMONIReportsType type, WalterModemRsp* rsp,
                                     walterModemCb cb, void* args)
{
  _runCmd(arr("AT+SQNMONI=", _digitStr(type)), "OK", rsp, cb, args);
  _returnAfterReply();
}

bool WalterModem::getIdentity(WalterModemRsp* rsp, walterModemCb cb, void* args)
{
  _runCmd(arr("AT+CGSN=2"), "OK", rsp, cb, args);
  _returnAfterReply();
}

#pragma endregion

bool WalterModem::tlsConfigProfile(int profile_id, WalterModemTlsValidation tls_valid,
                                   WalterModemTlsVersion tls_version, uint8_t ca_cert_id,
                                   uint8_t client_ca_id, uint8_t client_priv_key_id,
                                   WalterModemRsp* rsp, walterModemCb cb, void* args)
{
  if(profile_id >= WALTER_MODEM_MAX_TLS_PROFILES) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
  }

  WalterModemBuffer* stringsBuffer = _getFreeBuffer();
  stringsBuffer->size += sprintf((char*) stringsBuffer->data, "AT+SQNSPCFG=%u,%d,\"\",%d,",
                                 profile_id, tls_version, tls_valid);
  if(ca_cert_id != 0xff) {
    stringsBuffer->size +=
        sprintf((char*) stringsBuffer->data + stringsBuffer->size, "%d", ca_cert_id);
  }
  stringsBuffer->size += sprintf((char*) stringsBuffer->data + stringsBuffer->size, ",");
  if(client_ca_id != 0xff) {
    stringsBuffer->size +=
        sprintf((char*) stringsBuffer->data + stringsBuffer->size, "%d", client_ca_id);
  }
  stringsBuffer->size += sprintf((char*) stringsBuffer->data + stringsBuffer->size, ",");
  if(client_priv_key_id != 0xff) {
    stringsBuffer->size +=
        sprintf((char*) stringsBuffer->data + stringsBuffer->size, "%d", client_priv_key_id);
  }
  stringsBuffer->size +=
      sprintf((char*) stringsBuffer->data + stringsBuffer->size, ",\"\",\"\",0,1,0");

  _runCmd(arr((const char*) stringsBuffer->data), "OK", rsp, cb, args, NULL, NULL,
          WALTER_MODEM_CMD_TYPE_TX_WAIT, NULL, 0, stringsBuffer);

  _returnAfterReply();
}

#pragma region MODEM_STATE

WalterModemNetworkRegState WalterModem::getNetworkRegState()
{
  WalterModemRsp* rsp = NULL;
  walterModemCb cb = NULL;
  void* args = NULL;

  const char* _cmdArr[WALTER_MODEM_COMMAND_MAX_ELEMS + 1] = arr("AT+CEREG?");
  WalterModemCmd* cmd = _queueModemCMD(_cmdArr, "OK", rsp, cb, args);
  if(cmd == NULL) {
    return WalterModemNetworkRegState::WALTER_MODEM_NETWORK_REG_NOT_SEARCHING;
  }

  std::unique_lock<std::mutex> lock { cmd->cmdLock.mutex };
  cmd->cmdLock.cond.wait(lock, [cmd] {
    return cmd->state == WALTER_MODEM_CMD_STATE_SYNC_LOCK_NOTIFIED;
    ;
  });
  cmd->state = WALTER_MODEM_CMD_STATE_COMPLETE;
  lock.unlock();

  return _regState;
}

bool WalterModem::getOpState(WalterModemRsp* rsp, walterModemCb cb, void* args)
{
  _runCmd({ "AT+CFUN?" }, "OK", rsp, cb, args);
  _returnAfterReply();
}

bool WalterModem::setOpState(WalterModemOpState op_state, WalterModemRsp* rsp, walterModemCb cb,
                             void* args)
{
  _runCmd(arr("AT+CFUN=", _digitStr(op_state)), "OK", rsp, cb, args);
  _returnAfterReply();
}

#pragma endregion
#pragma region RADIO

bool WalterModem::getRAT(WalterModemRsp* rsp, walterModemCb cb, void* args)
{
  if(_ratType != WALTER_MODEM_RAT_UNKNOWN) {
    if(rsp) {
      rsp->type = WALTER_MODEM_RSP_DATA_TYPE_RAT;
      rsp->data.rat = _ratType;
    }

    return true;
  }

  _runCmd(arr("AT+SQNMODEACTIVE?"), "OK", rsp, cb, args);
  _returnAfterReply();
}

bool WalterModem::setRAT(WalterModemRAT rat, WalterModemRsp* rsp, walterModemCb cb, void* args)
{
  _runCmd(arr("AT+SQNMODEACTIVE=", _digitStr(rat + 1)), "OK", rsp, cb, args);
  _returnAfterReply();
}

bool WalterModem::getRadioBands(WalterModemRsp* rsp, walterModemCb cb, void* args)
{
  _runCmd(arr("AT+SQNBANDSEL?"), "OK", rsp, cb, args);
  _returnAfterReply();
}

bool WalterModem::setRadioBands(WalterModemRAT rat, uint32_t bands, WalterModemRsp* rsp,
                                walterModemCb cb, void* args)
{
  WalterModemBuffer* stringsbuffer = _getFreeBuffer();
  stringsbuffer->size +=
      sprintf((char*) stringsbuffer->data, "AT+SQNBANDSEL=%d,\"standard\",\"", rat);

  if(bands & WALTER_MODEM_BAND_B1) {
    stringsbuffer->size += sprintf((char*) stringsbuffer->data + stringsbuffer->size, "1,");
  }
  if(bands & WALTER_MODEM_BAND_B2) {
    stringsbuffer->size += sprintf((char*) stringsbuffer->data + stringsbuffer->size, "2,");
  }
  if(bands & WALTER_MODEM_BAND_B3) {
    stringsbuffer->size += sprintf((char*) stringsbuffer->data + stringsbuffer->size, "3,");
  }
  if(bands & WALTER_MODEM_BAND_B4) {
    stringsbuffer->size += sprintf((char*) stringsbuffer->data + stringsbuffer->size, "4,");
  }
  if(bands & WALTER_MODEM_BAND_B5) {
    stringsbuffer->size += sprintf((char*) stringsbuffer->data + stringsbuffer->size, "5,");
  }
  if(bands & WALTER_MODEM_BAND_B8) {
    stringsbuffer->size += sprintf((char*) stringsbuffer->data + stringsbuffer->size, "8,");
  }
  if(bands & WALTER_MODEM_BAND_B12) {
    stringsbuffer->size += sprintf((char*) stringsbuffer->data + stringsbuffer->size, "12,");
  }
  if(bands & WALTER_MODEM_BAND_B13) {
    stringsbuffer->size += sprintf((char*) stringsbuffer->data + stringsbuffer->size, "13,");
  }
  if(bands & WALTER_MODEM_BAND_B14) {
    stringsbuffer->size += sprintf((char*) stringsbuffer->data + stringsbuffer->size, "14,");
  }
  if(bands & WALTER_MODEM_BAND_B17) {
    stringsbuffer->size += sprintf((char*) stringsbuffer->data + stringsbuffer->size, "17,");
  }
  if(bands & WALTER_MODEM_BAND_B18) {
    stringsbuffer->size += sprintf((char*) stringsbuffer->data + stringsbuffer->size, "18,");
  }
  if(bands & WALTER_MODEM_BAND_B19) {
    stringsbuffer->size += sprintf((char*) stringsbuffer->data + stringsbuffer->size, "19,");
  }
  if(bands & WALTER_MODEM_BAND_B20) {
    stringsbuffer->size += sprintf((char*) stringsbuffer->data + stringsbuffer->size, "20,");
  }
  if(bands & WALTER_MODEM_BAND_B25) {
    stringsbuffer->size += sprintf((char*) stringsbuffer->data + stringsbuffer->size, "25,");
  }
  if(bands & WALTER_MODEM_BAND_B26) {
    stringsbuffer->size += sprintf((char*) stringsbuffer->data + stringsbuffer->size, "26,");
  }
  if(bands & WALTER_MODEM_BAND_B28) {
    stringsbuffer->size += sprintf((char*) stringsbuffer->data + stringsbuffer->size, "28,");
  }
  if(bands & WALTER_MODEM_BAND_B66) {
    stringsbuffer->size += sprintf((char*) stringsbuffer->data + stringsbuffer->size, "66,");
  }
  if(bands & WALTER_MODEM_BAND_B71) {
    stringsbuffer->size += sprintf((char*) stringsbuffer->data + stringsbuffer->size, "71,");
  }
  if(bands & WALTER_MODEM_BAND_B85) {
    stringsbuffer->size += sprintf((char*) stringsbuffer->data + stringsbuffer->size, "85,");
  }

  if(stringsbuffer->data[stringsbuffer->size - 1] == ',') {
    stringsbuffer->data[stringsbuffer->size - 1] = '"';
  } else {
    stringsbuffer->size += sprintf((char*) stringsbuffer->data + stringsbuffer->size, "\"");
  }

  _runCmd(arr((const char*) stringsbuffer->data), "OK", rsp, cb, args);
  _returnAfterReply();
}

#pragma endregion
#pragma region SIM_MANAGEMENT

bool WalterModem::getSIMState(WalterModemRsp* rsp, walterModemCb cb, void* args)
{
  _runCmd({ "AT+CPIN?" }, "OK", rsp, cb, args);
  _returnAfterReply();
}

bool WalterModem::getSIMCardID(WalterModemRsp* rsp, walterModemCb cb, void* args)
{
  _runCmd({ "AT+SQNCCID" }, "OK", rsp, cb, args);
  _returnAfterReply();
}

bool WalterModem::getSIMCardIMSI(WalterModemRsp* rsp, walterModemCb cb, void* args)
{
  _runCmd({ "AT+CIMI" }, "OK", rsp, cb, args);
  _returnAfterReply();
}

bool WalterModem::unlockSIM(WalterModemRsp* rsp, walterModemCb cb, void* args, const char* pin)
{
  _simPIN = pin;

  if(_simPIN == NULL) {
    return getSIMState(rsp, cb, args);
  }

  _runCmd(arr("AT+CPIN=", _atStr(_simPIN)), "OK", rsp, cb, args);
  _returnAfterReply();
}

#pragma endregion

bool WalterModem::setNetworkSelectionMode(WalterModemNetworkSelMode mode, const char* operator_name,
                                          WalterModemOperatorFormat format, WalterModemRsp* rsp,
                                          walterModemCb cb, void* args)
{
  _networkSelMode = mode;
  _operator.format = format;
  _strncpy_s(_operator.name, operator_name, WALTER_MODEM_OPERATOR_MAX_SIZE);

  if(mode == WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC ||
     mode == WALTER_MODEM_NETWORK_SEL_MODE_UNREGISTER) {
    _runCmd(arr("AT+COPS=", _digitStr(mode)), "OK", rsp, cb, args);
    _returnAfterReply();
  } else {
    _runCmd(arr("AT+COPS=", _digitStr(_networkSelMode), ",", _digitStr(_operator.format), ",",
                _atStr(_operator.name)),
            "OK", rsp, cb, args);
    _returnAfterReply();
  }
}

#pragma region POWER_SAVING

bool WalterModem::configPSM(WalterModemPSMMode mode, const char* req_tau, const char* req_active,
                            WalterModemRsp* rsp, walterModemCb cb, void* args)
{
  if(mode == WALTER_MODEM_PSM_ENABLE) {
    WalterModemBuffer* stringsbuffer = _getFreeBuffer();
    _bufPrintf(stringsbuffer, "AT+CPSMS=1,,,\"%s\",\"%s\"", req_tau, req_active);

    _runCmd(arr((const char*) stringsbuffer->data), "OK", rsp, cb, args, NULL, NULL,
            WALTER_MODEM_CMD_TYPE_TX_WAIT, NULL, 0, stringsbuffer);
    _returnAfterReply();
  } else {
    _runCmd(arr("AT+CPSMS=", _digitStr(mode)), "OK", rsp, cb, args);
    _returnAfterReply();
  }
}

bool WalterModem::configEDRX(WalterModemEDRXMode mode, const char* req_edrx_val,
                             const char* req_ptw, WalterModemRsp* rsp, walterModemCb cb, void* args)
{
  if(mode == WALTER_MODEM_EDRX_ENABLE || mode == WALTER_MODEM_EDRX_ENABLE_WITH_RESULT) {
    getRAT(rsp, cb, args);

    WalterModemBuffer* stringsbuffer = _getFreeBuffer();
    _bufPrintf(stringsbuffer, "AT+SQNEDRX=%d,%d,\"%s\",\"%s\"", mode, _ratType + 4, req_edrx_val,
               req_ptw);

    _runCmd(arr((const char*) stringsbuffer->data), "OK", rsp, cb, args, NULL, NULL,
            WALTER_MODEM_CMD_TYPE_TX_WAIT, NULL, 0, stringsbuffer);
    _returnAfterReply();
  } else {
    _runCmd(arr("AT+SQNEDRX=", _digitStr(mode)), "OK", rsp, cb, args);
    _returnAfterReply();
  }
}

uint8_t WalterModem::_convertDuration(const uint32_t* base_times, size_t base_times_len,
                                      uint32_t duration_seconds, uint32_t* actual_duration_seconds)
{
  uint32_t smallest_modulo = UINT32_MAX;
  uint8_t final_base = 0;
  uint8_t final_mult = 0;

  for(uint8_t base = 0; base < base_times_len; ++base) {
    uint32_t multiplier = duration_seconds / base_times[base];
    if(multiplier == 0 || multiplier > 31) {
      continue;
    }

    uint32_t modulo = duration_seconds % base_times[base];
    if(modulo < smallest_modulo) {
      final_base = base;
      final_mult = multiplier;
    }
  }

  if(actual_duration_seconds) {
    *actual_duration_seconds = (uint32_t) final_base * (uint32_t) final_mult;
  }

  return (final_base << 5) | final_mult;
}

uint8_t WalterModem::durationToTAU(uint32_t seconds, uint32_t minutes, uint32_t hours,
                                   uint32_t* actual_duration_seconds)
{
  static const uint32_t base_times[] = { 600, 3600, 36000, 2, 30, 60, 1152000 };
  uint32_t duration_seconds = seconds + (60 * minutes) + (60 * 60 * hours);

  return _convertDuration(base_times, 7, duration_seconds, actual_duration_seconds);
}

uint8_t WalterModem::durationToActiveTime(uint32_t seconds, uint32_t minutes,
                                          uint32_t* actual_duration_seconds)
{
  static const uint32_t base_times[] = { 2, 60, 360 };
  uint32_t duration_seconds = seconds + (60 * minutes);

  return _convertDuration(base_times, 3, duration_seconds, actual_duration_seconds);
}

#pragma endregion
#pragma region PDP_CONTEXT

bool WalterModem::definePDPContext(
    int pdp_ctx_id, const char* apn, WalterModemRsp* rsp, walterModemCb cb, void* args,
    WalterModemPDPType type, const char* pdpAddress, WalterModemPDPHeaderCompression headerComp,
    WalterModemPDPDataCompression dataComp, WalterModemPDPIPv4AddrAllocMethod ipv4AllocMethod,
    WalterModemPDPRequestType requestType, WalterModemPDPPCSCFDiscoveryMethod pcscfMethod,
    bool forIMCN, bool useNSLPI, bool useSecurePCO, bool useNASIPv4MTUDiscovery,
    bool useLocalAddrInd, bool useNASNonIPMTUDiscovery)
{
  walter_modem_pdp_context_t* ctx = _pdpContextGet(pdp_ctx_id);
  if(ctx == NULL) {
    _returnState(WALTER_MODEM_STATE_NO_FREE_PDP_CONTEXT);
  }

  ctx->id = pdp_ctx_id;

  ctx->state = WALTER_MODEM_PDP_CONTEXT_STATE_NOT_ATTACHED;

  ctx->type = type;
  _strncpy_s(ctx->apn, apn, WALTER_MODEM_APN_MAX_SIZE);
  _strncpy_s(ctx->pdpAddress, pdpAddress, WALTER_MODEM_PDP_ADDR_MAX_SIZE);
  ctx->headerComp = headerComp;
  ctx->dataComp = dataComp;
  ctx->ipv4AllocMethod = ipv4AllocMethod;
  ctx->requestType = requestType;
  ctx->pcscfMethod = pcscfMethod;
  ctx->forIMCN = forIMCN;
  ctx->useNSLPI = useNSLPI;
  ctx->useSecurePCO = useSecurePCO;
  ctx->useNASIPv4MTUDiscovery = useNASIPv4MTUDiscovery;
  ctx->useLocalAddrInd = useLocalAddrInd;
  ctx->useNASNonIPMTUDiscovery = useNASNonIPMTUDiscovery;

  auto completeHandler = [](WalterModemCmd* cmd, WalterModemState result) {
    walter_modem_pdp_context_t* ctx = (walter_modem_pdp_context_t*) cmd->completeHandlerArg;

    cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_PDP_CTX_ID;
    cmd->rsp->data.pdpCtxId = ctx->id;

    if(result == WALTER_MODEM_STATE_OK) {
      ctx->state = WALTER_MODEM_PDP_CONTEXT_STATE_INACTIVE;
    }
  };

  _runCmd(arr("AT+CGDCONT=", _digitStr(ctx->id), ",", _atStr(_pdpTypeStr(ctx->type)), ",",
              _atStr(ctx->apn), ",", _atStr(ctx->pdpAddress), ",", _digitStr(ctx->dataComp), ",",
              _digitStr(ctx->headerComp), ",", _digitStr(ctx->ipv4AllocMethod), ",",
              _digitStr(ctx->requestType), ",", _digitStr(ctx->pcscfMethod), ",",
              _atBool(ctx->forIMCN), ",", _atBool(ctx->useNSLPI), ",", _atBool(ctx->useSecurePCO),
              ",", _atBool(ctx->useNASIPv4MTUDiscovery), ",", _atBool(ctx->useLocalAddrInd), ",",
              _atBool(ctx->useNASNonIPMTUDiscovery)),
          "OK", rsp, cb, args, completeHandler, ctx);
  _returnAfterReply();
}

bool WalterModem::setPDPAuthParams(WalterModemPDPAuthProtocol auth_proto, const char* auth_user,
                                   const char* auth_pass, int pdp_ctx_id, WalterModemRsp* rsp,
                                   walterModemCb cb, void* args)
{
  walter_modem_pdp_context_t* ctx = _pdpContextGet(pdp_ctx_id);
  if(ctx == NULL) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_PDP_CONTEXT);
  }

  if(ctx->auth_proto == WALTER_MODEM_PDP_AUTH_PROTO_NONE) {
    _returnState(WALTER_MODEM_STATE_OK);
  }

  ctx->auth_proto = auth_proto;
  _strncpy_s(ctx->auth_user, auth_user, WALTER_MODEM_PDP_AUTH_USER_MAX_SIZE);
  _strncpy_s(ctx->auth_pass, auth_pass, WALTER_MODEM_PDP_AUTH_PASS_MAX_SIZE);

  _runCmd(arr("AT+CGAUTH=", _digitStr(ctx->id), ",", _digitStr(ctx->auth_proto), ",",
              _atStr(ctx->auth_user), ",", _atStr(ctx->auth_pass)),
          "OK", rsp, cb, args);
  _returnAfterReply();
}

bool WalterModem::setPDPContextActive(bool active, int pdp_ctx_id, WalterModemRsp* rsp,
                                      walterModemCb cb, void* args)
{
  walter_modem_pdp_context_t* ctx = _pdpContextGet(pdp_ctx_id);
  if(ctx == NULL) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_PDP_CONTEXT);
  }

  auto completeHandler = [](WalterModemCmd* cmd, WalterModemState result) {
    walter_modem_pdp_context_t* ctx = (walter_modem_pdp_context_t*) cmd->completeHandlerArg;

    if(result == WALTER_MODEM_STATE_OK) {
      ctx->state = WALTER_MODEM_PDP_CONTEXT_STATE_ACTIVE;
      for(size_t i = 0; i < WALTER_MODEM_MAX_PDP_CTXTS; i++) {
        walter_modem_pdp_context_t* _ctx = _pdpContextGet(i);
        if(_ctx->id != ctx->id) {
          _ctx->state = WALTER_MODEM_PDP_CONTEXT_STATE_INACTIVE;
        }
      }
      /* Reset the active PDP context */
      _pdpCtx = ctx;
    }
  };

  _runCmd(arr("AT+CGACT=", _atBool(active), ",", _digitStr(ctx->id)), "OK", rsp, cb, args,
          completeHandler, ctx);
  _returnAfterReply();
}

bool WalterModem::setNetworkAttachmentState(bool attach, WalterModemRsp* rsp, walterModemCb cb,
                                            void* args)
{
  _runCmd(arr("AT+CGATT=", _atBool(attach)), "OK", rsp, cb, args, NULL, NULL,
          WALTER_MODEM_CMD_TYPE_TX_WAIT);
  _returnAfterReply();
}

bool WalterModem::getPDPAddress(WalterModemRsp* rsp, walterModemCb cb, void* args, int pdp_ctx_id)
{
  walter_modem_pdp_context_t* ctx = _pdpContextGet(pdp_ctx_id);
  if(ctx == NULL) {
    _returnState(WALTER_MODEM_STATE_NO_SUCH_PDP_CONTEXT);
  }

  _runCmd(arr("AT+CGPADDR=", _digitStr(ctx->id)), "OK", rsp, cb, args);
  _returnAfterReply();
}

#pragma endregion

bool WalterModem::getClock(WalterModemRsp* rsp, walterModemCb cb, void* args)
{
  _runCmd(arr("AT+CCLK?"), "OK", rsp, cb, args);
  _returnAfterReply();
}

#pragma region TEMPERATURE_MONITORING

bool WalterModem::configTemperatureMonitor(WalterModemTempMonitorMode mode, int8_t extreme_low,
                                           int8_t warning_low, int8_t warning_high,
                                           int8_t extreme_high, WalterModemRsp* rsp,
                                           walterModemCb cb, void* args)
{
  WalterModemBuffer* stringsbuffer = _getFreeBuffer();
  stringsbuffer->size +=
      sprintf((char*) stringsbuffer->data, "AT+SQNTMON=%d,%d,%d,%d,%d", mode, (int) extreme_low,
              (int) warning_low, (int) warning_high, (int) extreme_high);

  _runCmd(arr((const char*) stringsbuffer->data), "OK", rsp, cb, args, NULL, NULL,
          WALTER_MODEM_CMD_TYPE_TX_WAIT, NULL, 0, stringsbuffer);
  _returnAfterReply();
}

bool WalterModem::getTemperature(WalterModemRsp* rsp, walterModemCb cb, void* args)
{
  _runCmd(arr("AT+SQNTMON?"), "OK", rsp, cb, args);
  _returnAfterReply();
}

#pragma endregion // TEMPERATURE_MONITORING
#pragma region VOLTAGE_MONITORING

bool WalterModem::configVoltageMonitor(WalterModemVoltageMonitorMode mode, uint16_t threshold,
                                       uint16_t period, WalterModemRsp* rsp, walterModemCb cb,
                                       void* args)
{
  WalterModemBuffer* stringsbuffer = _getFreeBuffer();
  stringsbuffer->size +=
      sprintf((char*) stringsbuffer->data, "AT+SQNVMON=%d,%d,%d", mode, threshold, period);

  _runCmd(arr((const char*) stringsbuffer->data), "OK", rsp, cb, args, NULL, NULL,
          WALTER_MODEM_CMD_TYPE_TX_WAIT, NULL, 0, stringsbuffer);
  _returnAfterReply();
}

bool WalterModem::getVoltage(WalterModemRsp* rsp, walterModemCb cb, void* args)
{
  _runCmd(arr("AT+SQNVMON?"), "OK", rsp, cb, args);
  _returnAfterReply();
}

#pragma endregion // VOLTAGE_MONITORING
#pragma region EVENT_HANDLERS

void WalterModem::setNetworkEventHandler(walterModemNetworkEventHandler handler, void* args)
{
  _eventHandlers[WALTER_MODEM_EVENT_TYPE_NETWORK].netHandler = handler;
  _eventHandlers[WALTER_MODEM_EVENT_TYPE_NETWORK].args = args;
}

void WalterModem::setSystemEventHandler(walterModemSystemEventHandler handler, void* args)
{
  _eventHandlers[WALTER_MODEM_EVENT_TYPE_SYSTEM].sysHandler = handler;
  _eventHandlers[WALTER_MODEM_EVENT_TYPE_SYSTEM].args = args;
}

void WalterModem::setTemperatureEventHandler(walterModemTemperatureEventHandler handler, void* args)
{
  _eventHandlers[WALTER_MODEM_EVENT_TYPE_TEMPERATURE].tempHandler = handler;
  _eventHandlers[WALTER_MODEM_EVENT_TYPE_TEMPERATURE].args = args;
}

void WalterModem::setVoltageEventHandler(walterModemVoltageEventHandler handler, void* args)
{
  _eventHandlers[WALTER_MODEM_EVENT_TYPE_VOLTAGE].voltageHandler = handler;
  _eventHandlers[WALTER_MODEM_EVENT_TYPE_VOLTAGE].args = args;
}

#pragma endregion
#pragma endregion
