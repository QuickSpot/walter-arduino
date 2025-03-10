/**
 * @file WalterModem.cpp
 * @author Daan Pape <daan@dptechnics.com>
 * @date 9 Jan 2023
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
 * This file contains Walter's modem library implementation.
 */

#include "WalterModem.h"

#include <ctime>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef ARDUINO
#include <Arduino.h>
#endif

#include <esp_log.h>
#include <esp_sleep.h>
#include <esp_system.h>
#include <esp_ota_ops.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_task_wdt.h>
#include <esp_partition.h>
#include <esp_image_format.h>

/**
 * @brief The RX pin on which modem data is received.
 */
#define WALTER_MODEM_PIN_RX 14

/**
 * @brief The TX to which modem data must be transmitted.
 */
#define WALTER_MODEM_PIN_TX 48

/**
 * @brief The RTS pin on the ESP32 side.
 */
#define WALTER_MODEM_PIN_RTS 21

/**
 * @brief The CTS pin on the ESP32 size.
 */
#define WALTER_MODEM_PIN_CTS 47

/**
 * @brief The active low modem reset pin.
 */
#define WALTER_MODEM_PIN_RESET 45

/**
 * @brief The baud rate used to talk to the modem.
 */
#define WALTER_MODEM_BAUD 115200

/**
 * @brief The maximum number of milliseconds to wait.
 */
#define WALTER_MODEM_CMD_TIMEOUT_MS 300000

/**
 * @brief The command timeout expressed in system ticks.
 */
#define WALTER_MODEM_CMD_TIMEOUT_TICKS pdMS_TO_TICKS(WALTER_MODEM_CMD_TIMEOUT_MS)

/**
 * @brief Any modem time below 1 Jan 2023 00:00:00 UTC is considered an invalid time.
 */
#define WALTER_MODEM_MIN_VALID_TIMESTAMP 1672531200

/**
 * @brief The event handlers of the Walter modem library are as lightweight as possible and are not
 * executed in their own thread. Therefore an application should handle them as interrupt handlers. 
 * It is not allowed to call other WalterModem functions from within an event handler and no 
 * blocking operations should be performed in an event handler. To aid the user in achieving this 
 * the library prints a warning when the handler takes more than the defined number of milliseconds.
 */
#define WALTER_MODEM_MAX_EVENT_DURATION_MS 500

/**
 * @brief The length of a string literal at compile time.
 */
#define _strLitLen(str) (sizeof(str) - 1)

/**
 * @brief uart buffer size
 */
#define UART_BUF_SIZE 128

/**
 * @brief Check if a WalterModemBuffer starts with a given string literal.
 */
#define _buffStartsWith(buff, str) ((buff->size >= _strLitLen(str)) && \
    memcmp(str, buff->data, _strLitLen(str)) == 0)

/**
 * @brief Check if a WalterModemBuffer starts with an ASCII digit [0-9].
 */
#define _buffStartsWithDigit(buff) ((buff->size > 0) && \
    (buff->data[0] >= '0' && buff->data[0] <= '9'))

/**
 * @brief 0-terminate a WalterModemBuffer. This macro is meant to be used as an assignment in the
 * form of x = _buffStr(buff);
 */
#define _buffStr(buff) (const char*) buff->data; buff->data[buff->size] = '\0'

/**
 * Check if the data string in a buffer equals an expected value which comes after a prefix.
 */
#define _dataStrIs(buff, prefix, expected) ((buff->size - _strLitLen(prefix)) > 0 && \
    memcmp(buff->data + _strLitLen(prefix), expected, buff->size - _strLitLen(prefix)) == 0)

/**
 * @brief Convert a string literal into an AT command array. An empty string will result in "","",""
 * and an non-empty string will result in "\"", "<string>", "\"".
 */
#define _atStr(str) strlen(str) > 0 ? "\"" : "", str, strlen(str) > 0 ? "\"" : ""

/**
 * @brief Convert a boolean flag into a string literal.
 */
#define _atBool(bool) bool ? "1" : "0"

/**
 * @brief Convert a number of up to 6 digits in length into an array of digits which can be passed
 * to an AT command array.
 */
#define _atNum(num) \
    _intToStrDigit(num, 0),\
    _intToStrDigit(num, 1),\
    _intToStrDigit(num, 2),\
    _intToStrDigit(num, 3),\
    _intToStrDigit(num, 4),\
    _intToStrDigit(num, 5)

/**
 * @brief Perform a string copy and ensure that the destination is 0-terminated.
 */
#define _strncpy_s(dst, src, max) strncpy(dst, src == NULL ? "" : src, max); dst[max - 1] = '\0';

/**
 * @brief Make an array of a list of arguments.
 */
#define arr(...) {__VA_ARGS__}

/**
 * @brief Return an error.
 */
#define _returnState(state) \
if(cb == NULL) { \
    if(rsp != NULL) { \
        rsp->result = state; \
    } \
    return state == WALTER_MODEM_STATE_OK; \
} else { \
    WalterModemRsp cbRsp = {}; \
    cbRsp.result = state; \
    cb(&cbRsp, args); \
    return true; \
}

/**
 * @brief Convert little endian to big endian for 16-bit integers.
 */
#define _switchEndian16(x) ((((x) << 8) | ((x) >> 8)) & 0xffff)

/**
 * @brief Convert little endian to big endian 
 */
#define _switchEndian32(x) ((((x) << 24) | (((x) << 8) & 0x00ff0000) | \
    (((x) >> 8) & 0x0000ff00) | ((x) >> 24)) & 0xffffffff)

/**
 * @brief Add a command to the queue for it to be execute it.
 * 
 * This macro will add a command to the command queue for it to be executed. If the command could
 * not be added to the queue the macro will make the surrounding function return false (when blocking
 * API is used) or call the callback with an out-of-memory error.
 * 
 * @param atCmd The NULL terminated AT command elements to send to the modem.
 * @param atRsp The expected AT response from the modem.
 * @param rsp Pointer to the response structure to save the response in.
 * @param cb Optional user callback for asynchronous API.
 * @param args Optional argument to pass to the callback.
 * @param ... Optional arguments for the _addQueueCmd function.
 */
#define _runCmd(atCmd, atRsp, rsp, cb, args, ...) \
const char *_cmdArr[WALTER_MODEM_COMMAND_MAX_ELEMS + 1] = atCmd; \
WalterModemCmd *cmd = _addQueueCmd(_cmdArr, atRsp, rsp, cb, args, ##__VA_ARGS__); \
if(cmd == NULL) { \
    _returnState(WALTER_MODEM_STATE_NO_MEMORY); \
} \
std::unique_lock<std::mutex> lock{cmd->cmdLock.mutex};

/**
 * @brief When working asynchronously, it will return true because the response is handled through
 * the user callback. If the blocking API is being used and the expected command state is reached,
 * the mutex is released, the correct result of the command is passed and the function returns with
 * true when the state is WALTER_MODEM_STATE_OK, false otherwise.
 */
#define _returnAfterReply() \
if(cmd->userCb != NULL) { \
    lock.unlock(); \
    return true; \
} \
cmd->cmdLock.cond.wait(lock, [cmd] {  \
    return cmd->state == WALTER_MODEM_CMD_STATE_SYNC_LOCK_NOTIFIED; \
}); \
WalterModemState rspResult = cmd->rsp->result; \
cmd->state = WALTER_MODEM_CMD_STATE_COMPLETE; \
lock.unlock(); \
return rspResult == WALTER_MODEM_STATE_OK;

/**
 * @brief Transmit all elements of a command.
 */
static bool endOfLine;
#ifdef ARDUINO
#define _transmitCmd(type, atCmd) { \
    ESP_LOGD((endOfLine = false, "WalterModem"), \
            "TX: %s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s" \
            "%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s", \
            (!endOfLine && atCmd[0]) ? atCmd[0] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[1]) ? atCmd[1] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[2]) ? atCmd[2] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[3]) ? atCmd[3] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[4]) ? atCmd[4] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[5]) ? atCmd[5] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[6]) ? atCmd[6] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[7]) ? atCmd[7] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[8]) ? atCmd[8] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[9]) ? atCmd[9] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[10]) ? atCmd[10] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[11]) ? atCmd[11] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[12]) ? atCmd[12] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[13]) ? atCmd[13] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[14]) ? atCmd[14] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[15]) ? atCmd[15] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[16]) ? atCmd[16] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[17]) ? atCmd[17] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[18]) ? atCmd[18] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[19]) ? atCmd[19] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[20]) ? atCmd[20] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[21]) ? atCmd[21] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[22]) ? atCmd[22] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[23]) ? atCmd[23] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[24]) ? atCmd[24] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[25]) ? atCmd[25] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[26]) ? atCmd[26] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[27]) ? atCmd[27] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[28]) ? atCmd[28] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[29]) ? atCmd[29] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[30]) ? atCmd[30] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[31]) ? atCmd[31] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[32]) ? atCmd[32] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[33]) ? atCmd[33] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[34]) ? atCmd[34] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[35]) ? atCmd[35] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[36]) ? atCmd[36] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[37]) ? atCmd[37] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[38]) ? atCmd[38] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[39]) ? atCmd[39] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[40]) ? atCmd[40] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[41]) ? atCmd[41] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[42]) ? atCmd[42] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[43]) ? atCmd[43] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[44]) ? atCmd[44] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[45]) ? atCmd[45] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[46]) ? atCmd[46] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[47]) ? atCmd[47] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[48]) ? atCmd[48] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[49]) ? atCmd[49] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[50]) ? atCmd[50] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[51]) ? atCmd[51] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[52]) ? atCmd[52] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[53]) ? atCmd[53] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[54]) ? atCmd[54] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[55]) ? atCmd[55] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[56]) ? atCmd[56] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[57]) ? atCmd[57] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[58]) ? atCmd[58] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[59]) ? atCmd[59] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[60]) ? atCmd[60] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[61]) ? atCmd[61] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[62]) ? atCmd[62] : (endOfLine = true, "")); \
    for(int i = 0; i < WALTER_MODEM_COMMAND_MAX_ELEMS; ++i) { \
        if(atCmd[i] == NULL) { \
            break; \
        } \
        _uart->write(atCmd[i]); \
    } \
    _uart->write(type == WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT ? "\n" : "\r\n"); \
}
#else
#define _transmitCmd(type, atCmd) { \
    ESP_LOGD((endOfLine = false, "WalterModem"), \
            "TX: %s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s" \ 
            "%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s", \
            (!endOfLine && atCmd[0]) ? atCmd[0] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[1]) ? atCmd[1] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[2]) ? atCmd[2] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[3]) ? atCmd[3] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[4]) ? atCmd[4] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[5]) ? atCmd[5] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[6]) ? atCmd[6] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[7]) ? atCmd[7] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[8]) ? atCmd[8] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[9]) ? atCmd[9] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[10]) ? atCmd[10] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[11]) ? atCmd[11] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[12]) ? atCmd[12] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[13]) ? atCmd[13] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[14]) ? atCmd[14] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[15]) ? atCmd[15] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[16]) ? atCmd[16] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[17]) ? atCmd[17] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[18]) ? atCmd[18] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[19]) ? atCmd[19] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[20]) ? atCmd[20] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[21]) ? atCmd[21] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[22]) ? atCmd[22] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[23]) ? atCmd[23] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[24]) ? atCmd[24] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[25]) ? atCmd[25] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[26]) ? atCmd[26] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[27]) ? atCmd[27] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[28]) ? atCmd[28] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[29]) ? atCmd[29] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[30]) ? atCmd[30] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[31]) ? atCmd[31] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[32]) ? atCmd[32] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[33]) ? atCmd[33] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[34]) ? atCmd[34] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[35]) ? atCmd[35] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[36]) ? atCmd[36] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[37]) ? atCmd[37] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[38]) ? atCmd[38] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[39]) ? atCmd[39] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[40]) ? atCmd[40] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[41]) ? atCmd[41] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[42]) ? atCmd[42] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[43]) ? atCmd[43] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[44]) ? atCmd[44] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[45]) ? atCmd[45] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[46]) ? atCmd[46] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[47]) ? atCmd[47] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[48]) ? atCmd[48] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[49]) ? atCmd[49] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[50]) ? atCmd[50] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[51]) ? atCmd[51] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[52]) ? atCmd[52] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[53]) ? atCmd[53] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[54]) ? atCmd[54] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[55]) ? atCmd[55] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[56]) ? atCmd[56] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[57]) ? atCmd[57] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[58]) ? atCmd[58] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[59]) ? atCmd[59] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[60]) ? atCmd[60] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[61]) ? atCmd[61] : (endOfLine = true, ""), \
            (!endOfLine && atCmd[62]) ? atCmd[62] : (endOfLine = true, "")); \
    for(int i = 0; i < WALTER_MODEM_COMMAND_MAX_ELEMS; ++i) { \
        if(atCmd[i] == NULL) { \
            break; \
        } \
        uart_write_bytes(_uartNo, atCmd[i], strlen(atCmd[i])); \
    } \
    if(type == WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT) \
        uart_write_bytes(_uartNo, "\n", 1); \
    else \
        uart_write_bytes(_uartNo, "\r\n", 2); \
}
#endif

//TODO: move this inside the WalterModem class
struct WalterModemStpRequest stpRequest;
struct WalterModemStpResponseSessionOpen stpResponseSessionOpen;
struct WalterModemStpRequestTransferBlockCmd stpRequestTransferBlockCmd;
struct WalterModemStpResponseTransferBlock stpResponseTransferBlock;
RTC_DATA_ATTR WalterModemPDPContext _pdpCtxSetRTC[WALTER_MODEM_MAX_PDP_CTXTS] = {};
RTC_DATA_ATTR WalterModemPDPContext _coapCtxSetRTC[WALTER_MODEM_MAX_COAP_PROFILES] = {};
RTC_DATA_ATTR WalterModemBlueCherryState blueCherryRTC = {};
RTC_DATA_ATTR WalterModemMqttTopic _mqttTopicSetRTC[WALTER_MODEM_MQTT_MAX_TOPICS] = {};

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
static const char* _digitStr(int val) 
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
static const char* _intToStrDigit(int num, int pos)
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
       digit = tmp/pow;
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
static const char* _pdpTypeStr(WalterModemPDPType type)
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
static int64_t strTotime(const char *timeStr, const char *format = "%Y-%m-%dT%H:%M:%S") {
    struct tm tm{};
    if(strptime(timeStr, format, &tm) == NULL) {
        return -1;
    }

    /* Without setting time zone, mktime will assume UTC+00 on arduino, thus behaving like timegm */
    time_t utcTime = std::mktime(&tm);
    return (int64_t) utcTime;
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
static bool strToUint32(
    const char *str,
    int len,
    uint32_t *result,
    int radix = 10,
    uint32_t max = UINT32_MAX)
{
    size_t l = len == -1 ? strlen(str) : (size_t) len;

    /* Create a temp stack buffer to make the string 0-terminated */
    char buff[l + 1];
    memcpy(buff, str, l);
    buff[l] = '\0';

    char *end;
    errno = 0;
    long long int sl = strtol(buff, &end, radix);

    if(end == buff) {
        return false;
    } else if ('\0' != *end) {
        return false;
    } else if(errno == ERANGE) {
        return false;
    } else if (sl > max) {
        return false;
    } else if (sl < 0) {
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
static bool strToUint16(const char *str, int len, uint16_t *result, int radix = 10)
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
static bool strToUint8(const char *str, int len, uint8_t *result, int radix = 10)
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
bool strToFloat(const char *str, int len, float *result)
{
    size_t l = len == -1 ? strlen(str) : (size_t) len;

    /* Create a temp buffer to make the string 0-terminated */
    char buff[l + 1];
    memcpy(buff, str, l);
    buff[l] = '\0';

    char *end;
    errno = 0;
    *result = strtof(buff, &end);

    if(end == buff) {
        return false;
    } else if ('\0' != *end) {
        return false;
    } else if(errno == ERANGE) {
        return false;
    }

    return true;
}

WalterModemCmd* WalterModem::_cmdPoolGet()
{
    for(size_t i = 0; i < WALTER_MODEM_MAX_PENDING_COMMANDS; ++i) {
        WalterModemCmd *cmd = _cmdPool + i;
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

    WalterModemCmd *cmd = _cmdQueue.queue[_cmdQueue.outIdx];
    _cmdQueue.queue[_cmdQueue.outIdx] = NULL;
    _cmdQueue.outIdx += 1;
    if(_cmdQueue.outIdx == WALTER_MODEM_MAX_PENDING_COMMANDS) {
        _cmdQueue.outIdx = 0;
    }
    return cmd;
}

bool WalterModem::_cmdQueuePut(WalterModemCmd *cmd)
{
    if(_cmdQueue.inIdx == _cmdQueue.outIdx && _cmdQueue.queue[_cmdQueue.outIdx] != NULL) {
        /* The queue is full */
        return false;
    }

    _cmdQueue.queue[_cmdQueue.inIdx] = cmd;

    _cmdQueue.inIdx += 1;
    if(_cmdQueue.inIdx == WALTER_MODEM_MAX_PENDING_COMMANDS) {
        _cmdQueue.inIdx = 0;
    }
    return true;
}

WalterModemPDPContext* WalterModem::_pdpContextReserve()
{
    WalterModemPDPContext *ctx = NULL;

    for(int i = 0; i < WALTER_MODEM_MAX_PDP_CTXTS; ++i) {
        if(_pdpCtxSet[i].state == WALTER_MODEM_PDP_CONTEXT_STATE_FREE) {
            ctx = _pdpCtxSet + i;
            ctx->state = WALTER_MODEM_PDP_CONTEXT_STATE_RESERVED;
            ctx->id = i + 1;
            break;
        }
    }

    if(ctx != NULL) {
        _pdpCtx = ctx;
    }

    return ctx;
}

WalterModemPDPContext* WalterModem::_pdpContextGet(int id)
{
    if(id < 0) {
        return _pdpCtx;
    }

    for(int i = 0; i < WALTER_MODEM_MAX_PDP_CTXTS; ++i) {
        if(_pdpCtxSet[i].state != WALTER_MODEM_PDP_CONTEXT_STATE_FREE && _pdpCtxSet[i].id == id) {
            _pdpCtx = _pdpCtxSet + i;
            return _pdpCtxSet + i;
        }
    }

    return NULL;
}

void WalterModem::_pdpContextRelease(WalterModemPDPContext *ctx)
{
    if(ctx == NULL) {
        return;
    }

    ctx->state = WALTER_MODEM_PDP_CONTEXT_STATE_FREE;
}

void WalterModem::_saveRTCPdpContextSet(WalterModemPDPContext *_pdpCtxSetRTC)
{
    for(int i = 0; i < WALTER_MODEM_MAX_PDP_CTXTS; ++i) {
        _pdpCtxSetRTC[i] = _pdpCtxSet[i];
    }
}

void WalterModem::_loadRTCPdpContextSet(WalterModemPDPContext *_pdpCtxSetRTC)
{
    if(_pdpCtxSetRTC == NULL) {
        return;
    }

    for(int i = 0; i < WALTER_MODEM_MAX_PDP_CTXTS; ++i) {
        _pdpCtxSet[i] = _pdpCtxSetRTC[i];
    }

    _pdpCtx = _pdpCtxSet;
}

WalterModemSocket* WalterModem::_socketReserve()
{
    WalterModemSocket *sock = NULL;

    for(int i = 0; i < WALTER_MODEM_MAX_SOCKETS; ++i) {
        if(_socketSet[i].state == WALTER_MODEM_SOCKET_STATE_FREE) {
            sock = _socketSet + i;
            sock->state = WALTER_MODEM_SOCKET_STATE_RESERVED;
            sock->id = i + 1;
            break;
        }
    }

    if(sock != NULL) {
        _socket = sock;
    }

    return sock;
}

WalterModemSocket* WalterModem::_socketGet(int id)
{
    if(id < 0) {
        return _socket;
    }

    for(int i = 0; i < WALTER_MODEM_MAX_SOCKETS; ++i) {
        if(_socketSet[i].state != WALTER_MODEM_SOCKET_STATE_FREE && _socketSet[i].id == id) {
            _socket = _socketSet + i;
            return _socketSet + i;
        }
    }

    return NULL;
}

void WalterModem::_socketRelease(WalterModemSocket *sock)
{
    if(sock == NULL) {
        return;
    }

    sock->state = WALTER_MODEM_SOCKET_STATE_FREE;
}

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

    return chosenBuf;
}

void WalterModem::_addATByteToBuffer(char data, bool raw)
{
    //TODO: in the future we must be aware of length, or at least check if the ending \r\n is
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

uint16_t WalterModem::_extractRawBufferChunkSize()
{
    if(_parserData.buf == NULL || !_parserData.buf->size) {
        return 0;
    }

    //TODO: add cases to parse SQNSRING length

    /* parse +SQNCOAPRCV raw data chunk size */
    if(_parserData.buf->size > _strLitLen("+SQNCOAPRCV: ")
            && _parserData.buf->data[0] == '+'
            && _parserData.buf->data[1] == 'S'
            && _parserData.buf->data[2] == 'Q'
            && _parserData.buf->data[3] == 'N'
            && _parserData.buf->data[4] == 'C'
            && _parserData.buf->data[5] == 'O'
            && _parserData.buf->data[6] == 'A'
            && _parserData.buf->data[7] == 'P'
            && _parserData.buf->data[8] == 'R'
            && _parserData.buf->data[9] == 'C'
            && _parserData.buf->data[10] == 'V'
            && _parserData.buf->data[11] == ':'
            && _parserData.buf->data[12] == ' ') {
        uint8_t nrCommasSeen = 0;
        short i = _strLitLen("+SQNCOAPRCV: ");
        for(; i < _parserData.buf->size && nrCommasSeen < 6; i++) {
            if(_parserData.buf->data[i] == ',') {
                nrCommasSeen++;
            }
        }

        if(nrCommasSeen == 6) {
            uint16_t chunkSize = 0;
            for(; i < _parserData.buf->size; i++) {
                chunkSize *= 10;
                chunkSize += (_parserData.buf->data[i] - '0');
            }

            return chunkSize + _strLitLen("\r\nOK\r\n") + (chunkSize ? _strLitLen("\r\n") : 0);
        } else {
            return _strLitLen("\r\nOK\r\n");
        }
    }

    return 0;
}

void WalterModem::_queueRxBuffer()
{
    if(_parserData.buf != NULL) {
        if(_parserData.buf->size > 0) {
            WalterModemTaskQueueItem qItem = {};
            qItem.rsp = _parserData.buf;

            if(xQueueSend(_taskQueue.handle, &qItem, 0) != pdTRUE) {
                /* 
                 * When we can not send the buffer to the queue we release it immediately and thus
                 * drop the packet. In the other case the buffer will be released by the queue
                 * consumer.
                 */
                _parserData.buf->free = true;
            }
        }

        _parserData.buf = NULL;
    }
}

size_t WalterModem::_uartRead(uint8_t *buf, int readSize, bool tryHard)
{
    size_t totalBytesRead = 0;

#ifdef ARDUINO
    do {
        totalBytesRead += _uart->readBytes(buf, readSize - totalBytesRead);
    } while(tryHard && totalBytesRead < readSize);
#else
    do {
       int bytesRead = uart_read_bytes(_uartNo, buf, readSize - totalBytesRead,
            WALTER_MODEM_CMD_TIMEOUT_TICKS);
       if(bytesRead < 0) {
           break;
       }
       totalBytesRead += bytesRead;
    } while(tryHard && totalBytesRead < readSize);
#endif

    return totalBytesRead;
}

size_t WalterModem::_uartWrite(uint8_t *buf, int writeSize)
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

void WalterModem::_parseRxData(char *rxData, size_t len)
{
    for(size_t i = 0; i < len; ++i) {
        char data = rxData[i];
        
        switch (_parserData.state) {
        case WALTER_MODEM_RSP_PARSER_START_CR:
            if(data == '\r') {
                _parserData.state = WALTER_MODEM_RSP_PARSER_START_LF;
            } else if(data == '+') {
                /* This is the start of a new line in a multiline response */
                _parserData.state = WALTER_MODEM_RSP_PARSER_DATA;
                _addATByteToBuffer(data, false);
            }
            break;

        case WALTER_MODEM_RSP_PARSER_START_LF:
            if(data == '\n') {
                _parserData.state = WALTER_MODEM_RSP_PARSER_DATA;
            }
            break;

        case WALTER_MODEM_RSP_PARSER_DATA:
            if(data == '>') {
                _parserData.state = WALTER_MODEM_RSP_PARSER_DATA_PROMPT;
            } else if(data == '<') {
                _parserData.state = WALTER_MODEM_RSP_PARSER_DATA_HTTP_START1;
            }
            _addATByteToBuffer(data, false);
            break;
       
        case WALTER_MODEM_RSP_PARSER_DATA_PROMPT:
            _addATByteToBuffer(data, false);
            if(data == ' ') {
                _parserData.state = WALTER_MODEM_RSP_PARSER_START_CR;
                _queueRxBuffer();
            } else if(data == '>') {
                _parserData.state = WALTER_MODEM_RSP_PARSER_DATA_PROMPT_HTTP;
            } else {
                /* state might have changed after detecting end \r */
                if(_parserData.state == WALTER_MODEM_RSP_PARSER_DATA_PROMPT) {
                    _parserData.state = WALTER_MODEM_RSP_PARSER_DATA;
                }
            }
            break;

        case WALTER_MODEM_RSP_PARSER_DATA_PROMPT_HTTP:
            _addATByteToBuffer(data, false);
            if(data == '>') {
                _parserData.state = WALTER_MODEM_RSP_PARSER_START_CR;
                _queueRxBuffer();
            } else {
                /* state might have changed after detecting end \r */
                if(_parserData.state == WALTER_MODEM_RSP_PARSER_DATA_PROMPT_HTTP) {
                    _parserData.state = WALTER_MODEM_RSP_PARSER_DATA;
                }
            }
            break;

        case WALTER_MODEM_RSP_PARSER_DATA_HTTP_START1:
            if(data == '<') {
                _parserData.state = WALTER_MODEM_RSP_PARSER_DATA_HTTP_START2;
            } else {
                _parserData.state = WALTER_MODEM_RSP_PARSER_DATA;
            }
            _addATByteToBuffer(data, false);
            break;

        case WALTER_MODEM_RSP_PARSER_DATA_HTTP_START2:
            if(data == '<' && _httpCurrentProfile < WALTER_MODEM_MAX_HTTP_PROFILES) {
                /* FIXME:
                 * - modem might block longer than cmd timeout,
                 *   will lead to retry, error etc - fix properly
                 * - no buffer size checking!
                 */
                _parserData.rawChunkSize =
                    _httpContextSet[_httpCurrentProfile].contentLength + _strLitLen("\r\nOK\r\n");
                _parserData.state = WALTER_MODEM_RSP_PARSER_RAW;
            } else {
                _parserData.state = WALTER_MODEM_RSP_PARSER_DATA;
            }
            _addATByteToBuffer(data, false);
            break;

        case WALTER_MODEM_RSP_PARSER_END_LF:
            if(data == '\n') {
                uint16_t chunkSize = _extractRawBufferChunkSize();
                if(chunkSize) {
                    _parserData.rawChunkSize = chunkSize;
                    _parserData.buf->data[_parserData.buf->size++] = '\r';
                    _parserData.state = WALTER_MODEM_RSP_PARSER_RAW;
                } else {
                    _parserData.state = WALTER_MODEM_RSP_PARSER_START_CR;
                    _queueRxBuffer();
                }
            } else {
                /* only now we know the \r was thrown away for no good reason */
                _parserData.buf->data[_parserData.buf->size++] = '\r';

                /* next byte gets the same treatment; since we really are
                 * back in semi DATA state, as we now know
                 * (but > will not lead to data prompt mode)
                 */
                _addATByteToBuffer(data, false);
                if(data != '\r') {
                    _parserData.state = WALTER_MODEM_RSP_PARSER_DATA;
                }
            }
            break;

        case WALTER_MODEM_RSP_PARSER_RAW:
            _addATByteToBuffer(data, true);
            _parserData.rawChunkSize--;

            if(_parserData.rawChunkSize == 0) {
                _parserData.state = WALTER_MODEM_RSP_PARSER_START_CR;
                _queueRxBuffer();
            }
            break;
        }
    }
}

#ifdef ARDUINO
void WalterModem::_handleRxData(void)
{
    size_t uartBufLen;
    static char incomingBuf[UART_BUF_SIZE];

    if(_rxHandlerInterrupted) {
        return;
    }

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
void WalterModem::_handleRxData(void *params)
{
    size_t uartBufLen;
    static char incomingBuf[UART_BUF_SIZE];

    /* init watchdog for uart rx task on ESP-IDF */
    if(_watchdogTimeout != 0) {
        esp_task_wdt_add(NULL);
    }

    for(;;) {
        tickleWatchdog();

        if(_rxHandlerInterrupted) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

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

void WalterModem::_queueProcessingTask(void *args)
{
    /* init watchdog for internal queue processing task */
    if(_watchdogTimeout) {
        esp_task_wdt_add(NULL);
    }

    WalterModemTaskQueueItem qItem = { };
    WalterModemCmd *curCmd = NULL;
    TickType_t blockTime = WALTER_MODEM_CMD_TIMEOUT_TICKS;

    while(true) {
        tickleWatchdog();

        if(xQueueReceive(_taskQueue.handle, &qItem, blockTime) == pdTRUE) {
            if(qItem.cmd != NULL) {
                if(curCmd == NULL) {
                    curCmd = qItem.cmd;
                } else {
                    if(!_cmdQueuePut(qItem.cmd)) {
                        _processQueueCmd(qItem.cmd, true);
                    }
                }
            } else if(qItem.rsp != NULL) {
                _processQueueRsp(curCmd, qItem.rsp);
            }
        }

        blockTime = WALTER_MODEM_CMD_TIMEOUT_TICKS;

        if(curCmd != NULL) {
            switch(curCmd->state) {
                case WALTER_MODEM_CMD_STATE_FREE:
                case WALTER_MODEM_CMD_STATE_POOLED:
                    /* 
                     * There was a programming issue, commands with these states should never be
                     * sent to the queue. We are going to ignore them.
                    */
                    curCmd = NULL;
                    break;

                case WALTER_MODEM_CMD_STATE_RETRY_AFTER_ERROR:
                    blockTime = _processQueueCmd(curCmd);
                    break;

                case WALTER_MODEM_CMD_STATE_NEW:
                case WALTER_MODEM_CMD_STATE_PENDING:
                    blockTime = _processQueueCmd(curCmd);
                    if(curCmd->state == WALTER_MODEM_CMD_STATE_COMPLETE) {
                        curCmd->state = WALTER_MODEM_CMD_STATE_FREE;
                        curCmd = NULL;
                    }
                    break;

                
                case WALTER_MODEM_CMD_STATE_SYNC_LOCK_NOTIFIED:
                    /* We need to wait until the other thread is ready */
                    break;

                case WALTER_MODEM_CMD_STATE_COMPLETE:
                    curCmd->state = WALTER_MODEM_CMD_STATE_FREE;
                    curCmd = NULL;
                    break;
            }
        }

        if(curCmd == NULL) {
            curCmd = _cmdQueuePop();
            if(curCmd != NULL) {
                blockTime = 0;
            }
        }
    }
}

WalterModemCmd* WalterModem::_addQueueCmd(
    const char *atCmd[WALTER_MODEM_COMMAND_MAX_ELEMS + 1],
    const char *atRsp,
    WalterModemRsp *rsp,
    walterModemCb userCb,
    void *userCbArgs,
    void (*completeHandler)(struct sWalterModemCmd *cmd, WalterModemState result),
    void *completeHandlerArg,
    WalterModemCmdType type,
    uint8_t *data,
    uint16_t dataSize,
    WalterModemBuffer* stringsBuffer,
    uint8_t maxAttempts)
{
    WalterModemCmd *cmd = _cmdPoolGet();
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
    cmd->data = data;
    cmd->dataSize = dataSize;
    cmd->maxAttempts = maxAttempts;
    cmd->atRspLen = atRsp == NULL ? 0 : strlen(atRsp);
    cmd->state = WALTER_MODEM_CMD_STATE_NEW;
    cmd->attempt = 0;
    cmd->attemptStart = 0;
    cmd->stringsBuffer = stringsBuffer;
    *(cmd->rsp) = {};

    WalterModemTaskQueueItem qItem = {};
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

void WalterModem::_finishQueueCmd(WalterModemCmd *cmd, WalterModemState result)
{
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
        std::unique_lock<std::mutex> lock{cmd->cmdLock.mutex};
        cmd->cmdLock.cond.notify_one();
        lock.unlock();
    }
}

TickType_t WalterModem::_processQueueCmd(WalterModemCmd *cmd, bool queueError)
{
    if(queueError) {
        _finishQueueCmd(cmd, WALTER_MODEM_STATE_NO_MEMORY);
        return WALTER_MODEM_CMD_TIMEOUT_TICKS;
    }

    switch(cmd->type) {
        case WALTER_MODEM_CMD_TYPE_TX:
            _transmitCmd(cmd->type, cmd->atCmd);
            cmd->state = WALTER_MODEM_CMD_STATE_PENDING;
            _finishQueueCmd(cmd);
            break;

        case WALTER_MODEM_CMD_TYPE_TX_WAIT:
        case WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT:
            if(cmd->state == WALTER_MODEM_CMD_STATE_NEW) {
                /* hack for handling AT+SQNSMQTTRCVMESSAGE response */
                if(cmd->type == WALTER_MODEM_CMD_TYPE_TX_WAIT &&
                   !strcmp(cmd->atCmd[0], "AT+SQNSMQTTRCVMESSAGE=0,")) {
                    _parserData.state = WALTER_MODEM_RSP_PARSER_RAW;
                    /* add 4 bytes for prepending and trailing \r\n */
                    _parserData.rawChunkSize = cmd->dataSize + 4;
                }
                _transmitCmd(cmd->type, cmd->atCmd);
                cmd->attempt = 1;
                cmd->attemptStart = xTaskGetTickCount();
                cmd->state = WALTER_MODEM_CMD_STATE_PENDING;
                return WALTER_MODEM_CMD_TIMEOUT_TICKS;
            } else {
                TickType_t diff = xTaskGetTickCount() - cmd->attemptStart;
                bool timedOut = diff >= WALTER_MODEM_CMD_TIMEOUT_TICKS;
                if(timedOut || cmd->state == WALTER_MODEM_CMD_STATE_RETRY_AFTER_ERROR) {
                    if(cmd->attempt >= cmd->maxAttempts) {
                        _finishQueueCmd(cmd, timedOut ?
                            WALTER_MODEM_STATE_TIMEOUT : 
                            WALTER_MODEM_STATE_ERROR);
                    } else {
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
                    _finishQueueCmd(cmd, WALTER_MODEM_STATE_TIMEOUT);
                } else {
                    return diff;
                }
            }
            break;
    }

    return WALTER_MODEM_CMD_TIMEOUT_TICKS;
}

static void coap_received_from_bluecherry(const WalterModemRsp *rsp, void *args)
{
    WalterModemBlueCherryState *blueCherry = (WalterModemBlueCherryState*) args;

    if(rsp->type != WALTER_MODEM_RSP_DATA_TYPE_COAP) {
        return;
    }

    /* 
     * Silently discard unexpected requests or responses, to keep the basic BlueCherry cloud API
     * with blueCherryDidRing simple. Users who want full control can manually use CoAP with profile
     * id 1 or 2.
     */

    /* sanity checks: valid message (response) ? */
    if(blueCherry->status == WALTER_MODEM_BLUECHERRY_STATUS_AWAITING_RESPONSE &&
       rsp->data.coapResponse.messageId == blueCherry->curMessageId &&
       rsp->data.coapResponse.sendType == WALTER_MODEM_COAP_SEND_TYPE_ACK &&
       (rsp->data.coapResponse.methodRsp == WALTER_MODEM_COAP_SEND_RSP_CODE_VALID ||
        rsp->data.coapResponse.methodRsp == WALTER_MODEM_COAP_SEND_RSP_CODE_CONTINUE)) {
        blueCherry->lastAckedMessageId = rsp->data.coapResponse.messageId;
        blueCherry->messageInLen = rsp->data.coapResponse.length;
        blueCherry->status = WALTER_MODEM_BLUECHERRY_STATUS_RESPONSE_READY;
        blueCherry->moreDataAvailable =
            rsp->data.coapResponse.methodRsp == WALTER_MODEM_COAP_SEND_RSP_CODE_CONTINUE;
    }
}

void WalterModem::_processQueueRsp(WalterModemCmd *cmd, WalterModemBuffer *buff)
{
    ESP_LOGD("WalterModem", "RX: %.*s", buff->size, buff->data);
    _dispatchEvent((const char*) (buff->data), buff->size);

    WalterModemState result = WALTER_MODEM_STATE_OK;

    if(_buffStartsWith(buff, "+CEREG: ")) {
        const char *rspStr = _buffStr(buff);
        int ceReg = atoi(rspStr + _strLitLen("+CEREG: "));
        _regState = (WalterModemNetworkRegState) ceReg;
        _dispatchEvent(_regState);
    }
    else if(_buffStartsWith(buff, "> ") || _buffStartsWith(buff, ">>>"))
    {
        if(cmd != NULL && cmd->type == WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT && cmd->data != NULL) {
#ifdef ARDUINO
            _uart->write(cmd->data, cmd->dataSize);
#else
            uart_write_bytes(_uartNo, cmd->data, cmd->dataSize);
#endif
        }
    }
    else if(_buffStartsWith(buff, "ERROR"))
    {
        if(cmd != NULL) {
            cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_NO_DATA;
            cmd->state = WALTER_MODEM_CMD_STATE_RETRY_AFTER_ERROR;
        }
        buff->free = true;
        return;
    }
    else if(_buffStartsWith(buff, "+CME ERROR: "))
    {
        if(cmd != NULL) {
            const char *rspStr = _buffStr(buff);
            int cmeError = atoi(rspStr + _strLitLen("+CME ERROR: "));
            cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_CME_ERROR;
            cmd->rsp->data.cmeError = (WalterModemCMEError) cmeError;
        }

        cmd->state = WALTER_MODEM_CMD_STATE_RETRY_AFTER_ERROR;
        buff->free = true;
        return;
    }
    else if(_buffStartsWith(buff, "+CFUN: "))
    {
        const char *rspStr = _buffStr(buff);
        int opState = atoi(rspStr + _strLitLen("+CFUN: "));
        _opState = (WalterModemOpState) opState;

        if(cmd == NULL) {
            buff->free = true;
            return;
        }
        cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_OPSTATE;
        cmd->rsp->data.opState = (WalterModemOpState) opState;

    }
    else if(_buffStartsWith(buff, "+CPIN: ") && cmd != NULL)
    {
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
    }
    else if(_buffStartsWith(buff, "+SQNCCID: ") && cmd != NULL)
    {
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
                    i += 3;

                    if(offset >= 22) { 
                        break;
                    } else {
                        continue;
                    }
                }

                cmd->rsp->data.simCardID.iccid[offset++] = buff->data[i];
            } else {
                if(buff->data[i] == '"' || offset >= 22) {
                    cmd->rsp->data.simCardID.euiccid[offset] = '\0';
                    break;
                }

                cmd->rsp->data.simCardID.euiccid[offset++] = buff->data[i];
            }
        }
    }
    else if(_buffStartsWith(buff, "+CGPADDR: "))
    {
        uint16_t dataSize = buff->size - _strLitLen("+CGPADDR: ");
        uint8_t *data = buff->data + _strLitLen("+CGPADDR: ");

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

        int pdpCtxId = atoi((const char*) data);
        WalterModemPDPContext *ctx = _pdpContextGet(pdpCtxId);
        if(ctx != NULL) {
            _strncpy_s(ctx->pdpAddress, (const char*) data + addr1Offset,
                WALTER_MODEM_PDP_ADDR_MAX_SIZE);
            
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
    }
    else if(_buffStartsWith(buff, "+CSQ: "))
    {
        const char *rspStr = _buffStr(buff);
        char *data = (char*) rspStr + _strLitLen("+CSQ: ");

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
    }
    else if(_buffStartsWith(buff, "+CESQ: "))
    {
        cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_SIGNAL_QUALITY;

        uint16_t dataSize = buff->size - _strLitLen("+CESQ: ");
        const char *rspStr = _buffStr(buff);
        char *data = (char*) rspStr + _strLitLen("+CESQ: ");

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
    }
    else if(_buffStartsWith(buff, "+SQNMONI: "))
    {
        cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_CELL_INFO;

        const char *rspStr = _buffStr(buff);
        char *data = (char*) rspStr;
        uint16_t dataSize = buff->size;
        uint16_t offset = _strLitLen("+SQNMONI: ") - 1;
        int8_t lastColon = -1;
        bool firstKeyParsed = false;

        for(int i = offset + 1; i < dataSize; ++i) {
            if(data[i] == ':') {
                lastColon = i;
            } else if(data[i] == ' ' || data[i] == '\r') {
                if(lastColon > 0) {
                    const char *key = data + offset + 1;
                    int key_len = lastColon - offset - 1;
                    const char *value = data + lastColon + 1;
                    int value_len = i - lastColon - 1;
                    if(!firstKeyParsed) {
                        if(key_len > 2) {
                            /* The operator name is present */
                            memcpy(cmd->rsp->data.cellInformation.netName, key,
                                   key_len - 3 > (WALTER_MODEM_OPERATOR_MAX_SIZE - 1) ?
                                    WALTER_MODEM_OPERATOR_MAX_SIZE - 1 : key_len - 3);
                            key += key_len - 2;
                            key_len = 2;
                        }
                        firstKeyParsed = true;
                    }
                    if(strncmp("Cc", key, key_len) == 0) {
                        strToUint16(value, value_len, &(cmd->rsp->data.cellInformation.cc));
                    } else if(strncmp("Nc", key, key_len) == 0) {
                        strToUint8(value, value_len, &(cmd->rsp->data.cellInformation.nc));
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
    }
    else if(_buffStartsWith(buff, "+CGSN: "))
    {
        if(cmd == NULL || 
           buff->size < _strLitLen("+CGSN: \"xxxxxxxxxxxxxxxx\""))
        {
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
        cmd->rsp->data.identity.svn[3] = '\0';

        /* Copy the 14-digit long IMEI number, and add the checksum */
        memcpy(cmd->rsp->data.identity.imei, cmd->rsp->data.identity.imeisv, 14);
        cmd->rsp->data.identity.imei[14] =
            _getLuhnChecksum((const char*) cmd->rsp->data.identity.imei);
        cmd->rsp->data.identity.imei[15] = '\0';
    }
    else if(_buffStartsWith(buff, "+SQNMODEACTIVE: "))
    {
        const char *rspStr = _buffStr(buff);
        int rat = atoi(rspStr + _strLitLen("+SQNMODEACTIVE: "));
        _ratType = (WalterModemRAT) (rat - 1);

        if(cmd == NULL) {
            buff->free = true;
            return;
        }
        cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_RAT;
        cmd->rsp->data.rat = (WalterModemRAT) (rat - 1);
    }
    else if(_buffStartsWith(buff, "+SQNBANDSEL: "))
    {
        if(buff->size <= _strLitLen("+SQNBANDSEL: 1,,\"\"")) {
            buff->free = true;
            return;
        }

        uint16_t dataSize = buff->size - _strLitLen("+SQNBANDSEL: ");
        uint8_t *data = buff->data + _strLitLen("+SQNBANDSEL: ");

        cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_BANDSET_CFG_SET;
        WalterModemBandSelection *bSel =
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
    }
    else if(_buffStartsWith(buff, "+LPGNSSFIXREADY: "))
    {
        uint16_t dataSize = buff->size - _strLitLen("+LPGNSSFIXREADY: ");
        uint8_t *data = buff->data + _strLitLen("+LPGNSSFIXREADY: ");

        char *start = (char*) data;
        char *lastCharacter = NULL;
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
                        _GNSSfix.fixId = atoi(start);
                        break;
                    
                    case 1:
                        *lastCharacter = '\0';
                        _GNSSfix.timestamp = strTotime(start + 1);
                        break;

                    case 2:
                        _GNSSfix.timeToFix = atoi(start);
                        break;

                    case 3:
                        *lastCharacter = '\0';
                        _GNSSfix.estimatedConfidence = strtod(start + 1, NULL);
                        break;

                    case 4:
                        *lastCharacter = '\0';
                        _GNSSfix.latitude = atof(start + 1);
                        break;

                    case 5:
                        *lastCharacter = '\0';
                        _GNSSfix.longitude = atof(start + 1);
                        break;

                    case 6:
                        *lastCharacter = '\0';
                        _GNSSfix.height = atof(start + 1);
                        break;

                    case 7:
                        *lastCharacter = '\0';
                        _GNSSfix.northSpeed = atof(start + 1);
                        break;

                    case 8:
                        *lastCharacter = '\0';
                        _GNSSfix.eastSpeed = atof(start + 1);
                        break;

                    case 9:
                        *lastCharacter = '\0';
                        _GNSSfix.downSpeed = atof(start + 1);
                        break;

                    case 10:
                        /* 
                         * Raw satellite signal sample is ignored, we use this occasion to reset the
                         * satellite count.
                         */
                        _GNSSfix.satCount = 0;
                        break;

                    default:
                        if(_GNSSfix.satCount >= WALTER_MODEM_GNSS_MAX_SATS) {
                            continue;
                        }

                        const char *satNoStr = start + 1;   /* skip '(' */
                        const char *satSigStr = start;
                        for(int i = 0; start[i] != '\0'; ++i) {
                            if(start[i] == ',') {
                                start[i] = '\0';
                                satSigStr = start + i + 1;
                                break;
                            }
                        }

                        _GNSSfix.sats[_GNSSfix.satCount].satNo = atoi(satNoStr);
                        _GNSSfix.sats[_GNSSfix.satCount].signalStrength = atoi(satSigStr);
                        _GNSSfix.satCount += 1;
                        break;
                }

                /* +1 for the comma */
                start = (char*) data + i + 1;
                partNo += 1;
            }
        }

        _dispatchEvent(&_GNSSfix);
    }
    else if(_buffStartsWith(buff, "+LPGNSSASSISTANCE: "))
    {
        cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_GNSS_ASSISTANCE_DATA;

        uint16_t dataSize = buff->size - _strLitLen("+LPGNSSASSISTANCE: ");
        uint8_t *data = buff->data + _strLitLen("+LPGNSSASSISTANCE: ");

        char *start = (char*) data;
        uint8_t partNo = 0;

        WalterModemGNSSAssistanceTypeDetails *details = NULL;

        for(uint16_t i = 0; i < dataSize; ++i) {
            bool partComplete = false;

            if(data[i] == ',') {
                data[i] = '\0';
                partComplete = true;
            } else if(i + 1 == dataSize) {
                data[i + 1] = '\0';
                partComplete = true;
            }

            if(partComplete) {
                switch(partNo) {
                    case 0:
                        switch(start[0]) {
                            case '0':
                                details = &(cmd->rsp->data.gnssAssistance.almanac);
                                break;

                            case '1':
                                details = &(cmd->rsp->data.gnssAssistance.realtimeEphemeris);
                                break;

                            case '2':
                                details = &(cmd->rsp->data.gnssAssistance.predictedEphemeris);
                                break;
                        }
                        break;

                    case 1:
                        if(details != NULL) {
                            details->available = atoi(start) == 1;
                        }
                        break;

                    case 2:
                        if(details != NULL) {
                            details->lastUpdate = atoi(start);
                        }
                        break;

                    case 3:
                        if(details != NULL) {
                            details->timeToUpdate = atoi(start);
                        }
                        break;

                    case 4:
                        if(details != NULL) {
                            details->timeToExpire = atoi(start);
                        }
                        break;
                }
                
                /* +1 for the comma */
                start = (char*) data + i + 1;
                partNo += 1;
            }
        }
    }
    else if(_buffStartsWith(buff, "+CCLK: \""))
    {
        buff->data[buff->size - 1] = '\0';
        char *data = (char*) buff->data + _strLitLen("+CCLK: \"");
        cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_CLOCK;
        int64_t utcTime = strTotime(data, "%y/%m/%d,%H:%M:%S");
        uint16_t tzOffset;
        if(data[17] == '+') {
            tzOffset = atoi(data + 18) * 15 * 60;
        } else {
            tzOffset = atoi(data + 18) * -15 * 60;
        }
        cmd->rsp->data.clock = utcTime - tzOffset;
        if(cmd->rsp->data.clock < WALTER_MODEM_MIN_VALID_TIMESTAMP) {
            cmd->rsp->data.clock = -1;
        }
    }
    else if(_buffStartsWith(buff, "+SQNCOAPRCV: "))
    {
        const char *rspStr = _buffStr(buff);
        char *payload = strchr(rspStr, '\r');
        if(payload != 0) {
            payload++;
        }

        char *commaPos = strchr(rspStr, ',');
        char *start = (char*) rspStr + _strLitLen("+SQNCOAPRCV: ");
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

            /* find message id in the stored rings for this profile */
            uint8_t ringIdx;
            for(ringIdx = 0; ringIdx < WALTER_MODEM_COAP_MAX_PENDING_RINGS; ringIdx++) {
                if(_coapContextSet[profileId].rings[ringIdx].messageId == messageId &&
                   _coapContextSet[profileId].rings[ringIdx].sendType == sendType &&
                   _coapContextSet[profileId].rings[ringIdx].methodRsp == reqRspCodeRaw) {
                    break;
                }
            }

            if(ringIdx < WALTER_MODEM_COAP_MAX_PENDING_RINGS) {
                /* free ring entry */
                _coapContextSet[profileId].rings[ringIdx].messageId = 0;
            }

            cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_COAP;
            cmd->rsp->data.coapResponse.profileId = profileId;
            cmd->rsp->data.coapResponse.messageId = messageId;
            cmd->rsp->data.coapResponse.sendType = sendType;
            cmd->rsp->data.coapResponse.methodRsp =
                (WalterModemCoapSendMethodRsp) reqRspCodeRaw;

            if(length > cmd->dataSize) {
                cmd->rsp->data.coapResponse.length = cmd->dataSize;
            }
            else {
                cmd->rsp->data.coapResponse.length = length;
            }

            /* 
             * If data and dataSize are null, we cannot store the result. We can only hope the user
             * is using a callback which has access to the raw buffer.
             */
            if(cmd->data) {
                memcpy(cmd->data, payload, cmd->rsp->data.coapResponse.length);
            }
        }
    }
    else if(_buffStartsWith(buff, "<<<"))   
    {
        /* <<< is start of SQNHTTPRCV answer */
        if(_httpCurrentProfile >= WALTER_MODEM_MAX_HTTP_PROFILES ||
           _httpContextSet[_httpCurrentProfile].state != WALTER_MODEM_HTTP_CONTEXT_STATE_GOT_RING) {
            result = WALTER_MODEM_STATE_ERROR;
            goto after_processing_logic;
        }

        cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_HTTP_RESPONSE;
        cmd->rsp->data.httpResponse.httpStatus = _httpContextSet[_httpCurrentProfile].httpStatus;
        if(_httpContextSet[_httpCurrentProfile].contentLength > cmd->dataSize - 1) {
            cmd->rsp->data.httpResponse.contentLength = cmd->dataSize - 1;
        } else {
            cmd->rsp->data.httpResponse.contentLength =
                _httpContextSet[_httpCurrentProfile].contentLength;
        }

        /* 
         * If data and dataSize are null, we cannot store the result. We can only hope the user is
         * using a callback which has access to the raw buffer.
         */
        if(cmd->data) {
            memcpy(cmd->data, buff->data + 3, cmd->rsp->data.httpResponse.contentLength);
            cmd->data[cmd->rsp->data.httpResponse.contentLength] = '\0';
        }
    }
    else if(_buffStartsWith(buff, "+SQNHTTPRING: "))
    {
        const char *rspStr = _buffStr(buff);
        char *commaPos = strchr(rspStr, ',');
        char *start = (char*) rspStr + _strLitLen("+SQNHTTPRING: ");

        uint8_t profileId = 0;
        uint8_t httpStatus = 0;
        char *contentType = NULL;
        uint16_t contentLength = 0;

        if(commaPos) {
            *commaPos = '\0';
            profileId = atoi(start);
            start = ++commaPos;
            commaPos = strchr(commaPos, ',');

            if(profileId >= WALTER_MODEM_MAX_HTTP_PROFILES) {
                //TODO: return error if modem returns invalid profile id. problem: this message is
                //an URC: the associated cmd may be any random command currently executing.
                buff->free = true;
                return;
            }
        }

        if(commaPos) {
            *commaPos = '\0';
            httpStatus = atoi(start);
            start = ++commaPos;
            commaPos = strchr(commaPos, ',');
        }

        if(commaPos) {
            *commaPos = '\0';
            contentType = start;
            contentLength = atoi(commaPos + 1);

            //TODO: if not expecting a ring, it may be a bug in the modem or at our side and we
            //should report an error + read the content to free the modem buffer (knowing that this
            //is a URC so there is no command to give feedback to)
            if(_httpContextSet[profileId].state != WALTER_MODEM_HTTP_CONTEXT_STATE_EXPECT_RING) {
                buff->free = true;
                return;
            }

            //TODO: remember ring info -once we implement events, call the event handler if any
            _httpContextSet[profileId].state = WALTER_MODEM_HTTP_CONTEXT_STATE_GOT_RING;
            _httpContextSet[profileId].httpStatus = httpStatus;
            if(_httpContextSet[profileId].contentType) {
                _strncpy_s(_httpContextSet[profileId].contentType, contentType,
                           _httpContextSet[profileId].contentTypeSize - 1);
            }
            _httpContextSet[profileId].contentLength = contentLength;
        } else {
            //TODO: report this incomplete ring message as an error. 
            buff->free = true;
            return;
        }
    }
    else if(_buffStartsWith(buff, "+SQNCOAPRING: "))
    {
        const char *rspStr = _buffStr(buff);
        char *commaPos = strchr(rspStr, ',');
        char *start = (char*) rspStr + _strLitLen("+SQNCOAPRING: ");

        char *profileIdStr = NULL;
        char *messageIdStr = NULL;
        char *sendTypeStr = NULL;
        char *reqRspCodeRawStr = NULL;
        char *lengthStr = NULL;

        if(commaPos) {
            *commaPos = '\0';
            profileIdStr = start;
            start = ++commaPos;
            commaPos = strchr(commaPos, ',');

            if(!_coapContextSet[atoi(profileIdStr)].connected ||
               atoi(profileIdStr) >= WALTER_MODEM_MAX_COAP_PROFILES) {
                //TODO: return error if modem returns invalid profile id.
                buff->free = true;
                return;
            }
        }

        if(commaPos) {
            *commaPos = '\0';
            messageIdStr = start;
            start = ++commaPos;
            commaPos = strchr(commaPos, ',');
        }

        if(commaPos) {
            *commaPos = '\0';
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
            reqRspCodeRawStr = start;
            lengthStr = commaPos + 1;

            /* convert parameters to int */
            uint8_t profileId = atoi(profileIdStr);
            uint16_t messageId = atoi(messageIdStr);
            WalterModemCoapSendType sendType = (WalterModemCoapSendType) atoi(sendTypeStr);
            uint8_t reqRspCodeRaw = atoi(reqRspCodeRawStr);
            uint16_t length = atoi(lengthStr);

            if(profileId == 0) {
                /* profile id 0 is the internal BlueCherry cloud coap profile */
                if(blueCherry.lastAckedMessageId != messageId) {
                    WalterModemBuffer *stringsBuffer = _getFreeBuffer();
                    stringsBuffer->size +=
                        sprintf((char*) stringsBuffer->data, "AT+SQNCOAPRCV=%s,%s,%s",
                            profileIdStr, messageIdStr, lengthStr);
                    const char *_cmdArr[WALTER_MODEM_COMMAND_MAX_ELEMS + 1] =
                        arr((const char*) stringsBuffer->data);

                    _addQueueCmd(_cmdArr, "+SQNCOAPRCV: ", NULL, coap_received_from_bluecherry,
                        &blueCherry, NULL, NULL, WALTER_MODEM_CMD_TYPE_TX_WAIT,
                        blueCherry.messageIn, WALTER_MODEM_MAX_INCOMING_MESSAGE_LEN, stringsBuffer);
                }
            } else {
                /* store ring in ring list for this coap context */
                uint8_t ringIdx;
                for(ringIdx = 0; ringIdx < WALTER_MODEM_COAP_MAX_PENDING_RINGS; ringIdx++) {
                    if(!_coapContextSet[profileId].rings[ringIdx].messageId) {
                        break;
                    }
                    if(_coapContextSet[profileId].rings[ringIdx].messageId == messageId &&
                       _coapContextSet[profileId].rings[ringIdx].sendType == sendType &&
                       _coapContextSet[profileId].rings[ringIdx].methodRsp == reqRspCodeRaw) {
                        break;
                    }
                }

                if(ringIdx == WALTER_MODEM_COAP_MAX_PENDING_RINGS) {
                    //TODO: error reporting mechanism for this failed URC
                    buff->free = true;
                    return;
                }

                if(!_coapContextSet[profileId].rings[ringIdx].messageId) {
                    _coapContextSet[profileId].rings[ringIdx].messageId = messageId;
                    _coapContextSet[profileId].rings[ringIdx].sendType = sendType;
                    _coapContextSet[profileId].rings[ringIdx].methodRsp =
                        (WalterModemCoapSendMethodRsp ) reqRspCodeRaw;
                    _coapContextSet[profileId].rings[ringIdx].length = length;
                }
            }
        }
    }
    else if(cmd && cmd->atCmd[0] && !strcmp(cmd->atCmd[0], "AT+SQNSMQTTRCVMESSAGE=0,") &&
            cmd->rsp->type != WALTER_MODEM_RSP_DATA_TYPE_MQTT)
    {
        const char *rspStr = _buffStr(buff);
        uint8_t ringIdx = (uint32_t) cmd->completeHandlerArg;

        cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_MQTT;
        cmd->rsp->data.mqttResponse.messageId = _mqttRings[ringIdx].messageId;
        cmd->rsp->data.mqttResponse.qos = _mqttRings[ringIdx].qos;
        cmd->rsp->data.mqttResponse.length = cmd->dataSize;

        /* free ring entry */
        _mqttRings[ringIdx].messageId = 0;

        if(cmd->data) {
            /* skip leading \r\n */
            memcpy(cmd->data, rspStr + 2, cmd->dataSize);
        }
    }
    else if(_buffStartsWith(buff, "+SQNCOAPCONNECTED: "))
    {
        const char *rspStr = _buffStr(buff);
        char *commaPos = strchr(rspStr, ',');
        if(commaPos) {
            *commaPos = '\0';
        }

        uint8_t profileId = atoi(rspStr + _strLitLen("+SQNCOAPCONNECTED: "));

        if(profileId < WALTER_MODEM_MAX_COAP_PROFILES) {
            _coapContextSet[profileId].connected = true;
        }
    }
    else if(_buffStartsWith(buff, "+SQNCOAPCLOSED: "))
    {
        const char *rspStr = _buffStr(buff);
        char *commaPos = strchr(rspStr, ',');
        if(commaPos) {
            *commaPos = '\0';
        }

        uint8_t profileId = atoi(rspStr + _strLitLen("+SQNCOAPCLOSED: "));

        if(profileId < WALTER_MODEM_MAX_COAP_PROFILES) {
            _coapContextSet[profileId].connected = false;
            /* Clear all pending rings on connection close */
            memset(_coapContextSet[profileId].rings, 0, sizeof(_coapContextSet[profileId].rings));

            if(profileId == 0) {
                /* our own coap profile for BlueCherry was just closed */
                if(blueCherry.status == WALTER_MODEM_BLUECHERRY_STATUS_AWAITING_RESPONSE) {
                    blueCherry.status = WALTER_MODEM_BLUECHERRY_STATUS_TIMED_OUT;
                }
            }
        }
    }
    else if(_buffStartsWith(buff, "+SQNHTTPCONNECT: "))
    {
        const char *rspStr = _buffStr(buff);
        char *commaPos = strchr(rspStr, ',');
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

        //TODO: implement event hook for arduino developers
    }
    else if(_buffStartsWith(buff, "+SQNHTTPDISCONNECT: "))
    {
        const char *rspStr = _buffStr(buff);
        uint8_t profileId = atoi(rspStr + _strLitLen("+SQNHTTPDISCONNECT: "));

        if(profileId < WALTER_MODEM_MAX_HTTP_PROFILES) {
            _httpContextSet[profileId].connected = false;
        }

        //TODO: implement event hook for arduino developers
    }
    else if(_buffStartsWith(buff, "+SQNHTTPSH: "))
    {
        const char *rspStr = _buffStr(buff);
        uint8_t profileId = atoi(rspStr + _strLitLen("+SQNHTTPSH: "));

        if(profileId < WALTER_MODEM_MAX_HTTP_PROFILES) {
            _httpContextSet[profileId].connected = false;
        }
    }
    else if(_buffStartsWith(buff, "+SQNSH: "))
    {
        const char *rspStr = _buffStr(buff);
        int sockId = atoi(rspStr + _strLitLen("+SQNSH: "));

        WalterModemSocket *sock = _socketGet(sockId);

        if(sock) {
            _socketRelease(sock);
        }
    }
    else if(_buffStartsWith(buff, "+SQNSMQTTONCONNECT:0,"))
    {
        const char *rspStr = _buffStr(buff);
        int status = atoi(rspStr + _strLitLen("+SQNSMQTTONCONNECT:0,"));

        _mqttStatus = (WalterModemMqttStatus) status;


        for(size_t i = 0; i < WALTER_MODEM_MQTT_MAX_TOPICS; i++) {
            if(!_mqttTopics[i].free) {
                _mqttSubscribeRaw(_mqttTopics[i].topic,_mqttTopics[i].qos);
            }
        }
        

        if(cmd != NULL) {
            cmd->rsp->data.mqttResponse.mqttStatus = _mqttStatus;
        }

        if(status < 0) {
            result = WALTER_MODEM_STATE_ERROR;
        }
    }
    else if(_buffStartsWith(buff, "+SQNSMQTTONDISCONNECT:0,"))
    {
        const char *rspStr = _buffStr(buff);
        int status = atoi(rspStr + _strLitLen("+SQNSMQTTONDISCONNECT:0,"));

        _mqttStatus = (WalterModemMqttStatus)status;

        if(status < 0) {
            result = WALTER_MODEM_STATE_ERROR;
        }
    }
    else if(_buffStartsWith(buff, "+SQNSMQTTONPUBLISH:0,"))
    {
        const char *rspStr = _buffStr(buff);
        const char *pmid = rspStr + _strLitLen("+SQNSMQTTONPUBLISH:0,");
        const char *statusComma = strchr(pmid, ',');

        if (statusComma) {
            int status = atoi(statusComma + 1);
            _mqttStatus = (WalterModemMqttStatus) status;

            if (cmd != NULL) {
                cmd->rsp->data.mqttResponse.mqttStatus = _mqttStatus;
            }

            if (status < 0) {
                result = WALTER_MODEM_STATE_ERROR;
            }
        }
    }
    else if(_buffStartsWith(buff, "+SQNSMQTTONSUBSCRIBE:0,"))
    {
        const char *rspStr = _buffStr(buff);
        const char *topic = rspStr + _strLitLen("+SQNSMQTTONSUBSCRIBE:0,");
        const char *statusComma = strchr(topic, ',');

        if(statusComma) {
            int status = atoi(statusComma + 1);
            _mqttStatus = (WalterModemMqttStatus) status;

            if (cmd != NULL) {
                cmd->rsp->data.mqttResponse.mqttStatus = _mqttStatus;
            }

            if (status < 0) {
                result = WALTER_MODEM_STATE_ERROR;
            }
        }
    }
    else if(_buffStartsWith(buff, "+SQNSMQTTONMESSAGE:0,"))
    {
        const char *rspStr = _buffStr(buff);
        char *commaPos = strchr(rspStr, ',');
        char *start = (char*) rspStr + _strLitLen("+SQNSMQTTONMESSAGE:0");

        char *topic = NULL;
        char *lenStr = NULL;
        char *qosStr = NULL;
        char *midStr = NULL;

        if(commaPos) {
            /* got mqtt profile (must be 0) */
            *commaPos = '\0';
            // atoi(start) should be 0
            start = ++commaPos;
            commaPos = strchr(commaPos, ',');
        }

        if(commaPos) {
            /* got topic */
            *commaPos = '\0';
            commaPos[-1] = '\0';
            topic = start + 1;
            start = ++commaPos;
            commaPos = strchr(commaPos, ',');
        }

        if(commaPos) {
            /* got len and qos */
            *commaPos = '\0';
            lenStr = start;
            qosStr = commaPos + 1;
            start = ++commaPos;
            commaPos = strchr(commaPos, ',');

            if(commaPos) {
                /* got optional msg id */
                *commaPos = '\0';
                midStr = commaPos + 1;
            }

            /* convert parameters to int */
            uint16_t length = atoi(lenStr);
            uint8_t qos = atoi(qosStr);
            uint16_t messageId = midStr ? atoi(midStr) : 0;

            uint8_t ringIdx;
            for(ringIdx = 0; ringIdx < WALTER_MODEM_MQTT_MAX_PENDING_RINGS; ringIdx++) {
                if(_mqttRings[ringIdx].free) {
                    break;
                }

                if (strncmp(topic, _mqttRings[ringIdx].topic, strlen(topic)) == 0 && _mqttRings[ringIdx].messageId == messageId && _mqttRings[ringIdx].qos == qos) {
                    ESP_LOGD("WalterModem", "mqtt duplicate message!");
                    goto after_processing_logic;
                }
            }

            if(ringIdx == WALTER_MODEM_MQTT_MAX_PENDING_RINGS) {
                _mqttStatus = WALTER_MODEM_MQTT_NOMEM;
                ESP_LOGD("WalterModem","mqtt message buffer was full!");
                goto after_processing_logic;
            }
            
            /* store ring in ring list for this mqtt context */
            if (_mqttRings[ringIdx].free) {
                _mqttRings[ringIdx].messageId = messageId;
                _mqttRings[ringIdx].length = length;
                _mqttRings[ringIdx].qos = qos;
                _strncpy_s(_mqttRings[ringIdx].topic, topic, WALTER_MODEM_MQTT_TOPIC_BUF_SIZE);
            }
        }
    }
    else if(_buffStartsWithDigit(buff))
    {
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
    }

after_processing_logic:
    if(cmd == NULL ||
       cmd->type == WALTER_MODEM_CMD_TYPE_TX ||
       cmd->state == WALTER_MODEM_CMD_STATE_FREE ||
       cmd->atRsp == NULL ||
       cmd->atRspLen > buff->size ||
       memcmp(cmd->atRsp, buff->data, cmd->atRspLen) != 0)
    {
        buff->free = true;
        return;
    }

    _finishQueueCmd(cmd, result);
    buff->free = true;
}

bool WalterModem::_processOtaInitializeEvent(uint8_t *data, uint16_t len)
{
    if(!blueCherry.otaBuffer || len != sizeof(uint32_t)) {
        return true;
    }

    blueCherry.otaSize = *((uint32_t*) data);
    
    /* check if there is enough space on the update partition */
    blueCherry.otaPartition = esp_ota_get_next_update_partition(NULL);
    if(!blueCherry.otaPartition ||
       blueCherry.otaSize > blueCherry.otaPartition->size ||
       blueCherry.otaSize == 0) {
        ESP_LOGD("WalterModem", "OTA init: no OTA partition or size 0 or %lu > %lu",
                blueCherry.otaSize, blueCherry.otaPartition->size);
        return true;
    }

    /* initialize buffer and state */
    blueCherry.otaBufferPos = 0;
    blueCherry.otaProgress = 0;

    ESP_LOGD("WalterModem", "OTA init: size %lu <= partition size %lu",
            blueCherry.otaSize, blueCherry.otaPartition->size);

    return false;
}

bool WalterModem::_otaBufferToFlash(void)
{
    /* first bytes of new firmware must be postponed so
     * partially written firmware is not bootable just yet
     */
    uint8_t skip = 0;

    if(!blueCherry.otaProgress) {
        /* meanwhile check for the magic byte */
        if(blueCherry.otaBuffer[0] != ESP_IMAGE_HEADER_MAGIC) {
            ESP_LOGD("WalterModem", "OTA chunk: magic header not found");
            return false;
        }

        skip = ENCRYPTED_BLOCK_SIZE;
        memcpy(blueCherry.otaSkipBuffer, blueCherry.otaBuffer, skip);
    }

    size_t flashOffset = blueCherry.otaPartition->address + blueCherry.otaProgress;

    // if it's the block boundary, than erase the whole block from here
    bool blockErase = (blueCherry.otaSize - blueCherry.otaProgress >= SPI_FLASH_BLOCK_SIZE) &&
        (flashOffset % SPI_FLASH_BLOCK_SIZE == 0);

    // sector belong to unaligned partition heading block
    bool partitionHeadSectors = blueCherry.otaPartition->address % SPI_FLASH_BLOCK_SIZE &&
        flashOffset <
            (blueCherry.otaPartition->address / SPI_FLASH_BLOCK_SIZE + 1) * SPI_FLASH_BLOCK_SIZE;

    // sector belong to unaligned partition tailing block
    bool partitionTailSectors =
        flashOffset >=(blueCherry.otaPartition->address + blueCherry.otaSize)
        / SPI_FLASH_BLOCK_SIZE
        * SPI_FLASH_BLOCK_SIZE;

    if (blockErase || partitionHeadSectors || partitionTailSectors) {
        if(esp_partition_erase_range(blueCherry.otaPartition, blueCherry.otaProgress,
           blockErase ? SPI_FLASH_BLOCK_SIZE : SPI_FLASH_SEC_SIZE) != ESP_OK) {
            ESP_LOGD("WalterModem", "OTA chunk: could not erase partition");
            return false;
        }
    }

    if(esp_partition_write(blueCherry.otaPartition, blueCherry.otaProgress + skip,
       (uint32_t*) blueCherry.otaBuffer + skip / sizeof(uint32_t),
       blueCherry.otaBufferPos - skip) != ESP_OK) {
        ESP_LOGD("WalterModem", "OTA chunk: could not write data to partition");
        return false;
    }

    blueCherry.otaProgress += blueCherry.otaBufferPos;
    blueCherry.otaBufferPos = 0;

    return true;
}

bool WalterModem::_processOtaChunkEvent(uint8_t *data, uint16_t len)
{
    if(!blueCherry.otaSize || len == 0 || blueCherry.otaProgress + len > blueCherry.otaSize) {
        ESP_LOGD("WalterModem", "OTA: cancelled because empty chunk or chunk beyond update size");
        return true;
    }

    size_t left = len;

    while((blueCherry.otaBufferPos + left) > SPI_FLASH_SEC_SIZE) {
        size_t toBuff = SPI_FLASH_SEC_SIZE - blueCherry.otaBufferPos;

        memcpy(blueCherry.otaBuffer + blueCherry.otaBufferPos, data + (len - left), toBuff);
        blueCherry.otaBufferPos += toBuff;

        if(!_otaBufferToFlash()) {
            ESP_LOGD("WalterModem", "OTA chunk: failed to write to flash (within loop)");
            return true;
        } else {
            ESP_LOGD("WalterModem", "OTA chunk written to flash; progress = %lu / %lu",
                    blueCherry.otaProgress, blueCherry.otaSize);
        }

        left -= toBuff;
    }

    memcpy(blueCherry.otaBuffer + blueCherry.otaBufferPos, data + (len - left), left);
    blueCherry.otaBufferPos += left;

    if(blueCherry.otaProgress + blueCherry.otaBufferPos == blueCherry.otaSize) {
        if(!_otaBufferToFlash()) {
            ESP_LOGD("WalterModem", "OTA chunk: failed to write to flash (remainder)");
            return true;
        } else {
            ESP_LOGD("WalterModem", "OTA remainder written to flash; progress = %lu / %lu",
                    blueCherry.otaProgress, blueCherry.otaSize);
        }
    }

    return false;
}

bool WalterModem::_processOtaFinishEvent(void)
{
    if(!blueCherry.otaSize || blueCherry.otaProgress != blueCherry.otaSize) {
        return true;
    }

    /* enable partition: write the stashed first bytes */
    if(esp_partition_write(blueCherry.otaPartition, 0, (uint32_t*) blueCherry.otaSkipBuffer,
       ENCRYPTED_BLOCK_SIZE) != ESP_OK) {
        ESP_LOGD("WalterModem", "OTA Finish: Could not write start of boot sector to partition");
        return true;
    }

    /* check if partition is bootable */
    if(esp_partition_read(blueCherry.otaPartition, 0, (uint32_t*) blueCherry.otaSkipBuffer,
       ENCRYPTED_BLOCK_SIZE) != ESP_OK) {
        ESP_LOGD("WalterModem", "OTA Finish: Could not read boot partition");
        return true;
    }
    if(blueCherry.otaSkipBuffer[0] != ESP_IMAGE_HEADER_MAGIC) {
        ESP_LOGD("WalterModem", "OTA Finish: Magic header is missing on partition");
        return true;
    }

    if(esp_ota_set_boot_partition(blueCherry.otaPartition)) {
        ESP_LOGD("WalterModem", "OTA Finish: Could not set boot partition");
        return true;
    }

    ESP_LOGD("WalterModem", "OTA Finish: set boot partition. Booting in new firmware.");
    esp_restart();

    return false;
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
    const esp_partition_t *ffat_partition =
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
    esp_vfs_fat_mount_config_t conf = {
      .format_if_mount_failed = true,
      .max_files = 1,
      .allocation_unit_size = CONFIG_WL_SECTOR_SIZE
    };

    result = esp_vfs_fat_spiflash_mount("/ffat", "ffat", &conf, &_wl_handle);
    if(result != ESP_OK) {
        ESP_LOGD("WalterModem", "Mount/format FAT partition failed!");
        _wl_handle = WL_INVALID_HANDLE;
        return false;
    }

    return true;
}

bool WalterModem::_processMotaInitializeEvent(uint8_t *data, uint16_t len)
{
    if(!blueCherry.otaBuffer || len != sizeof(uint32_t)) {
        return true;
    }

    blueCherry.otaSize = *((uint32_t*) data);
    
    if(!_motaFormatAndMount()) {
        ESP_LOGD("WalterModem", "MOTA initialize (format partition) failed");
        return true;
    }

    _mota_file_ptr = fopen("/ffat/mota.dup", "w+");

    /* Initialize bytes written counter */
    blueCherry.otaProgress = 0;

    ESP_LOGD("WalterModem", "MOTA procedure initialized");

    return false;
}

bool WalterModem::_processMotaChunkEvent(uint8_t *data, uint16_t len)
{
    if(!blueCherry.otaSize || len == 0 || blueCherry.otaProgress + len > blueCherry.otaSize) {
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

    blueCherry.otaProgress += len;

    ESP_LOGD("WalterModem", "MOTA: appending new chunk of %d bytes to ESP32 flash; so far %"PRIu32
        "/%"PRIu32" bytes written", len, blueCherry.otaProgress, blueCherry.otaSize);

    return false;
}

uint16_t WalterModem::_calculateStpCrc16(const void *input, size_t length)
{
    uint16_t crc = 0;
    const uint8_t *data = (const uint8_t*) input;

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

uint16_t WalterModem::_modemFirmwareUpgradeStart(void)
{
    char *atCmd[WALTER_MODEM_COMMAND_MAX_ELEMS + 1] = { NULL };
    int len;

    _rxHandlerInterrupted = true;

    /* reboot to recovery */
    vTaskDelay(pdMS_TO_TICKS(5000));
    tickleWatchdog();
    atCmd[0] = "AT+SMSWBOOT=3,1";
    atCmd[1] = NULL;
    _transmitCmd(WALTER_MODEM_CMD_TYPE_TX, atCmd);
    ESP_LOGD("WalterModem", "sent reboot to recovery command, waiting 10 seconds");
    vTaskDelay(pdMS_TO_TICKS(10000));
    tickleWatchdog();

    /* check if booted in recovery mode */
    atCmd[0] = "AT";
    atCmd[1] = NULL;
    _transmitCmd(WALTER_MODEM_CMD_TYPE_TX, atCmd);

    len = _uartRead(blueCherry.otaBuffer, 6);
    blueCherry.otaBuffer[len] = 0;
    ESP_LOGD("WalterModem", "sent AT, got %d:%s", len, blueCherry.otaBuffer);

    atCmd[0] = "AT+SMLOG?";
    atCmd[1] = NULL;
    _transmitCmd(WALTER_MODEM_CMD_TYPE_TX, atCmd);

    len = _uartRead(blueCherry.otaBuffer, 25);
    blueCherry.otaBuffer[len] = 0;
    ESP_LOGD("WalterModem", "sent AT+SMLOG?, got %d:%s", len, blueCherry.otaBuffer);

    atCmd[0] = "AT+SMOD?";
    atCmd[1] = NULL;
    _transmitCmd(WALTER_MODEM_CMD_TYPE_TX, atCmd);

    len = _uartRead(blueCherry.otaBuffer, 7);
    blueCherry.otaBuffer[len] = 0;
    ESP_LOGD("WalterModem", "sent AT+SMOD?, got %d:%s", len, blueCherry.otaBuffer);

    /* prepare modem firmware data transfer - must wait for OK still!! */
    atCmd[0] = "AT+SMSTPU=\"ON_THE_FLY\"";
    atCmd[1] = NULL;
    _transmitCmd(WALTER_MODEM_CMD_TYPE_TX, atCmd);

    vTaskDelay(pdMS_TO_TICKS(2000));
    len = _uartRead(blueCherry.otaBuffer, 64);
    blueCherry.otaBuffer[len] = 0;
    ESP_LOGD("WalterModem", "started STP mode, got %d:%s", len, blueCherry.otaBuffer);

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
        "sent STP reset: tx=%d header: 0x%"PRIx32" 0x%x 0x%x %d %"PRIu32" 0x%x 0x%x",
        bytesSent,
        _switchEndian32(stpRequest.signature),
        stpRequest.operation,
        stpRequest.sessionId,
        _switchEndian16(stpRequest.payloadLength),
        _switchEndian32(stpRequest.transactionId),
        _switchEndian16(stpRequest.headerCrc16),
        _switchEndian16(stpRequest.payloadCrc16));

    bytesReceived = _uartRead((uint8_t*) &stpRequest, sizeof(stpRequest), true);

    ESP_LOGD("WalterModem",
        "received STP reset ack: rx=%d header: 0x%"PRIx32" 0x%x 0x%x %d %"PRIu32" 0x%x 0x%x",
        bytesReceived,
        _switchEndian32(stpRequest.signature),
        stpRequest.operation,
        stpRequest.sessionId,
        _switchEndian16(stpRequest.payloadLength),
        _switchEndian32(stpRequest.transactionId),
        _switchEndian16(stpRequest.headerCrc16),
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
        "sent STP open session: tx=%d header: 0x%"PRIx32" 0x%x 0x%x %d %"PRIu32" 0x%x 0x%x",
        bytesSent,
        _switchEndian32(stpRequest.signature),
        stpRequest.operation,
        stpRequest.sessionId,
        _switchEndian16(stpRequest.payloadLength),
        _switchEndian32(stpRequest.transactionId),
        _switchEndian16(stpRequest.headerCrc16),
        _switchEndian16(stpRequest.payloadCrc16));

    bytesReceived = _uartRead((uint8_t*) &stpRequest, sizeof(stpRequest), true);

    ESP_LOGD("WalterModem",
        "received STP open session ack: rx=%d header: 0x%"PRIx32" 0x%x 0x%x %d %"PRIu32" 0x%x 0x%x",
        bytesReceived,
        _switchEndian32(stpRequest.signature),
        stpRequest.operation,
        stpRequest.sessionId,
        _switchEndian16(stpRequest.payloadLength),
        _switchEndian32(stpRequest.transactionId),
        _switchEndian16(stpRequest.headerCrc16),
        _switchEndian16(stpRequest.payloadCrc16));

    bytesReceived =
        _uartRead((uint8_t*) &stpResponseSessionOpen, sizeof(stpResponseSessionOpen), true);

    ESP_LOGD("WalterModem", "received STP open session ack data: rx=%d payload: %d %d %d",
        bytesReceived,
        stpResponseSessionOpen.success,
        stpResponseSessionOpen.version,
        _switchEndian16(stpResponseSessionOpen.maxTransferSize));

    tickleWatchdog();

    return _switchEndian16(stpResponseSessionOpen.maxTransferSize) - sizeof(stpRequest);
}

void WalterModem::_modemFirmwareUpgradeFinish(bool success)
{
    char *atCmd[WALTER_MODEM_COMMAND_MAX_ELEMS + 1] = { NULL };
    int len;

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
        "sent STP reset: tx=%d header: 0x%"PRIx32" 0x%x 0x%x %d %"PRIu32" 0x%x 0x%x",
        bytesSent,
        _switchEndian32(stpRequest.signature),
        stpRequest.operation,
        stpRequest.sessionId,
        _switchEndian16(stpRequest.payloadLength),
        _switchEndian32(stpRequest.transactionId),
        _switchEndian16(stpRequest.headerCrc16),
        _switchEndian16(stpRequest.payloadCrc16));

    bytesReceived = _uartRead((uint8_t*) &stpRequest, sizeof(stpRequest), true);

    ESP_LOGD("WalterModem",
        "received STP reset ack: rx=%d header: 0x%"PRIx32" 0x%x 0x%x %d %"PRIu32" 0x%x 0x%x",
        bytesReceived,
        _switchEndian32(stpRequest.signature),
        stpRequest.operation,
        stpRequest.sessionId,
        _switchEndian16(stpRequest.payloadLength),
        _switchEndian32(stpRequest.transactionId),
        _switchEndian16(stpRequest.headerCrc16),
        _switchEndian16(stpRequest.payloadCrc16));

    /* send AT and retry until we get OK */
    for(;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));

        atCmd[0] = "AT";
        atCmd[1] = NULL;
        _transmitCmd(WALTER_MODEM_CMD_TYPE_TX, atCmd);

        /* reuse otaBuffer which is guaranteed to be 4K */
        len = _uartRead(blueCherry.otaBuffer, 32);
        blueCherry.otaBuffer[len] = 0;
        ESP_LOGD("WalterModem", "sent AT, got %d:%s", len, blueCherry.otaBuffer);

        if(!strcmp((char*) blueCherry.otaBuffer, "\r\nOK\r\n")) {
            break;
        }
    }

    /* we got OK so ready to boot into new firmware; switch back to FFF mode */
    atCmd[0] = "AT+SMSWBOOT=1,0";
    atCmd[1] = NULL;
    _transmitCmd(WALTER_MODEM_CMD_TYPE_TX, atCmd);

    len = _uartRead(blueCherry.otaBuffer, 16);
    blueCherry.otaBuffer[len] = 0;
    ESP_LOGD("WalterModem", "switched modem to FFF mode, got %d:%s", len, blueCherry.otaBuffer);

    /* now reboot into new firmware */
    atCmd[0] = "AT^RESET";
    atCmd[1] = NULL;
    _transmitCmd(WALTER_MODEM_CMD_TYPE_TX, atCmd);
    ESP_LOGD("WalterModem", "sent reset command, waiting 10 seconds");
    vTaskDelay(pdMS_TO_TICKS(10000));

    len = _uartRead(blueCherry.otaBuffer, 64);
    blueCherry.otaBuffer[len] = 0;
    ESP_LOGD("WalterModem", "assuming modem boot complete; got %d:%s", len, blueCherry.otaBuffer);

    /* check if we are back in fff mode and check update status */
    atCmd[0] = "AT+SMLOG?";
    atCmd[1] = NULL;
    _transmitCmd(WALTER_MODEM_CMD_TYPE_TX, atCmd);

    len = _uartRead(blueCherry.otaBuffer, 64);
    blueCherry.otaBuffer[len] = 0;
    ESP_LOGD("WalterModem", "AT+SMLOG? got %d:%s", len, blueCherry.otaBuffer);

    atCmd[0] = "AT+SMOD?";
    atCmd[1] = NULL;
    _transmitCmd(WALTER_MODEM_CMD_TYPE_TX, atCmd);

    len = _uartRead(blueCherry.otaBuffer, 64);
    blueCherry.otaBuffer[len] = 0;
    ESP_LOGD("WalterModem", "AT+SMOD? got %d:%s", len, blueCherry.otaBuffer);

    atCmd[0] = "AT+SMUPGRADE?";
    atCmd[1] = NULL;
    _transmitCmd(WALTER_MODEM_CMD_TYPE_TX, atCmd);

    len = _uartRead(blueCherry.otaBuffer, 64);
    blueCherry.otaBuffer[len] = 0;
    ESP_LOGD("WalterModem", "AT+SMUPGRADE? got %d:%s", len, blueCherry.otaBuffer);

    _rxHandlerInterrupted = false;
}

void WalterModem::_modemFirmwareUpgradeBlock(size_t blockSize, uint32_t transactionId)
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
        "sent STP transfer block command: tx=%d header: 0x%"PRIx32" 0x%x 0x%x %d %"PRIu32" 0x%x 0x%x",
        bytesSent,
        _switchEndian32(stpRequest.signature),
        stpRequest.operation,
        stpRequest.sessionId,
        _switchEndian16(stpRequest.payloadLength),
        _switchEndian32(stpRequest.transactionId),
        _switchEndian16(stpRequest.headerCrc16),
        _switchEndian16(stpRequest.payloadCrc16));

    bytesSent =
        _uartWrite((uint8_t*) &stpRequestTransferBlockCmd, sizeof(stpRequestTransferBlockCmd));

    ESP_LOGD("WalterModem", "sent STP transfer block command data: sent=%d payload: %d",
        bytesSent, _switchEndian16(stpRequestTransferBlockCmd.blockSize));

    bytesReceived = _uartRead((uint8_t*) &stpRequest, sizeof(stpRequest), true);

    ESP_LOGD("WalterModem",
        "received STP transfer block command ack: rx=%d header: "
        "0x%"PRIx32" 0x%x 0x%x %d %"PRIu32" 0x%x 0x%x",
        bytesReceived,
        _switchEndian32(stpRequest.signature),
        stpRequest.operation,
        stpRequest.sessionId,
        _switchEndian16(stpRequest.payloadLength),
        _switchEndian32(stpRequest.transactionId),
        _switchEndian16(stpRequest.headerCrc16),
        _switchEndian16(stpRequest.payloadCrc16));

    /* transfer block: actual data transfer */
    stpRequest.signature = _switchEndian32(WALTER_MODEM_STP_SIGNATURE_REQUEST);
    stpRequest.operation = WALTER_MODEM_STP_OPERATION_TRANSFER_BLOCK;
    stpRequest.sessionId = 1;
    stpRequest.payloadLength = _switchEndian16(blockSize);
    stpRequest.transactionId = _switchEndian32(transactionId + 1);
    stpRequest.headerCrc16 = 0;
    stpRequest.payloadCrc16 = _calculateStpCrc16(blueCherry.otaBuffer, blockSize);
    stpRequest.headerCrc16 = _calculateStpCrc16(&stpRequest, sizeof(stpRequest));

    bytesSent = _uartWrite((uint8_t*) &stpRequest, sizeof(stpRequest));

    ESP_LOGD("WalterModem",
        "sent STP transfer block: tx=%d header: 0x%"PRIx32" 0x%x 0x%x %d %"PRIu32" 0x%x 0x%x",
        bytesSent,
        _switchEndian32(stpRequest.signature),
        stpRequest.operation,
        stpRequest.sessionId,
        _switchEndian16(stpRequest.payloadLength),
        _switchEndian32(stpRequest.transactionId),
        _switchEndian16(stpRequest.headerCrc16),
        _switchEndian16(stpRequest.payloadCrc16));

    bytesSent = _uartWrite(blueCherry.otaBuffer, blockSize);

    ESP_LOGD("WalterModem",
        "sent STP transfer block data: tx=%d payload: %d bytes data from flash dup file",
        bytesSent, blockSize);

    bytesReceived = _uartRead((uint8_t*) &stpRequest, sizeof(stpRequest), true);

    ESP_LOGD("WalterModem",
        "received STP transfer block ack: rx=%d header: "
        "0x%"PRIx32" 0x%x 0x%x %d %"PRIu32" 0x%x 0x%x",
        bytesReceived,
        _switchEndian32(stpRequest.signature),
        stpRequest.operation,
        stpRequest.sessionId,
        _switchEndian16(stpRequest.payloadLength),
        _switchEndian32(stpRequest.transactionId),
        _switchEndian16(stpRequest.headerCrc16),
        _switchEndian16(stpRequest.payloadCrc16));

    bytesReceived =
        _uartRead((uint8_t*) &stpResponseTransferBlock, sizeof(stpResponseTransferBlock), true);

    ESP_LOGD("WalterModem", "received STP transfer block ack data: received=%d payload: %d",
        bytesReceived, _switchEndian16(stpResponseTransferBlock.residue));
}

bool WalterModem::_processMotaFinishEvent(void)
{
    char *atCmd[WALTER_MODEM_COMMAND_MAX_ELEMS + 1] = { NULL };

    if(!blueCherry.otaSize || blueCherry.otaProgress != blueCherry.otaSize || !_mota_file_ptr) {
        ESP_LOGD("WalterModem", "MOTA error: incomplete or missing dup file");
        return true;
    }

    /* prepare modem, disable rx until done so we can talk with modem directly */
    uint16_t blockSize = _modemFirmwareUpgradeStart();
    if(blockSize > SPI_FLASH_BLOCK_SIZE) {
        blockSize = SPI_FLASH_BLOCK_SIZE;
    }

    fseek(_mota_file_ptr, 0L, SEEK_SET);

    uint32_t transactionId = 2;
    long bytesLeft = blueCherry.otaSize;

    while(bytesLeft > 0) {
        /* we can reuse otaBuffer since we expect it to be at least 4K (SPI_FLASH_BLOCK_SIZE) */
        size_t bytesRead = fread(blueCherry.otaBuffer, 1, blockSize, _mota_file_ptr);
        if(bytesRead <= 0) {
            break;
        }

        /* send this chunk to the modem */
        _modemFirmwareUpgradeBlock(bytesRead, transactionId);
        transactionId += 2;

        /* next block */
        bytesLeft -= bytesRead;
        ESP_LOGD("WalterModem",
            "sent chunk of %d bytes from ESP flash to modem; so far %"PRIu32"/%"PRIu32" bytes sent",
            bytesRead, blueCherry.otaSize - bytesLeft, blueCherry.otaSize);

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

bool WalterModem::_processBlueCherryEvent(uint8_t *data, uint8_t len)
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
            ESP_LOGD("WalterModem", "Error: invalid BlueCherry event type 0x%x from cloud server",
                data[0]);
            return true;
    }

    return true;
}

void WalterModem::offlineMotaUpgrade(uint8_t *otaBuffer)
{
    if(_wl_handle == WL_INVALID_HANDLE) {
        esp_err_t result;
        esp_vfs_fat_mount_config_t conf = {
          .format_if_mount_failed = false,
          .max_files = 1,
          .allocation_unit_size = CONFIG_WL_SECTOR_SIZE
        };
        result = esp_vfs_fat_spiflash_mount("/ffat", "ffat", &conf, &_wl_handle);
        if (result != ESP_OK) {
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
            blueCherry.otaBuffer = otaBuffer;

            fseek(_mota_file_ptr, 0L, SEEK_END);
            blueCherry.otaProgress = blueCherry.otaSize = ftell(_mota_file_ptr);
            fseek(_mota_file_ptr, 0L, SEEK_SET);

            if(_processMotaFinishEvent()) {
                ESP_LOGD("WalterModem", "MOTA offline update failed");
            } else {
                ESP_LOGD("WalterModem", "MOTA offline update succeeded");
            }
        }
    }
}

#ifdef ARDUINO
bool WalterModem::begin(HardwareSerial *uart, uint8_t watchdogTimeout)
#else
bool WalterModem::begin(uart_port_t uartNo, uint8_t watchdogTimeout)
#endif
{
    if(_initialized) {
        return true;
    }

    _watchdogTimeout = watchdogTimeout;
    if(_watchdogTimeout) {
        /* wdt timeout must be longer than max wait time for a modem response */
        if(_watchdogTimeout * 1000UL < WALTER_MODEM_CMD_TIMEOUT_MS + 5000UL) {
            _watchdogTimeout = (WALTER_MODEM_CMD_TIMEOUT_MS / 1000UL) + 5;
        }
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
        esp_task_wdt_init(_watchdogTimeout, true);
#else
        esp_task_wdt_config_t twdt_config = {
            .timeout_ms = (uint32_t) (_watchdogTimeout * 1000UL),
            .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
            .trigger_panic = true
        };
#if CONFIG_ESP_TASK_WDT_INIT
        esp_task_wdt_reconfigure(&twdt_config);
#else
        esp_task_wdt_init(&twdt_config);
#endif
#endif
        esp_task_wdt_add(NULL);
    }
    
    _taskQueue.handle = xQueueCreateStatic(WALTER_MODEM_TASK_QUEUE_MAX_ITEMS,
        sizeof(WalterModemTaskQueueItem), _taskQueue.mem, &(_taskQueue.memHandle));

    gpio_set_direction((gpio_num_t) WALTER_MODEM_PIN_RESET, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode((gpio_num_t) WALTER_MODEM_PIN_RESET, GPIO_FLOATING);
    gpio_deep_sleep_hold_en();

#ifdef ARDUINO
    _uart = uart;
    _uart->begin(
        WALTER_MODEM_BAUD,
        SERIAL_8N1,
        WALTER_MODEM_PIN_RX,
        WALTER_MODEM_PIN_TX);

    _uart->setPins(
        WALTER_MODEM_PIN_RX,
        WALTER_MODEM_PIN_TX,
        WALTER_MODEM_PIN_CTS,
        WALTER_MODEM_PIN_RTS);

    _uart->setHwFlowCtrlMode();
    _uart->setRxTimeout(1);
    _uart->onReceive(_handleRxData);
#else
    const uart_config_t uart_config = {
        .baud_rate = WALTER_MODEM_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT
    };
    _uartNo = uartNo;
    uart_driver_install(uartNo, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(uartNo, &uart_config);
    uart_set_pin(uartNo,
        WALTER_MODEM_PIN_TX,
        WALTER_MODEM_PIN_RX,
        WALTER_MODEM_PIN_RTS,
        WALTER_MODEM_PIN_CTS);

    xTaskCreateStaticPinnedToCore(_handleRxData, "uart_rx_task", WALTER_MODEM_TASK_STACK_SIZE, NULL,
        tskIDLE_PRIORITY, _rxTaskStack, &_rxTaskBuf, 0);
#endif

#ifdef ARDUINO
    _queueTask = xTaskCreateStaticPinnedToCore(_queueProcessingTask, "queueProcessingTask",
        WALTER_MODEM_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY, _queueTaskStack, &_queueTaskBuf, 1);
#else
    _queueTask = xTaskCreateStaticPinnedToCore(_queueProcessingTask, "queueProcessingTask",
        WALTER_MODEM_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY, _queueTaskStack, &_queueTaskBuf, 0);
#endif

    esp_sleep_wakeup_cause_t wakeupReason;
    wakeupReason = esp_sleep_get_wakeup_cause();

    if(wakeupReason == ESP_SLEEP_WAKEUP_UNDEFINED) {
        if (!reset()) {
            return false;
        }
    } else {
        _sleepWakeup();
    }

    /* Configure reports as we expect them in the library */
    if(!configCMEErrorReports()) {
        return false;
    }

    if(!configCEREGReports()) {
        return false;
    }

    _initialized = true;
    return true;
}

void WalterModem::tickleWatchdog(void)
{
    if(_watchdogTimeout) {
        esp_task_wdt_reset();
    }
}

bool WalterModem::sendCmd(const char *cmd)
{
    const char *_cmdArr[WALTER_MODEM_COMMAND_MAX_ELEMS + 1] = { cmd };
    return _addQueueCmd(_cmdArr, NULL, NULL, NULL, NULL, NULL, NULL, WALTER_MODEM_CMD_TYPE_TX)
        != nullptr;
}

bool WalterModem::softReset(WalterModemRsp *rsp, walterModemCb cb, void *args)
{
    _runCmd({"AT^RESET"}, "+SYSSTART", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::reset(WalterModemRsp *rsp, walterModemCb cb, void *args)
{
    _runCmd({}, "+SYSSTART", rsp, cb, args, NULL, NULL, WALTER_MODEM_CMD_TYPE_WAIT);

    gpio_hold_dis((gpio_num_t) WALTER_MODEM_PIN_RESET);
    gpio_set_level((gpio_num_t) WALTER_MODEM_PIN_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level((gpio_num_t) WALTER_MODEM_PIN_RESET, 1);
    gpio_hold_en((gpio_num_t) WALTER_MODEM_PIN_RESET);

    /* Also (re)initialize internal modem related library state */
    _regState = WALTER_MODEM_NETWORK_REG_NOT_SEARCHING;
    _httpCurrentProfile = 0xff;
    _opState = WALTER_MODEM_OPSTATE_MINIMUM;
    _networkSelMode = WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC;
    _socket = NULL;
    _pdpCtx = NULL;
    _simPIN = NULL;

    for(int i = 0; i < WALTER_MODEM_MAX_PDP_CTXTS; ++i) {
      _pdpCtxSet[i] = {};
    }

    for(int i = 0; i < WALTER_MODEM_MAX_SOCKETS; ++i) {
      _socketSet[i] = {};
    }

    for(int i = 0; i < WALTER_MODEM_MAX_COAP_PROFILES; ++i) {
      _coapContextSet[i] = {};
    }

    for(int i = 0; i < WALTER_MODEM_MAX_HTTP_PROFILES; ++i) {
      _httpContextSet[i] = {};
    }

    for(int i = 0; i < WALTER_MODEM_MQTT_MAX_PENDING_RINGS; i++) {
        _mqttRings[i] = {};
    }

    _operator = {};

    _returnAfterReply();
}

void WalterModem::_sleepPrepare()
{
    memcpy(_pdpCtxSetRTC, _pdpCtxSet, WALTER_MODEM_MAX_PDP_CTXTS * sizeof(WalterModemPDPContext));
    memcpy(_coapCtxSetRTC, _coapContextSet,
        WALTER_MODEM_MAX_COAP_PROFILES * sizeof(WalterModemCoapContext));
    memcpy(_mqttTopicSetRTC, _mqttTopics,
        WALTER_MODEM_MQTT_MAX_TOPICS *sizeof(WalterModemMqttTopic));
    blueCherryRTC = blueCherry;
}

void WalterModem::_sleepWakeup()
{
    memcpy(_pdpCtxSet, _pdpCtxSetRTC, WALTER_MODEM_MAX_PDP_CTXTS * sizeof(WalterModemPDPContext));
    memcpy(_coapContextSet, _coapCtxSetRTC,
        WALTER_MODEM_MAX_COAP_PROFILES * sizeof(WalterModemCoapContext));
    memcpy(_mqttTopics, _mqttTopicSetRTC,
        WALTER_MODEM_MQTT_MAX_TOPICS * sizeof(WalterModemMqttTopic));

    for (size_t i = 0; i < WALTER_MODEM_MAX_PDP_CTXTS; i++) {
        if(_pdpCtxSet[i].state == WALTER_MODEM_PDP_CONTEXT_STATE_ACTIVE) {
            _pdpCtx = _pdpCtxSet + i;
        }
    }
    

    blueCherry = blueCherryRTC;
}

void WalterModem::sleep(uint32_t sleepTime, bool lightSleep)
{
    if(lightSleep) {
        /* Disable RTS (make it high) so the modem can go to sleep */
#ifdef ARDUINO
        _uart->setHwFlowCtrlMode(UART_HW_FLOWCTRL_DISABLE);
        pinMode((gpio_num_t)WALTER_MODEM_PIN_RTS, OUTPUT);
        digitalWrite((gpio_num_t)WALTER_MODEM_PIN_RTS, HIGH);
#else
        uart_set_hw_flow_ctrl(_uartNo, UART_HW_FLOWCTRL_DISABLE, 0);
        gpio_set_direction((gpio_num_t)WALTER_MODEM_PIN_RTS, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)WALTER_MODEM_PIN_RTS, 1);
#endif

        esp_sleep_enable_timer_wakeup(sleepTime * 1000000);
        esp_light_sleep_start();

        /* Re-enable RTS after waking up */
#ifdef ARDUINO
        _uart->setPins(
            WALTER_MODEM_PIN_RX,
            WALTER_MODEM_PIN_TX,
            WALTER_MODEM_PIN_CTS,
            WALTER_MODEM_PIN_RTS);
        _uart->setHwFlowCtrlMode();
#else
        uart_set_pin(_uartNo,
            WALTER_MODEM_PIN_TX,
            WALTER_MODEM_PIN_RX,
            WALTER_MODEM_PIN_RTS,
            WALTER_MODEM_PIN_CTS);
        uart_set_hw_flow_ctrl(_uartNo, UART_HW_FLOWCTRL_CTS_RTS, 122);
#endif
    } else {
        _sleepPrepare();
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_deep_sleep(sleepTime * 1000000);
    }
}

bool WalterModem::checkComm(WalterModemRsp *rsp, walterModemCb cb, void *args)
{
    _runCmd({"AT"}, "OK", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::configCMEErrorReports(
    WalterModemCMEErrorReportsType type,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    _runCmd(arr("AT+CMEE=", _digitStr(type)), "OK", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::configCEREGReports(
    WalterModemCEREGReportsType type, 
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    _runCmd(arr("AT+CEREG=", _digitStr(type)), "OK", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::getRSSI(WalterModemRsp *rsp, walterModemCb cb, void *args)
{
    _runCmd(arr("AT+CSQ"), "OK", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::getSignalQuality(WalterModemRsp *rsp, walterModemCb cb, void *args)
{
    _runCmd(arr("AT+CESQ"), "OK", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::getCellInformation(
    WalterModemSQNMONIReportsType type,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    _runCmd(arr("AT+SQNMONI=", _digitStr(type)), "OK", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::getIdentity(WalterModemRsp *rsp, walterModemCb cb, void *args)
{
    _runCmd(arr("AT+CGSN=2"), "OK", rsp, cb, args);
    _returnAfterReply();
}   

WalterModemMqttStatus WalterModem::getMqttStatus(){
    return _mqttStatus;
}

bool WalterModem::mqttConfig(
    const char *clientId,
    const char *userName,
    const char *password,
    uint8_t tlsProfileId)
{
    WalterModemRsp *rsp = NULL;
    walterModemCb cb = NULL;
    void *args = NULL;

    WalterModemBuffer*  stringsBuffer = _getFreeBuffer();
    stringsBuffer->size += sprintf((char *)stringsBuffer->data,
        "AT+SQNSMQTTCFG=0,\"%s\"", clientId);

    if(userName && password) {
        stringsBuffer->size += sprintf(
            (char *)stringsBuffer->data + stringsBuffer->size,
            ",\"%s\",\"%s\"", userName, password);
    } else {
        stringsBuffer->size += sprintf(
            (char *) stringsBuffer->data + stringsBuffer->size,
            ",,");
    }

    if(tlsProfileId) {
            stringsBuffer->size += sprintf(
            (char *) stringsBuffer->data + stringsBuffer->size, 
            ",%u", tlsProfileId);
    }
    _runCmd(arr((const char *) stringsBuffer->data), "OK", rsp, cb, args);
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

    _runCmd(arr(
        "AT+SQNSMQTTCONNECT=0,",
        _atStr(serverName), ",",
        _atNum(port),",",
        _atNum(keepAlive)),
        "+SQNSMQTTONCONNECT:0,", rsp, cb, args);
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

    if (getNetworkRegState() != WALTER_MODEM_NETWORK_REG_REGISTERED_HOME 
     && getNetworkRegState() != WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING) {
        ESP_LOGD("WalterModem","network is not connected!");
        _returnState(WALTER_MODEM_STATE_ERROR);
    }

    _runCmd(arr(
        "AT+SQNSMQTTPUBLISH=0,",
        _atStr(topicString), ",",
        _atNum(qos), ",",
        _atNum(dataSize)),
        "+SQNSMQTTONPUBLISH:0,",
        rsp, cb, args, NULL, NULL, WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT, data, dataSize);
    _returnAfterReply();
}

static void mqtt_resubscribe_callback(const WalterModemRsp *rsp, void *args) {
    /*This is an empty callback so the _runCmd() runs async*/
}
bool WalterModem::_mqttSubscribeRaw(
    const char *topicString,
    uint8_t qos,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    _runCmd(arr("AT+SQNSMQTTSUBSCRIBE=0,", _atStr(topicString), ",", _atNum(qos)), NULL, rsp, mqtt_resubscribe_callback, args);
    _returnState(WALTER_MODEM_STATE_OK);
}

bool WalterModem::mqttSubscribe(
    const char *topicString,
    uint8_t qos,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    
    int index = -1;

    for (size_t i = 0; i < WALTER_MODEM_MQTT_MAX_TOPICS; i++) {
        if(_mqttTopics[i].free) {
            index = i;
            /*Reserve the topic by setting free to false*/
            _mqttTopics[i].free = false;
            _mqttTopics[i].qos = qos;
            _currentTopic = &_mqttTopics[i];
            _strncpy_s(_mqttTopics[i].topic, topicString, WALTER_MODEM_MQTT_TOPIC_BUF_SIZE);

            break;
        }
    }

    if(index < 0) {
        _currentTopic = NULL;
        rsp->data.mqttResponse.mqttStatus = WALTER_MODEM_MQTT_UNAVAILABLE;
        _returnState(WALTER_MODEM_STATE_ERROR);
    }

    auto completeHandler = [](WalterModemCmd *cmd, WalterModemState result) {
        if(result == WALTER_MODEM_STATE_ERROR) {
            /*If subscription was not succesfull free the topic so we can try again.*/
            _currentTopic->free = true;
        }
    };


    _runCmd(arr("AT+SQNSMQTTSUBSCRIBE=0,", _atStr(topicString), ",", _atNum(qos)), "+SQNSMQTTONSUBSCRIBE:0,", rsp, cb, args, completeHandler);
    _returnAfterReply();
}

bool WalterModem::tlsConfigProfile(
    uint8_t profileId,
    WalterModemTlsValidation tlsValid,
    WalterModemTlsVersion tlsVersion,
    uint8_t caCertificateId,
    uint8_t clientCertificateId,
    uint8_t clientPrivKeyId,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    if(profileId >= WALTER_MODEM_MAX_TLS_PROFILES) {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
    }

    WalterModemBuffer *stringsBuffer = _getFreeBuffer();
    stringsBuffer->size +=
        sprintf((char*) stringsBuffer->data, "AT+SQNSPCFG=%u,%d,\"\",%d,", profileId, tlsVersion,
        tlsValid);
    if(caCertificateId != 0xff) {
        stringsBuffer->size +=
            sprintf((char*) stringsBuffer->data + stringsBuffer->size, "%d", caCertificateId);
    }
    stringsBuffer->size +=
        sprintf((char*) stringsBuffer->data + stringsBuffer->size, ",");
    if(clientCertificateId != 0xff) {
        stringsBuffer->size +=
            sprintf((char*) stringsBuffer->data + stringsBuffer->size, "%d", clientCertificateId);
    }
    stringsBuffer->size +=
        sprintf((char*) stringsBuffer->data + stringsBuffer->size, ",");
    if(clientPrivKeyId != 0xff) {
        stringsBuffer->size +=
            sprintf((char*) stringsBuffer->data + stringsBuffer->size, "%d", clientPrivKeyId);
    }
    stringsBuffer->size +=
        sprintf((char*) stringsBuffer->data + stringsBuffer->size, ",\"\",\"\",0,0,0");

    _runCmd(arr(
        (const char*) stringsBuffer->data),
        "OK", rsp, cb, args, NULL, NULL, WALTER_MODEM_CMD_TYPE_TX_WAIT, NULL, 0, stringsBuffer);

    _returnAfterReply();
}

bool WalterModem::tlsWriteCredential(
    bool isPrivateKey,
    uint8_t slotIdx,
    const char *credential)
{
    WalterModemRsp *rsp = NULL;
    walterModemCb cb = NULL;
    void *args = NULL;

    const char *keyType = isPrivateKey ? "privatekey" : "certificate";

    _runCmd(arr(
        "AT+SQNSNVW=",
        _atStr(keyType), ",",
        _atNum(slotIdx), ",",
        _atNum(strlen(credential))),
        "OK", rsp, cb, args, NULL, NULL, WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT,
        (uint8_t*) credential, strlen(credential));

    _returnAfterReply();
}

char WalterModem::_getLuhnChecksum(const char *imei)
{
    int sum = 0;

    for(int i = 13; i >= 0; --i) {
        int digit = imei[i] - '0';

        if((13 - i) % 2 == 0) {
            int double_digit = digit * 2;
            if (double_digit > 9) {
                double_digit -= 9;
            }
            sum += double_digit;
        } else {
            sum += digit;
        }
    }

    return (char) (((10 - (sum % 10)) % 10) + '0');
}

void WalterModem::_checkEventDuration(
    const std::chrono::time_point<std::chrono::steady_clock>& start)
{
    auto end = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    if (elapsedTime > WALTER_MODEM_MAX_EVENT_DURATION_MS) {
        ESP_LOGW("WalterModem","The event handler took %lldms, preferred max is %dms",
            static_cast<long long>(elapsedTime), WALTER_MODEM_MAX_EVENT_DURATION_MS);
    }
}

void WalterModem::_dispatchEvent(WalterModemNetworkRegState state)
{
    WalterModemEventHandler *handler = _eventHandlers + WALTER_MODEM_EVENT_TYPE_REGISTRATION;
    if(handler->regHandler == nullptr) {
        return;
    }

    auto start = std::chrono::steady_clock::now();
    handler->regHandler(state, handler->args);
    _checkEventDuration(start);
}

void WalterModem::_dispatchEvent(WalterModemSystemEvent event)
{
    WalterModemEventHandler *handler = _eventHandlers + WALTER_MODEM_EVENT_TYPE_SYSTEM;
    if(handler->sysHandler == nullptr) {
        return;
    }

    auto start = std::chrono::steady_clock::now();
    handler->sysHandler(event, handler->args);
    _checkEventDuration(start);
}

void WalterModem::_dispatchEvent(const char *buff, size_t len)
{
    WalterModemEventHandler *handler = _eventHandlers + WALTER_MODEM_EVENT_TYPE_AT;
    if(handler->atHandler == nullptr) {
        return;
    }

    auto start = std::chrono::steady_clock::now();
    handler->atHandler(buff, len, handler->args);
    _checkEventDuration(start);
}

void WalterModem::_dispatchEvent(const WalterModemGNSSFix *fix)
{
    WalterModemEventHandler *handler = _eventHandlers + WALTER_MODEM_EVENT_TYPE_GNSS;
    if(handler->gnssHandler == nullptr) {
        return;
    }

    auto start = std::chrono::steady_clock::now();
    handler->gnssHandler(fix, handler->args);
    _checkEventDuration(start);
}

bool WalterModem::httpConfigProfile(
    uint8_t profileId,
    const char *serverName,
    uint16_t port,
    uint8_t tlsProfileId,
    bool useBasicAuth,
    const char *authUser,
    const char *authPass,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    if(profileId >= WALTER_MODEM_MAX_HTTP_PROFILES) {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
    }

    if(tlsProfileId && port == 80) {
        port = 443;
    }

    WalterModemBuffer *stringsBuffer = _getFreeBuffer();
    stringsBuffer->size +=
        sprintf((char*) stringsBuffer->data, "AT+SQNHTTPCFG=%d,\"%s\",%d,%d,\"%s\",\"%s\"",
        profileId, serverName, port, useBasicAuth, authUser, authPass);

    if(tlsProfileId) {
        stringsBuffer->size +=
            sprintf((char*) stringsBuffer->data + stringsBuffer->size, ",1,,,%u", tlsProfileId);
    }

    _runCmd(arr(
        (const char*) stringsBuffer->data),
        "OK", rsp, cb, args, NULL, NULL, WALTER_MODEM_CMD_TYPE_TX_WAIT, NULL, 0, stringsBuffer);

    _returnAfterReply();
}

bool WalterModem::httpConnect(
    uint8_t profileId,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    if(profileId >= WALTER_MODEM_MAX_HTTP_PROFILES) {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
    }

    if(_httpContextSet[profileId].connected) {
        _returnState(WALTER_MODEM_STATE_OK);
    }

    _runCmd(arr("AT+SQNHTTPCONNECT=", _atNum(profileId)), "+SQNHTTPCONNECT: ", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::httpClose(
    uint8_t profileId,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    if(profileId >= WALTER_MODEM_MAX_HTTP_PROFILES) {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
    }

    _runCmd(arr("AT+SQNHTTPDISCONNECT=", _atNum(profileId)), "OK", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::httpGetContextStatus(uint8_t profileId)
{
    if(profileId >= WALTER_MODEM_MAX_HTTP_PROFILES) {
        return false;
    }

    /* note: in my observation the SQNHTTPCONNECT command is to be avoided.
     * if the connection is closed by the server, you will not even
     * receive a +SQNHTTPSH disconnected message (you will on the next
     * connect attempt). reconnect will be impossible even if you try
     * to manually disconnect.
     * and a SQNHTTPQRY will still work and create its own implicit connection.
     *
     * (too bad: according to the docs SQNHTTPCONNECT is mandatory for
     * TLS connections)
     */
    return _httpContextSet[profileId].connected;
}

bool WalterModem::httpQuery(
    uint8_t profileId,
    const char *uri,
    WalterModemHttpQueryCmd httpQueryCmd,
    char *contentTypeBuf,
    uint16_t contentTypeBufSize,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    if(profileId >= WALTER_MODEM_MAX_HTTP_PROFILES) {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
    }

    if(_httpContextSet[profileId].state != WALTER_MODEM_HTTP_CONTEXT_STATE_IDLE) {
        _returnState(WALTER_MODEM_STATE_BUSY);
    }

    _httpContextSet[profileId].contentType = contentTypeBuf;
    _httpContextSet[profileId].contentTypeSize = contentTypeBufSize;

    auto completeHandler = [](WalterModemCmd *cmd, WalterModemState result)
    {
        WalterModemHttpContext *ctx = (WalterModemHttpContext*) cmd->completeHandlerArg;

        if(result == WALTER_MODEM_STATE_OK) {
            ctx->state = WALTER_MODEM_HTTP_CONTEXT_STATE_EXPECT_RING;
        }
    };

    WalterModemBuffer *stringsBuffer = _getFreeBuffer();
    stringsBuffer->size += sprintf((char*) stringsBuffer->data, "AT+SQNHTTPQRY=%d,%d,\"%s\"",
        profileId, httpQueryCmd, uri);

    _runCmd(arr(
        (const char*) stringsBuffer->data),
        "OK", rsp, cb, args, completeHandler, (void*) (_httpContextSet + profileId),
        WALTER_MODEM_CMD_TYPE_TX_WAIT, NULL, 0,stringsBuffer);

    _returnAfterReply();
}

bool WalterModem::httpSend(
    uint8_t profileId,
    const char *uri,
    uint8_t *data,
    uint16_t dataSize,
    WalterModemHttpSendCmd httpSendCmd,
    WalterModemHttpPostParam httpPostParam,
    char *contentTypeBuf,
    uint16_t contentTypeBufSize,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    if(profileId >= WALTER_MODEM_MAX_HTTP_PROFILES) {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
    }

    if(_httpContextSet[profileId].state != WALTER_MODEM_HTTP_CONTEXT_STATE_IDLE) {
        _returnState(WALTER_MODEM_STATE_BUSY);
    }

    _httpContextSet[profileId].contentType = contentTypeBuf;
    _httpContextSet[profileId].contentTypeSize = contentTypeBufSize;

    auto completeHandler = [](WalterModemCmd *cmd, WalterModemState result)
    {
        WalterModemHttpContext *ctx = (WalterModemHttpContext*) cmd->completeHandlerArg;

        if(result == WALTER_MODEM_STATE_OK) {
            ctx->state = WALTER_MODEM_HTTP_CONTEXT_STATE_EXPECT_RING;
        }
    };

    WalterModemBuffer *stringsBuffer = _getFreeBuffer();
    if(httpPostParam == WALTER_MODEM_HTTP_POST_PARAM_UNSPECIFIED) {
        stringsBuffer->size += sprintf((char*) stringsBuffer->data,
            "AT+SQNHTTPSND=%d,%d,\"%s\",%d", profileId, httpSendCmd, uri, dataSize);
    } else {
        stringsBuffer->size += sprintf((char*) stringsBuffer->data,
            "AT+SQNHTTPSND=%d,%d,\"%s\",%d,\"%d\"", profileId, httpSendCmd, uri, dataSize,
            httpPostParam);
    }

    _runCmd(arr(
        (const char*) stringsBuffer->data),
        "OK", rsp, cb, args, completeHandler, (void*) (_httpContextSet + profileId),
        WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT, data, dataSize, stringsBuffer);

    _returnAfterReply();
}

bool WalterModem::coapDidRing(
    uint8_t profileId,
    uint8_t *targetBuf,
    uint16_t targetBufSize,
    WalterModemRsp *rsp)
{
    /* this is by definition a blocking call without callback.
     * it is only used when the arduino user is not taking advantage of
     * the (TBI) ring notification events which give access to the raw
     * buffer (a targetBuf is not needed).
     */
    walterModemCb cb = NULL;
    void *args = NULL;

    if(profileId == 0) {
        _returnState(WALTER_MODEM_STATE_ERROR);
    }

    if(profileId >= WALTER_MODEM_MAX_COAP_PROFILES) {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
    }

    uint8_t ringIdx;
    for(ringIdx = 0; ringIdx < WALTER_MODEM_COAP_MAX_PENDING_RINGS; ringIdx++) {
        if(_coapContextSet[profileId].rings[ringIdx].messageId) {
            break;
        }
    }

    if(ringIdx == WALTER_MODEM_COAP_MAX_PENDING_RINGS) {
        _returnState(WALTER_MODEM_STATE_NO_DATA);
    }

    WalterModemBuffer *stringsBuffer = _getFreeBuffer();
    stringsBuffer->size +=
        sprintf((char*) stringsBuffer->data, "AT+SQNCOAPRCV=%d,%u,%u", profileId,
        _coapContextSet[profileId].rings[ringIdx].messageId,
        _coapContextSet[profileId].rings[ringIdx].length);

    _runCmd(arr(
        (const char*) stringsBuffer->data),
        "+SQNCOAPRCV: ", rsp, cb, args, NULL, NULL, WALTER_MODEM_CMD_TYPE_TX_WAIT, targetBuf,
        targetBufSize, stringsBuffer);

    _returnAfterReply();
}

bool WalterModem::httpDidRing(
    uint8_t profileId,
    uint8_t *targetBuf,
    uint16_t targetBufSize,
    WalterModemRsp *rsp)
{
    /* this is by definition a blocking call without callback.
     * it is only used when the arduino user is not taking advantage of
     * the (TBI) ring notification events.
     */
    walterModemCb cb = NULL;
    void *args = NULL;

    if(_httpCurrentProfile != 0xff) {
        _returnState(WALTER_MODEM_STATE_ERROR);
    }

    if(profileId >= WALTER_MODEM_MAX_HTTP_PROFILES) {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
    }

    if(_httpContextSet[profileId].state == WALTER_MODEM_HTTP_CONTEXT_STATE_IDLE) {
        _returnState(WALTER_MODEM_STATE_NOT_EXPECTING_RING);
    }

    if(_httpContextSet[profileId].state == WALTER_MODEM_HTTP_CONTEXT_STATE_EXPECT_RING) {
        _returnState(WALTER_MODEM_STATE_AWAITING_RING);
    }

    if(_httpContextSet[profileId].state != WALTER_MODEM_HTTP_CONTEXT_STATE_GOT_RING) {
        _returnState(WALTER_MODEM_STATE_ERROR);
    }

    /* ok, got ring. http context fields have been filled.
     * http status 0 means: timeout (or also disconnected apparently) */
    if(_httpContextSet[profileId].httpStatus == 0) {
        _httpContextSet[profileId].state = WALTER_MODEM_HTTP_CONTEXT_STATE_IDLE;
        _returnState(WALTER_MODEM_STATE_ERROR);
    }

    if(_httpContextSet[profileId].contentLength == 0) {
        _httpContextSet[profileId].state = WALTER_MODEM_HTTP_CONTEXT_STATE_IDLE;
        rsp->type = WALTER_MODEM_RSP_DATA_TYPE_HTTP_RESPONSE;
        rsp->data.httpResponse.httpStatus = _httpContextSet[profileId].httpStatus;
        rsp->data.httpResponse.contentLength = 0;
        _returnState(WALTER_MODEM_STATE_NO_DATA);
    }

    _httpCurrentProfile = profileId;

    auto completeHandler = [](WalterModemCmd *cmd, WalterModemState result) {
        _httpContextSet[_httpCurrentProfile].state = WALTER_MODEM_HTTP_CONTEXT_STATE_IDLE;
        _httpCurrentProfile = 0xff;
    };

    _runCmd(arr(
        "AT+SQNHTTPRCV=",
        _atNum(profileId)), "<<<",
        rsp, cb, args, completeHandler, NULL, WALTER_MODEM_CMD_TYPE_TX_WAIT, targetBuf,
        targetBufSize);
    _returnAfterReply();
}

bool WalterModem::mqttDidRing(
    const char *topic,
    uint8_t *targetBuf,
    uint16_t targetBufSize,
    WalterModemRsp *rsp)
{
    /* this is by definition a blocking call without callback.
     * it is only used when the arduino user is not taking advantage of
     * the (TBI) ring notification events which give access to the raw
     * buffer (a targetBuf is not needed).
     */
    walterModemCb cb = NULL;
    void *args = NULL;

    uint8_t idx;
    for(idx = 0; idx < WALTER_MODEM_MQTT_MAX_PENDING_RINGS; idx++) {
        if(!strncmp(topic, _mqttRings[idx].topic, WALTER_MODEM_MQTT_TOPIC_MAX_SIZE) &&
            _mqttRings[idx].messageId) {
            break;
        }
    }

    if(idx == WALTER_MODEM_MQTT_MAX_PENDING_RINGS) {
        _returnState(WALTER_MODEM_STATE_NO_DATA);
    }

    if(targetBufSize < _mqttRings[idx].length) {
        _returnState(WALTER_MODEM_STATE_NO_MEMORY);
    }

    targetBufSize = _mqttRings[idx].length;
    _mqttRings[idx].free = true;

    if (_mqttRings[idx].qos == 0)
    {
        /* no msg id means qos 0 message */
        _runCmd(arr(
            "AT+SQNSMQTTRCVMESSAGE=0,",
            _atStr(topic)),
            "OK", rsp, cb, args, NULL, (void*) idx, WALTER_MODEM_CMD_TYPE_TX_WAIT, targetBuf,
            targetBufSize);

        _returnAfterReply();
    }
    else
    {
        _runCmd(arr(
            "AT+SQNSMQTTRCVMESSAGE=0,",
            _atStr(topic), ",",
            _atNum(_mqttRings[idx].messageId)),
            "OK", rsp, cb, args, NULL, (void*) idx, WALTER_MODEM_CMD_TYPE_TX_WAIT, targetBuf,
            targetBufSize);

        _returnAfterReply();
    }
}

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

bool WalterModem::_tlsIsCredentialPresent(bool isPrivateKey, uint8_t slotIdx)
{
    WalterModemRsp *rsp = NULL;
    walterModemCb cb = NULL;
    void *args = NULL;

    const char *keyType = isPrivateKey ? "privatekey" : "certificate";

    _runCmd(arr("AT+SQNSNVR=", _atStr(keyType), ",", _atNum(slotIdx)), "+SQNSNVR: ", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::_blueCherryIsProvisioned()
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
    if(!_blueCherryIsProvisioned() ||
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

    if(!coapCreateContext(0, blueCherry.serverName, blueCherry.port, blueCherry.tlsProfileId)) {
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

bool WalterModem::coapCreateContext(
    uint8_t profileId,
    const char *serverName,
    int port,
    uint8_t tlsProfileId,
    int localPort,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    if(profileId >= WALTER_MODEM_MAX_COAP_PROFILES) {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
    }

    if(_coapContextSet[profileId].connected) {
        _returnState(WALTER_MODEM_STATE_OK);
    }

    WalterModemBuffer *stringsBuffer = _getFreeBuffer();
    stringsBuffer->size +=
        sprintf((char*) stringsBuffer->data, "AT+SQNCOAPCREATE=%d,\"%s\",%d,", profileId,
        serverName, port);

    if(localPort > -1) {
        stringsBuffer->size +=
            sprintf((char*) stringsBuffer->data + stringsBuffer->size, "%d", localPort);
    }

    stringsBuffer->size +=
        sprintf((char*) stringsBuffer->data + stringsBuffer->size, ",%d,60", tlsProfileId != 0);

    if(tlsProfileId) {
        stringsBuffer->size +=
            sprintf((char*) stringsBuffer->data + stringsBuffer->size, ",,%d", tlsProfileId);
    }

    _runCmd(arr(
        (const char*) stringsBuffer->data),
        "+SQNCOAPCONNECTED: ", rsp, cb, args, NULL, NULL, WALTER_MODEM_CMD_TYPE_TX_WAIT, NULL, 0,
        stringsBuffer);

    _returnAfterReply();
}

bool WalterModem::coapClose(uint8_t profileId, WalterModemRsp *rsp, walterModemCb cb, void *args)
{
    if(profileId == 0) {
        _returnState(WALTER_MODEM_STATE_ERROR);
    }

    if(profileId >= WALTER_MODEM_MAX_COAP_PROFILES) {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_PROFILE);
    }

    _runCmd(arr(
        "AT+SQNCOAPCLOSE=",
        _atNum(profileId)),
        "+SQNCOAPCLOSED: ", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::coapGetContextStatus(uint8_t profileId)
{
    if(profileId >= WALTER_MODEM_MAX_COAP_PROFILES) {
        return false;
    }

    return _coapContextSet[profileId].connected;
}

bool WalterModem::coapSetHeader(
    uint8_t profileId,
    int messageId,
    const char *token,
    WalterModemRsp *rsp ,
    walterModemCb cb,
    void *args)
{
    _runCmd(arr(
        "AT+SQNCOAPHDR=",
        _atNum(profileId), ",",
        _atNum(messageId),
        ",\"", token, "\""),
        "OK", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::coapSetOptions(
    uint8_t profileId,
    WalterModemCoapOptAction action,
    WalterModemCoapOptCode code,
    const char *const values,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    if(profileId == 0) {
        _returnState(WALTER_MODEM_STATE_ERROR);
    }

    if(action == WALTER_MODEM_COAP_OPT_READ) {
         /* not yet supported - add together with incoming socket con/coap response */
        _returnState(WALTER_MODEM_STATE_ERROR);
    }

    WalterModemBuffer *stringsBuffer = _getFreeBuffer();

    if(action == WALTER_MODEM_COAP_OPT_SET || action == WALTER_MODEM_COAP_OPT_EXTEND) {
         if(values && *values) {
            stringsBuffer->size +=
                sprintf((char*) stringsBuffer->data, "AT+SQNCOAPOPT=%d,%d,%d,\"%s\"", profileId,
                action, code, values);
         } else {
            stringsBuffer->size +=
                sprintf((char*) stringsBuffer->data, "AT+SQNCOAPOPT=%d,%d,%d", profileId, action,
                code);
         }
     } else if(action == WALTER_MODEM_COAP_OPT_DELETE) {
        stringsBuffer->size +=
            sprintf((char*) stringsBuffer->data, "AT+SQNCOAPOPT=%d,%d,%d", profileId, action, code);
     } else {
         /* make sure something sane is in the buffer if wrong action */
         stringsBuffer->size += sprintf((char*) stringsBuffer->data, "AT");
     }

    _runCmd(arr(
        (const char*) stringsBuffer->data),
        "OK", rsp, cb, args, NULL, NULL, WALTER_MODEM_CMD_TYPE_TX_WAIT, NULL, 0, stringsBuffer);
     _returnAfterReply();
}

bool WalterModem::coapSendData(
    uint8_t profileId,
    WalterModemCoapSendType type,
    WalterModemCoapSendMethodRsp methodRsp,
    int length,
    uint8_t *payload,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    _runCmd(arr(
        "AT+SQNCOAPSEND=",
        _atNum(profileId), ",",
        _atNum(type), ",",
        _atNum(methodRsp), ",",
        _atNum(length)),
        "OK", rsp, cb, args, NULL, NULL, WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT, payload, length);
    _returnAfterReply();
}

WalterModemNetworkRegState WalterModem::getNetworkRegState()
{
    return _regState;
}

bool WalterModem::getOpState(WalterModemRsp *rsp, walterModemCb cb, void *args)
{
    _runCmd({"AT+CFUN?"}, "OK", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::setOpState(
    WalterModemOpState opState,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    _runCmd(arr("AT+CFUN=", _digitStr(opState)), "OK", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::getRAT(WalterModemRsp *rsp, walterModemCb cb, void *args)
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

bool WalterModem::setRAT(WalterModemRAT rat, WalterModemRsp *rsp, walterModemCb cb, void *args)
{
    _runCmd(arr("AT+SQNMODEACTIVE=", _digitStr(rat + 1)), "OK", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::getRadioBands(WalterModemRsp *rsp, walterModemCb cb, void *args)
{
    _runCmd(arr("AT+SQNBANDSEL?"), "OK", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::setRadioBands(
    WalterModemRAT rat,
    uint32_t bands,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    WalterModemBuffer *stringsbuffer = _getFreeBuffer();
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

bool WalterModem::getSIMState(WalterModemRsp *rsp, walterModemCb cb, void *args)
{
    _runCmd({"AT+CPIN?"}, "OK", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::getSIMCardID(WalterModemRsp *rsp, walterModemCb cb, void *args)
{
    _runCmd({"AT+SQNCCID"}, "OK", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::getSIMCardIMSI(WalterModemRsp *rsp, walterModemCb cb, void *args)
{
    _runCmd({"AT+CIMI"}, "OK", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::unlockSIM(WalterModemRsp *rsp, walterModemCb cb, void *args, const char *pin)
{
    _simPIN = pin;

    if(_simPIN == NULL) {
        return getSIMState(rsp, cb, args);
    }
    
    _runCmd(arr("AT+CPIN=", _simPIN), "OK", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::setNetworkSelectionMode(
    WalterModemNetworkSelMode mode,
    const char *operatorName,
    WalterModemOperatorFormat format,
    WalterModemRsp *rsp,
    walterModemCb cb, 
    void *args)
{
    _networkSelMode = mode;
    _operator.format = format;
    _strncpy_s(_operator.name, operatorName, WALTER_MODEM_OPERATOR_MAX_SIZE);

    if(mode == WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC ||
       mode == WALTER_MODEM_NETWORK_SEL_MODE_UNREGISTER) {
        _runCmd(arr("AT+COPS=", _digitStr(mode)), "OK", rsp, cb, args);
        _returnAfterReply();
    } else {
        _runCmd(arr(
            "AT+COPS=",
            _digitStr(_networkSelMode), ",",
            _digitStr(_operator.format),",",
            _atStr(_operator.name)),
            "OK", rsp, cb, args);
        _returnAfterReply();
    }
}

bool WalterModem::configPSM(
    WalterModemPSMMode mode,
    const char *reqTAU,
    const char *reqActive,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    if(mode == WALTER_MODEM_PSM_ENABLE) {
        WalterModemBuffer *stringsbuffer = _getFreeBuffer();
        stringsbuffer->size +=
            sprintf((char*) stringsbuffer->data, "AT+CPSMS=1,,,\"%s\",\"%s\"", reqTAU, reqActive);

        _runCmd(arr(
            (const char*) stringsbuffer->data),
            "OK", rsp, cb, args, NULL, NULL, WALTER_MODEM_CMD_TYPE_TX_WAIT, NULL, 0, stringsbuffer);
        _returnAfterReply();
    } else {
        _runCmd(arr("AT+CPSMS=", _digitStr(mode)), "OK", rsp, cb, args);
        _returnAfterReply();
    }
}

bool WalterModem::configEDRX(
    WalterModemEDRXMode mode,
    const char *reqEDRXVal,
    const char *reqPtw,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    if(mode == WALTER_MODEM_EDRX_ENABLE || mode == WALTER_MODEM_EDRX_ENABLE_WITH_RESULT) {
        getRAT(rsp, cb, args);

        WalterModemBuffer *stringsbuffer = _getFreeBuffer();
        stringsbuffer->size +=
            sprintf((char*) stringsbuffer->data, "AT+SQNEDRX=%d,%d,\"%s\",\"%s\"", mode,
                _ratType + 4, reqEDRXVal, reqPtw);

        _runCmd(arr(
            (const char*) stringsbuffer->data),
            "OK", rsp, cb, args, NULL, NULL, WALTER_MODEM_CMD_TYPE_TX_WAIT, NULL, 0, stringsbuffer);
        _returnAfterReply();
    } else {
        _runCmd(arr("AT+SQNEDRX=", _digitStr(mode)), "OK", rsp, cb, args);
        _returnAfterReply();
    }
}

bool WalterModem::createPDPContext(
    const char *apn,
    WalterModemPDPAuthProtocol authProto,
    const char *authUser,
    const char *authPass,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args,
    WalterModemPDPType type,
    const char *pdpAddress, 
    WalterModemPDPHeaderCompression headerComp, 
    WalterModemPDPDataCompression dataComp,
    WalterModemPDPIPv4AddrAllocMethod ipv4AllocMethod,
    WalterModemPDPRequestType requestType,
    WalterModemPDPPCSCFDiscoveryMethod pcscfMethod,
    bool forIMCN,
    bool useNSLPI,
    bool useSecurePCO,
    bool useNASIPv4MTUDiscovery,
    bool useLocalAddrInd,
    bool useNASNonIPMTUDiscovery)
{
    WalterModemPDPContext *ctx = _pdpContextReserve();
    if(ctx == NULL) {
        _returnState(WALTER_MODEM_STATE_NO_FREE_PDP_CONTEXT);
    }

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
    ctx->authProto = authProto;
    _strncpy_s(ctx->authUser, authUser, WALTER_MODEM_PDP_AUTH_USER_MAX_SIZE);
    _strncpy_s(ctx->authPass, authPass, WALTER_MODEM_PDP_AUTH_PASS_MAX_SIZE);

    auto completeHandler = [](WalterModemCmd *cmd, WalterModemState result)
    {
        WalterModemPDPContext *ctx = (WalterModemPDPContext*) cmd->completeHandlerArg;

        cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_PDP_CTX_ID;
        cmd->rsp->data.pdpCtxId = ctx->id;

        if(result == WALTER_MODEM_STATE_OK) {
            ctx->state = WALTER_MODEM_PDP_CONTEXT_STATE_INACTIVE;
        }
    };

    _runCmd(arr(
        "AT+CGDCONT=",
        _digitStr(ctx->id), ",",
        _atStr(_pdpTypeStr(ctx->type)), ",",
        _atStr(ctx->apn), ",",
        _atStr(ctx->pdpAddress), ",",
        _digitStr(ctx->dataComp), ",",
        _digitStr(ctx->headerComp), ",",
        _digitStr(ctx->ipv4AllocMethod), ",",
        _digitStr(ctx->requestType), ",",
        _digitStr(ctx->pcscfMethod), ",",
        _atBool(ctx->forIMCN), ",",
        _atBool(ctx->useNSLPI), ",",
        _atBool(ctx->useSecurePCO), ",",
        _atBool(ctx->useNASIPv4MTUDiscovery), ",",
        _atBool(ctx->useLocalAddrInd), ",",
        _atBool(ctx->useNASNonIPMTUDiscovery)),
        "OK", rsp, cb, args, completeHandler, ctx);
    _returnAfterReply();
}

bool WalterModem::authenticatePDPContext(
    int pdpCtxId,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    WalterModemPDPContext *ctx = _pdpContextGet(pdpCtxId);
    if(ctx == NULL) {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_PDP_CONTEXT);
    }

    if(ctx->authProto == WALTER_MODEM_PDP_AUTH_PROTO_NONE) {
        _returnState(WALTER_MODEM_STATE_OK);
    }

    _runCmd(arr(
        "AT+CGAUTH=",
        _digitStr(ctx->id),",",
        _digitStr(ctx->authProto),",",
        _atStr(ctx->authUser),",",
        _atStr(ctx->authPass)),
        "OK", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::setPDPContextActive(
    bool active,
    int pdpCtxId,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    WalterModemPDPContext *ctx = _pdpContextGet(pdpCtxId);
    if(ctx == NULL) {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_PDP_CONTEXT);
    }

    auto completeHandler = [](WalterModemCmd *cmd, WalterModemState result) 
    {
        WalterModemPDPContext *ctx = (WalterModemPDPContext*) cmd->completeHandlerArg;
        
        if(result == WALTER_MODEM_STATE_OK) {
            ctx->state = WALTER_MODEM_PDP_CONTEXT_STATE_ACTIVE;
            for (size_t i = 0; i < WALTER_MODEM_MAX_PDP_CTXTS; i++) {
                WalterModemPDPContext *_ctx = _pdpContextGet(i);
                if(_ctx->id != ctx->id) {
                    _ctx->state = WALTER_MODEM_PDP_CONTEXT_STATE_INACTIVE;
                }
            }
            /* Reset the active PDP context */
            _pdpCtx = ctx;
        }
    };

    _runCmd(arr(
        "AT+CGACT=",
        _digitStr(ctx->id),",",
        _atBool(active)),
        "OK", rsp, cb, args, completeHandler, ctx);
    _returnAfterReply();
}

bool WalterModem::attachPDPContext(
    bool attach,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    auto completeHandler = [](WalterModemCmd *cmd, WalterModemState result) 
    {
        if(result == WALTER_MODEM_STATE_OK) {
            WalterModemPDPContext *ctx = _pdpContextGet();
            ctx->state = WALTER_MODEM_PDP_CONTEXT_STATE_ATTACHED;
        }
    };

    _runCmd(arr("AT+CGATT=", _atBool(attach)), "OK", rsp, cb, args, completeHandler);
    _returnAfterReply();
}

bool WalterModem::getPDPAddress(
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args,
    int pdpCtxId)
{
    WalterModemPDPContext *ctx = _pdpContextGet(pdpCtxId);
    if(ctx == NULL) {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_PDP_CONTEXT);
    }

    _runCmd(arr("AT+CGPADDR=", _digitStr(ctx->id)), "OK", rsp, cb, args);
    _returnAfterReply();
}

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
    if(ctx == NULL) {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_PDP_CONTEXT);
    }

    WalterModemSocket *sock = _socketReserve();
    if(sock == NULL) {
        _returnState(WALTER_MODEM_STATE_NO_FREE_SOCKET);
    }

    sock->pdpContextId = ctx->id;
    sock->mtu = mtu;
    sock->exchangeTimeout = exchangeTimeout;
    sock->connTimeout = connTimeout;
    sock->sendDelayMs = sendDelayMs;

    auto completeHandler = [](WalterModemCmd *cmd, WalterModemState result) 
    { 
        WalterModemSocket *sock = (WalterModemSocket*) cmd->completeHandlerArg;
        
        cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_SOCKET_ID;
        cmd->rsp->data.socketId = sock->id;

        if(result == WALTER_MODEM_STATE_OK) {
            sock->state = WALTER_MODEM_SOCKET_STATE_CREATED;
        }
    };

    _runCmd(arr(
        "AT+SQNSCFG=",
        _digitStr(sock->id),",",
        _digitStr(sock->pdpContextId),",",
        _atNum(sock->mtu),",",
        _atNum(sock->exchangeTimeout),",",
        _atNum(sock->connTimeout * 10),",",
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
    if(sock == NULL) {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
    }

    auto completeHandler = [](WalterModemCmd *cmd, WalterModemState result) 
    { 
        WalterModemSocket *sock = (WalterModemSocket*) cmd->completeHandlerArg;
        
        if(result == WALTER_MODEM_STATE_OK) {
            sock->state = WALTER_MODEM_SOCKET_STATE_CONFIGURED;
        }
    };

    _runCmd(arr(
        "AT+SQNSCFGEXT=",
        _digitStr(sock->id),",2,0,0,0,0,0"),
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
    if(sock == NULL) {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
    }

    sock->protocol = protocol;
    sock->acceptAnyRemote = acceptAnyRemote;
    _strncpy_s(sock->remoteHost, remoteHost, WALTER_MODEM_HOSTNAME_MAX_SIZE);
    sock->remotePort = remotePort;
    sock->localPort = localPort; 

    auto completeHandler = [](WalterModemCmd *cmd, WalterModemState result) 
    { 
        WalterModemSocket *sock = (WalterModemSocket*) cmd->completeHandlerArg;
        
        if(result == WALTER_MODEM_STATE_OK) {
            sock->state = WALTER_MODEM_SOCKET_STATE_OPENED;
        }
    };

    _runCmd(arr(
        "AT+SQNSD=",
        _digitStr(sock->id),",",
        _digitStr(sock->protocol),",",
        _atNum(sock->remotePort),",",
        _atStr(sock->remoteHost),",0,",
        _atNum(sock->localPort),",1,",
        _digitStr(sock->acceptAnyRemote),",0"),
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
    if(sock == NULL) {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
    }

    auto completeHandler = [](WalterModemCmd *cmd, WalterModemState result) 
    { 
        WalterModemSocket *sock = (WalterModemSocket*) cmd->completeHandlerArg;
        
        if(result == WALTER_MODEM_STATE_OK) {
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
    if(sock == NULL) {
        _returnState(WALTER_MODEM_STATE_NO_SUCH_SOCKET);
    }

    _runCmd(arr(
        "AT+SQNSSENDEXT=",
        _digitStr(sock->id),",",
        _atNum(dataSize),",",
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
    return socketSend((uint8_t*) str, strlen(str), rsp, cb, args, rai, socketId);
}

bool WalterModem::getClock(WalterModemRsp *rsp, walterModemCb cb, void *args)
{
    _runCmd(arr("AT+CCLK?"), "OK", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::configGNSS(
    WalterModemGNSSSensMode sensMode,
    WalterModemGNSSAcqMode acqMode,
    WalterModemGNSSLocMode locMode,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    _runCmd(arr(
        "AT+LPGNSSCFG=",
        _digitStr(locMode),",",
        _digitStr(sensMode),
        ",2,,1,",
        _digitStr(acqMode)), "OK", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::getGNSSAssistanceStatus(
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    _runCmd(arr("AT+LPGNSSASSISTANCE?"), "OK", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::updateGNSSAssistance(
    WalterModemGNSSAssistanceType type,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    _runCmd(arr("AT+LPGNSSASSISTANCE=", _digitStr(type)), "+LPGNSSASSISTANCE:", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::performGNSSAction(
    WalterModemGNSSAction action,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    auto gnssActionStr = [](WalterModemGNSSAction action) 
    {
        switch(action) {
            case WALTER_MODEM_GNSS_ACTION_GET_SINGLE_FIX:
                return "single";

            case WALTER_MODEM_GNSS_ACTION_CANCEL:
                return "stop";
        }
        return "";
    };

    _runCmd(arr("AT+LPGNSSFIXPROG=\"", gnssActionStr(action),"\""), "OK", rsp, cb, args);
    _returnAfterReply();
}

uint8_t WalterModem::_convertDuration(
    const uint32_t *base_times,
    size_t base_times_len,
    uint32_t duration_seconds,
    uint32_t *actual_duration_seconds)
{
    uint32_t smallest_modulo = UINT32_MAX;
    uint8_t final_base = 0;
    uint8_t final_mult = 0;

    for (uint8_t base = 0; base < base_times_len; ++base) {
        uint32_t multiplier = duration_seconds / base_times[base];
        if (multiplier == 0 || multiplier > 31) {
            continue;
        }

        uint32_t modulo = duration_seconds % base_times[base];
        if (modulo < smallest_modulo) {
            final_base = base;
            final_mult = multiplier;
        }
    }

    if (actual_duration_seconds) {
        *actual_duration_seconds = (uint32_t)final_base * (uint32_t)final_mult;
    }

    return  (final_base << 5) | final_mult;
}

uint8_t WalterModem::durationToTAU(
    uint32_t seconds,
    uint32_t minutes,
    uint32_t hours,
    uint32_t *actual_duration_seconds)
{
    static const uint32_t base_times[] = { 600, 3600, 36000, 2, 30, 60, 1152000 };
    uint32_t duration_seconds = seconds + (60 * minutes) + (60 * 60 * hours);

    return _convertDuration(base_times,7,duration_seconds,actual_duration_seconds);
}

uint8_t WalterModem::durationToActiveTime(
    uint32_t seconds,
    uint32_t minutes,
    uint32_t *actual_duration_seconds) 
{
    static const uint32_t base_times[] = { 2, 60, 360 };
    uint32_t duration_seconds = seconds + (60 * minutes);
    
    return _convertDuration(base_times,3, duration_seconds,actual_duration_seconds);
}

void WalterModem::setRegistrationEventHandler(
    walterModemRegistrationEventHandler handler,
    void *args)
{
    _eventHandlers[WALTER_MODEM_EVENT_TYPE_REGISTRATION].regHandler = handler;
    _eventHandlers[WALTER_MODEM_EVENT_TYPE_REGISTRATION].args = args;
}

void WalterModem::setSystemEventHandler(walterModemSystemEventHandler handler, void *args)
{
    _eventHandlers[WALTER_MODEM_EVENT_TYPE_SYSTEM].sysHandler = handler;
    _eventHandlers[WALTER_MODEM_EVENT_TYPE_SYSTEM].args = args;
}

void WalterModem::setATEventHandler(walterModemATEventHandler handler, void *args)
{
    _eventHandlers[WALTER_MODEM_EVENT_TYPE_AT].atHandler = handler;
    _eventHandlers[WALTER_MODEM_EVENT_TYPE_AT].args = args;
}

void WalterModem::setGNSSEventHandler(walterModemGNSSEventHandler handler, void *args)
{
    _eventHandlers[WALTER_MODEM_EVENT_TYPE_AT].gnssHandler = handler;
    _eventHandlers[WALTER_MODEM_EVENT_TYPE_AT].args = args;
}