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
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 * 
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 * 
 *   3. Neither the name of DPTechnics bv nor the names of its contributors may
 *      be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 * 
 *   4. This software, with or without modification, must only be used with a
 *      Walter board from DPTechnics bv.
 * 
 *   5. Any software provided in binary form under this license must not be
 *      reverse engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY DPTECHNICS BV “AS IS” AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL DPTECHNICS BV OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @section DESCRIPTION
 * 
 * This file contains Walter's modem library implementation.
 */

#include "WalterModem.h"

#include <mutex>
#include <ctime>
#include <bitset>
#include <cstdint>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <Arduino.h>
#include <condition_variable>

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
#define WALTER_MODEM_CMD_TIMEOUT_MS 30000

/**
 * @brief The command timeout expressed in system ticks.
 */
#define WALTER_MODEM_CMD_TIMEOUT_TICKS \
    pdMS_TO_TICKS(WALTER_MODEM_CMD_TIMEOUT_MS)

/**
 * @brief Any modem time below 1 Jan 2023 00:00:00 UTC is considered an
 * invalid time.
 */
#define WALTER_MODEM_MIN_VALID_TIMESTAMP 1672531200

/**
 * @brief Library debug statements macro, this wraps printf on the serial port.
 */
#if WALTER_MODEM_DEBUG_ENABLED == 1
    #define _dbgPrintf(...) Serial.printf(__VA_ARGS__)
#else
    #define _dbgPrintf(...)
#endif

/**
 * @brief The length of a string literal at compile time.
 */
#define _strLitLen(str) (sizeof(str) - 1)

/**
 * @brief Check if a WalterModemParserBuffer starts with a given string literal.
 */
#define _buffStartsWith(buff, str) ((buff->size >= _strLitLen(str)) && \
    memcmp(str, buff->data, _strLitLen(str)) == 0)

/**
 * @brief 0-terminate a WalterModemParserBuffer by replacing the '\r' of the
 * trailing '\r\n' with a zero. This macro is meant to be used as an assignment
 * in the form of x = _buffStr(buff);
 */
#define _buffStr(buff) (const char*) buff->data; \
    buff->data[buff->size] = '\0'

/**
 * Check if the data string in a buffer equals a certain expected value which
 * comes after a certain prefix.
 */
#define _dataStrIs(buff, prefix, expected) \
    ((buff->size - _strLitLen(prefix)) > 0 && \
    memcmp(buff->data + _strLitLen(prefix), expected, \
    buff->size - _strLitLen(prefix)) == 0)

/**
 * @brief Convert a string literal into an AT command array. An empty string
 * will result in "","","" and an non-empty string will result in
 * "\"", "<string>", "\"".
 */
#define _atStr(str) \
    strlen(str) > 0 ? "\"" : "", str, strlen(str) > 0 ? "\"" : ""

/**
 * @brief Convert a boolean flag into a string literal.
 */
#define _atBool(bool) bool ? "1" : "0"

/**
 * @brief Convert a number of up to 6 digits in length into an array of digits
 * which can be passed to an AT command array.
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
#define _strncpy_s(dst, src, maxLen) \
    strncpy(dst, src == NULL ? "" : src, maxLen); \
    dst[maxLen] = '\0';

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
 * @brief Add a command to the queue for it to be execute it.
 * 
 * This macro will add a command to the command queue for it to be executed. If 
 * the command could not be added to the queue the macro will make the
 * surrounding function return false (when blocking API is used) or call the 
 * callback with an out-of-memory error.
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
WalterModemCmd *cmd = \
    _addQueueCmd(_cmdArr, atRsp, rsp, cb, args, ##__VA_ARGS__); \
if(cmd == NULL) { \
    _returnState(WALTER_MODEM_STATE_NO_MEMORY); \
} \
std::unique_lock<std::mutex> lock{cmd->cmdLock.mutex};

/**
 * @brief When working asynchronously, it will return true because the response
 * is handled through the user callback. If the blocking API is being used and
 * the expected command state is reached, the mutex is released, the correct
 * result of the command is passed and the function returns with true when the
 * state is WALTER_MODEM_STATE_OK, false otherwise.
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
#define _transmitCmd(type, atCmd) { \
    _dbgPrintf("TX: "); \
    for(int i = 0; i < WALTER_MODEM_COMMAND_MAX_ELEMS; ++i) { \
        if(atCmd[i] == NULL) { \
            break; \
        } \
        _uart->write(atCmd[i]); \
        _dbgPrintf("%s", atCmd[i]); \
    } \
    _uart->write(type == WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT ? "\n" : "\r\n"); \
    _dbgPrintf("\n"); \
}

/**
 * @brief Convert a digit to a string literal.
 * 
 * This function converts a digit [0,9] into a string literal so that it can be
 * passed as an AT command element.
 * 
 * @param val The value to convert.
 * 
 * @return The resulting string literal or an empty string when the value is 
 * not in the [0,9] range.
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
 * This function will convert the digit from a certain integer into a string
 * literal. The position starts at 0 which is the leftmost digit of the number.
 * When position is bigger than the number of digits in the number an empty 
 * string literal is returned.
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

    while(tmp > 10) {
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
 * @brief Convert a radio access technology to a string.
 * 
 * This function will convert a radio access technology into a string that can
 * be used in AT commands.
 * 
 * @param rat The radio access technology to convert.
 * 
 * @return The resulting string literal.
 */
static const char* _ratStr(WalterModemRAT rat)
{
    switch(rat) {
        case WALTER_MODEM_RAT_NBIOT:
            return "NBIOT";
        
        case WALTER_MODEM_RAT_LTEM:
            return "CatM";

        case WALTER_MODEM_RAT_AUTO:
            return "AUTO"; 
    }

    return "";
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
 * @brief Convert an UTC time to an UTC timestamp.
 * 
 * This function converts a string to an UTC timestamp.
 * 
 * @param timeStr The string to convert.
 * @param format The time format to parse.
 * 
 * @return The UTC timestamp or -1 on error. 
 */
static int64_t strTotime(
    const char *timeStr,
    const char *format = "%Y-%m-%dT%H:%M:%S")
{
    struct tm tm{};
    if(strptime(timeStr, format, &tm) == NULL) {
        return -1;
    }

    time_t utcTime = std::mktime(&tm);
    time_t localTime = std::mktime(std::localtime(&utcTime));
    time_t offset = localTime - utcTime;

    return (int64_t) (utcTime - offset);
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
    if(_cmdQueue.inIdx == _cmdQueue.outIdx &&
       _cmdQueue.queue[_cmdQueue.outIdx] == NULL)
    {
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
    if(_cmdQueue.inIdx == _cmdQueue.outIdx &&
       _cmdQueue.queue[_cmdQueue.outIdx] != NULL)
    {
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
        if(_pdpCtxSet[i].state != WALTER_MODEM_PDP_CONTEXT_STATE_FREE &&
           _pdpCtxSet[i].id == id)
        {
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
        if(_socketSet[i].state != WALTER_MODEM_SOCKET_STATE_FREE &&
           _socketSet[i].id == id)
        {
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

void WalterModem::_addATByteToBuffer(char data)
{
    //TODO: in the future we must be aware of length, or at least check if the
    //ending \r\n is escaped when data is transmitted.
    if(data == '\r') {
        _parserData.state = WALTER_MODEM_RSP_PARSER_END_LF;
        return;
    }

    /* Try to get a free response buffer from the pool */
    if(_parserData.buf == NULL) {
        for(int i = 0; i < WALTER_MODEM_AT_RSP_POOL_SIZE; ++i) {
            if(_parserData.pool[i].free) {
                _parserData.buf = _parserData.pool + i;
                _parserData.buf->free = false;
                _parserData.buf->size = 0;
                break;
            }
        }
    }
    
    /* 
     * The buffer could be NULL when all buffers in the pool are in use. When
     * this happens we will continue parsing but drop the fully parsed command.
     */
    if(_parserData.buf != NULL) {
        _parserData.buf->data[_parserData.buf->size++] = data;
    }
}

void WalterModem::_queueRxBuffer()
{
    if(_parserData.buf != NULL) {
        if(_parserData.buf->size > 0) {
            WalterModemTaskQueueItem qItem = {};
            qItem.rsp = _parserData.buf;

            if(xQueueSend(_taskQueue.handle, &qItem, 0) != pdTRUE) {
                /* 
                 * When we can not send the buffer to the queue we
                 * release it immediately and thus drop the packet.
                 * In the other case the buffer will be released by
                 * the queue consumer.
                 */
                _parserData.buf->free = true;
            }
        }

        _parserData.buf = NULL;
    }
}

//TODO: check if the modem response is not longer than the buffer size
void WalterModem::_handleRxData()
{
    int uartBufSize = _uart->available();
    for(int i = 0; i < uartBufSize; ++i) {
        char data = _uart->read();
        
        switch (_parserData.state) {
        case WALTER_MODEM_RSP_PARSER_START_CR:
            if(data == '\r') {
                _parserData.state = WALTER_MODEM_RSP_PARSER_START_LF;
            } else if(data == '+') {
                /* This is the start of a new line in a multiline response */
                _parserData.state = WALTER_MODEM_RSP_PARSER_DATA;
                _addATByteToBuffer(data);
            }
            break;

        case WALTER_MODEM_RSP_PARSER_START_LF:
            if(data == '\n') {
                _parserData.state = WALTER_MODEM_RSP_PARSER_DATA;
            }
            break;

        case WALTER_MODEM_RSP_PARSER_DATA:
            _addATByteToBuffer(data);

            if(_parserData.buf->size == strlen("+SQNSRING: 1,x") &&
               _buffStartsWith(_parserData.buf, "+SQNSRING: 1,"))
            {
            
            }

            if(data == '>') {
                _parserData.state = WALTER_MODEM_RSP_PARSER_DATA_PROMPT;
            }
            break;
        
        //TODO: add cases to parse SQNSRING length

        case WALTER_MODEM_RSP_PARSER_DATA_PROMPT:
            _addATByteToBuffer(data);
            if(data == ' ') {
                _queueRxBuffer();
                _parserData.state = WALTER_MODEM_RSP_PARSER_START_CR;
            } else {
                _parserData.state = WALTER_MODEM_RSP_PARSER_DATA;
            }
            break;

        case WALTER_MODEM_RSP_PARSER_END_LF:
            if(data == '\n') {
                _queueRxBuffer();
                _parserData.state = WALTER_MODEM_RSP_PARSER_START_CR;
            } else {
                if(_parserData.buf != NULL) {
                    _parserData.buf->data[_parserData.buf->size++] = '\r';
                    if(data != '\r') {
                        _parserData.buf->data[_parserData.buf->size++] = data;
                    }
                }
            }
            break;
        }
    }
}

void WalterModem::_queueProcessingTask(void *args)
{
    WalterModemTaskQueueItem qItem = { };
    WalterModemCmd *curCmd = NULL;
    TickType_t blockTime = portMAX_DELAY;

    while(true) {
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

        blockTime = portMAX_DELAY;

        if(curCmd != NULL) {
            switch(curCmd->state) {
                case WALTER_MODEM_CMD_STATE_FREE:
                case WALTER_MODEM_CMD_STATE_POOLED:
                    /* 
                    * There was a programming issue, commands with these states
                    * should never be sent to the queue. We are going to ignore
                    * them.
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
    void (*completeHandler)(struct sWalterModemCmd *cmd,
        WalterModemState result),
    void *completeHandlerArg,
    WalterModemCmdType type,
    const uint8_t *data,
    uint16_t dataSize,
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

    memset(cmd->rsp, 0, sizeof(WalterModemRsp));

    WalterModemTaskQueueItem qItem = {};
    qItem.cmd = cmd;

    if(xQueueSend(_taskQueue.handle, &qItem, 0) != pdTRUE) {
        cmd->state = WALTER_MODEM_CMD_STATE_FREE;
        return NULL;
    }

    return cmd;
}

void WalterModem::_finishQueueCmd(WalterModemCmd *cmd, WalterModemState result)
{
    cmd->rsp->result = result;

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
        return portMAX_DELAY;
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
                _transmitCmd(cmd->type, cmd->atCmd);
                cmd->attempt = 1;
                cmd->attemptStart = xTaskGetTickCount();
                cmd->state = WALTER_MODEM_CMD_STATE_PENDING;
                return WALTER_MODEM_CMD_TIMEOUT_TICKS;
            } else {
                TickType_t diff = xTaskGetTickCount() - cmd->attemptStart;
                bool timedOut = diff >= WALTER_MODEM_CMD_TIMEOUT_TICKS;
                if(timedOut || 
                   cmd->state == WALTER_MODEM_CMD_STATE_RETRY_AFTER_ERROR)
                {
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

    return portMAX_DELAY;
}

void WalterModem::_processQueueRsp(
    WalterModemCmd *cmd,
    WalterModemParserBuffer *buff)
{
    _dbgPrintf("RX: %.*s\n", buff->size, buff->data);

    if(_buffStartsWith(buff, "+CEREG: ")) {
        const char *rspStr = _buffStr(buff);
        int ceReg = atoi(rspStr + _strLitLen("+CEREG: "));
        _regState = (WalterModemNetworkRegState) ceReg;
        //TODO: call correct handlers
    }
    else if(_buffStartsWith(buff, "> "))
    {
        if(cmd != NULL &&
           cmd->type == WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT &&
           cmd->data != NULL)
        {
            _uart->write(cmd->data, cmd->dataSize);
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
            cmd->rsp->data.simState = 
                WALTER_MODEM_SIM_STATE_PHONE_TO_SIM_PIN_REQUIRED;
        } else if(_dataStrIs(buff, "+CPIN: ", "PH-FSIM PIN")) {
            cmd->rsp->data.simState = 
                WALTER_MODEM_SIM_STATE_PHONE_TO_FIRST_SIM_PIN_REQUIRED;
        } else if(_dataStrIs(buff, "+CPIN: ", "PH-FSIM PUK")) {
            cmd->rsp->data.simState = 
                WALTER_MODEM_SIM_STATE_PHONE_TO_FIRST_SIM_PUK_REQUIRED;
        } else if(_dataStrIs(buff, "+CPIN: ", "SIM PIN2")) {
            cmd->rsp->data.simState = WALTER_MODEM_SIM_STATE_PIN2_REQUIRED;
        } else if(_dataStrIs(buff, "+CPIN: ", "SIM PUK2")) {
            cmd->rsp->data.simState = WALTER_MODEM_SIM_STATE_PUK2_REQUIRED;
        } else if(_dataStrIs(buff, "+CPIN: ", "PH-NET PIN")) {
            cmd->rsp->data.simState = 
                WALTER_MODEM_SIM_STATE_NETWORK_PIN_REQUIRED;
        } else if(_dataStrIs(buff, "+CPIN: ", "PH-NET PUK")) {
            cmd->rsp->data.simState = 
                WALTER_MODEM_SIM_STATE_NETWORK_PUK_REQUIRED;
        } else if(_dataStrIs(buff, "+CPIN: ", "PH-NETSUB PIN")) {
            cmd->rsp->data.simState = 
                WALTER_MODEM_SIM_STATE_NETWORK_SUBSET_PIN_REQUIRED;
        } else if(_dataStrIs(buff, "+CPIN: ", "PH-NETSUB PUK")) {
            cmd->rsp->data.simState = 
                WALTER_MODEM_SIM_STATE_NETWORK_SUBSET_PUK_REQUIRED;
        } else if(_dataStrIs(buff, "+CPIN: ", "PH-SP PIN")) {
            cmd->rsp->data.simState = 
                WALTER_MODEM_SIM_STATE_SERVICE_PROVIDER_PIN_REQUIRED;
        } else if(_dataStrIs(buff, "+CPIN: ", "PH-SP PUK")) {
            cmd->rsp->data.simState = 
                WALTER_MODEM_SIM_STATE_SERVICE_PROVIDER_PUK_REQUIRED;
        } else if(_dataStrIs(buff, "+CPIN: ", "PH-CORP PIN")) {
            cmd->rsp->data.simState = 
                WALTER_MODEM_SIM_STATE_CORPORATE_SIM_REQUIRED;
        } else if(_dataStrIs(buff, "+CPIN: ", "PH-CORP PUK")) {
            cmd->rsp->data.simState = 
                WALTER_MODEM_SIM_STATE_CORPORATE_PUK_REQUIRED;
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
                cmd->rsp->type = WALTER_MDOEM_RSP_DATA_TYPE_PDP_ADDR;
                cmd->rsp->data.pdpAddressList.pdpAddress = ctx->pdpAddress;
                cmd->rsp->data.pdpAddressList.pdpAddress2 = ctx->pdpAddress2;
            }
        }
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
        WalterModemBandSelection *bSel = cmd->rsp->data.bandSelCfgSet.config +
            cmd->rsp->data.bandSelCfgSet.count;
        cmd->rsp->data.bandSelCfgSet.count += 1;

        /* Parse RAT */
        bSel->rat = data[0] == '0' ?
            WALTER_MODEM_RAT_LTEM : WALTER_MODEM_RAT_NBIOT;
        data += 2;
        dataSize -= 2;

        /* Parse operator name */
        bSel->netOperator.format =
            WALTER_MODEM_OPERATOR_FORMAT_LONG_ALPHANUMERIC;
        for(uint16_t i = 0; i < dataSize; ++i) {
            if(data[i] == ',') {
                data += i + 1;
                dataSize -= i + 1;

                if(i < WALTER_MODEM_OPERATOR_MAX_SIZE) {
                    bSel->netOperator.name[i] = '\0';
                } else {
                    bSel->netOperator.name[WALTER_MODEM_OPERATOR_MAX_SIZE]
                        = '\0';
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

            data[i] == 0;
            int band = atoi((const char *) data + start);
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
        uint8_t partNo = 0;
        bool parenthesisOpen = false;

        for(uint16_t i = 0; i < dataSize; ++i) {
            bool partComplete = false;

            if(data[i] == ',' && !parenthesisOpen) {
                data[i] = '\0';
                partComplete = true;
            } else if(data[i] == '(') {
                parenthesisOpen = true;
            } else if(data[i] == ')') {
                parenthesisOpen = false;
            }
            
            if(i + 1 == dataSize) {
                data[i + 1] = '\0';
                partComplete = true;
            }

            if(partComplete) {
                switch(partNo) {
                    case 0:
                        _GNSSfix.fixId = atoi(start);
                        break;
                    
                    case 1:
                        _GNSSfix.timestamp = strTotime(start);
                        break;

                    case 2:
                        _GNSSfix.timeToFix = atoi(start);
                        break;

                    case 3:
                        _GNSSfix.estimatedConfidence = strtod(start, NULL);
                        break;

                    case 4:
                        _GNSSfix.latitude = atof(start);
                        break;

                    case 5:
                        _GNSSfix.longitude = atof(start);
                        break;

                    case 6:
                        _GNSSfix.height = atof(start);
                        break;

                    case 7:
                        _GNSSfix.northSpeed = atof(start);
                        break;

                    case 8:
                        _GNSSfix.eastSpeed = atof(start);
                        break;

                    case 9:
                        _GNSSfix.downSpeed = atof(start);
                        break;

                    case 10:
                        /* 
                         * Raw satellite signal sample is ignored, we use this
                         * occasion to reset the satellite count.
                         */
                        _GNSSfix.satCount = 0;
                        break;

                    default:
                        if(_GNSSfix.satCount >= WALTER_MODEM_GNSS_MAX_SATS) {
                            continue;
                        }

                        const char *satNoStr = start;
                        const char *satSigStr = start;
                        for(int i = 0; start[i] != '\0'; ++i) {
                            if(start[i] == ',') {
                                start[i] = '\0';
                                satSigStr = start + i + 1;
                                break;
                            }
                        }

                        _GNSSfix.sats[_GNSSfix.satCount].satNo = atoi(satNoStr);
                        _GNSSfix.sats[_GNSSfix.satCount].signalStrength
                            = atoi(satSigStr);
                        _GNSSfix.satCount += 1;
                        break;
                }

                /* +2 for the comma and trailing space */
                start = (char*) data + i + 2;
                partNo += 1;
            }
        }

        if(_usrGNSSfixHandler != NULL) {
            _usrGNSSfixHandler(&_GNSSfix, _usrGNSSfixHandlerArgs);
        }
    }
    else if(_buffStartsWith(buff, "+LPGNSSASSISTANCE: "))
    {
        cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_GNSS_ASSISTANCE_DATA;

        uint16_t dataSize = buff->size - _strLitLen("+LPGNSSASSISTANCE: ");
        uint8_t *data = buff->data + _strLitLen("+LPGNSSASSISTANCE: ");

        char *start = (char*) data;
        uint8_t partNo = 0;

        WalterMOdemGNSSAssistanceTypeDetails *details = NULL;

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
                                details =
                                    &(cmd->rsp->data.gnssAssistance.almanac);
                                break;

                            case '1':
                                details =
                                    &(cmd->rsp->data.gnssAssistance.ephemeris);
                                break;
                        }
                        break;

                    case 1:
                        if(details != NULL) {
                            details->available = start[0] == '1';
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
                
                /* +2 for the comma and trailing space */
                start = (char*) data + i + 2;
                partNo += 1;
            }
        }
    }
    else if(_buffStartsWith(buff, "+CCLK: \""))
    {
        buff->data[buff->size - 1] = '\0';
        char *data = (char*) buff->data + _strLitLen("+CCLK: \"");
        cmd->rsp->type = WALTER_MODEM_RSP_DATA_TYPE_CLOCK;
        cmd->rsp->data.clock = strTotime(data, "%y/%m/%d,%H:%M:%S");

        if(cmd->rsp->data.clock < WALTER_MODEM_MIN_VALID_TIMESTAMP) {
            cmd->rsp->data.clock = -1;
        }
    }

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

    /* Got expected answer of response */
    _finishQueueCmd(cmd);
    buff->free = true;
}

bool WalterModem::begin(HardwareSerial *uart)
{
    if(_initialized) {
        return true;
    }
    
    _taskQueue.handle = xQueueCreateStatic(WALTER_MODEM_TASK_QUEUE_MAX_ITEMS,
        sizeof(WalterModemTaskQueueItem), _taskQueue.mem,
        &(_taskQueue.memHandle));

    _uart = uart;
    pinMode(WALTER_MODEM_PIN_RESET, OUTPUT);
    digitalWrite(WALTER_MODEM_PIN_RESET, LOW);

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

    _queueTask = xTaskCreateStaticPinnedToCore(_queueProcessingTask,
        "queueProcessingTask", WALTER_MODEM_TASK_STACK_SIZE, NULL,
        tskIDLE_PRIORITY, _queueTaskStack, &_queueTaskBuf, 1);

    if(!reset()) {
        return false;
    }

    if(!configCMEErrorReports()) {
        return false;
    }

    if(!configCEREGReports()) {
        return false;
    }

    _initialized = true;
    return true;
}

void WalterModem::setATHandler(
    void (*handler)(const uint8_t*, uint16_t, void*),
    void *args)
{
    _usrATHandler = handler;
    _usrATHandlerArgs = args;
}

void WalterModem::setGNSSfixHandler(
    void (*handler)(const WalterModemGNSSFix*, void*),
    void *args)
{
    _usrGNSSfixHandler = handler;
    _usrGNSSfixHandlerArgs = args;
}

bool WalterModem::sendCmd(const char *cmd)
{
    const char *_cmdArr[WALTER_MODEM_COMMAND_MAX_ELEMS + 1] = { cmd };
    return _addQueueCmd(_cmdArr, NULL, NULL, NULL, NULL, NULL, NULL,
        WALTER_MODEM_CMD_TYPE_TX) != NULL;
}

bool WalterModem::reset(WalterModemRsp *rsp, walterModemCb cb, void *args)
{
    _runCmd({}, "+SYSSTART", rsp, cb, args, NULL, NULL,
        WALTER_MODEM_CMD_TYPE_WAIT);

    digitalWrite(WALTER_MODEM_PIN_RESET, LOW);
    delay(10);
    digitalWrite(WALTER_MODEM_PIN_RESET, HIGH);

    /* Reset internal library state */
    _regState = WALTER_MODEM_NETWORK_REG_NOT_SEARCHING;

    _returnAfterReply();
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

bool WalterModem::getRadioBands(
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    _runCmd(arr("AT+SQNBANDSEL?"), "OK", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::getSIMState(WalterModemRsp *rsp, walterModemCb cb, void *args)
{
    _runCmd({"AT+CPIN?"}, "OK", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::unlockSIM(
    WalterModemRsp *rsp, 
    walterModemCb cb,
    void *args,
    const char *pin)
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

    if(mode == WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC) {
        _runCmd(arr("AT+COPS=", _digitStr(mode)), "OK", rsp, cb, args);
        _returnAfterReply();
    } else {
        _runCmd(arr(
            "AT+COPS=",
            _digitStr(_networkSelMode),",",
            _digitStr(_operator.format),",",
            _atStr(_operator.name)
        ), "OK", rsp, cb, args);
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

    //TODO: check if the response type is not overwritten when the Async
    // callback system is used.
    if(rsp != NULL) {
        rsp->type = WALTER_MODEM_RSP_DATA_TYPE_PDP_CTX_ID;
        rsp->data.pdpCtxId = ctx->id;
    }

    auto completeHandler = [](WalterModemCmd *cmd, WalterModemState result) 
    {
        WalterModemPDPContext *ctx =
            (WalterModemPDPContext*) cmd->completeHandlerArg;
        
        if(result == WALTER_MODEM_STATE_OK) {
            ctx->state = WALTER_MODEM_PDP_CONTEXT_STATE_INACTIVE;
        }
    };

    _runCmd(arr(
        "AT+CGDCONT=",
        _digitStr(ctx->id),",",
        _atStr(_pdpTypeStr(ctx->type)),",",
        _atStr(ctx->apn),",",
        _atStr(ctx->pdpAddress),",",
        _digitStr(ctx->dataComp),",",
        _digitStr(ctx->headerComp),",",
        _digitStr(ctx->ipv4AllocMethod),",",
        _digitStr(ctx->requestType),",",
        _digitStr(ctx->pcscfMethod),",",
        _atBool(ctx->forIMCN),",",
        _atBool(ctx->useNSLPI),",",
        _atBool(ctx->useSecurePCO),",",
        _atBool(ctx->useNASIPv4MTUDiscovery),",",
        _atBool(ctx->useLocalAddrInd),",",
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
        WalterModemPDPContext *ctx =
            (WalterModemPDPContext*) cmd->completeHandlerArg;
        
        if(result == WALTER_MODEM_STATE_OK) {
            ctx->state = WALTER_MODEM_PDP_CONTEXT_STATE_ACTIVE;
            //TODO: set all other PDP contexts to inactive.
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
            //TODO: set active PDP context to ATTACHED.
        }
    };

    _runCmd(arr("AT+CGATT=",_atBool(attach)), "OK", rsp, cb, args,
        completeHandler);
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

    _runCmd(arr("AT+CGPADDR=",_digitStr(ctx->id)), "OK", rsp, cb, args);
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

    //TODO: check if the response type is not overwritten when the Async
    // callback system is used.
    if(rsp != NULL) {
        rsp->type = WALTER_MODEM_RSP_DATA_TYPE_SOCKET_ID;
        rsp->data.socketId = sock->id;
    }

    auto completeHandler = [](WalterModemCmd *cmd, WalterModemState result) 
    { 
        WalterModemSocket *sock = (WalterModemSocket*) cmd->completeHandlerArg;
        
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
            sock->state = WALTER_MODEM_SOCKET_STATE_CLOSED;
        }
    };

    _runCmd(arr(
        "AT+SQNSH=",
        _digitStr(sock->id)),
        "OK", rsp, cb, args, completeHandler, sock);
    _returnAfterReply();
}

bool WalterModem::socketSend(
    const uint8_t *data,
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
        "OK", rsp, cb, args, NULL, NULL, WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT, 
            data, dataSize);
    _returnAfterReply();
}

bool WalterModem::getClock(WalterModemRsp *rsp, walterModemCb cb, void *args)
{
    _runCmd(arr("AT+CCLK?"), "OK", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::socketSend(
    const char *str,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args,
    WalterModemRAI rai,
    int socketId)
{
    return socketSend((const uint8_t*) str, strlen(str), rsp, cb, args, rai, 
        socketId);
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
    _runCmd(arr("AT+LPGNSSASSISTANCE=", _digitStr(type)),
        type == WALTER_MODEM_GNSS_ASSISTANCE_TYPE_ALMANAC ?
            "+LPGNSSASSISTANCE:0" : "+LPGNSSASSISTANCE:1", rsp, cb, args);
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

    _runCmd(arr(
        "AT+LPGNSSFIXPROG=\"",
        gnssActionStr(action),"\""), "OK", rsp, cb, args);
    _returnAfterReply();
}
        