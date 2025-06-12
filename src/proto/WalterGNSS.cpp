/**
 * @file WalterGNSS.cpp
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

#if CONFIG_WALTER_MODEM_ENABLE_GNSS
    #pragma region PRIVATE_METHODS
void WalterModem::_dispatchEvent(const WalterModemGNSSFix *fix)
{
    WalterModemEventHandler *handler = _eventHandlers + WALTER_MODEM_EVENT_TYPE_GNSS;
    if (handler->gnssHandler == nullptr) {
        return;
    }

    auto start = std::chrono::steady_clock::now();
    handler->gnssHandler(fix, handler->args);
    _checkEventDuration(start);
}
    #pragma endregion

    #pragma region PUBLIC_METHODS
bool WalterModem::gnssConfig(
    WalterModemGNSSSensMode sensMode,
    WalterModemGNSSAcqMode acqMode,
    WalterModemGNSSLocMode locMode,
    WalterModemRsp *rsp,
    walterModemCb cb,
    void *args)
{
    _runCmd(
        arr("AT+LPGNSSCFG=",
            _digitStr(locMode),
            ",",
            _digitStr(sensMode),
            ",2,,1,",
            _digitStr(acqMode)),
        "OK",
        rsp,
        cb,
        args);
    _returnAfterReply();
}

bool WalterModem::gnssGetAssistanceStatus(WalterModemRsp *rsp, walterModemCb cb, void *args)
{
    _runCmd(arr("AT+LPGNSSASSISTANCE?"), "OK", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::gnssUpdateAssistance(
    WalterModemGNSSAssistanceType type, WalterModemRsp *rsp, walterModemCb cb, void *args)
{
    _runCmd(arr("AT+LPGNSSASSISTANCE=", _digitStr(type)), "+LPGNSSASSISTANCE:", rsp, cb, args);
    _returnAfterReply();
}

bool WalterModem::gnssPerformAction(
    WalterModemGNSSAction action, WalterModemRsp *rsp, walterModemCb cb, void *args)
{
    auto gnssActionStr = [](WalterModemGNSSAction action) {
        switch (action) {
        case WALTER_MODEM_GNSS_ACTION_GET_SINGLE_FIX:
            return "single";

        case WALTER_MODEM_GNSS_ACTION_CANCEL:
            return "stop";
        }
        return "";
    };

    _runCmd(arr("AT+LPGNSSFIXPROG=\"", gnssActionStr(action), "\""), "OK", rsp, cb, args);
    _returnAfterReply();
}
bool WalterModem::gnssSetUTCTime(uint64_t epochTime, WalterModemRsp *rsp, walterModemCb cb, void *args){
    char utcTimeStr[32] = {0};

    if (!timeToStr(epochTime, utcTimeStr, sizeof(utcTimeStr))) {
        return false; // conversion failed
    }

    _runCmd(arr("AT+LPGNSSUTCTIME=",_atStr(utcTimeStr)), "OK", rsp, cb, args);
    _returnAfterReply();
}
bool WalterModem::gnssGetUTCTime(WalterModemRsp *rsp, walterModemCb cb, void *args){
    _runCmd(arr("AT+LPGNSSUTCTIME?"), "OK", rsp, cb, args);
    _returnAfterReply();
}
void WalterModem::gnssSetEventHandler(walterModemGNSSEventHandler handler, void *args)
{
    _eventHandlers[WALTER_MODEM_EVENT_TYPE_GNSS].gnssHandler = handler;
    _eventHandlers[WALTER_MODEM_EVENT_TYPE_GNSS].args = args;
}
    #pragma endregion
#endif