/**
 * @file WalterModem.h
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
 * This file contains the headers of Walter's modem library.
 */

#ifndef WALTER_MODEM_H
#define WALTER_MODEM_H

#include <mutex>
#include <bitset>
#include <cstdint>
#include <Arduino.h>
#include <condition_variable>

/**
 * @brief When this define is set to 1 the raw modem communication will be 
 * printed onto the serial console.
 */
#ifndef WALTER_MODEM_DEBUG_ENABLED
    #define WALTER_MODEM_DEBUG_ENABLED 1
#endif 

/**
 * @brief The maximum number of items in the task queue.
 */
#define WALTER_MODEM_TASK_QUEUE_MAX_ITEMS 32

/**
 * @brief The size in bytes of the task queue.
 */
#define WALTER_MODEM_TASK_QUEUE_SIZE \
    WALTER_MODEM_TASK_QUEUE_MAX_ITEMS * sizeof(WalterModemTaskQueueItem)

/**
 * @brief The maximum number of pending commands.
 */
#define WALTER_MODEM_MAX_PENDING_COMMANDS 32

/**
 * @brief The size of an AT response buffer.
 */
#define WALTER_MODEM_RSP_BUF_SIZE 1536

/**
 * @brief The maximum number of AT response buffers in the parser's pool.
 */
#define WALTER_MODEM_AT_RSP_POOL_SIZE 8

/**
 * @brief The size of the stack of the command and response processing task.
 */
#define WALTER_MODEM_TASK_STACK_SIZE 2048

/**
 * @brief The default number of attempts to execute a command.
 */
#define WALTER_MODEM_DEFAULT_CMD_ATTEMTS 3

/**
 * The maximum number of elements allowed to build an AT command.
*/
#define WALTER_MODEM_COMMAND_MAX_ELEMS 63

/**
 * @brief The maximum numbers of characters of the APN. 
 */
#define WALTER_MODEM_APN_MAX_SIZE 99

/**
 * @brief The size of an APN buffer.
 */
#define WALTER_MODEM_APN_BUF_SIZE (WALTER_MODEM_APN_MAX_SIZE + 1)

/**
 * @brief The maximum number of characters in the string representation of the 
 * PDP address.
 */
#define WALTER_MODEM_PDP_ADDR_MAX_SIZE 63

/**
 * @brief The size of a PDP address buffer.
 */
#define WALTER_MODEM_PDP_ADDR_BUF_SIZE (WALTER_MODEM_PDP_ADDR_MAX_SIZE + 1)

/**
 * @brief The maximum number of characters of a PDP context username.
 */
#define WALTER_MODEM_PDP_AUTH_USER_MAX_SIZE 63

/**
 * @brief The size of a PDP context username buffer.
 */
#define WALTER_MODEM_PDP_AUTH_USER_BUF_SIZE \
    (WALTER_MODEM_PDP_AUTH_USER_MAX_SIZE + 1)

/**
 * @brief The maximum number of characters of a PDP context password. 
 */
#define WALTER_MODEM_PDP_AUTH_PASS_MAX_SIZE 63

/**
 * @brief The size of a PDP context password buffer.
 */
#define WALTER_MODEM_PDP_AUTH_PASS_BUF_SIZE \
    (WALTER_MODEM_PDP_AUTH_PASS_MAX_SIZE + 1)

/**
 * @brief The maximum number of PDP contexts that the library can support.
 */
#define WALTER_MODEM_MAX_PDP_CTXTS 8

/**
 * @brief The maximum number of characters of an operator name.
 */
#define WALTER_MODEM_OPERATOR_MAX_SIZE 16

/**
 * @brief The size of an operator name buffer.
 */
#define WALTER_MODEM_OPERATOR_BUF_SIZE \
    (WALTER_MODEM_OPERATOR_MAX_SIZE + 1)

/**
 * @brief The maximum number of band selection configurations 
 */
#define WALTER_MODEM_MAX_BANDSEL_SETSIZE 32

/**
 * @brief The maximum number of sockets.
 */
#define WALTER_MODEM_MAX_SOCKETS 6

/**
 * @brief The maximum number of characters in a hostname.
 */
#define WALTER_MODEM_HOSTNAME_MAX_SIZE 127

/**
 * @brief The size of a hostname buffer.
 */
#define WALTER_MODEM_HOSTNAME_BUF_SIZE \
    (WALTER_MODEM_HOSTNAME_MAX_SIZE + 1)

/**
 * @brief The maximum number of tracked GNSS satellites.
 */
#define WALTER_MODEM_GNSS_MAX_SATS 32

/**
 * @brief This enum groups status codes of functions and operational components
 * of the modem.
 */
typedef enum {
    WALTER_MODEM_STATE_OK = 0,
    WALTER_MODEM_STATE_ERROR,
    WALTER_MODEM_STATE_TIMEOUT,
    WALTER_MODEM_STATE_NO_MEMORY,
    WALTER_MODEM_STATE_NO_FREE_PDP_CONTEXT,
    WALTER_MODEM_STATE_NO_SUCH_PDP_CONTEXT,
    WALTER_MODEM_STATE_NO_FREE_SOCKET,
    WALTER_MODEM_STATE_NO_SUCH_SOCKET,
} WalterModemState;

/**
 * @brief The possible states that the SIM card can be in.
 */
typedef enum {
    WALTER_MODEM_SIM_STATE_READY,
    WALTER_MODEM_SIM_STATE_PIN_REQUIRED,
    WALTER_MODEM_SIM_STATE_PUK_REQUIRED,
    WALTER_MODEM_SIM_STATE_PHONE_TO_SIM_PIN_REQUIRED,
    WALTER_MODEM_SIM_STATE_PHONE_TO_FIRST_SIM_PIN_REQUIRED,
    WALTER_MODEM_SIM_STATE_PHONE_TO_FIRST_SIM_PUK_REQUIRED,
    WALTER_MODEM_SIM_STATE_PIN2_REQUIRED,
    WALTER_MODEM_SIM_STATE_PUK2_REQUIRED,
    WALTER_MODEM_SIM_STATE_NETWORK_PIN_REQUIRED,
    WALTER_MODEM_SIM_STATE_NETWORK_PUK_REQUIRED,
    WALTER_MODEM_SIM_STATE_NETWORK_SUBSET_PIN_REQUIRED,
    WALTER_MODEM_SIM_STATE_NETWORK_SUBSET_PUK_REQUIRED,
    WALTER_MODEM_SIM_STATE_SERVICE_PROVIDER_PIN_REQUIRED,
    WALTER_MODEM_SIM_STATE_SERVICE_PROVIDER_PUK_REQUIRED,
    WALTER_MODEM_SIM_STATE_CORPORATE_SIM_REQUIRED,
    WALTER_MODEM_SIM_STATE_CORPORATE_PUK_REQUIRED,
} WalterModemSIMState;

/**
 * @brief The different types of 3GPP access technologies supported by Walter.
 */
typedef enum {
    WALTER_MODEM_RAT_LTEM = 0,
    WALTER_MODEM_RAT_NBIOT = 1,
    WALTER_MODEM_RAT_AUTO = 2
} WalterModemRAT;

/**
 * @brief The different operational modes of the modem.
 */
typedef enum {
    WALTER_MODEM_OPSTATE_MINIMUM = 0,
    WALTER_MODEM_OPSTATE_FULL = 1,
    WALTER_MODEM_OPSTATE_NO_RF = 4,
    WALTER_MODEM_OPSTATE_MANUFACTURING = 5,
} WalterModemOpState;

/**
 * @brief The different network registration states that the modem can be in.
 */
typedef enum {
    WALTER_MODEM_NETWORK_REG_NOT_SEARCHING = 0,
    WALTER_MODEM_NETOWRK_REG_REGISTERED_HOME = 1,
    WALTER_MODEM_NETOWRK_REG_SEARCHING = 2,
    WALTER_MODEM_NETOWRK_REG_DENIED = 3,
    WALTER_MODEM_NETOWRK_REG_UNKNOWN = 4,
    WALTER_MODEM_NETOWRK_REG_REGISTERED_ROAMING = 5,
    WALTER_MODEM_NETOWRK_REG_REGISTERED_SMS_ONLY_HOME = 6,
    WALTER_MODEM_NETOWRK_REG_REGISTERED_SMS_ONLY_ROAMING = 7,
    WALTER_MODEM_NETOWRK_REG_ATTACHED_EMERGENCY_ONLY = 8,
    WALTER_MODEM_NETOWRK_REG_REGISTERED_CSFB_NOT_PREFERRED_HOME = 9,
    WALTER_MODEM_NETOWRK_REG_REGISTERED_CSFB_NOT_PREFERRED_ROAMING = 10,
    WALTER_MODEM_NETOWRK_REG_REGISTERED_TEMP_CONN_LOSS = 80
} WalterModemNetworkRegState;

/**
 * @brief The CME error reporting methods.
 */
typedef enum {
    WALTER_MODEM_CME_ERROR_REPORTS_OFF,
    WALTER_MODEM_CME_ERROR_REPORTS_NUMERIC,
    WALTER_MODEM_CME_ERROR_REPORTS_VERBOSE
} WalterModemCMEErrorReportsType;

/**
 * @brief This CEREG unsolicited reporting methods.
 */
typedef enum {
    WALTER_MODEM_CEREG_REPORTS_OFF = 0,
    WALTER_MODEM_CEREG_REPORTS_ENABLED = 1,
    WALTER_MODEM_CEREG_REPORTS_ENABLED_WITH_LOCATION = 2,
    WALTER_MODEM_CEREG_REPORTS_ENABLED_WITH_LOCATION_EMM_CAUSE = 3,
    WALTER_MODEM_CEREG_REPORTS_ENABLED_UE_PSM_WITH_LOCATION= 4,
    WALTER_MODEM_CEREG_REPORTS_ENABLED_UE_PSM_WITH_LOCATION_EMM_CAUSE = 5
} WalterModemCEREGReportsType;

/**
 * @brief All supported CME error codes.
 */
typedef enum {
    WALTER_MODEM_CME_EQUIPMENT_FAILURE = 0,
    WALTER_MODEM_CME_NO_CONNECTION = 1,
    WALTER_MODEM_CME_PHONE_ADAPTER_LINK_RESERVED = 2,
    WALTER_MODEM_CME_OPERATION_NOT_ALLOWED = 3,
    WALTER_MODEM_CME_OPERATION_NOT_SUPPORTED = 4,
    WALTER_MODEM_CME_PH_SIM_PIN_REQUIRED = 5,
    WALTER_MODEM_CME_PH_FSIM_PIN_REQUIRED = 6,
    WALTER_MODEM_CME_PH_FSIM_PUK_REQUIRED = 7,
    WALTER_MODEM_CME_SIM_NOT_INSERTED = 10,
    WALTER_MODEM_CME_SIM_PIN_REQUIRED = 11,
    WALTER_MODEM_CME_SIM_PUK_REQUIRED = 12,
    WALTER_MODEM_CME_SIM_FAILURE = 13,
    WALTER_MODEM_CME_SIM_BUSY = 14,
    WALTER_MODEM_CME_SIM_WRONG = 15,
    WALTER_MODEM_CME_INCORRECT_PASSWORD = 16,
    WALTER_MODEM_CME_SIM_PIN2_REQUIRED = 17,
    WALTER_MODEM_CME_SIM_PUK2_REQUIRED = 18,
    WALTER_MODEM_CME_MEMORY_FULL = 20,
    WALTER_MODEM_CME_INVALID_INDEX = 21,
    WALTER_MODEM_CME_NOT_FOUND = 22,
    WALTER_MODEM_CME_MEMORY_FAILURE = 23,
    WALTER_MODEM_CME_TEXT_STRING_TOO_LONG = 24,
    WALTER_MODEM_CME_INVALID_CHARS_IN_TEXT_STRING = 25,
    WALTER_MODEM_CME_DIAL_STRING_TOO_LONG = 26,
    WALTER_MODEM_CME_INVALID_CHARS_IN_DIAL_STRING = 27,
    WALTER_MODEM_CME_NO_NETWORK_SERVICE = 30,
    WALTER_MODEM_CME_NETWORK_TIMEOUT = 31,
    WALTER_MODEM_CME_NETWORK_NOT_ALLOWED_EMERGENCY_CALS_ONLY = 32,
    WALTER_MODEM_CME_NETWORK_PERSONALISATION_PIN_REQUIRED = 40,
    WALTER_MODEM_CME_NETWORK_PERSONALISATION_PUK_REQUIRED = 41,
    WALTER_MODEM_CME_NETWORK_SUBSET_PERSONALISATION_PIN_REQUIRED = 42,
    WALTER_MODEM_CME_NETWORK_SUBSET_PERSONALISATION_PUK_REQUIRED = 43,
    WALTER_MODEM_CME_SERVICE_PROVIDER_PERSONALISATION_PIN_REQUIRED = 44,
    WALTER_MODEM_CME_SERVICE_PROVIDER_PERSONALISATION_PUK_REQUIRED = 45,
    WALTER_MODEM_CME_CORPORATE_PERSONALISATION_PIN_REQUIRED = 46,
    WALTER_MODEM_CME_CORPORATE_PERSONALISATION_PUK_REQUIRED = 47,
    WALTER_MODEM_CME_HIDDEN_KEY_REQUIRED = 48,
    WALTER_MODEM_CME_EAP_METHOD_NOT_SUPPORTED = 49,
    WALTER_MODEM_CME_INCORRECT_PARAMETERS = 50,
    WALTER_MODEM_CME_SYSTEM_FAILURE = 60,
    WALTER_MODEM_CME_UNKNOWN_ERROR = 100,
    WALTER_MODEM_CME_UPGRADE_FAILED_GENERAL_ERROR = 528,
    WALTER_MODEM_CME_UPGRADE_FAILED_CORRUPTED_IMAGE = 529,
    WALTER_MODEM_CME_UPGRADE_FAILED_INVALID_SIGNATURE = 530,
    WALTER_MODEM_CME_UPGRADE_FAILED_NETWORK_ERROR = 531,
    WALTER_MODEM_CME_UPGRADE_FAILED_ALREADY_IN_PROGRESS = 532,
    WALTER_MODEM_CME_UPGRADE_CANCEL_FAILED_NO_UPGRADE_IN_PROGRESS = 533,
    WALTER_MODEM_CME_HW_CONFIG_FAILED_GENERAL_ERROR = 540,
    WALTER_MODEM_CME_HW_CONFIG_FAILED_INVALID_FUNCTION = 541,
    WALTER_MODEM_CME_HW_CONFIG_FAILED_INVALID_FUNCTION_PARAM = 542,
    WALTER_MODEM_CME_HW_CONFIG_FAILED_PINS_ALREADY_ASSIGNED = 543,
} WalterModemCMEError;

/**
 * @brief The different states the raw RX response parser can be in.
 */
typedef enum {
    WALTER_MODEM_RSP_PARSER_START_CR,
    WALTER_MODEM_RSP_PARSER_START_LF,
    WALTER_MODEM_RSP_PARSER_DATA,
    WALTER_MODEM_RSP_PARSER_DATA_PROMPT,
    WALTER_MODEM_RSP_PARSER_END_LF
} WalterModemRspParserState;

/**
 * @brief The types of command supported by the queue task.
 */
typedef enum {
    WALTER_MODEM_CMD_TYPE_TX,
    WALTER_MODEM_CMD_TYPE_TX_WAIT,
    WALTER_MODEM_CMD_TYPE_WAIT,
    WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT
} WalterModemCmdType;

/**
 * @brief The different states the AT command FSM can be in.
 */
typedef enum {
    WALTER_MODEM_CMD_STATE_FREE,
    WALTER_MODEM_CMD_STATE_POOLED,
    WALTER_MODEM_CMD_STATE_NEW,
    WALTER_MODEM_CMD_STATE_PENDING,
    WALTER_MODEM_CMD_STATE_RETRY_AFTER_ERROR,
    WALTER_MODEM_CMD_STATE_SYNC_LOCK_NOTIFIED,
    WALTER_MODEM_CMD_STATE_COMPLETE,
} WalterModemCmdState;

/**
 * @brief This enumeration represents the different states a PDP context can be 
 * in.
 */
typedef enum {
    WALTER_MODEM_PDP_CONTEXT_STATE_FREE = 0,
    WALTER_MODEM_PDP_CONTEXT_STATE_RESERVED = 1,
    WALTER_MODEM_PDP_CONTEXT_STATE_INACTIVE = 2,
    WALTER_MODEM_PDP_CONTEXT_STATE_ACTIVE = 3,
    WALTER_MODEM_PDP_CONTEXT_STATE_ATTACHED = 4
} WalterModemPDPContextState;

/**
 * @brief The supported packet data protocol types.
 */
typedef enum {
    WALTER_MODEM_PDP_TYPE_X25,
    WALTER_MODEM_PDP_TYPE_IP,
    WALTER_MODEM_PDP_TYPE_IPV6,
    WALTER_MODEM_PDP_TYPE_IPV4V6,
    WALTER_MODEM_PDP_TYPE_OSPIH,
    WALTER_MODEM_PDP_TYPE_PPP,
    WALTER_MODEM_PDP_TYPE_NON_IP
} WalterModemPDPType;

/**
 * @brief The supported packet data protocol header compression mechanisms.
 */
typedef enum {
    WALTER_MODEM_PDP_HCOMP_OFF = 0,
    WALTER_MODEM_PDP_HCOMP_ON = 1,
    WALTER_MODEM_PDP_HCOMP_RFC1144 = 2,
    WALTER_MODEM_PDP_HCOMP_RFC2507 = 3,
    WALTER_MODEM_PDP_HCOMP_RFC3095 = 4,
    WALTER_MODEM_PDP_HCOMP_UNSPEC = 99
} WalterModemPDPHeaderCompression;

/**
 * @brief The supported packet data protocol data compression mechanisms.
 */
typedef enum {
    WALTER_MODEM_PDP_DCOMP_OFF = 0,
    WALTER_MODEM_PDP_DCOMP_ON = 1,
    WALTER_MODEM_PDP_DCOMP_V42BIS = 2,
    WALTER_MODEM_PDP_DCOMP_V44 = 3,
    WALTER_MODEM_PDP_DCOMP_UNSPEC = 99
} WalterModemPDPDataCompression;

/**
 * @brief The supported packet data protocol IPv4 address allocation methods.
 */
typedef enum {
    WALTER_MODEM_PDP_IPV4_ALLOC_NAS = 0,
    WALTER_MODEM_PDP_IPV4_ALLOC_DHCP = 1
} WalterModemPDPIPv4AddrAllocMethod;

/**
 * @brief The supported packet data protocol request types.
 */
typedef enum {
    WALTER_MODEM_PDP_REQUEST_NEW_OR_HANDOVER = 0,
    WALTER_MODEM_PDP_REQUEST_EMERGENCY = 1,
    WALTER_MODEM_PDP_REQUEST_NEW = 2,
    WALTER_MODEM_PDP_REQUEST_HANDOVER = 3,
    WALTER_MODEM_PDP_REQUEST_EMERGENCY_HANDOVER = 4,
} WalterModemPDPRequestType;

/**
 * @brief The supported types of P-CSCF discovery in a packet data context.
 */
typedef enum {
    WALTER_MODEM_PDP_PCSCF_AUTO = 0,
    WALTER_MDOEM_PDP_PCSCF_NAS = 1
} WalterModemPDPPCSCFDiscoveryMethod;

/**
 * @brief The authentication protocol used within the PDP context.
 */
typedef enum {
    WALTER_MODEM_PDP_AUTH_PROTO_NONE = 0,
    WALTER_MODEM_PDP_AUTH_PROTO_PAP = 1,
    WALTER_MODEM_PDP_AUTH_PROTO_CHAP = 2
} WalterModemPDPAuthProtocol;

/**
 * @brief This enum represents the different implemented response types.
 */
typedef enum {
    WALTER_MODEM_RSP_DATA_TYPE_NO_DATA,
    WALTER_MODEM_RSP_DATA_TYPE_OPSTATE,
    WALTER_MODEM_RSP_DATA_TYPE_SIM_STATE,
    WALTER_MODEM_RSP_DATA_TYPE_CME_ERROR,
    WALTER_MODEM_RSP_DATA_TYPE_PDP_CTX_ID,
    WALTER_MODEM_RSP_DATA_TYPE_BANDSET_CFG_SET,
    WALTER_MDOEM_RSP_DATA_TYPE_PDP_ADDR,
    WALTER_MODEM_RSP_DATA_TYPE_SOCKET_ID,
    WALTER_MODEM_RSP_DATA_TYPE_GNSS_ASSISTANCE_DATA,
    WALTER_MODEM_RSP_DATA_TYPE_CLOCK
} WalterModemRspDataType;

/**
 * @brief The supported network selection modes.
 */
typedef enum {
    WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC,
    WALTER_MODEM_NETWORK_SEL_MODE_MANUAL,
    WALTER_MODEM_NETWORK_SEL_MODE_UNREGISTER,
    WALTER_MODEM_NETWORK_SEL_MODE_MANUAL_AUTO_FALLBACK
} WalterModemNetworkSelMode;

/**
 * @brief The supported netowrk operator formats. 
 */
typedef enum {
    WALTER_MODEM_OPERATOR_FORMAT_LONG_ALPHANUMERIC = 0,
    WALTER_MODEM_OPERATOR_FORMAT_SHORT_ALPHANUMERIC = 1,
    WALTER_MODEM_OPERATOR_FORMAT_NUMERIC = 2
} WalterModemOperatorFormat;

/**
 * @brief This enumeration represents the different bands that the modem
 * can support. The enum can be used as a mask over the 'bands' member in a 
 * band selection configuration to check if the band is configured.
 */
typedef enum {
    WALTER_MODEM_BAND_B1 = 0x00001,
    WALTER_MODEM_BAND_B2 = 0x00002,
    WALTER_MODEM_BAND_B3 = 0x00004,
    WALTER_MODEM_BAND_B4 = 0x00008,
    WALTER_MODEM_BAND_B5 = 0x00010,
    WALTER_MODEM_BAND_B8 = 0x00020,
    WALTER_MODEM_BAND_B12 = 0x00040,
    WALTER_MODEM_BAND_B13 = 0x00080,
    WALTER_MODEM_BAND_B14 = 0x00100,
    WALTER_MODEM_BAND_B17 = 0x00200,
    WALTER_MODEM_BAND_B18 = 0x00400,
    WALTER_MODEM_BAND_B19 = 0x00800,
    WALTER_MODEM_BAND_B20 = 0x01000,
    WALTER_MODEM_BAND_B25 = 0x02000,
    WALTER_MODEM_BAND_B26 = 0x04000,
    WALTER_MODEM_BAND_B28 = 0x08000,
    WALTER_MODEM_BAND_B66 = 0x10000,
    WALTER_MODEM_BAND_B71 = 0x20000,
    WALTER_MODEM_BAND_B85 = 0x40000,
} WalterModemBand;

/**
 * @brief The state of a socket.
 */
typedef enum {
    WALTER_MODEM_SOCKET_STATE_FREE = 0,
    WALTER_MODEM_SOCKET_STATE_RESERVED = 1,
    WALTER_MODEM_SOCKET_STATE_CREATED = 2,
    WALTER_MODEM_SOCKET_STATE_CONFIGURED = 3,
    WALTER_MODEM_SOCKET_STATE_OPENED = 4,
    WALTER_MODEM_SOCKET_STATE_LISTENING = 5,
    WALTER_MODEM_SOCKET_STATE_CLOSED = 6
} WalterModemSocketState;

/**
 * @brief The protocol that us used by the socket. 
 */
typedef enum {
    WALTER_MODEM_SOCKET_PROTO_TCP = 0,
    WALTER_MODEM_SOCKET_PROTO_UDP = 1
} WalterModemSocketProto;

/**
 * @brief Possible methodologies on how a socket handles data from other 
 * hosts besides the IP-address and remote port it is configured for.
 */
typedef enum {
    WALTER_MODEM_ACCEPT_ANY_REMOTE_DISABLED = 0,
    WALTER_MODEM_ACCEPT_ANY_REMOTE_RX_ONLY = 1,
    WALTER_MODEM_ACCEPT_ANY_REMOTE_RX_AND_TX = 2
} WalterModemSocketAcceptAnyRemote;

/**
 * @brief In case of an NB-IoT connection the RAI (Release Assistance
 * Information). The RAI is used to indicate to the network (MME) if there 
 * are going to be other transmissions or not.
 */
typedef enum {
    WALTER_MODEM_RAI_NO_INFO = 0,
    WALTER_MODEM_RAI_NO_FURTHER_RXTX_EXPECTED = 1,
    WALTER_MODEM_RAI_ONLY_SINGLE_RXTX_EXPECTED = 2
} WalterModemRAI;

/**
 * @brief The GNSS location modus. When set to 'on-device location' the GNSS 
 * subsystem will compute position and speed and estimate the error on these
 * parameters.
 */
typedef enum { 
    WALTER_MODEM_GNSS_LOC_MODE_ON_DEVICE_LOCATION = 0
} WalterModemGNSSLocMode;

/**
 * @brief The possible sensitivity settings use by Walter's GNSS receiver. This
 * sets the amount of time that the receiver is actually on. More sensitivity
 * requires more power.
 */
typedef enum {
    WALTER_MODEM_GNSS_SENS_MODE_LOW = 1,
    WALTER_MODEM_GNSS_SENS_MODE_MEDIUM = 2,
    WALTER_MODEM_GNSS_SENS_MODE_HIGH = 3
} WalterModemGNSSSensMode;

/**
 * @brief The possible GNSS acquistion modes. In a cold or warm start situation 
 * Walter has no clue where he is on earth. In hot start mode Walter must know
 * where he is within 100km. When no ephemerides are available and/or the time
 * is not known cold start will be used automatically.
 */
typedef enum {
    WALTER_MODEM_GNSS_ACQ_MODE_COLD_WARM_START = 0,
    WALTER_MODEM_GNSS_ACQ_MODE_HOT_START = 1
} WalterModemGNSSAcqMode;

/**
 * @brief The supported actions that Walter's GNSS can execute.
 */
typedef enum {
    WALTER_MODEM_GNSS_ACTION_GET_SINGLE_FIX = 0,
    WALTER_MODEM_GNSS_ACTION_CANCEL = 1
} WalterModemGNSSAction;

/**
 * @brief The possible GNSS fix statuses.
 */
typedef enum {
    WALTER_MODEM_GNSS_FIX_STATUS_READY = 0,
    WALTER_MODEM_GNSS_FIX_STATUS_STOPPED_BY_USER = 1,
    WALTER_MODEM_GNSS_FIX_STATUS_NO_RTC = 2,
    WALTER_MODEM_GNSS_FIX_STATUS_LTE_CONCURRENCY = 3
} WalterModemGNSSFixStatus;

/**
 * @brief The possible GNSS assistance types.
 */
typedef enum {
    WALTER_MODEM_GNSS_ASSISTANCE_TYPE_ALMANAC = 0,
    WALTER_MODEM_GNSS_ASSISTANCE_TYPE_REALTIME_EPHEMERIS = 1,
} WalterModemGNSSAssistanceType;

/**
 * @brief This structure represents the 
 */
typedef struct {
    /**
     * @brief The number of the satellite.
     */
    uint8_t satNo;

    /**
     * @brief The CN0 signal strength of the satellite in dB/Hz. The minimum
     * required signal strength is 30dB/Hz.
     */
    uint8_t signalStrength;
} WalterModemGNSSSat;

/**
 * @brief This structure represents a GNSS fix.
 */
typedef struct {
    /**
     * @brief The status of the fix.
     */
    WalterModemGNSSFixStatus status = WALTER_MODEM_GNSS_FIX_STATUS_READY;

    /**
     * @brief The id of the fix, always in [0-9].
     */
    uint8_t fixId = 0;

    /**
     * @brief The time of the fix as a unix timestamp.
     */
    int64_t timestamp = 0;

    /**
     * @brief The number of milliseconds used to get the fix.
     */
    uint32_t timeToFix = 0;

    /**
     * @brief The estimated horizontal confidence of the fix in meters. 
     */
    double estimatedConfidence = 20000000;

    /**
     * @brief The latitude of the fix. 
     */
    double latitude = 0;

    /**
     * @brief The longitude of the fix.
     */
    double longitude = 0;

    /**
     * @brief The height above sea level.
     */
    double height = 0;

    /**
     * @brief The speed in northern direction in meters per second.
     */
    double northSpeed = 0;

    /**
     * @brief The speed in eastern direction in meters per second. 
     */
    double eastSpeed = 0;

    /**
     * @brief The downwards speed in meters per second.
     */
    double downSpeed = 0;

    /**
     * @brief The number of received satellites.
     */
    uint8_t satCount = 0;

    /**
     * @brief Satelite numbers and reception strength.
     */
    WalterModemGNSSSat sats[WALTER_MODEM_GNSS_MAX_SATS] = {};
} WalterModemGNSSFix;

/**
 * @brief This structure represents the details of a certain GNSS assistance 
 * type.
 */
typedef struct {
    /**
     * @brief The type of assistance the details are about.
     */
    WalterModemGNSSAssistanceType type;

    /**
     * @brief True when this type of assistance data is available.
     */
    bool available;

    /**
     * @brief The number of seconds since the last update of this type of 
     * assistance data.
     */
    int32_t lastUpdate;

    /**
     * @brief The number of seconds before this type of assistance data should
     * be updated in order not to degrade the GNSS performance.
     */
    int32_t timeToUpdate;

    /**
     * @brief The number of seconds after which this type of assistance data
     * expires and cannot be used by the GNSS system.
     */
    int32_t timeToExpire; 
} WalterMOdemGNSSAssistanceTypeDetails;

/**
 * @brief This structure contains GNSS assistance metadata.
 */
typedef struct {
    /**
     * @brief Almanac data details, this is not needed when real-time ephemeris
     * data is available.
     */
    WalterMOdemGNSSAssistanceTypeDetails almanac;

    /**
     * @brief Real-time ephemeris data details. Use this kind of assistance 
     * data for the fastest and most power efficient GNSS fix.
     */
    WalterMOdemGNSSAssistanceTypeDetails ephemeris;
} WalterModemGNSSAssistance;

/**
 * @brief This structure represents an operator.
 */
typedef struct {
    /**
     * @brief The format in which the operator is stored.
     */
    WalterModemOperatorFormat format;

    /**
     * @brief The name of the operator. 
     */
    char name[WALTER_MODEM_OPERATOR_BUF_SIZE];
} WalterModemOperator;

/**
 * @brief This structure represents a band selection for a given radio access
 * technology and operator.
 */
typedef struct {
    /**
     * @brief The radio access technology for which the bands are configured.
     */
    WalterModemRAT rat;

    /**
     * @brief The mobile network operator or "3GGP" or "standard" for unknown
     * or generic operators.
     */
    WalterModemOperator netOperator;

    /**
     * @brief When the bit is set the respective band is configured to be used.
     * The bands are B1, B2, B3, B4, B5, B8, B12, B13, B14, B17, B18, B19, B20, 
     * B25, B26, B28, B66, B71, B85. For example to check if B1 is configured
     * one must do 'bands & 0x01';
     */
    uint32_t bands;
} WalterModemBandSelection;

/**
 * @brief This structure represents a configuration set of band selection
 * configurations for one or more <RAT,operator> combinations.
 */
typedef struct {
    /**
     * @brief The number of configurations in the set.
     */
    uint8_t count;

    /**
     * @brief The set of band selection configurations.
     */
    WalterModemBandSelection config[WALTER_MODEM_MAX_BANDSEL_SETSIZE];
} WalterModemBandSelectionConfigSet;

/**
 * @brief This structure represents the two addresses that a certain PDP context
 * can have.
 */
typedef struct {
    /**
     * @brief The ID of the context which owns the addresses.
     */
    int pdpCtxId;

    /**
     * @brief Pointer to a 0-terminated string with the first address.
     */
    const char *pdpAddress;

    /**
     * @brief Pointer to a 0-terminated string with the seconds address.
     */
    const char *pdpAddress2;
} WalterModemPDPAddressList;

/**
 * @brief This union groups the response data of all different commands.
 */
union uWalterModemRspData {
    /**
     * @brief The operational state of the modem.
     */
    WalterModemOpState opState;

    /**
     * @brief The state of the SIM card.
     */
    WalterModemSIMState simState;

    /**
     * @brief The CME error received from the modem.
     */
    WalterModemCMEError cmeError;

    /**
     * @brief The ID of a PDP context.
     */
    int pdpCtxId;

    /**
     * @brief The band selection configuration set.
     */
    WalterModemBandSelectionConfigSet bandSelCfgSet;

    /**
     * @brief The list of addresses of a cert
     */
    WalterModemPDPAddressList pdpAddressList;

    /**
     * @brief The ID of a socket.
     */
    int socketId;

    /**
     * @brief The GNSS assistance data status.
     */
    WalterModemGNSSAssistance gnssAssistance;

    /**
     * @brief Unix timestamp of the current time and date in the modem.
     */
    int64_t clock;
};

/**
 * @brief This structure represents a response to a command.
 */
typedef struct {
    /**
     * @brief The result of the executed command.
     */
    WalterModemState result = WALTER_MODEM_STATE_OK;

    /**
     * @brief The type of response
     */
    WalterModemRspDataType type = WALTER_MODEM_RSP_DATA_TYPE_NO_DATA;

    /**
     * @brief The parsed response data based on the type of response.
     */
    union uWalterModemRspData data;
} WalterModemRsp;

/**
 * @brief The callback header used when this library is used asynchronously.
 */
typedef void (*walterModemCb)(const WalterModemRsp *rsp, void *args);

/**
 * @brief This structure groups a mutex and condition variable used to implement
 * the blocking version of the API.
 */
typedef struct {
    /**
     * @brief The condition variable.
     */
    std::condition_variable cond;

    /**
     * @brief The mutex which protects the shared object.
     */
    std::mutex mutex;
} WalterModemCmdLock;

/**
 * @brief This structure represents an AT command to be added to the command
 * queue.
 */
typedef struct sWalterModemCmd {
    /**
     * @brief The current state of the command.
     */
    WalterModemCmdState state = WALTER_MODEM_CMD_STATE_FREE;

    /**
     * @brief The type of AT command.
     */
    WalterModemCmdType type = WALTER_MODEM_CMD_TYPE_TX_WAIT;

    /**
     * @brief The AT command without the trailing \r\n.
     */
    const char *atCmd[WALTER_MODEM_COMMAND_MAX_ELEMS + 1] = {NULL};

    /**
     * @brief Pointer to the data buffer to transmit in case of a
     * WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT command.
     */
    const uint8_t *data;

    /**
     * @brief The number of bytes in the data buffer.
     */
    uint16_t dataSize;

    /**
     * @brief The expected command response starting string.
     */
    const char *atRsp = NULL;

    /**
     * @brief The length of the response starting string.
     */
    size_t atRspLen = 0;

    /**
     * @brief The maximum number of attempts to execute the command.
     */
    uint8_t maxAttempts = WALTER_MODEM_DEFAULT_CMD_ATTEMTS;

    /**
     * @brief The current attempt number.
     */
    uint8_t attempt = 0;

    /**
     * @brief The time on which the current attempt was started.
     */
    TickType_t attemptStart = 0;

    /**
     * @brief A lock and condition variable used to implement the blocking API.
     */
    WalterModemCmdLock cmdLock = {};

    /**
     * @brief The user callback function or NULL when using blocking API.
     */
    walterModemCb userCb = NULL;

    /**
     * @brief Arguments for the user callback function.
     */
    void *userCbArgs = NULL;

    /**
     * @brief Memory used to save response data in. When the user doesn't pass
     * a response object (in case the blocking API is used). The rsp pointer
     * will point to this memory.
     */
    WalterModemRsp rspMem;

    /**
     * @brief Pointer to the response object to store the command results in.
     */
    WalterModemRsp *rsp;
    
    /**
     * @brief Pointer to a function which is called before the command user
     * callback is called. This pointer is used to manage internal library 
     * state.
     */
    void (*completeHandler)(
        struct sWalterModemCmd *cmd,
        WalterModemState result) = NULL;

    /**
     * @brief Pointer to an argument used by the completeHandler.
     */
    void *completeHandlerArg = NULL;
} WalterModemCmd;

/**
 * @brief This structure groups commands which makes it easy to implement simple
 * finite state machines to perform actions that require more than 1 AT command.
 */
typedef struct {
    /**
     * @brief A lock and condition variable used to implement the blocking API.
     */
    WalterModemCmdLock cmdLock = {};

    /**
     * @brief The user callback function or NULL when using blocking API.
     */
    walterModemCb userCb = NULL;

    /**
     * @brief Arguments for the user callback function.
     */
    void *userCbArgs = NULL;

    /**
     * @brief Memory used to save response data in. When the user doesn't pass
     * a response object (in case the blocking API is used). The rsp pointer
     * will point to this memory.
     */
    WalterModemRsp rspMem;

    /**
     * @brief Pointer to the response object to store the command results in.
     */
    WalterModemRsp *rsp;
} WalterModemCmdFsm;

/**
 * @brief This structure represents a buffer from the parser's pool.
 */
typedef struct {
    /**
     * @brief Pointer to the data in the buffer.
     */
    uint8_t data[WALTER_MODEM_RSP_BUF_SIZE] = { 0 };

    /**
     * @brief The number of actual data bytes in the buffer.
     */
    uint16_t size = 0;

    /**
     * @brief This volatile flag is set to true when the buffer is currently
     * not in use and can be used to store the next response in.
     */
    volatile bool free = true;
} WalterModemParserBuffer;

/**
 * @brief This structure groups the AT parser's working data.
 */
typedef struct {
    /**
     * @brief The FSM state the parser currently is in.
     */
    WalterModemRspParserState state = WALTER_MODEM_RSP_PARSER_START_CR;

    /**
     * @brief The buffer currently used by the parser.
     */
    WalterModemParserBuffer *buf = NULL;

    /**
     * @brief The pool of buffers used by the parser and AT response handlers.
     */
    WalterModemParserBuffer pool[WALTER_MODEM_AT_RSP_POOL_SIZE] = {};
} WalterModemATParserData;

/**
 * @brief This structure represents an item in the task queue.
 */
typedef struct {
    /**
     * @brief Pointer to an AT response or NULL when this is an AT command.
     */
    WalterModemParserBuffer *rsp = NULL;

    /**
     * @brief The AT command pointer in case rsp is NULL.
     */
    WalterModemCmd *cmd = NULL;
} WalterModemTaskQueueItem;

/**
 * @brief This structure represents the task queue. This is the queue which 
 * contains both incoming modem data and AT commands to be sent to the modem.
 * This queue is used to synchronize between the modem receive task and the 
 * API (which may be called by one or more tasks).
 */
typedef struct {
    /**
     * @brief The queue handle.
     */
    QueueHandle_t handle;

    /**
     * @brief The memory handle.
     */
    StaticQueue_t memHandle;
    
    /**
     * @brief The statically allocated queue memory.
     */
    uint8_t mem[WALTER_MODEM_TASK_QUEUE_SIZE] = { 0 };
} WalterModemTaskQueue;

/**
 * @brief This structure represents the command queue. This queue is used inside
 * the libraries processing task to manage incoming and pending commands.
 */
typedef struct {
    /**
     * @brief The FiFo command queue.
     */
    WalterModemCmd *queue[WALTER_MODEM_MAX_PENDING_COMMANDS] = { NULL };

    /**
     * @brief Index of the outgoing queue item.
     */
    uint8_t outIdx = 0;

    /**
     * @brief Index of the ingoing queue item.
     */
    uint8_t inIdx = 0;
} WalterModemCmdQueue;

/**
 * @brief This structure represents a PDP context.
 */
typedef struct {
    /**
     * @brief The state of the PDP context.
     */
    WalterModemPDPContextState state = WALTER_MODEM_PDP_CONTEXT_STATE_FREE;

    /**
     * @brief The ID of this PDP data context.
     */
    int id = 0;

    /**
     * @brief The type of packet data protocol.
     */
    WalterModemPDPType type = WALTER_MODEM_PDP_TYPE_IP;

    /**
     * @brief The APN to use.
     */
    char apn[WALTER_MODEM_APN_BUF_SIZE] = { 0 };

    /**
     * @brief The PDP address of this context.
     */
    char pdpAddress[WALTER_MODEM_PDP_ADDR_BUF_SIZE] = { 0 };

    /**
     * @brief A secondary IPv6 PDP address when dual stack is enabled.
     */
    char pdpAddress2[WALTER_MODEM_PDP_ADDR_BUF_SIZE] = { 0 };

    /**
     * @brief The header compression used in the PDP context.
     */
    WalterModemPDPHeaderCompression headerComp = WALTER_MODEM_PDP_HCOMP_UNSPEC;

    /**
     * @brief The data compression method used in the PDP context.
     */
    WalterModemPDPDataCompression dataComp = WALTER_MODEM_PDP_DCOMP_UNSPEC;

    /**
     * @brief The IPv4 address allocation method used in the PDP context.
     */
    WalterModemPDPIPv4AddrAllocMethod ipv4AllocMethod =
        WALTER_MODEM_PDP_IPV4_ALLOC_NAS;

    /**
     * @brief The packet data protocol request type.
     */
    WalterModemPDPRequestType requestType =
        WALTER_MODEM_PDP_REQUEST_NEW_OR_HANDOVER;

    /**
     * @brief The method to use for P-CSCF discovery.
     */
    WalterModemPDPPCSCFDiscoveryMethod pcscfMethod =
        WALTER_MODEM_PDP_PCSCF_AUTO;

    /**
     * @brief This flag must be set when the PDP context is used for IM CN 
     * subsystem-related signalling.
     */
    bool forIMCN = false;

    /**
     * @brief This flag is set when the PDP context should use Non-Access
     * Stratum (NAS) Signalling Low Priority Indication (NSLPI).
     */
    bool useNSLPI = false;

    /**
     * @brief When this flag is set the Protocol Configuration Options (PCO) are
     * requested to be protected.
     */
    bool useSecurePCO = false;

    /**
     * @brief When this flag is set the PDP context will use NAS signalling to
     * discover the IPv4 MTU.
     */
    bool useNASIPv4MTUDiscovery = false;

    /**
     * @brief This flag should be set when the system supports local IP
     * addresses in the Traffic Flow Template (TFT).
     */
    bool useLocalAddrInd = false;

    /**
     * @brief This flag should be set when NAS should be used to discovery the 
     * MTU of non-IP PDP contexts.
     */
    bool useNASNonIPMTUDiscovery = false;

    /**
     * @brief The authentication protocol used to activate the PDP, typically
     * the APN authentication method.
     */
    WalterModemPDPAuthProtocol authProto = WALTER_MODEM_PDP_AUTH_PROTO_NONE;

    /**
     * @brief The user to authenticate.
     */
    char authUser[WALTER_MODEM_PDP_AUTH_USER_BUF_SIZE] = { 0 };

    /**
     * @brief The password to authenticate.
     */
    char authPass[WALTER_MODEM_PDP_AUTH_PASS_BUF_SIZE] = { 0 };
} WalterModemPDPContext;

/**
 * @brief This structure represents a socket.
 */
typedef struct {
    /**
     * @brief The state of 
     */
    WalterModemSocketState state = WALTER_MODEM_SOCKET_STATE_FREE;

    /**
     * @brief The socket identifier.
     */
    int id = 1;

    /**
     * @brief The PDP context id in which the socket is created.
     */
    uint8_t pdpContextId = 1;

    /**
     * @brief Maximum transmission unit used by the TCP/UDP/IP stack.
     */
    uint16_t mtu = 300;

    /**
     * @brief The socket exchange timeout in seconds. When no data is exchanged
     * within the timeout the socket is automatically closed. When this is set
     * to 0 the timeout is disabled. The maximum exchange timeout is 65535
     * seconds.
     */
    uint16_t exchangeTimeout = 90;

    /**
     * @brief The connection timeout in seconds. When a connection to the remote
     * host could not be established within the given timeout an error will be 
     * generated. When this is set to 0 the timeout is disabled. The maximum 
     * connection timeout is 120 seconds.
     */
    uint16_t connTimeout = 60;

    /**
     * @brief The number of milliseconds after which the transmit buffer is 
     * effectively transmitted. The maximum delay is 25500 milliseconds.
     */
    uint16_t sendDelayMs = 5000;

    /**
     * @brief The protocol to use.
     */
    WalterModemSocketProto protocol = WALTER_MODEM_SOCKET_PROTO_UDP;

    /**
     * @brief How to handle data from other hosts than the remote host and port
     * that the socket is configured for. This is only applicable when this is
     * an UDP socket.
     */
    WalterModemSocketAcceptAnyRemote acceptAnyRemote =
        WALTER_MODEM_ACCEPT_ANY_REMOTE_DISABLED;

    /**
     * @brief The IPv4 or IPv6 address of the remote host or a hostname in which
     * case a DNS query will be executed in the background.
     */
    char remoteHost[WALTER_MODEM_HOSTNAME_BUF_SIZE] = { 0 }; 

    /**
     * @brief The remote port to connect to.
     */
    uint16_t remotePort = 0;

    /**
     * @brief In case of UDP this is the local port number to which the remote
     * host can send an answer.
     */
    uint16_t localPort = 0;
} WalterModemSocket;

/**
 * @brief The WalterModem class allows you to use the Sequans Monarch 2 modem
 * and positioning functionality.
 */
class WalterModem
{
    private:
        /**
         * @brief This flag is set to true when the modem is initialized.
         */
        static inline bool _initialized = false;

        /**
         * @brief The hardware serial peripheral used to talk to the modem.
         */
        static inline HardwareSerial *_uart = NULL;

        /**
         * @brief The queue used by the processing task.
         */
        static inline WalterModemTaskQueue _taskQueue = {};

        /**
         * @brief The queue used to store pending tasks in.
         */
        static inline WalterModemCmdQueue _cmdQueue = {};

        /**
         * @brief The set with PDP contexts.
         */
        static inline WalterModemPDPContext
            _pdpCtxSet[WALTER_MODEM_MAX_PDP_CTXTS] = {};

        /**
         * @brief The set with sockets.
         */
        static inline WalterModemSocket
            _socketSet[WALTER_MODEM_MAX_SOCKETS] = {};

        /**
         * @brief The task in which AT commands and responses are handled.
         */
        static inline TaskHandle_t _queueTask;

        /**
         * @brief Handle used to manage the queue processing task stack.
         */
        static inline StaticTask_t _queueTaskBuf;

        /**
         * @brief The statically allocated queue processing task stack memory.
         */
        static inline StackType_t _queueTaskStack[WALTER_MODEM_TASK_STACK_SIZE];

        /**
         * @brief The data of the AT parser.
         */
        static inline WalterModemATParserData _parserData = {};  

        /**
         * @brief The memory pool to save pending commands in.
         */
        static inline WalterModemCmd
            _cmdPool[WALTER_MODEM_MAX_PENDING_COMMANDS] = {};

        /**
         * @brief The current operational state of the modem.
         */
        static inline WalterModemOpState
            _opState = WALTER_MODEM_OPSTATE_MINIMUM;

        /**
         * @brief The current network registration state of the modem.
         */
        static inline WalterModemNetworkRegState
            _regState = WALTER_MODEM_NETWORK_REG_NOT_SEARCHING;

        /**
         * @brief The PIN code when required for the installed SIM or NULL when
         * no PIN code is used.
         */
        static inline const char *_simPIN = NULL;

        /**
         * @brief The chosen network selection mode.
         */
        static inline WalterModemNetworkSelMode
            _networkSelMode = WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC;

        /**
         * @brief An operator to use, this is ignored when automatic operator
         * selection is used.
         */
        static inline WalterModemOperator _operator = {};

        /**
         * @brief The PDP context which is currently in use by the library or
         * NULL when no PDP context is in use. In use doesn't mean that the 
         * context is activated yet it is just a pointer to the PDP context
         * which was last used by any of the functions that work with a PDP
         * context.
         */
        static inline WalterModemPDPContext *_pdpCtx = NULL;

        /**
         * @brief The socket which is currently in use by the library or NULL
         * when no socket is in use.
         */
        static inline WalterModemSocket *_socket = NULL;

        /**
         * @brief The GNSS fix which is currently being processed.
         */
        static inline WalterModemGNSSFix _GNSSfix = {};

        /**
         * @brief Pointer to an optional user AT response handler. When an AT
         * response is received the function will be called with the response
         * body, length of the body and an optional user argument.
         * 
         * @param atBuf Pointer to the incoming AT buffer.
         * @param atBufLen The length of the incoming AT buffer.
         * @param args Optional user arguments.
         */
        static inline void (*_usrATHandler)(
            const uint8_t* atBuf,
            uint16_t atBufLen,
            void* args) = NULL;

        /**
         * @brief Pointer to pass as argument in the _usrATHandler function.
         */
        static inline void *_usrATHandlerArgs = NULL;

        /**
         * @brief Pointer to an optional user GNSS fix handler. When a GNSS fix
         * is acquired this function will be called.
         * 
         * @param fix The GNSS fix data.
         * @param args Optional user arguments.
         */
        static inline void (*_usrGNSSfixHandler)(
            const WalterModemGNSSFix *fix,
            void *args) = NULL;

        /**
         * @brief Pointer to pass as argument in the _usrGNSSfixHandler
         * function.
         */
        static inline void* _usrGNSSfixHandlerArgs = NULL;

        /**
         * @brief Get a command from the command pool.
         * 
         * This function will get a command from the command pool if there is 
         * still space.
         * 
         * @return Pointer to the command or NULL when no more free spaces are 
         * available in the pool.
         */
        static WalterModemCmd* _cmdPoolGet();

        /**
         * @brief Pop the last item off the queue.
         * 
         * This function will return the first inserted item on the queue and 
         * remove it. When the queue is empty this function will return NULL.
         *  
         * @return Pointer to the command or NULL when the queue is empty.
         */
        static WalterModemCmd* _cmdQueuePop();

        /**
         * @brief Put a new item onto the queue.
         * 
         * This function will add a new item on the queue.
         * 
         * @param cmd The command to add to the queue.
         * 
         * @return True on success, false when the queue is full.
         */
        static bool _cmdQueuePut(WalterModemCmd *cmd);

        /**
         * @brief Get a PDP context structure which is not in use.
         * 
         * This function will search for a PDP context structure which can be
         * used to save and register a new PDP context structure with. The
         * returned PDP context will automatically be assigned with a free
         * PDP context identifier.
         * 
         * @return Pointer to a PDP which is not yet in use or NULL when all
         * contexts are in use.
         */
        static WalterModemPDPContext* _pdpContextReserve();

        /**
         * @brief Get a reference to the PDP context with the given id.
         * 
         * This function will return a PDP context with the given id when it 
         * is in use, in all other cases this function will return NULL.
         * 
         * @param id The id of the PDP context or -1 to return the current
         * class PDP context.
         * 
         * @return Pointer to the PDP context with the given id or NULL.
         */
        static WalterModemPDPContext* _pdpContextGet(int id = -1);

        /**
         * @brief Release a PDP context structure back to the pool.
         * 
         * This function will release the PDP context back to the set.
         * 
         * @param ctx The context to release.
         * 
         * @return None.
         */
        static void _pdpContextRelease(WalterModemPDPContext *ctx);

        /**
         * @brief Get a socket structure which is not in use.
         * 
         * This function will search for a socket structure which can be used
         * to create a new socket with. The returned socket will automatically
         * be assigned with a free socket identifier.
         * 
         * @return Pointer to a socket which is not yet in use or NULL when
         * all sockets are in use.
         */
        static WalterModemSocket* _socketReserve();

        /**
         * @brief Get a reference to a socket with the given id.
         * 
         * This function will return a socket with the given id when it is in
         * use, in all other cases this function will return NULL.
         * 
         * @param id The id of the socket or -1 to return the current class
         * socket.
         * 
         * @return Pointer to the socket with the given id or NULL.
         */
        static WalterModemSocket* _socketGet(int id = -1);

        /**
         * @brief Release a socket structure back to the pool.
         * 
         * This function will release the socket back to the socket set.
         * 
         * @param sock The socket to release.
         * 
         * @return None.
         */
        static void _socketRelease(WalterModemSocket *sock);

        /**
         * @brief Handle an AT data byte.
         * 
         * This function is used by the AT data parser to add a databyte to 
         * the buffer currently in use or to reserve a new buffer to add a byte
         * to.
         * 
         * @param data The data byte to handle.
         * 
         * @return None.
         */
        static void _addATByteToBuffer(char data);

        /**
         * @brief Copy the currently received data buffer into the task queue.
         * 
         * This function will copy the current modem receive buffer into the
         * task queue. When the buffer could not be placed in the queue it will
         * be silently dropped.
         * 
         * @return None.
         */
        static void _queueRxBuffer();

        /**
         * @brief Handle and parse modem RX data.
         * 
         * This function is called when the modem placed data in the UART RX
         * buffer. The context is a vTask in the ESP32 Arduino core framework
         * and not an ISR, therefore this function also immediately parses the
         * incoming data into a free pool buffer.
         * 
         * @return None.
         */
        static void _handleRxData();

        /**
         * @brief This is the entrypoint of the queue processing task.
         * 
         * The WalterModem library relies on a single task to handle both
         * incoming data and outgoing commands. This reduces context switching
         * and allows a fully asynchronous (non-blocking) aswell as a
         * synchronous (blocking) API.
         * 
         * @param args A NULL pointer.
         * 
         * @return None.
         */
        static void _queueProcessingTask(void *args);

        /**
         * @brief Add a command to the command queue.
         * 
         * This function add a command to the task queue. This function will 
         * only fail when the command queue is full. The command which is put
         * onto the queue will automatically get the WALTER_MODEM_CMD_STATE_NEW
         * state. This function will never call any callbacks.
         * 
         * @param atCmd NULL terminated array of command elements. The elements
         * must stay available until the command is complete. The array is only
         * shallow copied.
         * @param atRsp The expected AT response.
         * @param rsp Pointer to the response used to save command results.
         * @param userCb Optional user callback.
         * @param userCbArgs Optional user callback arguments.
         * @param completeHandler Optional complete handler function.
         * @param completeHandlerArg Optional argument for the complete handler.
         * @param type The type of queue AT command.
         * @param data Pointer to the data buffer to transmit.
         * @param dataSize The number of bytes in the data buffer.
         * @param maxAttempts The maximum number of retries for this command.
         * 
         * @return Pointer to the command on success, NULL when no memory for
         * the command was available.
         */
        static WalterModemCmd* _addQueueCmd(
            const char *atCmd[WALTER_MODEM_COMMAND_MAX_ELEMS + 1] = { NULL },
            const char *atRsp = NULL,
            WalterModemRsp *rsp = NULL,
            walterModemCb userCb = NULL,
            void *userCbArgs = NULL,
            void (*completeHandler)(struct sWalterModemCmd *cmd,
                WalterModemState result) = NULL,
            void *completeHandlerArg = NULL,
            WalterModemCmdType type = WALTER_MODEM_CMD_TYPE_TX_WAIT,
            const uint8_t *data = NULL,
            uint16_t dataSize = 0,
            uint8_t maxAttempts = WALTER_MODEM_DEFAULT_CMD_ATTEMTS);

        /**
         * @brief Finish a queue command.
         * 
         * This function will call the command user callback in case the async 
         * API is used. When the blocking API is used, this function will notify
         * the condition variable and unlock it.
         * 
         * @param cmd The command to finish.
         * @param result The state to set the response result to. This will be 
         * set to OK when no parameter is passed in.
         * 
         * @return None.
         */
        static void _finishQueueCmd(
            WalterModemCmd *cmd,
            WalterModemState result = WALTER_MODEM_STATE_OK);

        /**
         * @brief Process an AT command from the queue.
         * 
         * This function is called in the queue processing task when an AT 
         * command destined for the modem is received from the queue. This
         * function will process the command by sending it to the modem and 
         * marking it as pending.
         * 
         * @param cmd The command to process.
         * @param queueError This flag is true when the command could not be 
         * added to the command queue of the processing task.
         * 
         * @return The number of ticks after which this function wants to be 
         * called again with the command to process.
         */
        static TickType_t _processQueueCmd(
            WalterModemCmd *cmd,
            bool queueError = false);

        /**
         * @brief Process an AT response from the queue.
         * 
         * This functioni is called in the queue processing task when an AT
         * response was received from the modem. The function will process the
         * response and notify blocked functions or call the correct callbacks.
         * This function will also release the response buffer back to the
         * parser's response buffer pool.
         * 
         * @param cmd The pending command or NULL when no command is pending.
         * @param rsp The AT response.
         * 
         * @return None.
         */
        static void _processQueueRsp(
            WalterModemCmd *cmd,
            WalterModemParserBuffer *rsp);
    
    public:
        /**
         * @brief Initialize the modem.
         * 
         * This function will initialize the modem. This is the first function
         * that needs to be called before using the modem device. This function
         * can only be called once, all consecutive calls will be no-ops. 
         * 
         * @param uart The hardware serial used to talk to the modem.
         * 
         * @return True on success, false on error.
         */
        static bool begin(HardwareSerial *uart);

        /**
         * @brief Set the AT response handler.
         * 
         * This function sets the handler that is called when an AT response
         * is received from the modem. When this function is called multiple
         * times only the last handler will be called. To remove the AT handler
         * this function must be called with a NULL pointer as handler.
         * 
         * @param handler The handler function or NULL.
         * @param args Optional handler arguments.
         * 
         * @return None.
         */
        static void setATHandler(
            void (*handler)(const uint8_t*, uint16_t, void*) = NULL,
            void *args = NULL);

        /**
         * @brief Set the GNSS fix handler.
         * 
         * This function sets the handler that is called when a GNSS fix was
         * obtained or when the receiver has given up. When this function is 
         * called multiple times only the last handler will be called. To
         * remove the GNSS fix handler this function must be called with a NULL
         * pointer as handler.
         * 
         * @param handler The handler function or NULL.
         * @param args Optional handler arguments.
         * 
         * @return None.
         */
        static void setGNSSfixHandler(
            void (*handler)(const WalterModemGNSSFix*, void*),
            void *args = NULL);

        /**
         * @brief Send an AT command.
         * 
         * This function will send an AT command. The necessary carriage return
         * and line feed will be added to the given command.
         * 
         * @param cmd The AT command to send.
         * 
         * @return True on success, false otherwise.
         */
        static bool sendCmd(const char *cmd);

        /**
         * @brief Physically reset the modem and wait for it to start. All 
         * connections will be lost when this function is called. The function
         * will fail when the modem doesn't start after the reset.
         * 
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * 
         * @return True on success, false otherwise.
         */
        static bool reset(
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL);

        /**
         * @brief Check communication between the ESP32 and the modem.
         * 
         * This function will send the 'AT' command and check if the modem
         * answers 'OK'.
         * 
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * 
         * @return True on success or false if the communication failed. 
         */
        static bool checkComm(
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL);

        /**
         * @brief Configure the CME error reports.
         * 
         * This function will set the CME error reports type. By default the 
         * library will set the error reports type to be enabled and numeric.
         * If this setting is changed, the library may not report errors
         * correctly.
         * 
         * @param type The CME error reports type.
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * 
         * @return True on success, false otherwise.
         */
        static bool configCMEErrorReports(
            WalterModemCMEErrorReportsType type =
                WALTER_MODEM_CME_ERROR_REPORTS_NUMERIC,
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL);

        /**
         * @brief Configure the CEREG status reports.
         * 
         * This function will set the CEREG status report type. By default the 
         * library will set the CEREG status reports to be enabled with minimal
         * operational info. If this setting is changed, the library may not 
         * work correctly.
         * 
         * @param type The CEREG status reports type.
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * 
         * @return True on success, false otherwise.
         */
        static bool configCEREGReports(
            WalterModemCEREGReportsType type = 
                WALTER_MODEM_CEREG_REPORTS_ENABLED,
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL);

        /**
         * @brief Get the network registration state.
         * 
         * This function returns the current network registration state. This
         * is buffered by the library and thus instantly available.
         * 
         * @return The current modem registration state. 
         */
        static WalterModemNetworkRegState getNetworkRegState();

        /**
         * @brief Get the operational state of the modem.
         * 
         * This function will request the operational state the modem is
         * currently in.
         * 
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         *  
         * @return True on success, false otherwise.
         */
        static bool getOpState(
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL);

        /**
         * @brief Set the operational state of the modem.
         * 
         * This function will set the operational state of the modem.
         * 
         * @param opState The new operational state of the modem.
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * 
         * @return True on success, false otherwise.
         */
        static bool setOpState(
            WalterModemOpState opState,
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL);

        /**
         * @brief Get the radio bands that the modem is configured to use.
         * 
         * This function will retrieve the bands which are used to connect to 
         * the mobile network.
         * 
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * 
         * @return True on success, false on error.
         */
        static bool getRadioBands(
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL);

        /**
         * @brief Get the SIM state.
         * 
         * This function will get the state of the SIM card.
         * 
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * 
         * @return True on success, false otherwise.
         */
        static bool getSIMState(
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL, 
            void *args = NULL);

        /**
         * @brief Set the SIM card's PIN code.
         * 
         * This function will set the PIN code of the SIM card. It is required
         * that the modem is in the FULL or NO_RF operational state.
         *
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * @param pin The PIN code of the SIM card or NULL for no pin.
         * 
         * @return True on success, false otherwise.
         */
        static bool unlockSIM(
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL,
            const char *pin = NULL);

        /**
         * @brief Set the network selection mode.
         * 
         * This function will set-up the network selection mode that Walter 
         * should use. This command is only available when the the modem is 
         * in the fully operational state.
         * 
         * @param mode The network selection mode.
         * @param operatorName The network operator name in case manual selection 
         * has been chosen.
         * @param format The format in which the network operator name is 
         * passed.
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * 
         * @return True on success, false otherwise.
         */
        static bool setNetworkSelectionMode(
            WalterModemNetworkSelMode mode = 
                WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC,
            const char *operatorName = NULL,
            WalterModemOperatorFormat format =
                WALTER_MODEM_OPERATOR_FORMAT_LONG_ALPHANUMERIC,
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL, 
            void *args = NULL);

        /**
         * @brief Create a new packet data protocol (PDP) context.
         * 
         * This function will create a new packet data protocol with the lowest
         * free context id.
         * 
         * @param apn The access point name.
         * @param authProto The used authentication protocol.
         * @param authUser Optional user to use for authentication.
         * @param authPass Optional password to use for authentication.
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * @param type The type of PDP context to create.
         * @param pdpAddress Optional PDP address.
         * @param headerComp The type of header compression to use.
         * @param dataComp The type of data compression to use.
         * @param ipv4AllocMethod The IPv4 alloction method.
         * @param requestType The type of PDP requests.
         * @param pcscfMethod The method to use for P-CSCF discovery.
         * @param forIMCN Set when this PDP ctx is used for IM CN signalling.
         * @param useNSLPI Set when NSLPI is used.
         * @param useSecurePCO Set to use secure protocol config options. 
         * @param useNASIPv4MTUDiscovery Set to use NAS for IPv4 MTU discovery.
         * @param useLocalAddrInd Set when local IPs are supported in the TFT.
         * @param useNASNonIPMTUDiscovery Set for NAS based no-IP MTU discovery.
         * 
         * @return True on success, false otherwise.
         */
        static bool createPDPContext(
            const char *apn = NULL,
            WalterModemPDPAuthProtocol authProto =
                WALTER_MODEM_PDP_AUTH_PROTO_NONE,
            const char *authUser = NULL,
            const char *authPass = NULL,
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL,
            WalterModemPDPType type = WALTER_MODEM_PDP_TYPE_IP,
            const char *pdpAddress = NULL, 
            WalterModemPDPHeaderCompression headerComp = 
                WALTER_MODEM_PDP_HCOMP_OFF, 
            WalterModemPDPDataCompression dataComp = 
                WALTER_MODEM_PDP_DCOMP_OFF,
            WalterModemPDPIPv4AddrAllocMethod ipv4AllocMethod =
                WALTER_MODEM_PDP_IPV4_ALLOC_DHCP,
            WalterModemPDPRequestType requestType =
                WALTER_MODEM_PDP_REQUEST_NEW_OR_HANDOVER,
            WalterModemPDPPCSCFDiscoveryMethod pcscfMethod =
                WALTER_MODEM_PDP_PCSCF_AUTO,
            bool forIMCN = false,
            bool useNSLPI = true,
            bool useSecurePCO = false,
            bool useNASIPv4MTUDiscovery = false,
            bool useLocalAddrInd = false,
            bool useNASNonIPMTUDiscovery = false);

        /**
         * @brief Authenticate a PDP context.
         * 
         * When a PDP context's APN requires authentication this function will
         * prepare the PDP context for this authentication. When this function
         * is executed for a PDP context with 'NONE' as the selected
         * authentication method this is a no-op.
         * 
         * @param pdpCtxId The PDP context id or -1 to re-use the last one.
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * 
         * @return True on success, false otherwise.
         */
        static bool authenticatePDPContext(
            int pdpCtxId = -1,
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL);

        /**
         * @brief Activate or deactivate a PDP context.
         * 
         * This function activates or deactivates a given PDP context. A PDP 
         * context must be activated before it can be attached to.
         * 
         * @param active True to activate the PDP context, false to deactivate.
         * @param pdpCtxId The PDP context id or -1 to re-use the last one.
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * 
         * @return True on success, false otherwise.
         */
        static bool setPDPContextActive(
            bool active = true,
            int pdpCtxId = -1,
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL);

        /**
         * @brief Attach to or detach the currently active PDP context.
         * 
         * This function will attach to or detach from a packet domain service.
         * 
         * @param attach True to attach, false to detach.
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * 
         * @return True on success, false otherwise.
         */
        static bool attachPDPContext(
            bool attach = true,
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL);

        /**
         * @brief Get a list of PDP addresses of a PDP context.
         * 
         * This function will retrieve the list of PDP addresses of the
         * requested PDP context id.
         * 
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * @param pdpCtxId The PDP context id or -1 to re-use the last one.
         * 
         * @return True on success, false otherwise.
         */
        static bool getPDPAddress(
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL,
            int pdpCtxId = -1);

        /**
         * @brief Create a new socket in a certain PDP context.
         * 
         * This function will create a new socket. After socket creation one 
         * can set additional socket settings and use the socket for
         * communication.
         * 
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * @param pdpCtxId The PDP context id or -1 to re-use the last one.
         * @param mtu The maximum transmission unit used by the socket.
         * @param exchangeTimeout The maximum number of seconds this socket can
         * be inactive.
         * @param connTimeout The maximum number of seconds this socket is
         * allowed to try to connect.
         * @param sendDelayMs The number of milliseconds send delay.
         * 
         * @return True on success, false otherwise.
         */
        static bool createSocket(
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL,
            int pdpCtxId = -1,
            uint16_t mtu = 300,
            uint16_t exchangeTimeout = 90,
            uint16_t connTimeout = 60,
            uint16_t sendDelayMs = 5000);

        /**
         * @brief Configure a newly created socket.
         * 
         * This step is required for the library to correctly configure the
         * modem to use this socket. 
         * 
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * @param socketId The id of the socket to connect or -1 to re-use the 
         * last one.
         * 
         * @return True on success, false otherwise.
         */
        static bool configSocket(
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL,
            int socketId = -1);

        /**
         * @brief Connect a socket after which data can be exchanged.
         * 
         * This function will connect a socket to a remote host. When the 
         * connection was successful data can be exchanged.
         * 
         * @param remoteHost The remote IPv4/IPv6 or hostname to connect to.
         * @param remotePort The remote port to connect on.
         * @param localPort The local port in case of an UDP socket.
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * @param protocol The protocol to use, UDP by default.
         * @param acceptAnyRemote How to accept remote UDP packets.
         * @param socketId The id of the socket to connect or -1 to re-use the 
         * last one.
         * 
         * @return True on success, false otherwise.
         */
        static bool connectSocket(
            const char *remoteHost,
            uint16_t remotePort,
            uint16_t localPort = 0,
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL,
            WalterModemSocketProto protocol = WALTER_MODEM_SOCKET_PROTO_UDP,
            WalterModemSocketAcceptAnyRemote acceptAnyRemote =
                WALTER_MODEM_ACCEPT_ANY_REMOTE_DISABLED,
            int socketId = -1);

        /**
         * @brief Close a socket.
         * 
         * This function closes a socket. Sockets can only be closed when they
         * are suspended, active socket connections cannot be closed.
         * 
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * @param socketId The id of the socket to close or -1 to re-use the
         * last one.
         * 
         * @return True on success, false otherwise.
         */
        static bool closeSocket(
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL,
            int socketId = -1);
        
        /**
         * @brief Send data over a socket.
         * 
         * This function will send data over a socket. The data buffer cannot
         * be freed until the send response is received (sync or async). The 
         * maximum size of the data buffer is 1500 bytes.
         * 
         * @param data The data to send.
         * @param dataSize The number of bytes to transmit.
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * @param rai The release assistance information.
         * @param socketId The id of the socket to close or -1 to re-use the
         * last one.
         * 
         * @return True on success, false otherwise.
         */
        static bool socketSend(
            const uint8_t *data,
            uint16_t dataSize,
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL,
            WalterModemRAI rai = WALTER_MODEM_RAI_NO_INFO,
            int socketId = -1);

        /**
         * @brief Send a string over a socket.
         * 
         * This function will send a string over a socket. The string cannot
         * be freed until the send response is received (sync or async). The 
         * maximum size of the string, not including the 0-terminator, is 1500
         * bytes.
         * 
         * @param str A zero-terminated string.
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * @param rai The release assistance information.
         * @param socketId The id of the socket to close or -1 to re-use the
         * last one.
         * 
         * @return True on success, false otherwise.
         */
        static bool socketSend(
            const char *str,
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL,
            WalterModemRAI rai = WALTER_MODEM_RAI_NO_INFO,
            int socketId = -1);

        /**
         * @brief Get the current modem time and date.
         * 
         * This function retrieves the current time and date from the modem.
         * 
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * 
         * @return True on success, false otherwise
         */
        static bool getClock(
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL);

        /**
         * @brief Configure Walter's GNSS receiver.
         * 
         * This function will configure the GNSS receiver. The settings are 
         * persistent over reboots but it could be that they need to be set
         * again after a modem firmware upgrade. Inbetween fixes this function
         * could be used to change the sensitivity mode. It is recommended to 
         * run this function at least once before GNSS is used.
         * 
         * @param sensMode The sensitivity mode.
         * @param acqMode The acquisition mode.
         * @param locMode The GNSS location mode.
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * 
         * @return True on success, false on error.
         */
        static bool configGNSS(
            WalterModemGNSSSensMode sensMode =
                WALTER_MODEM_GNSS_SENS_MODE_HIGH,
            WalterModemGNSSAcqMode acqMode =
                WALTER_MODEM_GNSS_ACQ_MODE_COLD_WARM_START,
            WalterModemGNSSLocMode locMode =
                WALTER_MODEM_GNSS_LOC_MODE_ON_DEVICE_LOCATION,
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL);

        /**
         * @brief Get the current GNSS assistance data status.
         * 
         * This function retrieves the status of the assistance data currently
         * loaded in the GNSS subsystem.
         * 
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * 
         * @return True on success, false on error.
         */
        static bool getGNSSAssistanceStatus(
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL);

        /**
         * @brief Update the GNSS assistance data.
         * 
         * This function will connect to the cloud to download the requested
         * type of assistance data and update the GNSS subsystem with this date.
         * The most efficient type of assistance data is real-time ephemeris.
         * 
         * @param type The type of GNSS assistance data to update.
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * 
         * @return True on success, false on error.
         */
        static bool updateGNSSAssistance(
            WalterModemGNSSAssistanceType type =
                WALTER_MODEM_GNSS_ASSISTANCE_TYPE_REALTIME_EPHEMERIS,
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL);

        /**
         * @brief Perform a GNSS action.
         * 
         * This function programs the GNSS subsystem to perform a certain
         * action.
         * 
         * @param action The action for the GNSS subsystem to perform.
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * 
         * @return True on success, false on error.
         */
        static bool performGNSSAction(
            WalterModemGNSSAction action =
                WALTER_MODEM_GNSS_ACTION_GET_SINGLE_FIX,
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL);
};

#endif