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
#ifdef ARDUINO
#include <Arduino.h>
#endif
#include <condition_variable>

#include <esp_partition.h>
#include <spi_flash_mmap.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>
#include <esp_vfs.h>
#include <esp_vfs_fat.h>
#include <esp_system.h>

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
 * @brief The number of buffers in the buffer pool.
 */
#define WALTER_MODEM_BUFFER_POOL_SIZE 8

/**
 * @brief The size of the stack of the command and response processing task.
 */
#define WALTER_MODEM_TASK_STACK_SIZE 4096

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
 * @brief The maximum number of COAP profiles that the library can support.
 */
#define WALTER_MODEM_MAX_COAP_PROFILES 3

/**
 * @brief The maximum number of HTTP profiles that the library can support.
 */
#define WALTER_MODEM_MAX_HTTP_PROFILES 3

/**
 * @brief The maximum number of TLS profiles that the library can support.
 */
#define WALTER_MODEM_MAX_TLS_PROFILES 6

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
 * @brief The maximum size of the message for the MQTT/COAP data to send to the
 * server.
 */
#define WALTER_MODEM_COAP_MAX_INCOMING_MESSAGE_LEN 1220
#define WALTER_MODEM_COAP_MAX_OUTGOING_MESSAGE_LEN 1024

/**
 * @brief Encrypted block size within flash
 */
#define ENCRYPTED_BLOCK_SIZE 16

/**
 * @brief SPI flash sectors per erase block
 */
#define SPI_SECTORS_PER_BLOCK   16      // usually large erase block is 32k/64k

/**
 * @brief SPI flash erase block size - sec size defined through esp_partition.h?
 */
#define SPI_FLASH_BLOCK_SIZE    (SPI_SECTORS_PER_BLOCK*SPI_FLASH_SEC_SIZE)

/**
 * @brief Get the character '0' or '1' at offset n in a byte.
 */
#define BITCHAR(x, n) (((x) >> (n)) & 1 ? '1' : '0')

/**
 * @brief Compound literal which represents a byte as a 0-terminated bitstring.
 */
#define BINBYTESTR(x) (char[9]){BITCHAR(x, 7), BITCHAR(x, 6), BITCHAR(x, 5), BITCHAR(x, 4), \
                                BITCHAR(x, 3), BITCHAR(x, 2), BITCHAR(x, 1), BITCHAR(x, 0), '\0'}

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
    WALTER_MODEM_STATE_NO_SUCH_PROFILE,
    WALTER_MODEM_STATE_NOT_EXPECTING_RING,
    WALTER_MODEM_STATE_AWAITING_RING,
    WALTER_MODEM_STATE_AWAITING_RESPONSE,
    WALTER_MODEM_STATE_BUSY,
    WALTER_MODEM_STATE_NO_DATA
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
    WALTER_MODEM_RAT_AUTO = 2,
    WALTER_MODEM_RAT_UNKNOWN = 99
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
    WALTER_MODEM_NETWORK_REG_REGISTERED_HOME = 1,
    WALTER_MODEM_NETWORK_REG_SEARCHING = 2,
    WALTER_MODEM_NETWORK_REG_DENIED = 3,
    WALTER_MODEM_NETWORK_REG_UNKNOWN = 4,
    WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING = 5,
    WALTER_MODEM_NETWORK_REG_REGISTERED_SMS_ONLY_HOME = 6,
    WALTER_MODEM_NETWORK_REG_REGISTERED_SMS_ONLY_ROAMING = 7,
    WALTER_MODEM_NETWORK_REG_ATTACHED_EMERGENCY_ONLY = 8,
    WALTER_MODEM_NETWORK_REG_REGISTERED_CSFB_NOT_PREFERRED_HOME = 9,
    WALTER_MODEM_NETWORK_REG_REGISTERED_CSFB_NOT_PREFERRED_ROAMING = 10,
    WALTER_MODEM_NETWORK_REG_REGISTERED_TEMP_CONN_LOSS = 80
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
 * @brief The SQNMONI cell information reporting scopes.
 */
typedef enum {
    WALTER_MODEM_SQNMONI_REPORTS_SERVING_CELL = 0,
    WALTER_MODEM_SQNMONI_REPORTS_INTRA_FREQUENCY_CELLS = 1,
    WALTER_MODEM_SQNMONI_REPORTS_INTER_FREQUENCY_CELLS = 2,
    WALTER_MODEM_SQNMONI_REPORTS_ALL_CELLS = 7,
    WALTER_MODEM_SQNMONI_REPORTS_SERVING_CELL_WITH_CINR = 9
} WalterModemSQNMONIReportsType;

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
    WALTER_MODEM_RSP_PARSER_DATA_PROMPT_HTTP,
    WALTER_MODEM_RSP_PARSER_DATA_HTTP_START1,
    WALTER_MODEM_RSP_PARSER_DATA_HTTP_START2,
    WALTER_MODEM_RSP_PARSER_END_LF,
    WALTER_MODEM_RSP_PARSER_RAW
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
    WALTER_MODEM_RSP_DATA_TYPE_RAT,
    WALTER_MODEM_RSP_DATA_TYPE_RSSI,
    WALTER_MODEM_RSP_DATA_TYPE_SIGNAL_QUALITY,
    WALTER_MODEM_RSP_DATA_TYPE_CELL_INFO,
    WALTER_MODEM_RSP_DATA_TYPE_SIM_STATE,
    WALTER_MODEM_RSP_DATA_TYPE_SIM_CARD_ID,
    WALTER_MODEM_RSP_DATA_TYPE_SIM_CARD_IMSI,
    WALTER_MODEM_RSP_DATA_TYPE_CME_ERROR,
    WALTER_MODEM_RSP_DATA_TYPE_PDP_CTX_ID,
    WALTER_MODEM_RSP_DATA_TYPE_BANDSET_CFG_SET,
    WALTER_MODEM_RSP_DATA_TYPE_PDP_ADDR,
    WALTER_MODEM_RSP_DATA_TYPE_SOCKET_ID,
    WALTER_MODEM_RSP_DATA_TYPE_GNSS_ASSISTANCE_DATA,
    WALTER_MODEM_RSP_DATA_TYPE_CLOCK,
    WALTER_MODEM_RSP_DATA_TYPE_IDENTITY,
    WALTER_MODEM_RSP_DATA_TYPE_BLUECHERRY,
    WALTER_MODEM_RSP_DATA_TYPE_HTTP_RESPONSE,
    WALTER_MODEM_RSP_DATA_TYPE_COAP,
    WALTER_MODEM_RSP_DATA_TYPE_MQTT
} WalterModemRspDataType;

/**
 * @brief The supported network selection modes.
 */
typedef enum {
    WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC = 0,
    WALTER_MODEM_NETWORK_SEL_MODE_MANUAL = 1,
    WALTER_MODEM_NETWORK_SEL_MODE_UNREGISTER = 2,
    WALTER_MODEM_NETWORK_SEL_MODE_MANUAL_AUTO_FALLBACK = 4
} WalterModemNetworkSelMode;

/**
 * @brief The supported network operator formats. 
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
 * @brief The supported PSM modes.
 */
typedef enum {
    WALTER_MODEM_PSM_DISABLE = 0,
    WALTER_MODEM_PSM_ENABLE = 1,
    WALTER_MODEM_PSM_RESET = 2
} WalterModemPSMMode;

/**
 * @brief The supported eDRX modes.
 */
typedef enum {
    WALTER_MODEM_EDRX_DISABLE = 0,
    WALTER_MODEM_EDRX_ENABLE = 1,
    WALTER_MODEM_EDRX_ENABLE_WITH_RESULT = 2,
    WALTER_MODEM_EDRX_RESET = 3
} WalterModemEDRXMode;

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
 * @brief The state of a http context.
 */
typedef enum {
    WALTER_MODEM_HTTP_CONTEXT_STATE_IDLE,
    WALTER_MODEM_HTTP_CONTEXT_STATE_EXPECT_RING,
    WALTER_MODEM_HTTP_CONTEXT_STATE_GOT_RING
} WalterModemHttpContextState;

/**
 * @brief The TLS version.
 */
typedef enum {
    WALTER_MODEM_TLS_VERSION_10 = 0,
    WALTER_MODEM_TLS_VERSION_11 = 1,
    WALTER_MODEM_TLS_VERSION_12 = 2,
    WALTER_MODEM_TLS_VERSION_13 = 3,
    WALTER_MODEM_TLS_VERSION_RESET = 255
} WalterModemTlsVersion;

/**
 * @brief The TLS validation policy.
 */
typedef enum {
    WALTER_MODEM_TLS_VALIDATION_NONE = 0,
    WALTER_MODEM_TLS_VALIDATION_CA = 1,
    WALTER_MODEM_TLS_VALIDATION_URL = 4,
    WALTER_MODEM_TLS_VALIDATION_URL_AND_CA = 5
} WalterModemTlsValidation;

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
    WALTER_MODEM_GNSS_ASSISTANCE_TYPE_PREDICTED_EPHEMERIS = 2,
} WalterModemGNSSAssistanceType;

/**
 * @brief The possible statuses of a BlueCherry communication cycle.
 */
typedef enum {
    WALTER_MODEM_BLUECHERRY_STATUS_NOT_PROVISIONED,
    WALTER_MODEM_BLUECHERRY_STATUS_IDLE,
    WALTER_MODEM_BLUECHERRY_STATUS_AWAITING_RESPONSE,
    WALTER_MODEM_BLUECHERRY_STATUS_RESPONSE_READY,
    WALTER_MODEM_BLUECHERRY_STATUS_PENDING_MESSAGES,
    WALTER_MODEM_BLUECHERRY_STATUS_TIMED_OUT
} WalterModemBlueCherryStatus;

/**
 * @brief The possible option codes for the COAP message.
 */
typedef enum {
    WALTER_MODEM_COAP_OPT_CODE_IF_MATCH = 1,
    WALTER_MODEM_COAP_OPT_CODE_URI_HOST = 3,
    WALTER_MODEM_COAP_OPT_CODE_ETAG = 4,
    WALTER_MODEM_COAP_OPT_CODE_IF_NONE_MATCH = 5,
    WALTER_MODEM_COAP_OPT_CODE_OBSERVE = 6,
    WALTER_MODEM_COAP_OPT_CODE_URI_PORT = 7,
    WALTER_MODEM_COAP_OPT_CODE_LOCATION_PATH = 8,
    WALTER_MODEM_COAP_OPT_CODE_URI_PATH = 11,
    WALTER_MODEM_COAP_OPT_CODE_CONTENT_TYPE = 12,
    WALTER_MODEM_COAP_OPT_CODE_MAX_AGE = 14,
    WALTER_MODEM_COAP_OPT_CODE_URI_QUERY = 15,
    WALTER_MODEM_COAP_OPT_CODE_ACCEPT = 17,
    WALTER_MODEM_COAP_OPT_CODE_TOKEN = 19,
    WALTER_MODEM_COAP_OPT_CODE_LOCATION_QUERY = 20,
    WALTER_MODEM_COAP_OPT_CODE_BLOCK2 = 23,
    WALTER_MODEM_COAP_OPT_CODE_SIZE2 = 28,
    WALTER_MODEM_COAP_OPT_CODE_PROXY_URI = 35,
    WALTER_MODEM_COAP_OPT_CODE_SIZE1 = 60
} WalterModemCoapOptCode;

/**
 * @brief The possible option values for the COAP message.
 */
typedef enum {
    WALTER_MODEM_COAP_OPT_VALUE_TEXT_PLAIN = 0,
    WALTER_MODEM_COAP_OPT_VALUE_TEXT_XML = 1,
    WALTER_MODEM_COAP_OPT_VALUE_TEXT_CSV = 2,    
    WALTER_MODEM_COAP_OPT_VALUE_TEXT_HTML = 3,
    WALTER_MODEM_COAP_OPT_VALUE_IMAGE_GIF = 21,
    WALTER_MODEM_COAP_OPT_VALUE_IMAGE_JPEG = 22,
    WALTER_MODEM_COAP_OPT_VALUE_IMAGE_PNG = 23,
    WALTER_MODEM_COAP_OPT_VALUE_IMAGE_TIFF = 24,
    WALTER_MODEM_COAP_OPT_VALUE_AUDIO_RAW = 25,
    WALTER_MODEM_COAP_OPT_VALUE_VIDEO_RAW = 26,
    WALTER_MODEM_COAP_OPT_VALUE_APPLICATION_LINK_FORMAT = 40,
    WALTER_MODEM_COAP_OPT_VALUE_APPLICATION_XML = 41,
    WALTER_MODEM_COAP_OPT_VALUE_APPLICATION_OCTET_STREAM = 42,
    WALTER_MODEM_COAP_OPT_VALUE_APPLICATION_RDF_XML = 43,
    WALTER_MODEM_COAP_OPT_VALUE_APPLICATION_SOAP_XML = 44,
    WALTER_MODEM_COAP_OPT_VALUE_APPLICATION_ATOM_XML = 45,
    WALTER_MODEM_COAP_OPT_VALUE_APPLICATION_XMPP_XML = 46,
    WALTER_MODEM_COAP_OPT_VALUE_APPLICATION_EXI = 47,
    WALTER_MODEM_COAP_OPT_VALUE_APPLICATION_FASTINFOSET = 48,
    WALTER_MODEM_COAP_OPT_VALUE_APPLICATION_SOAP_FASTINFOSET = 49,
    WALTER_MODEM_COAP_OPT_VALUE_APPLICATION_JSON = 50,
    WALTER_MODEM_COAP_OPT_VALUE_APPLICATION_X_OBIX_BINARY = 51,
    WALTER_MODEM_COAP_OPT_VALUE_APPLICATION_CBOR = 60
} WalterModemCoapOptValue;

/**
 * @brief The possible option values for the COAP message.
 */
typedef enum {
    /**
     * @brief Set of overwrite an option. 
     */
    WALTER_MODEM_COAP_OPT_SET = 0,

    /**
     * @brief Delete one or all options.
     */
    WALTER_MODEM_COAP_OPT_DELETE = 1,

    /**
     * @brief Read a single option.
     */
    WALTER_MODEM_COAP_OPT_READ = 2,

    /**
     * @brief Allows adding more values to repeatable options.
     */
    WALTER_MODEM_COAP_OPT_EXTEND = 3
} WalterModemCoapOptAction;

/**
 * @brief The possible CoAP send types.
 */
typedef enum {
    WALTER_MODEM_COAP_SEND_TYPE_CON = 0,
    WALTER_MODEM_COAP_SEND_TYPE_NON = 1,
    WALTER_MODEM_COAP_SEND_TYPE_ACK = 2,
    WALTER_MODEM_COAP_SEND_TYPE_RST = 3
} WalterModemCoapSendType;

/**
 * @brief The possible CoAP send methods.
 */
typedef enum {
    WALTER_MODEM_COAP_SEND_METHOD_NONE = 0,
    WALTER_MODEM_COAP_SEND_METHOD_GET = 1,
    WALTER_MODEM_COAP_SEND_METHOD_POST = 2,
    WALTER_MODEM_COAP_SEND_METHOD_PUT = 3,
    WALTER_MODEM_COAP_SEND_METHOD_DELETE = 4,
    WALTER_MODEM_COAP_SEND_RSP_CODE_CREATED = 201,
    WALTER_MODEM_COAP_SEND_RSP_CODE_DELETED = 202,
    WALTER_MODEM_COAP_SEND_RSP_CODE_VALID = 203,
    WALTER_MODEM_COAP_SEND_RSP_CODE_CHANGED = 204,
    WALTER_MODEM_COAP_SEND_RSP_CODE_CONTENT = 205,
    WALTER_MODEM_COAP_SEND_RSP_CODE_CONTINUE = 231,
    WALTER_MODEM_COAP_SEND_RSP_CODE_BAD_REQUEST = 400,
    WALTER_MODEM_COAP_SEND_RSP_CODE_UNAUTHORIZED = 401,
    WALTER_MODEM_COAP_SEND_RSP_CODE_BAD_OPTION = 402,
    WALTER_MODEM_COAP_SEND_RSP_CODE_FORBIDDEN = 403,
    WALTER_MODEM_COAP_SEND_RSP_CODE_NOT_FOUND = 404,
    WALTER_MODEM_COAP_SEND_RSP_CODE_METHOD_NOT_ALLOWED = 405,
    WALTER_MODEM_COAP_SEND_RSP_CODE_NOT_ACCEPTABLE = 406,
    WALTER_MODEM_COAP_SEND_RSP_CODE_PRECONDITION_FAILED = 412,
    WALTER_MODEM_COAP_SEND_RSP_CODE_REQUEST_ENTITY_TOO_LARGE = 413,
    WALTER_MODEM_COAP_SEND_RSP_CODE_UNSUPPORTED_MEDIA_TYPE = 415,
    WALTER_MODEM_COAP_SEND_RSP_CODE_INTERNAL_SERVER_ERROR = 500,
    WALTER_MODEM_COAP_SEND_RSP_CODE_NOT_IMPLEMENTED = 501,
    WALTER_MODEM_COAP_SEND_RSP_CODE_BAD_GATEWAY = 502,
    WALTER_MODEM_COAP_SEND_RSP_CODE_SERVICE_UNAVAILABLE = 503,
    WALTER_MODEM_COAP_SEND_RSP_CODE_GATEWAY_TIMEOUT = 504,
    WALTER_MODEM_COAP_SEND_RSP_CODE_PROXYING_NOT_SUPPORTED = 505
} WalterModemCoapSendMethodRsp;

/**
 * @brief The possible commands for a HTTP query operation.
 */
typedef enum {
    WALTER_MODEM_HTTP_QUERY_CMD_GET,
    WALTER_MODEM_HTTP_QUERY_CMD_HEAD,
    WALTER_MODEM_HTTP_QUERY_CMD_DELETE
} WalterModemHttpQueryCmd;

/**
 * @brief The possible commands for a HTTP send operation.
 */
typedef enum {
    WALTER_MODEM_HTTP_SEND_CMD_POST,
    WALTER_MODEM_HTTP_SEND_CMD_PUT
} WalterModemHttpSendCmd;

/**
 * @brief The possible post params for a HTTP send operation.
 */
typedef enum {
    WALTER_MODEM_HTTP_POST_PARAM_URL_ENCODED = 0,
    WALTER_MODEM_HTTP_POST_PARAM_TEXT_PLAIN = 1,
    WALTER_MODEM_HTTP_POST_PARAM_OCTET_STREAM = 2,
    WALTER_MODEM_HTTP_POST_PARAM_FORM_DATA = 3,
    WALTER_MODEM_HTTP_POST_PARAM_JSON = 4,
    WALTER_MODEM_HTTP_POST_PARAM_UNSPECIFIED = 99
} WalterModemHttpPostParam;

/**
 * @brief The possible types of BlueCherry events
 */
typedef enum {
    WALTER_MODEM_BLUECHERRY_EVENT_TYPE_OTA_INITIALIZE = 1, /* payload: 32 bit size */
    WALTER_MODEM_BLUECHERRY_EVENT_TYPE_OTA_CHUNK = 2, /* payload: chunk data */
    WALTER_MODEM_BLUECHERRY_EVENT_TYPE_OTA_FINISH = 3,
    WALTER_MODEM_BLUECHERRY_EVENT_TYPE_OTA_ERROR = 4,
    WALTER_MODEM_BLUECHERRY_EVENT_TYPE_MOTA_INITIALIZE = 5,
    WALTER_MODEM_BLUECHERRY_EVENT_TYPE_MOTA_CHUNK = 6,
    WALTER_MODEM_BLUECHERRY_EVENT_TYPE_MOTA_FINISH = 7,
    WALTER_MODEM_BLUECHERRY_EVENT_TYPE_MOTA_ERROR = 8
} WalterModemBlueCherryEventType;

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
} WalterModemGNSSAssistanceTypeDetails;

/**
 * @brief This structure contains GNSS assistance metadata.
 */
typedef struct {
    /**
     * @brief Almanac data details, this is not needed when real-time ephemeris
     * data is available.
     */
    WalterModemGNSSAssistanceTypeDetails almanac;

    /**
     * @brief Real-time ephemeris data details. Use this kind of assistance 
     * data for the fastest and most power efficient GNSS fix.
     */
    WalterModemGNSSAssistanceTypeDetails realtimeEphemeris;

    /**
     * @brief Predicted ephemeris data details.
     */
    WalterModemGNSSAssistanceTypeDetails predictedEphemeris;
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

typedef struct {
    /**
     * @brief A 0-terminated string representation of the SIM ICCID.
     */
    char iccid[23];
    
    /**
     * @brief A 0-terminated string representation of the SIM eUICCID.
     */
    char euiccid[23];
} WalterModemSIMCardID;

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
 * @brief This structure contains the IMEI, IMEISV and SVN identity of the
 * modem.
 */
typedef struct {
    /**
     * @brief A 0-terminated string representation of the IMEI number.
     */
    char imei[16];

    /**
     * @brief A 0-terminated string representation of the IMEISV number.
     */
    char imeisv[17];

    /**
     * @brief A 0-terminated string representation of the SVN number.
     */
    char svn[3];
} WalterModemIdentity;

/**
 * @brief This structure contains one of possibly multiple BlueCherry messages
 * delivered in a CoAP datagram. 
 */
typedef struct {
    /**
     * @brief The MQTT topic number.
     */
    int topic;

    /**
     * @brief The data size af the message.
     */
    uint8_t dataSize;

    /**
     * @brief The data of the message.
     */
    uint8_t *data;
} WalterModemBlueCherryMessage;

/**
 * @brief This structure represents the BlueCherry data with the individual messages. 
 */
typedef struct {
    /**
     * @brief The BlueCherry connection state
     */
    WalterModemBlueCherryStatus state;

    /**
     * @brief Flag to indicate if synchronisation is finished.
     */
    bool syncFinished;

    /**
     * @brief The amount of the messages.
     */
    int messageCount;

    /**
     * @brief The array containing the BlueCherry messages.
     */
    WalterModemBlueCherryMessage messages[16];
} WalterModemBlueCherryData;

/**
 * @brief This strucure represents the data in a walter coap message received.
 */
typedef struct {
    /**
     * @brief The id of the message. 
     */
    int messageId;

    /**
     * @brief The token of the message.
     */
    const char *token;

    /**
     * @brief The CoAP connection type. 
     */
    WalterModemCoapSendType type;

    /**
     * @brief This type contains the response code or method of the message.
     */
    WalterModemCoapSendMethodRsp methodRsp;

    /**
     * @brief The length of the payload data.
     * 
     */
    int length;

    /**
     * @brief The payload of the message.
     */
    uint8_t *payload;
} WalterModemCoapMessage;

/**
 * @brief This strucure represents a COAP response
 */
typedef struct {
    /**
     * @brief Profile id as received from the modem
     * (if one does not trust the modem, one might want to compare it
     * with the profile id for which data was requested)
     */
    uint8_t profileId;

    /**
     * @brief message id
     */
    uint16_t messageId;

    /**
     * @brief send type (con non ack rst)
     */
    WalterModemCoapSendType sendType;
    
    /**
     * @brief method or response code
     */
    WalterModemCoapSendMethodRsp methodRsp;

    /**
     * @brief length of the message
     */
    uint16_t length;
} WalterModemCoapResponse;

/**
 * @brief This strucure represents an incoming MQTT message
 */
typedef struct {
    /**
     * @brief Message id (0xffff means unknown, in case of qos 0)
     */
    uint16_t messageId;

    /**
     * @brief QOS
     */
    uint8_t qos;

    /**
     * @brief length of the message
     */
    uint16_t length;

    /**
     * topic not needed because mqttDidRing caller specifies desired topic
     */
} WalterModemMqttResponse;

/**
 * @brief This strucure represents a http response
 */
typedef struct {
    /*
     * @brief http response status code
     * including our own code to indicate errors during httpDidRing
     */
    uint8_t httpStatus;

    /**
     * @brief content length
     */
    uint16_t contentLength;
} WalterModemHttpResponse;

/**
 * @brief This structure groups the RSRQ and RSRP signal quality parameters.
 */
typedef struct {
    /**
     * @brief The RSRQ in 10ths of dB.
     */
    int rsrq;

    /**
     * @brief The RSRP in dBm.
     */
    int rsrp;
} WalterModemSignalQuality;

/**
 * @brief This structure groups all possible cell monitoring response values.
 */
typedef struct {
    /**
     * @brief Name of the network operator.
     */
    char netName[WALTER_MODEM_OPERATOR_MAX_SIZE];

    /**
     * @brief Mobile country code.
     */
    uint16_t cc;

    /**
     * @brief Mobile network operator code.
     */
    uint8_t nc;

    /**
     * @brief Reference Signal Received Power.
     */
    float rsrp;

    /**
     * @brief Carrier to Interference-plus-Noise Ratio.
     */
    float cinr;

    /**
     * @brief Reference Signal Received Quality.
     */
    float rsrq;

    /**
     * @brief Tracking Area Code.
     */
    uint16_t tac;

    /**
     * @brief Physical Cell ID.
     */
    uint16_t pci;

    /**
     * @brief E-UTRA Assigned Radio Channel.
     */
    uint16_t earfcn;

    /**
     * @brief Received signal strength in dBm.
     */
    float rssi;

    /**
     * @brief DRX cycle in number of radio frames (1 frame = 10 ms).
     */
    uint16_t paging;

    /**
     * @brief 28-bit E-UTRAN Cell Identity.
     */
    uint32_t cid;

    /**
     * @brief Band number.
     */
    uint8_t band;

    /**
     * @brief Downlink bandwidth in kHz.
     */
    uint16_t bw;

    /**
     * @brief Coverage Enhancement level.
     */
    uint8_t ceLevel;
} WalterModemCellInformation;

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
     * @brief The ICCID and/or eUICCID of the SIM card.
     */
    WalterModemSIMCardID simCardID;

    /**
     * @brief The 0-terminated string representation of the active IMSI.
     */
    char imsi[16];

    /**
     * @brief The CME error received from the modem.
     */
    WalterModemCMEError cmeError;

    /**
     * @brief The ID of a PDP context.
     */
    int pdpCtxId;

    /**
     * @brief The radio access technology.
     */
    WalterModemRAT rat;

    /**
     * @brief The RSSI of the signal in dBm.
     */
    int rssi;

    /**
     * @brief The current signal quality.
     */
    WalterModemSignalQuality signalQuality;

    /**
     * @brief Current cell information data
     */
    WalterModemCellInformation cellInformation;

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

    /**
     * @brief The modem identity.
     */
    WalterModemIdentity identity;

    /**
     * @brief The BlueCherry data 
     */
    WalterModemBlueCherryData blueCherry;

    /**
     * @brief HTTP response
     */
    WalterModemHttpResponse httpResponse;

    /**
     * @brief COAP response
     */
    WalterModemCoapResponse coapResponse;

    /**
     * @brief MQTT response
     */
    WalterModemMqttResponse mqttResponse;
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
 * @brief This structure represents a buffer from the pool.
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
} WalterModemBuffer;

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
     * WALTER_MODEM_CMD_TYPE_DATA_TX_WAIT command,
     * or as a target buffer for data coming from the library.
     */
    uint8_t *data;

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
     * @brief Optional temporary buffer (from the pool) used for non-static
     * string parameters
     */
    WalterModemBuffer* stringsBuffer = NULL;

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
    WalterModemBuffer *buf = NULL;

    /**
     * @brief In raw data chunk parser state, we remember nr expected bytes
     */
    uint16_t rawChunkSize = 0;
} WalterModemATParserData;

/**
 * @brief This structure represents an item in the task queue.
 */
typedef struct {
    /**
     * @brief Pointer to an AT response or NULL when this is an AT command.
     */
    WalterModemBuffer *rsp = NULL;

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
     * @brief The state of the socket (in its lifecycle)
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

typedef struct {
    /**
     * @brief Message id (initialized 0 so message id 0 is not permitted
     * in our COAP implementation)
     */
    uint16_t messageId;

    /**
     * @brief Send type (con / non / ack / rst)
     */
    WalterModemCoapSendType sendType;

    /**
     * @brief Method or response code
     */
    WalterModemCoapSendMethodRsp methodRsp;

    /**
     * @brief Message size
     */
    uint16_t length;
} WalterModemCoapRing;

/**
 * @brief This structure represents a coap context with
 * its current state info.
 */
typedef struct {
    /**
     * @brief Connection status: connected or disconnected
     */
    bool connected;

    /**
     * @brief Up to 8 COAP RING notifications
     */
    WalterModemCoapRing rings[8];
} WalterModemCoapContext;

/**
 * @brief This structure represents a http connection context with
 * its current state info.
 */
typedef struct {
    /**
     * @brief Connection status: connected or disconnected
     * (only relevant for TLS where you first need to call httpConnect
     * and poll until connected jumps to true)
     */
    bool connected;

    /**
     * @brief Context state: idle, expecting ring, got ring
     */
    WalterModemHttpContextState state;

    /**
     * @brief Last incoming ring: http status code
     */
    uint8_t httpStatus;

    /**
     * @brief Last incoming ring: length
     */
    uint16_t contentLength;

    /**
     * @brief Target buffer to hold content type header after ring URC
     */
    char *contentType;

    /**
     * @brief Target content type header buffer size
     */
    uint16_t contentTypeSize;
} WalterModemHttpContext;

typedef struct {
    /**
     * @brief Message id (initialized 0 so message id 0 is not permitted)
     */
    uint16_t messageId = 0;

    /**
     * @brief QOS
     */
    uint8_t qos;

    /**
     * @brief Topic
     */
    char topic[WALTER_MODEM_HOSTNAME_BUF_SIZE] = { 0 };

    /**
     * @brief Message size
     */
    uint16_t length;
} WalterModemMqttRing;

/**
 * @brief This structure represents the state of the BlueCherry association.
 */
typedef struct {
    /**
     * @brief The tls profile
     */
    uint8_t tlsProfileId;

    /**
     * @brief COAP server name
     */
    char serverName[WALTER_MODEM_HOSTNAME_BUF_SIZE] = "coap.bluecherry.io";

    /**
     * @brief COAP server port
     */
    uint16_t port = 5684;

    /**
     * @brief Timeout for ACK of outgoing BlueCherry COAP messages, in seconds
     */
    uint16_t ackTimeout = 60;

    /**
     * @brief COAP message being composed 
     */
    uint8_t messageOut[WALTER_MODEM_COAP_MAX_OUTGOING_MESSAGE_LEN];

    /**
     * @brief Length of the COAP message being composed so far
     * (containing a client id initially - FIXME)
     */
    uint16_t messageOutLen = 1;
    
    /**
     * @brief COAP message received
     */
    uint8_t messageIn[WALTER_MODEM_COAP_MAX_INCOMING_MESSAGE_LEN];

    /**
     * @brief Length of the COAP message received
     */
    uint16_t messageInLen = 0;

    /**
     * @brief COAP message id of the message being composed or sent
     */
    uint16_t curMessageId = 0x1;         /* start at 1; 0 is invalid value */

    /**
     * @brief Last acked message id so we know how much to catch up
     *
     */
    uint16_t lastAckedMessageId = 0x0;  /* 0 means nothing received yet */

    /**
     * @brief Flag that indicates whether more data is ready on bridge,
     * meaning an extra blueCherrySync call is needed
     */
    bool moreDataAvailable = false;

    /**
     * @brief Status indicator for last BlueCherry synchronization cycle
     */
    WalterModemBlueCherryStatus status = WALTER_MODEM_BLUECHERRY_STATUS_IDLE;

    /**
     * @brief Time when the last message was sent.
     */
    time_t lastTransmissionTime = 0;

    /**
     * @brief OTA update status
     */
    uint8_t *otaBuffer = NULL;
    uint32_t otaBufferPos = 0;
    uint8_t otaSkipBuffer[ENCRYPTED_BLOCK_SIZE];  /* first bytes are stashed aside till the end */
    bool emitErrorEvent = false;
    uint32_t otaSize = 0;       /* non-zero means transfer in progress */
    uint32_t otaProgress = 0;
    const esp_partition_t *otaPartition = NULL;
} WalterModemBlueCherryState;

#define WALTER_MODEM_STP_SIGNATURE_REQUEST 0x66617374
#define WALTER_MODEM_STP_SIGNATURE_RESPONSE 0x74736166

#define WALTER_MODEM_STP_OPERATION_RESET 0x00
#define WALTER_MODEM_STP_OPERATION_OPEN_SESSION 0x01
#define WALTER_MODEM_STP_OPERATION_TRANSFER_BLOCK_COMMAND 0x02
#define WALTER_MODEM_STP_OPERATION_TRANSFER_BLOCK 0x03

/**
 * @brief STP packet for modem firmware upload
 */
struct WalterModemStpRequest
{
    uint32_t signature;
    uint8_t operation;
    uint8_t sessionId;
    uint16_t payloadLength;
    uint32_t transactionId;
    uint16_t headerCrc16;
    uint16_t payloadCrc16;
};

struct WalterModemStpResponseSessionOpen
{
    uint8_t success;
    uint8_t version;
    uint16_t maxTransferSize;
};

struct WalterModemStpRequestTransferBlockCmd
{
    uint16_t blockSize;
};

struct WalterModemStpResponseTransferBlock
{
    uint16_t residue;
};

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
         * @brief We remember the configured watchdog timeout.
         */
        static inline uint8_t _watchdogTimeout = false;

        /**
         * @brief The hardware serial peripheral used to talk to the modem.
         */
#ifdef ARDUINO
        static inline HardwareSerial *_uart = NULL;
#else
        static inline uint8_t _uartNo = 1;
        static inline StackType_t _rxTaskStack[WALTER_MODEM_TASK_STACK_SIZE];
        static inline StaticTask_t _rxTaskBuf;
#endif

        /**
         * @brief The pool of buffers used by the parser, command strings
         * and responses
         */
        static inline WalterModemBuffer
            _bufferPool[WALTER_MODEM_BUFFER_POOL_SIZE] = {};

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
         * @brief The set with COAP contexts
         */
        static inline WalterModemCoapContext
            _coapContextSet[WALTER_MODEM_MAX_COAP_PROFILES] = {};

        /**
         * @brief The set with HTTP contexts (array index = profile id)
         */
        static inline WalterModemHttpContext
            _httpContextSet[WALTER_MODEM_MAX_HTTP_PROFILES] = {};

        /**
         * @brief HTTP profile for which we are currently awaiting data
         */
        static inline uint8_t _httpCurrentProfile = 0xff;

        /**
         * @brief MQTT incoming messages for subscribed topics backlog
         */
        static inline WalterModemMqttRing _mqttRings[8];

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
         * @brief The current type of Radio Access Technology in use.
         */
        static inline WalterModemRAT
            _ratType = WALTER_MODEM_RAT_UNKNOWN;

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

        /*
         * @brief BlueCherry state
         */
        static inline WalterModemBlueCherryState blueCherry;

        /*
         * @brief FAT partition mount handle (used for modem firmware upgrade)
         */
        static inline wl_handle_t _wl_handle = WL_INVALID_HANDLE;

        /*
         * @brief Current modem firmware file handle in the FAT partition
         */
        static inline FILE *_mota_file_ptr = NULL;

        /*
         * @brief Flag to interrupt the rx interrupt handler
         * during MOTA updates where we want to read the raw uart
         * data directly
         */
        static inline bool _rxHandlerInterrupted = false;

        /*
         * @brief Helper to boot modem to recovery modem and start upgrade
         */
        static uint16_t _modemFirmwareUpgradeStart(void);   // returns modem max blockSize
                                                            //
        /*
         * @brief Helper to boot modem into new firmware after upgrade
         */
        static void _modemFirmwareUpgradeFinish(bool success);

        /*
         * @brief Helper to transfer a chunk of the modem firmware to modem
         * during MOTA update
         */
        static void _modemFirmwareUpgradeBlock(size_t blockSize, uint32_t transactionId);

        /*
         * @brief Helper to abstract away the difference between arduino
         * and ESP-IDF for reading uart
         */
        static size_t _uartRead(uint8_t *buf, int readSize, bool tryHard = false);

        /*
         * @brief Helper to abstract away the difference between arduino
         * and ESP-IDF for writing to uart
         */
        static size_t _uartWrite(uint8_t *buf, int writeSize);

        /*
         * @brief Helper for the CRC used in the modem binary transfer
         * STP protocol
         */
        static uint16_t _calculateStpCrc16(const void *input, size_t length);

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
         * @brief Save the PDP context to RTC memory
         * 
         * This function will copy the active PDP context set to 
         * RTC memory, so it can be preserved during deep sleep.
         * 
         * @param _pdpCtxSetRTC The PDP context set saved in RTC memory.
         * 
         * @return None.
         */
        static void _saveRTCPdpContextSet(WalterModemPDPContext *_pdpCtxSetRTC = NULL);

        /**
         * @brief Load the PDP context from RTC memory
         * 
         * This function will fill in the WalterModem PDP context using the 
         * copy saved in RTC memory, after waking up from deep sleep.
         * 
         * @param _pdpCtxSetRTC The PDP context set saved in RTC memory.
         * 
         * @return None.
         */
        static void _loadRTCPdpContextSet(WalterModemPDPContext *_pdpCtxSetRTC = NULL);

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
         * @brief Test if the new buffer line starts a raw data chunk
         *
         * @return Size of the expected raw data chunk
         */
        static uint16_t _extractRawBufferChunkSize();

        /**
         * @brief Get a free buffer from the buffer pool.
         * 
         * @return None.
         */
        static WalterModemBuffer* _getFreeBuffer(void);

        /**
         * @brief Handle an AT data byte.
         * 
         * This function is used by the AT data parser to add a databyte to 
         * the buffer currently in use or to reserve a new buffer to add a byte
         * to.
         * 
         * @param data The data byte to handle.
         * @param raw Raw mode (do not scan for ending \r)
         * 
         * @return None.
         */
        static void _addATByteToBuffer(char data, bool raw);

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
         * @param params Incoming params for this freertos task handler
         * (in the ESP-IDF version)
         *
         * @return None.
         */
#ifdef ARDUINO
        static void _handleRxData(void);
#else
        static void _handleRxData(void *params);
#endif

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
         * @param stringsBuffer Optional buffer (from the pool) for remembering
         * non-static string parameters.
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
            uint8_t *data = NULL,
            uint16_t dataSize = 0,
            WalterModemBuffer* stringsBuffer = NULL,
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
         * buffer pool.
         * 
         * @param cmd The pending command or NULL when no command is pending.
         * @param rsp The AT response.
         * 
         * @return None.
         */
        static void _processQueueRsp(
            WalterModemCmd *cmd,
            WalterModemBuffer *rsp);
    
        /**
         * @brief Process incoming BlueCherry event
         *
         * This function is called when blueCherryDidRing encounters a
         * BlueCherry management packet, eg for OTA updates.
         *
         * @param data The event data.
         * @param len The length of the data block.
         *
         * @return Whether we should emit an error BC event on next sync.
         */
        static bool _processBlueCherryEvent(uint8_t *data, uint8_t len);

        /**
         * @brief Process OTA init event
         *
         * This function prepares a OTA update and checks the announced
         * update image size against the update partition size.
         *
         * @param data The event data, being the announced size of the image
         * @param len The length of the update data.
         *
         * @return Whether we should emit an error BC event on next sync,
         * in case announced size is too large for partition
         */
        static bool _processOtaInitializeEvent(uint8_t *data, uint16_t len);

        /**
         * @brief Process OTA chunk event
         *
         * This function accepts a chunk of the OTA update binary image.
         * If the chunk is empty, the BlueCherry cloud server signals a
         * cancel of the upload in progress.
         *
         * @param data The chunk data
         * @param len The length of the chunk data
         *
         * @return Whether we should emit an error BC event on next sync,
         * in case size so far exceeds announced size, or if it is an
         * empty chunk.
         */
        static bool _processOtaChunkEvent(uint8_t *data, uint16_t len);

        /**
         * @brief Write a flash sector to flash, erasing the block
         * first if on an as of yet uninitialized block
         *
         * @param None.
         *
         * @return True if succeeded, false if not.
         */
        static bool _otaBufferToFlash(void);

        /**
         * @brief Process OTA finish event
         *
         * This function verifies the exact announced size has been
         * flashed, could verify the optional included SHA256,
         *
         * @param None.
         *
         * @return Whether we should emit an error BC event on next sync,
         * in case the size mismatches the announced size, or the optional
         * included SHA256 digest mismatches the corresponding image.
         */
        static bool _processOtaFinishEvent(void);

        static bool _formatFat(void);
        static bool _processMotaInitializeEvent(uint8_t *data, uint16_t len);
        static bool _processMotaChunkEvent(uint8_t *data, uint16_t len);
        static bool _processMotaFinishEvent(void);

        /**
         * @brief Configure mqtt client in the modem
         *
         * This function configures the modem mqtt client,
         * without connecting.
         *
         * @param clientId MQTT client id to be used
         * @param userName Username for auth
         * @param password Password for auth
         * @param tlsProfileId TLS profile id to be used
         *
         * @return True if succeeded, false if not.
         */
        static bool _mqttConfig(const char *clientId,
            const char *userName,
            const char *password,
            uint8_t tlsProfileId);

        /**
         * @brief Upload key or certificate to modem NVRAM.
         *
         * This function uploads a key or certificate to the modem NVRAM.
         * It is recommended to save credentials in index 10-19 to avoid
         * overwriting preinstalled certificates and (if applicable)
         * BlueCherry cloud platform credentials.
         *
         * @param isPrivateKey true if it is a private key,
         * false if it is a certificate
         * @param slotIdx slot index within the modem NVRAM keystore
         * @param credential NULL-terminated string containing the PEM key/cert data
         *
         * @return True if succeeded, false if not.
         */
        static bool tlsWriteCredential(bool isPrivateKey, 
            uint8_t slotIdx, 
            const char *credential);

        /**
         * @brief Check if a key or certificate is present in modem NVRAM.
         *
         * This function checks if a key or certificate is present on a specific
         * slot index inside the modem NVRAM.
         *
         * @param isPrivateKey true to check for a private key,
         * false to check for a certificate
         * @param slotIdx slot index within the modem NVRAM keystore
         *
         * @return True if present, false if not.
         */
        static bool _tlsIsCredentialPresent(bool isPrivateKey, uint8_t slotIdx);

        /**
         * @brief Calculate the Luhn checksum for a 14-digit imei.
         * 
         * This function will return the Luhn checksum for a 14-digit IMEI
         * number and return it as an ASCII character.
         * 
         * @param imei The 14-digit IMEI number
         * 
         * @return The Luhn checksum as an ASCII character.
         */
        static char _getLuhnChecksum(const char *imei);

        /**
         * @brief Save context data in RTC memory before ESP deep sleep.
         *
         * This function will save the necessary state and context sets
         * in RTC memory to keep this information saved during ESP
         * deep sleep. 
         *
         * @return None.
         */
        static void _sleepPrepare();

        /**
         * @brief Load context data from RTC memory after ESP deep sleep.
         *
         * This function will load the saved state and context sets
         * from RTC memory to restore the contexts after ESP deep sleep. 
         *
         * @return None.
         */
        static void _sleepWakeup();
        
        /**
         * @brief Converts a given duration to encoded uint8_t according to the base_times.
         *
         * This function will encode a the given duration according to the base_times / mulipliers for use in PSM.
         *
         * @warning This is an approximation based on the base_times array.
         *
         * @param base_times Pointer to an array containing the base times
         * @param base_times_len Length of the base_times array
         * @param duration_seconds The requested duration in seconds.
         * @param actual_duration_seconds Optional pointer in which the actual requested duration can be saved.
         *
         * @return The duration encoded into the 3GPP standard format.
         */
        static const uint8_t _convertDuration(const uint32_t *base_times, size_t base_times_len, uint32_t duration_seconds, uint32_t *actual_duration_seconds);

    public :
        /**
         * @brief Initialize the modem.
         * 
         * This function will initialize the modem. This is the first function
         * that needs to be called before using the modem device. This function
         * can only be called once, all consecutive calls will be no-ops. 
         * 
         * @param uart The hardware serial used to talk to the modem
         * (HardwareSerial pointer on arduino, uart number on esp-idf)
         * @param watchdogTimeout Timeout in seconds before auto-reboot.
         * If set to nonzero, you must call tickleWatchdog before the
         * timeout expires. This helps you guard against programming errors,
         * although it is still possible a part of your code never gets
         * executed while always tickling the watchdog in time.
         * It also guards against bugs in the walter modem library that
         * would cause it to block for too long.
         * Use a value larger than 30 seconds; say 40 seconds at least.
         * Note that a wdt may be set in the compile options, triggered
         * by a starting idle task. This is usually sufficient for
         * simple programs.
         * 
         * @return True on success, false on error.
         */
#ifdef ARDUINO
        static bool begin(HardwareSerial *uart, uint8_t watchdogTimeout = 0);
#else
        static bool begin(uint8_t uartNo, uint8_t watchdogTimeout = 0);
#endif

        /**
         * @brief Tickle watchdog
         *
         * This function will reset the watchdog timer. It must be called
         * regularly and before the configured timeout expires.
         *
         * @return None.
         */
        static void tickleWatchdog(void);

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
         * @brief Software reset the modem and wait for it to reset. 
         * (required when switching RAT) 
         * The function will fail when the modem doesn't reset.
         *
         * @param rsp Pointer to a modem response structure to save the result
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         *
         * @return True on success, false otherwise.
         */
        static bool softReset(
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL);
        
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
         * @brief Put Walter to deep or light sleep.
         * 
         * This function will put Walter into deep sleep or light sleep for 
         * a given duration. The typical power consumption in light sleep is 1mA
         * and in deep sleep it is 9.5uA. 
         * 
         * This function will have an immediate effect on the ESP32-S3 but the 
         * modem can be delayed or prevented to go to deep sleep. 
         * 
         * Deep sleep causes the ESP32 to restart program execution, the modem
         * libraries therefore saves state (such as PDP context and socket state
         * in RTC memory). This also means that any initialisation must be 
         * repeated after waking up from deep sleep. Deep sleep is typically
         * combined with PSM and/or eDRX.
         * 
         * @param sleepTime The duration of deep sleep in seconds.
         * @param lightSleep When set to true Walter will only go to light
         * sleep.
         * 
         * @return None.
         */
        static void sleep(uint32_t sleepTime = 0, bool lightSleep = false);

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
         * @brief Get the current signal quality.
         * 
         * This function returns the current signal quality in dBm. The signal
         * quality is in the range [-113dBm, -51dBm]. 
         * 
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * 
         * @return True on success, false otherwise.
         */
        static bool getRSSI(
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL);

        /**
         * @brief Get extended RSRQ and RSRP signal quality.
         * 
         * This function returns the RSRQ and RSRP signal quality indicators.
         * 
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * 
         * @return True on success, false otherwise.
         */
        static bool getSignalQuality(
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL);

        /**
         * @brief Get information on the serving and neighbouring cells.
         * 
         * This function returns information about the serving and
         * neighbouring cells such as operator, cell ID, RSSI, RSRP...
         * 
         * @param type The type of cell information to retreive, defaults to
         * the cell which is currently serving the connection.
         * @param rsp Pointer to a modem response structure to save the result
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * 
         * @return True on success, false otherwise.
         */
        static bool getCellInformation(
            WalterModemSQNMONIReportsType type =
                WALTER_MODEM_SQNMONI_REPORTS_SERVING_CELL,
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL);

        /**
         * @brief Get the identity of the modem (IMEI, IMEISV, SVN).
         * 
         * This function retrieves the IMEI, IMEISV and SVN from the modem.
         * 
         * @param rsp Pointer to a modem response structure to save the result
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * 
         * @return True on success, false otherwise.
         */
        static bool getIdentity(
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL);     

        /**
         * @brief Disconnect mqtt connection.
         *
         * This function disconnects the mqtt client connection
         * to the broker.
         *
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * 
         * @return True on success, false otherwise.
         */
        static bool mqttDisconnect(
            WalterModemRsp *rsp,
            walterModemCb cb,
            void *args);

        /**
         * @brief Initialize MQTT and establish connection in one call.
         *
         * This function initializes the mqtt client on the modem
         * and establishes a connection.
         *
         * @param serverName MQTT broker hostname
         * @param port Port to connect to
         * @param clientId Client id string to be used
         * @param userName Username
         * @param password Password
         * @param tlsProfileId TLS profile id to be used (default 0=plaintext)
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * 
         * @return True on success, false otherwise.
         */
        static bool mqttConnect(const char *serverName,
            uint16_t port,
            const char *clientId = "walter-mqtt-client",
            const char *userName = "",
            const char *password = "",
            uint8_t tlsProfileId = 0,
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL);

        /**
         * @brief Publish something through mqtt.
         *
         * This function publishes the passed data on the given mqtt topic
         * using the connection established earlier through mqttConnect.
         *
         * @param topicString topic to publish on
         * @param data Data to be published
         * @param dataSize Size of the data block
         * @param qos QOS 0=at most once 1=at least once 2=exactly once received
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * 
         * @return True on success, false otherwise.
         */
        static bool mqttPublish(const char *topicString,
            uint8_t *data,
            uint16_t dataSize,
            uint8_t qos = 1,
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL);

        /**
         * @brief Subscribe to a mqtt topic
         *
         * This function subscribes to a given topic using the
         * connection established earlier through mqttConnect.
         *
         * @param topicString topic to publish on
         * @param qos QOS 0=at most once 1=at least once 2=exactly once received
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * 
         * @return True on success, false otherwise.
         */
        static bool mqttSubscribe(const char *topicString,
            uint8_t qos = 1,
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL);

        /**
         * @brief Poll if there were incoming mqtt messages.
         *
         * Poll if the modem has reported any incoming mqtt messages on the
         * topics we are subscribed on
         *
         * @param topic Topic to poll
         * @param targetBuf Target buffer to write incoming mqtt data in
         * @param targetBufSize Size of the target buffer
         *
         * @return True on success, false otherwise.
         */
        static bool mqttDidRing(
            const char *topic,
            uint8_t *targetBuf,
            uint16_t targetBufSize,
            WalterModemRsp *rsp = NULL);

        /**
         * @brief Configure TLS profile.
         *
         * This function should be called once in an initializer
         * sketch that prepares the modem for its intended use on this
         * Walter. Configure a set of TLS profiles within the modem,
         * with optional client auth certificates, validation level
         * (none/url/ca/url and ca) and TLS version.
         * Later http/mqtt/coap/bluecherry/socket sessions can then
         * use these preconfigured profile ids.
         *
         * @param profileId Security profile id (1-6)
         * @param tlsValid TLS validation level: nothing, URL, CA+period or all
         * @param tlsVersion TLS version
         * @param caCertificateId CA certificate for certificate validation,
         * 0-19 or 0xff to specify none
         * @param clientCertificateId Client TLS certificate index,
         * 0-19 or 0xff to specify none
         * @param clientPrivKeyId Client TLS private key index,
         * 0-19 or 0xff to specify none
         * @param rsp Pointer to a modem response structure to save the result
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         *
         * @return True on success, false otherwise.
         */
        static bool tlsConfigProfile(
            uint8_t profileId,
            WalterModemTlsValidation tlsValid = WALTER_MODEM_TLS_VALIDATION_NONE,
            WalterModemTlsVersion tlsVersion = WALTER_MODEM_TLS_VERSION_12,
            uint8_t caCertificateId = 0xff,
            uint8_t clientCertificateId = 0xff,
            uint8_t clientPrivKeyId = 0xff,
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL);

        /**
         * @brief Configure a HTTP profile.
         * 
         * This function will configure a HTTP profile with parameters
         * such as server name and auth info. The profile info is stored
         * persistently in the modem, so it is possible to store connection
         * info once, using an Arduino sketch to prepare all settings,
         * and later rely on this preconfigured profile in the modem
         * without the need to set the parameters again in the actual
         * Arduino sketch used in production.
         *
         * TLS not supported for now. Neither are file uploads/downloads.
         * 
         * @param profileId HTTP profile id (0, 1 or 2)
         * @param serverName The server name to connect to.
         * @param port The port of the server to connect to.
         * @param tlsProfileId If not 0, TLS is used with the given profile (1-6).
         * @param useBasicAuth Set true to use basic auth and send username/pw.
         * @param authUser Username.
         * @param authPass Password.
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * 
         * @return True on success, false otherwise.
         */
        static bool httpConfigProfile(
            uint8_t profileId,
            const char *serverName,
            uint16_t port = 80,
            uint8_t tlsProfileId = 0,
            bool useBasicAuth = false,
            const char *authUser = "",
            const char *authPass = "",
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL);

        /**
         * @brief Make http connection using a predefined profile
         * configured using httpConfigProfile. Note that this
         * modem command is buggy (see comment in httpGetContextStatus
         * implementation). It will also return OK while establishing
         * the connection in the background, so you need to poll with
         * httpGetContextStatus to discover when the connection is ready
         * to be used.
         *
         * @param profileId HTTP profile id (0, 1 or 2)
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         *
         * @return True on success, false otherwise.
         */
        static bool httpConnect(
                uint8_t profileId,
                WalterModemRsp *rsp = NULL,
                walterModemCb cb = NULL,
                void *args = NULL);

        /**
         * @brief Close http connection for the given http context.
         * Avoid connect and disconnect if possible (see comments
         * in implementation)
         *
         * @param profileId HTTP profile id (0, 1 or 2)
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         *
         * @return True on success, false otherwise.
         */
        static bool httpClose(
                uint8_t profileId,
                WalterModemRsp *rsp = NULL,
                walterModemCb cb = NULL,
                void *args = NULL);

        /**
         * @brief Get connection status of a http context.
         * Avoid connect and disconnect if possible (see comments
         * in implementation)
         *
         * @param profileId The profile id (0, 1 or 2) of the context
         *
         * @return True if context for given http context is connected,
         * false if not.
         */
        static bool httpGetContextStatus(uint8_t profileId);

        /**
         * @brief Perform a http get, delete or head request.
         * No need to first open the connection with the buggy httpConnect
         * command unless you need TLS + a private key.
         *
         * @param profileId The profile id (0, 1 or 2) of the http context
         * @param uri The URI
         * @param httpQueryCmd get, delete or head
         * @param contentTypeBuf Optional user buffer to store content type
         * header in.
         * @param contentTypeBufSize Size of the user buffer, including
         * terminating null byte.
         * @param rsp Response object
         * @param cb Callback
         * @param args Callback args
         *
         * @return True on success, false otherwise.
         */
        static bool httpQuery(
                uint8_t profileId,
                const char *uri,
                WalterModemHttpQueryCmd httpQueryCmd = WALTER_MODEM_HTTP_QUERY_CMD_GET,
                char *contentTypeBuf = NULL,
                uint16_t contentTypeBufSize = 0,
                WalterModemRsp *rsp = NULL,
                walterModemCb cb = NULL,
                void *args = NULL);

        /**
         * @brief Perform a http post or put request.
         * No need to first open the connection with the buggy httpConnect
         * command unless you need TLS + a private key.
         *
         * @param profileId The profile id (0, 1 or 2) of the http context
         * @param uri The URI
         * @param data Data to be sent to the server
         * @param dataSize Length of the data buffer to be sent to the server
         * @param httpSendCmd post or put
         * @param httpPostParam content type (enum value)
         * @param contentTypeBuf Optional user buffer to store content type
         * header in.
         * @param contentTypeBufSize Size of the user buffer, including
         * terminating null byte.
         * @param rsp Response object
         * @param cb Callback
         * @param args Callback args
         *
         * @return True on success, false otherwise.
         */
        static bool httpSend(
                uint8_t profileId,
                const char *uri,
                uint8_t *data,
                uint16_t dataSize,
                WalterModemHttpSendCmd httpSendCmd = WALTER_MODEM_HTTP_SEND_CMD_POST,
                WalterModemHttpPostParam httpPostParam = WALTER_MODEM_HTTP_POST_PARAM_UNSPECIFIED,
                char *contentTypeBuf = NULL,
                uint16_t contentTypeBufSize = 0,
                WalterModemRsp *rsp = NULL,
                walterModemCb cb = NULL,
                void *args = NULL);

        /**
         * @brief Fetch http response to earlier http request, if any
         *
         * @param profileId Profile for which to get response
         * @param targetBuf User buffer to store response in.
         * @param targetBufSize Size of the user buffer, including space for a
         * terminating null byte.
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         *
         * @return True on success, false if no data arrived or error
         * or no data expected (eg no ring received).
         */
        static bool httpDidRing(
                uint8_t profileId,
                uint8_t *targetBuf,
                uint16_t targetBufSize,
                WalterModemRsp *rsp = NULL);

        /**
         * @brief Upload BlueCherry credentials to the modem.
         *
         * Upload the Walter certificate and private key and the BlueCherry
         * cloud server CA certificate to the modem.
         *
         * The key parameters are NULL terminated strings containing the
         * PEM data with each line terminated by CRLF.
         *
         * @param walterCertificate Walter X.509 certificate as PEM string
         * @param walterPrivateKey Walter private key as PEM string
         * @param caCertificate BlueCherry CA certificate
         * @param rsp Pointer to a modem response structure to save the result
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         *
         * @return True on success, false otherwise.
         */
        static bool blueCherryProvision(
            const char *walterCertificate,
            const char *walterPrivateKey,
            const char *caCertificate,
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL);

        /**
         * @brief Check if device is provisioned for BlueCherry.
         *
         * This function checks if the necessary certificates and private
         * key are present in the modem NVRAM to be used for BlueCherry.
         * note: it does not check if the credentials are valid, but only
         * checks if the BlueCherry reserved slot indexes are occupied
         * inside the modem NVRAM.
         *
         * @return True on success, false otherwise.
         */
        static bool _blueCherryIsProvisioned();

        /**
         * @brief Initialize BlueCherry COAP bridge.
         * 
         * This function will set the TLS profile id and initialize
         * the accumulated outgoing datagram, initialize the current
         * message id to 1, the last acknowledged id to 0 and set the
         * state machine to IDLE.
         * 
         * @param tlsProfileId DTLS is used with the given profile (1-6).
         * @param otaBuffer A user-supplied buffer for OTA updates to flash;
         * must be 4K = the flash sector size.
         * @param rsp The (optional) response object.
         * @param ackTimeout Timeout for ACK of outgoing BlueCherry COAP 
         * messages, in seconds.
         * 
         * @return True if succeeded, false on error.
         */
        static bool blueCherryInit(
                uint8_t tlsProfileId,
                uint8_t *otaBuffer = NULL,
                WalterModemRsp *rsp = NULL,
                uint16_t ackTimeout = 60);

        /**
         * @brief Enqueue a MQTT publish message.
         * 
         * This function will add the message to the accumulated outgoing
         * datagram, which will -after blueCherrySynchronize- be sent to the
         * BlueCherry cloud server and published through MQTT.
         * 
         * @param topic The topic of the message.
         * @param len The length of the data.
         * @param data The data to send.
         * 
         * @return True on success, false on error.
         */
        static bool blueCherryPublish(uint8_t topic, uint8_t len, uint8_t *data);

        /**
         * @brief Send accumulated MQTT messages and poll for incoming data.
         * 
         * This function will send all accumulated MQTT publish messages to
         * the BlueCherry cloud server, and ask the server for an acknowledgement
         * and for the new incoming MQTT messages since the last
         * BlueCherrySynchronize call.
         *
         * Even if nothing was enqueued for publish, this call must frequently
         * be executed if Walter is subscribed to one or more MQTT topics or
         * has enabled BlueCherry OTA updates.
         * 
         * A response might not fit in a single datagram response. As long as
         * rsp.data.BlueCherry.syncFinished is false, this function needs to
         * be called again repeatedly.
         *
         * @return True if communication is successfull and a response was
         * received, false if error occurred.
         */
        static bool blueCherrySync(WalterModemRsp *rsp);

        /**
         * @brief Poll the API for a received BlueCherry response.
         *
         * The call is named DidRing for consistancy with the HTTP, COAP
         * and other polling network calls. It can only be used after a
         * blueCherrySynchronize call, and will return the response from the
         * server (an ACK for any published messages, and a list of
         * incoming messages for MQTT topics Walter is subscribed to, if any).
         *
         * Only after the (possibly empty) response has been received, or a
         * timeout has been reported, it will be possible to publish more data or
         * do a new blueCherrySynchronize call.
         *
         * Timeout is reported using the nak flag in the response object.
         *
         * If blueCherrySynchronize has been called, one may consider this form if
         * there is no other work to be done in the mean time:
         *
         * bool moreDataAvailable;
         * while(!modem.blueCherryDidRing(&moreDataAvailable, &rsp)) {
         *   delay(100);
         * }
         *
         * @param moreDataAvailable This flag will be set true through the
         * pointer if more incoming MQTT data is available for Walter; it did
         * not fit in the single-datagram response, and will be sent with the
         * next blueCherrySynchronize call. False if no more data for now.
         * @param rsp The (optional) response object.
         *
         * @return True if the response/ack was received and is now available
         * in the rsp object, and also if there was a timeout (nak flag will
         * be true in that case). False if we were not expecting data because
         * blueCherrySynchronize has not been called, or if we are still waiting for
         * the data.
         *
         * In practice: true means you can now publish
         * (optionally) new messages or call blueCherrySynchronize, and false means
         * we are still waiting for data and need to call blueCherryDidRing again later
         * before we can perform a new blueCherrySynchronize cycle.
         */
        static bool blueCherryDidRing(bool *moreDataAvailable, WalterModemRsp *rsp = NULL);

        /**
         * @brief Close the BlueCherry platform CoAP connection.
         * 
         * This function will close the CoAP connection to the Bluecherry 
         * cloud platform. Usually there is no need to call this function, 
         * unless using deep sleep mode (which might cause a modem bug in 
         * the latest modem firmware versions).
         * 
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * 
         * @return True if succeeded, false on error.
         */
        static bool blueCherryClose(
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL);

        /**
         * @brief Create a COAP context.
         * 
         * This function will create a COAP context if it was not open yet.
         * This needs to be done before you can set headers or options or
         * send or receive data.
         * 
         * @param profileId COAP profile id (0 is used by BlueCherry
         * and should not be used for regular COAP)
         * @param serverName The server name to connect to.
         * @param port The port of the server to connect to.
         * @param tlsProfileId If not 0, DTLS is used with the given profile (1-6).
         * @param localPort The local port to use (default 0=random).
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * 
         * @return True on success, false otherwise.
         */
        static bool coapCreateContext(
            uint8_t profileId,
            const char *serverName,
            int port,    
            uint8_t tlsProfileId = 0,
            int localPort = -1,
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL);

        /**
         * @brief Close a COAP context.
         * 
         * This function will close a COAP context previously opened with
         * coapCreateContext. To change parameters such as the server name,
         * you must first close the context using this call. Eventually
         * the context will be automatically closed after the timeout.
         * 
         * @param profileId COAP profile id (0 is not permitted since it is
         * used by BlueCherry)
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * 
         * @return True on success, false otherwise.
         */
        static bool coapClose(
            uint8_t profileId,
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL);

        /**
         * @brief Get connection status of a COAP context.
         *
         * @param profileId The profile id (0, 1 or 2) of the context
         *
         * @return True if context with the profile id is connected,
         * false if not.
         */
        static bool coapGetContextStatus(uint8_t profileId);

        /**
         * @brief Set COAP header.
         * 
         * This function will set the header of the next message to send.
         * This is not necessary, when you do not set the header the message id 
         * and the token will be set to random values.
         *  
         * @param profileId COAP profile id (1 or 2 - 0 only intended for internal
         * use by BlueCherry)
         * @param messageId The message id of the next message to send.
         * @param token The token of the next message to send as a string of
         * 16 hex digits for a max token length of 8 bytes, with default
         * value "NO_TOKEN" which is the magic value to send a datagram
         * without token.
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * 
         * @return True on success, false otherwise.
         */
        static bool coapSetHeader(
            uint8_t profileId,
            int messageId = 1,
            const char *token = "NO_TOKEN",
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL);

        /**
         * @brief Set the options for the next COAP message.
         * 
         * @param profileId COAP profile id (1 or 2)
         * @param action The action code of the option.
         * @param code The code of the options.
         * @param values The optional values array, expected as a comma
         * delimited string of up to 6 strings or recognized option values
         * (see WalterModemCoapOptValue)
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         *  
         * @return True on success, false otherwise.
         */
        static bool coapSetOptions(
            uint8_t profileId,
             WalterModemCoapOptAction action,
             WalterModemCoapOptCode code,
             const char *const values = NULL,
             WalterModemRsp *rsp = NULL,
             walterModemCb cb = NULL,
             void *args = NULL);

        /**
         * @brief Send a datagram (with header set and options set before)
         * 
         * This function will send a COAP message.
         * 
         * @param profileId COAP profile id (1 or 2; 0 should not be used and
         * is used by BlueCherry)
         * @param type The type of message (NON, CON, ACK, RST) which implies
         * whether it is a request or response (reqtype).
         * @param methodRsp The method or response code.
         * @param length The length of the payload.
         * @param payload The payload to send max 1024 bytes.
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         *  
         * @return True on success, false otherwise.
         */
        static bool coapSendData(
            uint8_t profileId,
            WalterModemCoapSendType type,
            WalterModemCoapSendMethodRsp methodRsp,
            int length,
            uint8_t *payload,
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL);

        /**
         * @brief Fetch incoming COAP messages, if any
         *
         * @param profileId Profile for which to get incoming data
         * (0 not allowed because it is used internally for BlueCherry)
         * @param targetBuf User buffer to store response in.
         * @param targetBufSize Size of the user buffer, including space for a
         * terminating null byte.
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         *
         * @return True on success, false if no data arrived or error
         * or no data expected (eg no ring received).
         */
        static bool coapDidRing(
                uint8_t profileId,
                uint8_t *targetBuf,
                uint16_t targetBufSize,
                WalterModemRsp *rsp = NULL);

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
         * @brief Get the selected RAT (Radio Access Technology).
         * 
         * This function will request the Radio Access Technology which the
         * modem should apply.
         * 
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         *  
         * @return True on success, false otherwise.
         */
        static bool getRAT(
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL);

        /**
         * @brief Set the RAT (Radio Access Technology).
         * 
         * This function will set the Radio Access Technology which the modem
         * should apply.
         * 
         * @param rat The new RAT.
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         *  
         * @return True on success, false otherwise.
         */
        static bool setRAT(
            WalterModemRAT rat,
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
         * @brief Set the radio bands the modem will use.
         *
         * This function configures the radio bands the modem will use.
         *
         * @param rat Radio access technology
         * @param bands Bitset of WalterModemBand bits to specify the bands
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * 
         * @return True on success, false on error.
         */
        static bool setRadioBands(
            WalterModemRAT rat,
            uint32_t bands,
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
         * @brief Get the SIM ICCID and/or eUICCID.
         * 
         * The function will receive the ICCID (Integrated Circuit Card ID) and
         * eUICCID (embedded Universal Integrated Circuit Card ID) of the
         * installed SIM card. For this function to be able to actually read
         * these numbers from the SIM, the modem must be in the 
         * WALTER_MODEM_OPSTATE_FULL or WALTER_MODEM_OPSTATE_NO_RF operational
         * state.
         * 
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * 
         * @return True on success, false otherwise.
         */
        static bool getSIMCardID(
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL, 
            void *args = NULL);

        /**
         * @brief Get the IMSI on the SIM card.
         * 
         * This function will receive the IMSI (International Mobile Subscriber
         * Identity) number which is currently active on the SIM card.For this
         * function to be able to actually read the IMSI from the SIM, the modem
         * must be in the WALTER_MODEM_OPSTATE_FULL or 
         * WALTER_MODEM_OPSTATE_NO_RF operational state.
         * 
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * 
         * @return True on success, false otherwise.
         */
        static bool getSIMCardIMSI(
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
         * @brief Configure Power Saving Mode Setting.
         * 
         * This function will control whether PSM should be applied, and 
         * request the Power Saving Mode setting that Walter should use. 
         * This is only a request, see the unsolicited result codes provided 
         * by +CEREG for the Active Time value and the extended periodic 
         * TAU value that are allocated to Walter by the network.
         * 
         * @param mode Enable or disable the use of PSM.
         * @param reqTau The requested extended periodic TAU value (T3412). 
         * This is coded as one byte (octet 3) of the GPRS Timer 3 information element
         * coded as bit format (e.g. "00100001" equals 1 hour).
         * @param reqActive The requested Active Time value (T3324).
         * This is coded as one byte (octet 3) of the GPRS Timer 2 information element
         * coded as bit format (e.g. "00000101" equals 10 seconds).
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * 
         * @return True on success, false otherwise.
         */
        static bool configPSM(
            WalterModemPSMMode mode = WALTER_MODEM_PSM_DISABLE,
            const char *reqTAU = NULL,
            const char *reqActive = NULL,
            WalterModemRsp *rsp = NULL,
            walterModemCb cb = NULL,
            void *args = NULL);

        /**
         * @brief Configure extended DRX Setting.
         * 
         * This function will control whether extended DRX should be
         * applied, and configure the requested eDRX value and 
         * Paging Time Window.
         * 
         * @param mode Enable or disable the use of eDRX.
         * @param reqEDRXVal The requested eDRX value.
         * This refers to bits 4 to 1 of octet 3 of the extended DRX 
         * parameters information element.
         * @param reqPtw The requested Paging Time Window.
         * This refers to bits 8 to 5 of octet 3 of the extended DRX 
         * parameters information element.
         * @param rsp Pointer to a modem response structure to save the result 
         * of the command in. When NULL is given the result is ignored.
         * @param cb Optional callback argument, when not NULL this function
         * will return immediately.
         * @param args Optional argument to pass to the callback.
         * 
         * @return True on success, false otherwise.
         */
        static bool configEDRX(
            WalterModemEDRXMode mode = WALTER_MODEM_EDRX_DISABLE,
            const char *reqEDRXVal = NULL,
            const char *reqPtw = NULL,
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
            uint8_t *data,
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
            char *str,
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

        /**
         * @brief Offline update modem firmware from file on flash
         *
         * This function upgrades the modem firmware from a file called mota.dup
         * on the FAT filesystem on the flash. See the ModemFota example
         * sketch. Do not forget to put the supplied FAT image on
         * the flash using esptool - see comments in ModemFota.ino.
         *
         * Do not combine with initBlueCherry.
         *
         * @param otaBuffer Buffer we can use for block transfers to modem,
         * expected to be at least SPI_FLASH_SEC_SIZE = 4K
         */
        static void offlineMotaUpgrade(uint8_t *otaBuffer);

        /**
         * @brief Encode a TAU duration for use in PSM configuration.
         * 
         * This function will encode a given duration into the nearest duration that can be encoded
         * according to  the 3GPP specification for use in timer T3412 (TAU)
         *
         * @warning This function is an approximation because of the encoding used over the wire.
         *
         * @param seconds Duration in seconds
         * @param minutes Duration in minutes
         * @param hours  Duration in hours
         * @param actual_duration_seconds Optional pointer in which the actual requested duration can be saved.
         *
         * @return The interval encoded into the 3GPP standard format.
         */
        static const uint8_t durationToTAU(uint32_t seconds = 0, uint32_t minutes = 0, uint32_t hours = 0, uint32_t *actual_duration_seconds = nullptr);

        /**
         * @brief Converts a given duration of seconds, minutes to a reqActive approximation
         *
         * This function will encode a given duration into the nearest duration that can be encoded
         * according to  the 3GPP specification for use in timer T3324 (active time).
         *
         * @warning This function is an approximation because it uses a multiplier internally.
         *
         * @param seconds Duration in seconds
         * @param minutes Duration in minutes
         * @param actual_duration_seconds Optional pointer in which the actual requested duration can be saved.
         *
         * @return The duration encoded into the 3GPP standard format.
         */
        static const uint8_t durationToActiveTime(uint32_t seconds = 0, uint32_t minutes = 0, uint32_t *actual_duration_seconds = nullptr);
};

#endif
