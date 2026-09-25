/**
 * @file WalterBlueCherry.cpp
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
 * The BlueCherry cloud protocol: CoAP framing over a DTLS secured UDP socket, an outgoing message
 * queue, firmware updates and Zero-Touch Provisioning.
 *
 * The protocol logic is a port of the bluecherry-esp-idf client and is deliberately kept as close
 * to it as the platform allows, down to the names of the file static helpers, so that a fix on
 * either side can be carried to the other mechanically. What differs is everything below the
 * protocol: the modem owns the socket and the DTLS session and is driven over AT commands, the
 * credentials live in modem NVM instead of in flash, and a session survives deep sleep and is
 * resumed rather than rebuilt.
 */

#include <WalterBlueCherry.h>
#include <esp_log.h>

#if CONFIG_BLUECHERRY_ENABLE

#include <esp_attr.h>
#include <esp_heap_caps.h>
#include <esp_image_format.h>
#include <esp_mac.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_random.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <bootloader_random.h>
#include <freertos/message_buffer.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <mbedtls/ctr_drbg.h>
#include <mbedtls/ecp.h>
#include <mbedtls/entropy.h>
#include <mbedtls/net_sockets.h>
#include <mbedtls/pem.h>
#include <mbedtls/pk.h>
#include <mbedtls/ssl.h>
#include <mbedtls/x509_crt.h>
#include <mbedtls/x509_csr.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#pragma region PRIVATE_CONSTANTS

/**
 * @brief The logging tag for this BlueCherry module.
 */
static const char* TAG = "[BlueCherry]";

/**
 * @brief The library reported to the cloud in INIT_INFO.
 *
 * Several libraries share a toolchain, so the platform alone cannot identify the sender: this is
 * what makes the version numbers below mean anything to an operator identifying a fleet.
 */
#define BLUECHERRY_LIB_NAME "WalterModem"
#define BLUECHERRY_LIB_VERSION_MAJOR 1
#define BLUECHERRY_LIB_VERSION_MINOR 5
#define BLUECHERRY_LIB_VERSION_PATCH 1

/**
 * @brief The MCU reported to the cloud in INIT_INFO.
 */
#define BLUECHERRY_MCU "esp32s3"

/**
 * @brief The schema version of the INIT_INFO payload.
 */
#define BLUECHERRY_INIT_INFO_SCHEMA 1

/**
 * @brief Presence bits for the optional INIT_INFO fields.
 *
 * Optional fields are extended by presence bit, never by schema version: a parser stops at the
 * first bit it does not know, so a new field takes the next free bit and both ends degrade
 * gracefully. A field whose width changes must retire its bit and take a new one - bit 2 held a
 * one byte reset reason that became a string on bit 10, and must never be reassigned.
 */
#define BLUECHERRY_INFO_BIT_PLATFORM (1 << 0)      /* 1B  toolchain enum */
#define BLUECHERRY_INFO_BIT_LIB_VERSION (1 << 1)   /* 3B  major, minor, patch */
#define BLUECHERRY_INFO_BIT_OTA_SLOT_SIZE (1 << 3) /* 4B  usable slot size, LE */
#define BLUECHERRY_INFO_BIT_UPTIME (1 << 4)        /* 4B  seconds since boot, LE */
#define BLUECHERRY_INFO_BIT_TOTAL_HEAP (1 << 5)    /* 4B  total heap bytes, LE */
#define BLUECHERRY_INFO_BIT_LIB_NAME (1 << 7)      /* 1B len + UTF-8 */
#define BLUECHERRY_INFO_BIT_MCU (1 << 8)           /* 1B len + UTF-8 */
#define BLUECHERRY_INFO_BIT_OTA_SLOT (1 << 9)      /* 2B  running slot, target slot */
#define BLUECHERRY_INFO_BIT_RESET_REASON (1 << 10) /* 1B len + UTF-8 */

/**
 * @brief The longest string an INIT_INFO field carries.
 */
#define BLUECHERRY_INFO_STR_MAX 32

/**
 * @brief Reported when the running or target partition is not an OTA slot.
 */
#define BLUECHERRY_OTA_SLOT_NONE 0xFF

/**
 * @brief Reason byte appended to BLUECHERRY_EVENT_TYPE_ERROR: the probe reply.
 *
 * The cloud opens every update with BLUECHERRY_EVENT_TYPE_OTA_PROBE and sends nothing this client
 * accepts until it is answered with [4][0xB2]. The reply must be EXACTLY those two bytes: a one
 * byte [4] is the historic payload-less error reported on any topic 0x00 failure, and the length
 * is all that keeps the two apart.
 */
#define BLUECHERRY_OTA_ERROR_UNSUPPORTED_PROTOCOL 0xB2

/**
 * @brief The size of the CoAP header written in front of every message.
 */
#define BLUECHERRY_COAP_HEADER_SIZE 5U

/**
 * @brief The size of the [topic][length] header in front of every payload record.
 */
#define BLUECHERRY_MQTT_HEADER_SIZE 2U

/**
 * @brief The bytes a queued message costs on top of its payload.
 */
#define BLUECHERRY_PUBLISH_RECORD_OVERHEAD                                                         \
  (2U + BLUECHERRY_COAP_HEADER_SIZE + BLUECHERRY_MQTT_HEADER_SIZE)

/**
 * @brief The largest payload an internal channel event carries, and the slot that holds one.
 */
#define BLUECHERRY_EVENT_PAYLOAD_MAX 128
#define BLUECHERRY_PENDING_EVENT_SIZE                                                              \
  (BLUECHERRY_COAP_HEADER_SIZE + BLUECHERRY_MQTT_HEADER_SIZE + BLUECHERRY_EVENT_PAYLOAD_MAX)

/**
 * @brief Buffer sizes for the Zero-Touch Provisioning exchange.
 */
#define BLUECHERRY_ZTP_PKEY_BUF_SIZE 256
#define BLUECHERRY_ZTP_CERT_BUF_SIZE 576
#define BLUECHERRY_ZTP_SUBJ_BUF_SIZE 32
#define BLUECHERRY_ZTP_MAC_LEN 6
#define BLUECHERRY_ZTP_IMEI_LEN 15
#define BLUECHERRY_ZTP_MAX_DEVICE_ID_PARAMS 3
#define BLUECHERRY_ZTP_TX_BUF_SIZE (BLUECHERRY_ZTP_CERT_BUF_SIZE + 64)

/**
 * @brief The number of bytes of the ZTP response that are skipped unparsed.
 *
 * The response is not decoded: this many bytes are dropped and the remainder is treated as the
 * payload, so any change to the provisioning server's response header silently corrupts it.
 */
#define BLUECHERRY_ZTP_RSP_HEADER_LEN 7

/**
 * @brief The modem NVM slots BlueCherry owns.
 */
#define BLUECHERRY_SLOT_PRIVKEY 0
#define BLUECHERRY_SLOT_DEVCERT 5
#define BLUECHERRY_SLOT_CA 6

/**
 * @brief Marks the RTC block as holding a session worth resuming.
 */
#define BLUECHERRY_RTC_MAGIC 0x42433031UL

/**
 * @brief The size of the buffer holding received datagrams until the sync task reads them.
 *
 * Room for two of the largest, each stored behind a size_t length: the answer being waited for and
 * a duplicate of it. A message buffer holds one byte less than its size, hence the extra one.
 */
#define BLUECHERRY_RX_BUFFER_SIZE (2 * (BLUECHERRY_MAX_INCOMING_MESSAGE_LEN + sizeof(size_t)) + 1)

/**
 * @brief How many datagrams a resume drains before it gives up on the socket.
 */
#define BLUECHERRY_RESUME_DRAIN_MAX 8

/**
 * @brief The priority of the BlueCherry synchronisation task.
 */
static const UBaseType_t BLUECHERRY_SP = 10;

/**
 * @brief The retransmission parameters, deliberately slower than the RFC 7252 defaults to limit
 * retransmits on a slow UDP link. MAX_RETRANSMITS counts every transmission, the first included:
 * the waits are 4, 8 and 16 s times a random factor of 1 to 1.07, 28 to 30 s in total.
 *
 * The factor is drawn with esp_random(), not rand(), which is never seeded and would draw the same
 * one on every device.
 */
static const uint8_t BLUECHERRY_MAX_RETRANSMITS = 3;
static const double BLUECHERRY_ACK_TIMEOUT = 4.0;
static const double BLUECHERRY_ACK_RANDOM_FACTOR = 1.07;

/**
 * @brief How long a single read waits for a datagram before reporting a timeout, in milliseconds.
 */
static const uint32_t BLUECHERRY_SSL_READ_TIMEOUT = 100;

/**
 * @brief The handshake budget, handed to the modem as the socket connection timeout.
 */
#define BLUECHERRY_HANDSHAKE_TIMEOUT_SEC 20

/**
 * @brief Socket tunables for the BlueCherry session.
 */
#define BLUECHERRY_SOCKET_MTU 300
#define BLUECHERRY_SOCKET_EXCHANGE_TIMEOUT 0
#define BLUECHERRY_SOCKET_SEND_DELAY_MS 5000

/**
 * @brief The backoff applied between provisioning attempts, in milliseconds.
 */
#define BLUECHERRY_PROVISION_RETRY_MS 8000
#define BLUECHERRY_PROVISION_RETRY_MAX_MS 64000

/**
 * @brief The longest the synchronisation task sleeps in one go, so the watchdog stays fed.
 */
#define BLUECHERRY_SYNC_IDLE_MAX_MS 1000

/**
 * @brief How long a deep sleep waits for the synchronisation task to stop, in milliseconds.
 *
 * Long enough for an AT command already under way. A task still busy after it is in a dial, and
 * the next boot starts a new session instead.
 */
#define BLUECHERRY_SLEEP_WAIT_MS 5000

/**
 * @brief Returned by a cycle that left work outstanding, deliberately outside the esp_err_t range.
 */
#define BLUECHERRY_SYNC_CONTINUE 0x100

/**
 * @brief The hostname of the BlueCherry cloud.
 */
static const char* BLUECHERRY_HOST = BLUECHERRY_HOSTNAME;

/**
 * @brief The BlueCherry CA root + intermediate certificate used for CoAP DTLS communication.
 */
static const char* BLUECHERRY_CA = "-----BEGIN CERTIFICATE-----\r\n\
MIIBlTCCATqgAwIBAgICEAAwCgYIKoZIzj0EAwMwGjELMAkGA1UEBhMCQkUxCzAJ\r\n\
BgNVBAMMAmNhMB4XDTI0MDMyNDEzMzM1NFoXDTQ0MDQwODEzMzM1NFowJDELMAkG\r\n\
A1UEBhMCQkUxFTATBgNVBAMMDGludGVybWVkaWF0ZTBZMBMGByqGSM49AgEGCCqG\r\n\
SM49AwEHA0IABJGFt28UrHlbPZEjzf4CbkvRaIjxDRGoeHIy5ynfbOHJ5xgBl4XX\r\n\
hp/r8zOBLqSbu6iXGwgjp+wZJe1GCDi6D1KjZjBkMB0GA1UdDgQWBBR/rtuEomoy\r\n\
49ovMAnj5Hpmk2gTGjAfBgNVHSMEGDAWgBR3Vw0Y1sUvMhkX7xySsX55tvsu8TAS\r\n\
BgNVHRMBAf8ECDAGAQH/AgEAMA4GA1UdDwEB/wQEAwIBhjAKBggqhkjOPQQDAwNJ\r\n\
ADBGAiEApN7DmuufC/aqyt6g2Y8qOWg6AXFUyTcub8/Y28XY3KgCIQCs2VUXCPwn\r\n\
k8jR22wsqNvZfbndpHthtnPqI5+yFXrY4A==\r\n\
-----END CERTIFICATE-----\r\n\
-----BEGIN CERTIFICATE-----\r\n\
MIIBmDCCAT+gAwIBAgIUDjfXeosg0fphnshZoXgQez0vO5UwCgYIKoZIzj0EAwMw\r\n\
GjELMAkGA1UEBhMCQkUxCzAJBgNVBAMMAmNhMB4XDTI0MDMyMzE3MzU1MloXDTQ0\r\n\
MDQwNzE3MzU1MlowGjELMAkGA1UEBhMCQkUxCzAJBgNVBAMMAmNhMFkwEwYHKoZI\r\n\
zj0CAQYIKoZIzj0DAQcDQgAEB00rHNthOOYyKj80cd/DHQRBGSbJmIRW7rZBNA6g\r\n\
fbEUrY9NbuhGS6zKo3K59zYc5R1U4oBM3bj6Q7LJfTu7JqNjMGEwHQYDVR0OBBYE\r\n\
FHdXDRjWxS8yGRfvHJKxfnm2+y7xMB8GA1UdIwQYMBaAFHdXDRjWxS8yGRfvHJKx\r\n\
fnm2+y7xMA8GA1UdEwEB/wQFMAMBAf8wDgYDVR0PAQH/BAQDAgGGMAoGCCqGSM49\r\n\
BAMDA0cAMEQCID7AcgACnXWzZDLYEainxVDxEJTUJFBhcItO77gcHPZUAiAu/ZMO\r\n\
VYg4UI2D74WfVxn+NyVd2/aXTvSBp8VgyV3odA==\r\n\
-----END CERTIFICATE-----\r\n";

#pragma endregion
#pragma region PRIVATE_TYPES

/**
 * @brief The types of CoAP packets.
 */
typedef enum {
  BLUECHERRY_COAP_TYPE_CON = 0,
  BLUECHERRY_COAP_TYPE_NON = 1,
  BLUECHERRY_COAP_TYPE_ACK = 2,
  BLUECHERRY_COAP_TYPE_RST = 3
} _bluecherry_coap_type;

/**
 * @brief The types of CoAP responses.
 */
typedef enum {
  BLUECHERRY_COAP_RSP_VALID = 0x43,
  BLUECHERRY_COAP_RSP_CONTINUE = 0x61
} _bluecherry_coap_response;

/**
 * @brief The possible types of BlueCherry events.
 *
 * Events 5 to 8 are the modem firmware update, which is unique to Walter. Those numbers are
 * reserved by the protocol on every client, so they never collide with the application update.
 */
typedef enum {
  BLUECHERRY_EVENT_TYPE_OTA_PROBE = 1,
  BLUECHERRY_EVENT_TYPE_OTA_UNSUPPORTED_CHUNK = 2,
  BLUECHERRY_EVENT_TYPE_OTA_UNSUPPORTED_FINISH = 3,
  BLUECHERRY_EVENT_TYPE_ERROR = 4,
  BLUECHERRY_EVENT_TYPE_MOTA_INITIALIZE = 5,
  BLUECHERRY_EVENT_TYPE_MOTA_CHUNK = 6,
  BLUECHERRY_EVENT_TYPE_MOTA_FINISH = 7,
  BLUECHERRY_EVENT_TYPE_MOTA_ERROR = 8,
  BLUECHERRY_EVENT_TYPE_PARTITION_HASH = 9,
  BLUECHERRY_EVENT_TYPE_INIT_INFO = 10,
  BLUECHERRY_EVENT_TYPE_OTA_INITIALIZE = 11,
  BLUECHERRY_EVENT_TYPE_OTA_START = 12,
  BLUECHERRY_EVENT_TYPE_OTA_CHUNK = 13,
  BLUECHERRY_EVENT_TYPE_OTA_RESUME = 14,
  BLUECHERRY_EVENT_TYPE_OTA_VERIFIED = 15,
  BLUECHERRY_EVENT_TYPE_OTA_ERROR = 16
} _bluecherry_event_type;

/**
 * @brief The toolchain reported in INIT_INFO.
 */
typedef enum {
  BLUECHERRY_PLATFORM_UNKNOWN = 0,
  BLUECHERRY_PLATFORM_ESP_IDF = 1,
  BLUECHERRY_PLATFORM_ARDUINO = 2,
  BLUECHERRY_PLATFORM_NORDIC = 3,
  BLUECHERRY_PLATFORM_ZEPHYR = 4,
  BLUECHERRY_PLATFORM_LINUX = 5
} _bluecherry_platform;

/**
 * @brief Internal OTA state.
 */
typedef enum {
  BLUECHERRY_OTA_STATE_IDLE = 0,
  BLUECHERRY_OTA_STATE_OFFERED,
  BLUECHERRY_OTA_STATE_DOWNLOADING,
  BLUECHERRY_OTA_STATE_AWAITING_VERIFIED,
  BLUECHERRY_OTA_STATE_COMPLETE,
  BLUECHERRY_OTA_STATE_RESUMING
} _bluecherry_ota_state;

/**
 * @brief One device identifier value offered to the provisioning server.
 */
typedef union {
  const char* bc_type_id;
  unsigned char mac[BLUECHERRY_ZTP_MAC_LEN];
  char imei[BLUECHERRY_ZTP_IMEI_LEN + 1];
  unsigned long long oob_challenge;
} _bluecherry_ztp_device_id_value_t;

/**
 * @brief One device identifier offered to the provisioning server.
 */
typedef struct {
  BlueCherryZtpDeviceIdType type;
  _bluecherry_ztp_device_id_value_t value;
} _bluecherry_ztp_device_id_param_t;

/**
 * @brief A CSR in DER format.
 */
typedef struct {
  unsigned char buffer[BLUECHERRY_ZTP_CERT_BUF_SIZE];
  size_t length;
} _bluecherry_ztp_csr_t;

/**
 * @brief The set of device identifiers offered to the provisioning server.
 */
typedef struct {
  _bluecherry_ztp_device_id_param_t param[BLUECHERRY_ZTP_MAX_DEVICE_ID_PARAMS];
  int count;
} _bluecherry_ztp_device_id_t;

/**
 * @brief The CBOR encoder context.
 */
typedef struct {
  uint8_t* buffer;
  size_t capacity;
  size_t position;
} _ztp_cbor_t;

/**
 * @brief A message waiting to be published.
 */
typedef struct {
  size_t len;
  uint8_t* data;
} _bluecherry_msg_t;

/**
 * @brief The queue of messages waiting to be published, as a ring of framed records.
 *
 * Each record is a 2 byte little endian length followed by that many bytes, which are the message
 * with room for its CoAP header already in front of it. A record is never split across the end of
 * the buffer: the transmit path writes the CoAP header into the record in place and retransmits
 * from it, so what is handed out has to be one contiguous run of bytes. When a record does not fit
 * at the end, wrap remembers where the used bytes stopped and writing restarts at 0.
 */
typedef struct {
  uint8_t* buf;
  size_t size;
  size_t head;
  size_t tail;
  size_t wrap;
  size_t count;
  bool owned;
  SemaphoreHandle_t lock;
} _bluecherry_ring_t;

/**
 * @brief The operational data used by the BlueCherry cloud connection.
 */
typedef struct {
  BlueCherryState state;

  /** @brief The modem socket carrying the session, or -1 when there is none. */
  int sock;

  /** @brief The modem TLS profile the session is built from. */
  uint8_t tls_profile_id;

  /** @brief The PDP context the socket rides on. */
  int pdp_ctx_id;

  _bluecherry_ztp_device_id_t ztp_dev_id_params;
  _bluecherry_ztp_csr_t ztp_csr;
  mbedtls_x509write_csr ztp_mb_csr;
  mbedtls_x509_crt devcert;
  mbedtls_pk_context devkey;
  mbedtls_ctr_drbg_context ctr_drbg;
  mbedtls_entropy_context entropy;

  _bluecherry_ring_t out_ring;

  blueCherryMsgHandler msg_handler;
  void* msg_handler_args;

  uint16_t cur_message_id;
  uint16_t last_acked_message_id;

  /** @brief The message id of the last frame handed to the modem, acknowledged or not. */
  uint16_t sent_message_id;
  int64_t last_tx_us;

  size_t in_buf_len;

  /** @brief The receive buffer, sized for the largest datagram the modem will hand over. */
  uint8_t in_buf[BLUECHERRY_MAX_INCOMING_MESSAGE_LEN];

  /**
   * @brief One flash sector of firmware staged on its way to the partition.
   *
   * Distinct from in_buf on purpose: a chunk is copied out of in_buf into here, and a sector's
   * worth spans many datagrams, so in_buf is overwritten several times before this is flushed.
   *
   * The modem firmware upgrade borrows it for STP transfer blocks. That is safe because it runs
   * with the modem rx handler muted, so no OTA chunk can arrive while it holds it.
   */
  uint8_t ota_buffer[BLUECHERRY_OTA_BUFFER_SIZE];

  uint32_t ota_size;
  uint32_t ota_progress;
  uint32_t ota_buffer_pos;
  uint8_t ota_skip_buffer[ENCRYPTED_BLOCK_SIZE];
  const esp_partition_t* ota_partition;
  _bluecherry_ota_state ota_state;
  uint8_t ota_expected_hash[BLUECHERRY_PARTITION_HASH_LEN];
  bool ota_unverified;
  int8_t ota_target_version;

  /**
   * @brief True while a RESUME still has to be queued for the current session.
   */
  bool ota_resume_due;

  /**
   * @brief Priority slot for one outgoing internal channel frame.
   *
   * Checked BEFORE out_ring in the send step. One slot suffices because every internal event is a
   * reply the server then responds to, so only one is ever outstanding.
   */
  uint8_t pending_event[BLUECHERRY_PENDING_EVENT_SIZE];
  size_t pending_event_len;

  blueCherryOtaHandler ota_handler;
  void* ota_handler_args;
  blueCherryStateHandler state_handler;
  void* state_handler_args;
} _bluecherry_t;

/**
 * @brief The slice of BlueCherry state that has to survive deep sleep.
 *
 * Deliberately plain data and free of pointers. RTC slow memory is 8 kB on the ESP32-S3 and the
 * modem driver's own mirrors already take most of it, and any pointer kept here would be a stale
 * address after the application is updated. The OTA partition is therefore remembered as a slot
 * index and resolved again on wake, while the staging buffer and the handlers are supplied afresh
 * by the application on every boot.
 */
typedef struct {
  uint32_t magic;

  int8_t sock;
  uint8_t tls_profile_id;
  uint8_t pdp_ctx_id;

  /* NOT reset on resume: a wake is not a reconnect, and the server's view of the message id
   * sequence survived the sleep along with the session. */
  uint16_t cur_message_id;
  uint16_t last_acked_message_id;

  uint8_t ota_state;
  uint32_t ota_size;
  uint32_t ota_progress;
  int8_t ota_target_version;
  bool ota_unverified;
  uint8_t ota_partition_slot;
  uint8_t ota_expected_hash[BLUECHERRY_PARTITION_HASH_LEN];
  uint8_t ota_skip_buffer[ENCRYPTED_BLOCK_SIZE];
  bool ota_resume_due;

  uint16_t pending_event_len;
  uint8_t pending_event[BLUECHERRY_PENDING_EVENT_SIZE];

  /** @brief Set on the INIT_INFO ack, not by _sleepPrepare, and read without the magic check. */
  bool init_info_acked;
} _bluecherry_rtc_t;

#pragma endregion
#pragma region PRIVATE_STATE

/**
 * @brief The operational data of the one BlueCherry connection.
 */
static _bluecherry_t _bluecherry_opdata = {};

/**
 * @brief What survives deep sleep.
 */
RTC_DATA_ATTR static _bluecherry_rtc_t _bluecherry_rtc;

/**
 * @brief Received datagrams, read out of the modem by the event task and read by the sync task.
 *
 * Plays the part of the socket's own receive buffer on WiFi.
 */
static StaticMessageBuffer_t _bc_rx_buf_struct;
static uint8_t _bc_rx_buf_mem[BLUECHERRY_RX_BUFFER_SIZE];
static MessageBufferHandle_t _bc_rx_buf = NULL;

/**
 * @brief Where the event task reads a datagram out of the modem before it is buffered.
 */
static uint8_t _bc_rx_scratch[BLUECHERRY_MAX_INCOMING_MESSAGE_LEN];

/**
 * @brief Set when the modem reports the BlueCherry socket closed.
 */
static volatile bool _bc_peer_closed = false;

/**
 * @brief The buffer used to store a private key.
 */
static char ztp_pkey_buf[BLUECHERRY_ZTP_PKEY_BUF_SIZE];

/**
 * @brief The buffer used to store a certificate.
 */
static char ztp_cert_buf[BLUECHERRY_ZTP_CERT_BUF_SIZE];

/**
 * @brief The BlueCherry device ID received from the server.
 */
static char ztp_bc_dev_id[BLUECHERRY_ZTP_ID_LEN + 1];

/**
 * @brief The buffer used for the CSR subject.
 */
static char ztp_subj_buf[BLUECHERRY_ZTP_SUBJ_BUF_SIZE];

/**
 * @brief The BlueCherry type ID associated with this firmware.
 */
static const char* bc_type_id = NULL;

/**
 * @brief When the next provisioning attempt is due, or 0 for "due now".
 */
static int64_t _next_provision_us = 0;

/**
 * @brief The synchronisation task, or NULL before the first init.
 */
static TaskHandle_t _sync_task = NULL;

/**
 * @brief Seconds between automatic synchronisations, or 0 when they are off.
 */
static uint32_t _auto_sync_interval_sec = 0;

/**
 * @brief When the last synchronisation cycle ran.
 */
static int64_t _last_sync_us = 0;

/**
 * @brief When the next automatic synchronisation is due. Unused while the interval is 0.
 */
static int64_t _next_auto_sync_us = 0;

/**
 * @brief Update requests raised by the application, carried out on the sync task.
 *
 * The work they ask for writes the priority slot the task transmits out of, so the caller only
 * raises a flag: that keeps every write to that slot on one thread.
 */
static volatile bool _ota_start_req = false;
static volatile bool _ota_abort_req = false;
static volatile uint8_t _ota_abort_code = 0;

/**
 * @brief Guards the connection state and the request flag below.
 *
 * A spinlock rather than a mutex, because what it protects is a decision immediately followed by
 * the assignment it decided on, nothing under it ever blocks, and the two tasks involved can be on
 * different cores. The state handler is always called outside it.
 */
static portMUX_TYPE _bluecherry_state_lock = portMUX_INITIALIZER_UNLOCKED;

/**
 * @brief Raised when an exchange is asked for, cleared by the task as it starts one.
 *
 * Counting the outgoing ring takes the ring's mutex, which cannot be held inside a critical
 * section, so the end of a cycle has to read that count before it takes this lock. This flag
 * closes the gap: it is raised under the same lock the decision is made under, so a message queued
 * in between cannot be settled past.
 */
static bool _sync_requested = false;

/**
 * @brief Raised by _sleepPrepare. The synchronisation task stops at its next step and does not
 * run again before the deep sleep.
 */
static bool _sleep_requested = false;

/**
 * @brief Raised by the synchronisation task once it has stopped for the deep sleep.
 */
static bool _sync_parked = false;

/* Defined further down, but needed before their definitions. */
static esp_err_t _bluecherry_sync_once(void);
static void _bluecherry_ota_service_requests(void);
static void _bluecherry_cleanup_session(void);
static bool _bluecherry_dtls_connect(const char* host, uint16_t port);
static int _bluecherry_mbed_dtls_read(unsigned char* buf, size_t len);
static int _bluecherry_mbed_dtls_write(const unsigned char* buf, size_t len);

#pragma endregion
#pragma region HELPERS

/**
 * @brief Whether the synchronisation task is subscribed to the task watchdog.
 *
 * Feeding a watchdog a task is not subscribed to logs an error on every call, and a project that
 * compiled the task watchdog out has nothing to subscribe to at all, so the subscription decides
 * whether the feeding happens.
 */
static bool _watchdog = false;

/**
 * @brief Tickle the task watchdog.
 *
 * @return None.
 */
static void _bluecherry_tickle_watchdog(void)
{
  if(_watchdog) {
    esp_task_wdt_reset();
  }
}

/**
 * @brief Stop the synchronisation task for good once a deep sleep is requested.
 *
 * Called before anything is sent and before a response is read, so nothing goes out and nothing
 * changes after _sleepPrepare took its snapshot.
 *
 * @return None.
 */
static void _bluecherry_park_for_sleep(void)
{
  portENTER_CRITICAL(&_bluecherry_state_lock);
  bool sleeping = _sleep_requested;
  portEXIT_CRITICAL(&_bluecherry_state_lock);

  if(!sleeping) {
    return;
  }

  if(_watchdog) {
    esp_task_wdt_delete(NULL);
  }

  portENTER_CRITICAL(&_bluecherry_state_lock);
  _sync_parked = true;
  portEXIT_CRITICAL(&_bluecherry_state_lock);

  vTaskSuspend(NULL);
}

/**
 * @brief Widen the task watchdog to the budget a synchronisation cycle needs.
 *
 * The watchdog has one timeout shared by every subscribed task, so the budget the synchronisation
 * task needs is necessarily the whole timer's. A cycle feeds the watchdog around each blocking
 * step, but socketDial cannot be broken up: the DTLS handshake runs inside AT+SQNSD and is allowed
 * BLUECHERRY_HANDSHAKE_TIMEOUT_SEC on its own. Arduino has no Kconfig and its core ships a five
 * second timeout, which that dial trips on every cold connect.
 *
 * Only ever widened, and left alone when it is already wide enough, so an application that asked
 * for a longer budget keeps it.
 *
 * @param current_sec The timeout in force, or 0 when WalterModem left it at the project default.
 *
 * @return None.
 */
static void _bluecherry_widen_watchdog(uint16_t current_sec)
{
#if CONFIG_ESP_TASK_WDT_EN
  if(current_sec == 0) {
    current_sec = CONFIG_ESP_TASK_WDT_TIMEOUT_S;
  }

  if(current_sec >= BLUECHERRY_WDT_TIMEOUT_S) {
    return;
  }

  /* Rebuilt from the project's own settings rather than carried over from a getter there is none
   * of. Forcing an idle check onto a core the project excluded, or a panic it turned off, would
   * change behaviour that is not ours to change; only the timeout is. */
  esp_task_wdt_config_t twdt_config = {};
  twdt_config.timeout_ms = (uint32_t) BLUECHERRY_WDT_TIMEOUT_S * 1000UL;
#if CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0
  twdt_config.idle_core_mask |= 1 << 0;
#endif
#if CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1
  twdt_config.idle_core_mask |= 1 << 1;
#endif
#if CONFIG_ESP_TASK_WDT_PANIC
  twdt_config.trigger_panic = true;
#endif

#if CONFIG_ESP_TASK_WDT_INIT
  esp_err_t ret = esp_task_wdt_reconfigure(&twdt_config);
#else
  esp_err_t ret = esp_task_wdt_init(&twdt_config);
#endif

  if(ret == ESP_OK) {
    ESP_LOGD(TAG, "Task watchdog widened from %us to %ds", current_sec, BLUECHERRY_WDT_TIMEOUT_S);
  } else {
    ESP_LOGW(TAG, "Could not widen the %us task watchdog to %ds, a dial may trip it", current_sec,
             BLUECHERRY_WDT_TIMEOUT_S);
  }
#endif
}

/**
 * @brief Move to a new connection state and tell the application, if it asked.
 *
 * The single place the state is written, so the handler cannot miss a transition.
 *
 * @param next The state to enter.
 *
 * @return None.
 */
static void _bluecherry_set_state(BlueCherryState next)
{
  bool changed = false;

  portENTER_CRITICAL(&_bluecherry_state_lock);
  if(_bluecherry_opdata.state != next) {
    _bluecherry_opdata.state = next;
    changed = true;
  }
  portEXIT_CRITICAL(&_bluecherry_state_lock);

  /* Reported outside the lock, and only while this is still the current state. A publish or a sync
   * that promoted out of it in between has its own transition to report, and delivering a state it
   * has already superseded would tell the application the opposite of what is true. */
  if(changed && _bluecherry_opdata.state == next && _bluecherry_opdata.state_handler != NULL) {
    _bluecherry_opdata.state_handler(next, _bluecherry_opdata.state_handler_args);
  }
}

/**
 * @brief Record that an exchange has been asked for and take the state out of idle.
 *
 * Both halves happen under one lock, so a cycle settling concurrently on the task either sees the
 * request and holds the state at BLUECHERRY_STATE_PENDING_MESSAGES, or settles to idle first and
 * is promoted back out of it here. Either way the state is no longer idle by the time the caller
 * gets control back, which is what lets an application ask for an exchange and then look at the
 * state without racing the task.
 *
 * @return None.
 */
static void _bluecherry_request_sync(void)
{
  bool changed = false;

  portENTER_CRITICAL(&_bluecherry_state_lock);
  _sync_requested = true;
  if(_bluecherry_opdata.state == BLUECHERRY_STATE_IDLE) {
    _bluecherry_opdata.state = BLUECHERRY_STATE_PENDING_MESSAGES;
    changed = true;
  }
  portEXIT_CRITICAL(&_bluecherry_state_lock);

  if(changed && _bluecherry_opdata.state_handler != NULL) {
    _bluecherry_opdata.state_handler(BLUECHERRY_STATE_PENDING_MESSAGES,
                                     _bluecherry_opdata.state_handler_args);
  }
}

/**
 * @brief Settle the state at the end of a cycle, atomically against publish and sync.
 *
 * @param pending Whether anything is still outstanding, evaluated before the lock is taken.
 *
 * @return The state settled on.
 */
static BlueCherryState _bluecherry_settle_to(bool pending)
{
  BlueCherryState next;
  bool changed = false;

  portENTER_CRITICAL(&_bluecherry_state_lock);
  next = (pending || _sync_requested) ? BLUECHERRY_STATE_PENDING_MESSAGES : BLUECHERRY_STATE_IDLE;
  if(_bluecherry_opdata.state != next) {
    _bluecherry_opdata.state = next;
    changed = true;
  }
  portEXIT_CRITICAL(&_bluecherry_state_lock);

  if(changed && _bluecherry_opdata.state == next && _bluecherry_opdata.state_handler != NULL) {
    _bluecherry_opdata.state_handler(next, _bluecherry_opdata.state_handler_args);
  }

  return next;
}

#pragma endregion
#pragma region RING

/**
 * @brief Release the publish buffer, freeing it only if it was not the application's.
 *
 * @return None.
 */
static void _bluecherry_ring_deinit(void)
{
  _bluecherry_ring_t* ring = &_bluecherry_opdata.out_ring;

  if(ring->lock != NULL) {
    vSemaphoreDelete(ring->lock);
  }
  if(ring->owned) {
    free(ring->buf);
  }

  memset(ring, 0, sizeof(*ring));
}

/**
 * @brief Reserve the publish buffer.
 *
 * @param cfg The application's buffer, or NULL to allocate one here.
 *
 * @return ESP_OK on success.
 */
static esp_err_t _bluecherry_ring_init(const BlueCherryPublishBuffer* cfg)
{
  _bluecherry_ring_t* ring = &_bluecherry_opdata.out_ring;

  /* Idempotent: an init retried after a failed one would otherwise strand the previous buffer and
   * its lock. Safe because the synchronisation task does not touch the buffer while the state is
   * BLUECHERRY_STATE_UNINITIALIZED. */
  _bluecherry_ring_deinit();

  if(cfg != NULL && cfg->buffer != NULL) {
    if(cfg->size < BLUECHERRY_MIN_PUBLISH_BUFFER) {
      ESP_LOGE(TAG, "The publish buffer must be at least %uB", BLUECHERRY_MIN_PUBLISH_BUFFER);
      return ESP_ERR_INVALID_ARG;
    }
    ring->buf = cfg->buffer;
    ring->size = cfg->size;
    ring->owned = false;
  } else {
    ring->size = BLUECHERRY_PUBLISH_BUFFER_SIZE;
    ring->buf = (uint8_t*) malloc(ring->size);
    if(ring->buf == NULL) {
      ESP_LOGE(TAG, "Could not allocate the %uB publish buffer", (unsigned) ring->size);
      return ESP_ERR_NO_MEM;
    }
    ring->owned = true;
  }

  ring->lock = xSemaphoreCreateMutex();
  if(ring->lock == NULL) {
    ESP_LOGE(TAG, "Could not create the publish buffer lock");
    if(ring->owned) {
      free(ring->buf);
    }
    ring->buf = NULL;
    return ESP_FAIL;
  }

  ring->head = 0;
  ring->tail = 0;
  ring->wrap = ring->size;
  ring->count = 0;

  return ESP_OK;
}

/**
 * @brief How many messages are waiting to be published.
 *
 * @return The number of queued messages.
 */
static size_t _bluecherry_ring_count(void)
{
  _bluecherry_ring_t* ring = &_bluecherry_opdata.out_ring;

  if(ring->lock == NULL) {
    return 0;
  }

  xSemaphoreTake(ring->lock, portMAX_DELAY);
  size_t count = ring->count;
  xSemaphoreGive(ring->lock);

  return count;
}

/**
 * @brief Frame one message into the publish buffer.
 *
 * @param topic The topic of the message, passed as the topic index.
 * @param len The length of the topic payload data.
 * @param data The topic payload data.
 *
 * @return ESP_OK on success, ESP_ERR_NO_MEM when the buffer is full.
 */
static esp_err_t _bluecherry_ring_push(uint8_t topic, uint16_t len, const uint8_t* data)
{
  _bluecherry_ring_t* ring = &_bluecherry_opdata.out_ring;
  const size_t rec = BLUECHERRY_COAP_HEADER_SIZE + BLUECHERRY_MQTT_HEADER_SIZE + len;
  const size_t need = 2 + rec;

  if(ring->lock == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  if(need > ring->size) {
    ESP_LOGE(TAG, "The message does not fit in the %uB publish buffer", (unsigned) ring->size);
    return ESP_ERR_INVALID_SIZE;
  }

  xSemaphoreTake(ring->lock, portMAX_DELAY);

  if(ring->count == 0) {
    /* Nothing is held, so start over and give the message the whole buffer to fit in. */
    ring->head = 0;
    ring->tail = 0;
    ring->wrap = ring->size;
  }

  size_t room = ring->count == 0          ? ring->size
                : ring->head > ring->tail ? ring->size - ring->head
                                          : ring->tail - ring->head;
  size_t at;

  if(room >= need) {
    at = ring->head;
    ring->head += need;
  } else if(ring->head > ring->tail && ring->tail >= need) {
    /* Out of room at the end, but the records have moved on far enough to start again in front of
     * them. wrap is where the reader has to turn around. */
    ring->wrap = ring->head;
    at = 0;
    ring->head = need;
  } else {
    xSemaphoreGive(ring->lock);
    return ESP_ERR_NO_MEM;
  }

  uint8_t* p = ring->buf + at;
  p[0] = (uint8_t) (rec & 0xFF);
  p[1] = (uint8_t) (rec >> 8);
  p += 2;

  /* The CoAP header is left as it is: it carries the message id, which the transmit path only
   * knows once it is about to send, and writes into the record then. */
  p[BLUECHERRY_COAP_HEADER_SIZE] = topic;
  p[BLUECHERRY_COAP_HEADER_SIZE + 1] = (uint8_t) (len & 0xFF);
  memcpy(p + BLUECHERRY_COAP_HEADER_SIZE + BLUECHERRY_MQTT_HEADER_SIZE, data, len);

  ring->count += 1;
  xSemaphoreGive(ring->lock);

  return ESP_OK;
}

/**
 * @brief Point at the oldest message without removing it.
 *
 * The pointer stays valid until _bluecherry_ring_pop, which is what lets the message be
 * transmitted, and retransmitted, while other tasks keep publishing: a publish only ever writes
 * from head onwards, which never reaches into the record being sent.
 *
 * @param out Filled in with the message.
 *
 * @return True when there was one.
 */
static bool _bluecherry_ring_peek(_bluecherry_msg_t* out)
{
  _bluecherry_ring_t* ring = &_bluecherry_opdata.out_ring;

  if(ring->lock == NULL) {
    return false;
  }

  xSemaphoreTake(ring->lock, portMAX_DELAY);

  bool found = ring->count > 0;
  if(found) {
    out->len = (size_t) ring->buf[ring->tail] | ((size_t) ring->buf[ring->tail + 1] << 8);
    out->data = ring->buf + ring->tail + 2;
  }

  xSemaphoreGive(ring->lock);

  return found;
}

/**
 * @brief Drop the oldest message, which is only correct once the cloud has acknowledged it.
 *
 * @return None.
 */
static void _bluecherry_ring_pop(void)
{
  _bluecherry_ring_t* ring = &_bluecherry_opdata.out_ring;

  if(ring->lock == NULL) {
    return;
  }

  xSemaphoreTake(ring->lock, portMAX_DELAY);

  if(ring->count > 0) {
    size_t len = (size_t) ring->buf[ring->tail] | ((size_t) ring->buf[ring->tail + 1] << 8);
    ring->tail += 2 + len;
    ring->count -= 1;

    if(ring->tail >= ring->wrap) {
      /* The records tile the buffer exactly, so this lands on wrap rather than past it. */
      ring->tail = 0;
      ring->wrap = ring->size;
    }
    if(ring->count == 0) {
      ring->head = 0;
      ring->tail = 0;
      ring->wrap = ring->size;
    }
  }

  xSemaphoreGive(ring->lock);
}

#pragma endregion
#pragma region SYNC_SCHEDULING

/**
 * @brief Whether anything is still outstanding in either direction.
 *
 * The publish buffer counts because a cycle sends one message: without it the state would settle
 * to IDLE with publishes still waiting.
 *
 * @param want_resync Whether the server asked for another round.
 *
 * @return True while there is more to do.
 */
static bool _bluecherry_work_pending(bool want_resync)
{
  return want_resync || _bluecherry_opdata.pending_event_len > 0 || _bluecherry_ring_count() > 0;
}

/**
 * @brief Settle the state after a cycle that returned before it could do so itself.
 *
 * A cycle that reaches the end settles itself. One that returns early leaves AWAITING_RESPONSE
 * behind, which is the only case this cleans up - testing for exactly that is what stops it
 * overwriting a PENDING_MESSAGES the server asked for.
 *
 * @return None.
 */
static void _bluecherry_settle_state(void)
{
  if(_bluecherry_opdata.state != BLUECHERRY_STATE_AWAITING_RESPONSE) {
    return;
  }

  _bluecherry_settle_to(_bluecherry_work_pending(false));
}

/**
 * @brief How long the synchronisation task may wait before running the next cycle.
 *
 * The next deadline it knows about: a provisioning retry or the auto-sync interval. Capped at
 * BLUECHERRY_SYNC_IDLE_MAX_MS so the watchdog cannot be starved. A trigger cuts the wait short.
 *
 * @return The number of milliseconds to wait.
 */
static uint32_t _bluecherry_sync_wait_ms(void)
{
  int64_t deadline_us = 0;

  if(_bluecherry_opdata.state == BLUECHERRY_STATE_NOT_PROVISIONED) {
    deadline_us = _next_provision_us;
  } else if(_auto_sync_interval_sec > 0) {
    deadline_us = _next_auto_sync_us;
  }

  if(deadline_us == 0) {
    return BLUECHERRY_SYNC_IDLE_MAX_MS;
  }

  int64_t remaining_us = deadline_us - esp_timer_get_time();
  if(remaining_us <= 0) {
    return 0;
  }

  uint64_t remaining_ms = (uint64_t) remaining_us / 1000;
  return remaining_ms > BLUECHERRY_SYNC_IDLE_MAX_MS ? BLUECHERRY_SYNC_IDLE_MAX_MS
                                                    : (uint32_t) remaining_ms;
}

/**
 * @brief Whether a cycle is due without anything having asked for one.
 *
 * The wait is capped so the watchdog stays fed, so it expiring says nothing about whether there is
 * work: this is the actual test. Provisioning and connecting are always due, because their own
 * backoff gates decide whether the cycle does anything.
 *
 * @return True when the task should run a cycle of its own accord.
 */
static bool _bluecherry_sync_due(void)
{
  switch(_bluecherry_opdata.state) {
  case BLUECHERRY_STATE_UNINITIALIZED:
    return false;

  case BLUECHERRY_STATE_NOT_PROVISIONED:
    return esp_timer_get_time() >= _next_provision_us;

  case BLUECHERRY_STATE_AWAIT_CONNECTION:
    return true;

  default:
    break;
  }

  return _auto_sync_interval_sec > 0 && esp_timer_get_time() >= _next_auto_sync_us;
}

/**
 * @brief Arm the next automatic synchronisation, or clear it when they are off.
 *
 * Counted from the last synchronisation, not from now, so that changing the interval asks "how
 * long since the last one" rather than restarting the wait. Shortening it below the time already
 * elapsed therefore leaves the next one due immediately, which is the point.
 *
 * @return None.
 */
static void _bluecherry_arm_auto_sync(void)
{
  _next_auto_sync_us =
      _auto_sync_interval_sec > 0 ? _last_sync_us + (int64_t) _auto_sync_interval_sec * 1000000 : 0;
}

/**
 * @brief The entrypoint of the BlueCherry synchronisation task.
 *
 * Owns every network operation the library performs, and all the timing around them. It runs a
 * cycle when something triggers one, when an automatic synchronisation falls due, or when the
 * previous cycle finished with work still outstanding; otherwise it waits.
 *
 * @param args A NULL pointer.
 *
 * @return None.
 */
static void _bluecherry_sync_task(void* args)
{
  _watchdog = esp_task_wdt_add(NULL) == ESP_OK;

  while(true) {
    _bluecherry_tickle_watchdog();

    /* Clearing on take is what makes a sync request a trigger rather than a queue: any number of
     * calls collapse into one cycle, and one raised mid-cycle is still pending here. */
    uint32_t triggered = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(_bluecherry_sync_wait_ms()));

    _bluecherry_park_for_sleep();
    _bluecherry_tickle_watchdog();

    /* The task starts before init finishes, and init can still fail after that. */
    if(_bluecherry_opdata.state == BLUECHERRY_STATE_UNINITIALIZED) {
      continue;
    }

    /* Waking is not the same as having something to do: the wait is capped for the watchdog, so
     * it expires long before a deadline that is further out than the cap. */
    if(triggered == 0 && !_bluecherry_sync_due()) {
      continue;
    }

    /* Cleared as the cycle starts rather than when it ends, so a publish or a sync that arrives
     * while this one is running still holds the state out of idle afterwards. */
    portENTER_CRITICAL(&_bluecherry_state_lock);
    _sync_requested = false;
    portEXIT_CRITICAL(&_bluecherry_state_lock);

    _bluecherry_ota_service_requests();

    esp_err_t ret = _bluecherry_sync_once();
    _bluecherry_settle_state();

    /* Recorded at the end of a cycle, so the interval is "time since the last exchange". */
    _last_sync_us = esp_timer_get_time();
    _bluecherry_arm_auto_sync();

    /* Self-notify rather than loop, so every cycle takes the same path past the watchdog. */
    if(ret == BLUECHERRY_SYNC_CONTINUE) {
      xTaskNotifyGive(_sync_task);
    }

    /* Also fed on the way out, so a long sync is bracketed rather than merely preceded by a
     * reset. Feeding only before the call leaves the effective budget at "the watchdog timeout
     * minus one whole sync". */
    _bluecherry_tickle_watchdog();
  }
}

#pragma endregion
#pragma region OTA

/**
 * @brief Get the current OTA progress as a percentage.
 *
 * @return The percentage of the image written to flash, or 0 when no update is running.
 */
static float _bluecherry_ota_progress_percent(void)
{
  if(_bluecherry_opdata.ota_size == 0) {
    return 0.0f;
  }

  return ((float) _bluecherry_opdata.ota_progress / (float) _bluecherry_opdata.ota_size) * 100.0f;
}

/**
 * @brief Write the staging buffer to flash, erasing the block first if it is not yet erased.
 *
 * @return True if succeeded, false if not.
 */
static bool _bluecherry_ota_buffer_to_flash(void)
{
  uint8_t* ota_buffer = _bluecherry_opdata.ota_buffer;
  uint32_t& ota_size = _bluecherry_opdata.ota_size;
  uint32_t& ota_progress = _bluecherry_opdata.ota_progress;

  /* first bytes of new firmware must be postponed so
   * partially written firmware is not bootable just yet
   */
  uint8_t skip = 0;

  if(!ota_progress) {
    /* meanwhile check for the magic byte */
    if(ota_buffer[0] != ESP_IMAGE_HEADER_MAGIC) {
      ESP_LOGD(TAG, "OTA chunk: magic header not found");
      return false;
    }

    skip = ENCRYPTED_BLOCK_SIZE;
    memcpy(_bluecherry_opdata.ota_skip_buffer, ota_buffer, skip);
  }

  size_t flash_offset = _bluecherry_opdata.ota_partition->address + ota_progress;

  // if it's the block boundary, than erase the whole block from here
  bool block_erase = (ota_size - ota_progress >= SPI_FLASH_BLOCK_SIZE) &&
                     (flash_offset % SPI_FLASH_BLOCK_SIZE == 0);

  // sector belong to unaligned partition heading block
  bool partition_head_sectors =
      _bluecherry_opdata.ota_partition->address % SPI_FLASH_BLOCK_SIZE &&
      flash_offset < (_bluecherry_opdata.ota_partition->address / SPI_FLASH_BLOCK_SIZE + 1) *
                         SPI_FLASH_BLOCK_SIZE;

  // sector belong to unaligned partition tailing block
  bool partition_tail_sectors =
      flash_offset >= (_bluecherry_opdata.ota_partition->address + ota_size) /
                          SPI_FLASH_BLOCK_SIZE * SPI_FLASH_BLOCK_SIZE;

  if(block_erase || partition_head_sectors || partition_tail_sectors) {
    if(esp_partition_erase_range(_bluecherry_opdata.ota_partition, ota_progress,
                                 block_erase ? SPI_FLASH_BLOCK_SIZE : SPI_FLASH_SEC_SIZE) !=
       ESP_OK) {
      ESP_LOGE(TAG, "OTA chunk: could not erase partition");
      return false;
    }
  }

  if(esp_partition_write(_bluecherry_opdata.ota_partition, ota_progress + skip, ota_buffer + skip,
                         _bluecherry_opdata.ota_buffer_pos - skip) != ESP_OK) {
    ESP_LOGE(TAG, "OTA chunk: could not write data to partition");
    return false;
  }

  ota_progress += _bluecherry_opdata.ota_buffer_pos;
  _bluecherry_opdata.ota_buffer_pos = 0;

  return true;
}

/**
 * @brief Queue one internal channel (topic 0x00) frame in the priority slot.
 *
 * The slot is checked before out_ring in the send step, so a protocol reply goes out on the very
 * next sync instead of queueing behind the application's publishes. That matters most for the
 * probe reply: any topic 0x00 frame sets want_resync, so the sync task loops again in
 * milliseconds and the answer is back before the server has packed a single chunk.
 *
 * Only one internal event is ever outstanding - each is a reply the server then responds to - so a
 * single slot is enough. A second call overwrites the first.
 *
 * @param payload The event payload, starting with the event type byte.
 * @param len Payload length.
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_SIZE when the framed event does not fit the slot.
 */
static esp_err_t _bluecherry_publish_event(const uint8_t* payload, uint8_t len)
{
  const size_t total = BLUECHERRY_COAP_HEADER_SIZE + BLUECHERRY_MQTT_HEADER_SIZE + len;

  if(total > sizeof(_bluecherry_opdata.pending_event)) {
    ESP_LOGE(TAG, "Internal event of %uB does not fit the priority slot", len);
    return ESP_ERR_INVALID_SIZE;
  }

  uint8_t* p = _bluecherry_opdata.pending_event + BLUECHERRY_COAP_HEADER_SIZE;
  p[0] = 0x00; /* internal channel */
  p[1] = len;
  memcpy(p + BLUECHERRY_MQTT_HEADER_SIZE, payload, len);

  _bluecherry_opdata.pending_event_len = total;
  return ESP_OK;
}

/**
 * @brief Report an OTA event to the application, if it registered a handler.
 *
 * Two gates: the NULL check answers "is anyone watching" and the return value answers "who decides
 * this event". Only the second can differ per event, so a handler registered purely to log cannot
 * accidentally stop a device updating.
 *
 * @param event The event that occurred.
 * @param error_code The error code, for a failure event.
 *
 * @return True when the application took this event's decision, false when the library should
 * apply its own default.
 */
static bool _bluecherry_ota_notify(BlueCherryOtaEvent event, uint8_t error_code)
{
  if(_bluecherry_opdata.ota_handler == NULL) {
    return false;
  }

  BlueCherryOtaInfo info = {};
  info.version = _bluecherry_opdata.ota_target_version;
  info.size = _bluecherry_opdata.ota_size;
  info.bytes_received = _bluecherry_opdata.ota_progress;
  info.error_code = error_code;
  memcpy(info.sha256, _bluecherry_opdata.ota_expected_hash, BLUECHERRY_PARTITION_HASH_LEN);

  return _bluecherry_opdata.ota_handler(event, &info, _bluecherry_opdata.ota_handler_args);
}

/**
 * @brief Flush the staging buffer to flash and report the progress it made.
 *
 * The only place a progress event is emitted: the progress counter advances nowhere but in
 * _bluecherry_ota_buffer_to_flash, so tying the event to arriving chunks instead would repeat the
 * same byte count for a whole sector's worth of them.
 *
 * @return True if the flush succeeded, false if the write failed.
 */
static bool _bluecherry_ota_flush(void)
{
  if(!_bluecherry_ota_buffer_to_flash()) {
    return false;
  }

  ESP_LOGD(TAG, "OTA: %lu / %lu bytes written (%.2f%%)",
           (unsigned long) _bluecherry_opdata.ota_progress,
           (unsigned long) _bluecherry_opdata.ota_size, _bluecherry_ota_progress_percent());
  _bluecherry_ota_notify(BLUECHERRY_OTA_EVENT_PROGRESS, 0);
  return true;
}

/**
 * @brief Clear all OTA transfer state.
 *
 * Every field describing an update goes, not just the transfer counters: a stale target version or
 * unverified flag carried into the next offer would describe the previous one, and the partition
 * would be left pointing at one nothing is writing to.
 *
 * @return None.
 */
static void _bluecherry_ota_reset(void)
{
  _bluecherry_opdata.ota_state = BLUECHERRY_OTA_STATE_IDLE;
  _bluecherry_opdata.ota_size = 0;
  _bluecherry_opdata.ota_progress = 0;
  _bluecherry_opdata.ota_buffer_pos = 0;
  _bluecherry_opdata.ota_target_version = 0;
  _bluecherry_opdata.ota_unverified = false;
  _bluecherry_opdata.ota_resume_due = false;
  _bluecherry_opdata.ota_partition = NULL;
  memset(_bluecherry_opdata.ota_expected_hash, 0, BLUECHERRY_PARTITION_HASH_LEN);
}

/**
 * @brief Abandon the transfer, tell the cloud why, and inform the application.
 *
 * @param error_code A BlueCherryOtaError.
 *
 * @return None.
 */
static void _bluecherry_ota_fail(uint8_t error_code)
{
  ESP_LOGE(TAG, "OTA: failing with code %u", error_code);

  uint8_t payload[3] = { BLUECHERRY_EVENT_TYPE_OTA_ERROR,
                         (uint8_t) _bluecherry_opdata.ota_target_version, error_code };
  _bluecherry_publish_event(payload, sizeof(payload));

  _bluecherry_ota_notify(BLUECHERRY_OTA_EVENT_FAILED, error_code);
  _bluecherry_ota_reset();
}

/**
 * @brief Report a modem firmware update failure to the cloud.
 *
 * Exactly ONE byte. A two byte reply is the probe answer announcing the OTA protocol this client
 * speaks, and the length is the only thing that distinguishes the two, so a modem firmware failure
 * reported with two bytes would be read as a protocol announcement.
 *
 * @return None.
 */
static void _bluecherry_mota_fail(void)
{
  uint8_t payload[1] = { BLUECHERRY_EVENT_TYPE_ERROR };
  _bluecherry_publish_event(payload, sizeof(payload));
}

/**
 * @brief Accept the offered update and ask the server to start sending it.
 *
 * Runs on the synchronisation task: it writes the priority slot that the task transmits out of,
 * and a second writer there can splice a frame that is mid flight across its retransmits.
 *
 * @return None.
 */
static void _bluecherry_ota_begin(void)
{
  /* The authoritative check: the offer can be withdrawn between the request and this running. */
  if(_bluecherry_opdata.ota_state != BLUECHERRY_OTA_STATE_OFFERED) {
    ESP_LOGW(TAG, "OTA: the offer was withdrawn before the update could start");
    return;
  }

  uint8_t payload[2] = { BLUECHERRY_EVENT_TYPE_OTA_START,
                         (uint8_t) _bluecherry_opdata.ota_target_version };
  if(_bluecherry_publish_event(payload, sizeof(payload)) != ESP_OK) {
    return;
  }

  _bluecherry_opdata.ota_state = BLUECHERRY_OTA_STATE_DOWNLOADING;
  _bluecherry_opdata.ota_progress = 0;
  _bluecherry_opdata.ota_buffer_pos = 0;

  ESP_LOGI(TAG, "OTA: requesting firmware v%d (%lu bytes)", _bluecherry_opdata.ota_target_version,
           (unsigned long) _bluecherry_opdata.ota_size);
  _bluecherry_ota_notify(BLUECHERRY_OTA_EVENT_STARTED, 0);
}

/**
 * @brief Keep an interrupted download so it can resume from what is on flash.
 *
 * Only a download that already received a chunk is kept, since only then had the cloud committed
 * to it. The staged bytes are dropped: flash ends on a sector boundary while downloading, so the
 * cloud resends from there.
 *
 * @return True when the download is kept, false when there is nothing to resume.
 */
static bool _bluecherry_ota_prepare_resume(void)
{
  if(_bluecherry_opdata.ota_state != BLUECHERRY_OTA_STATE_RESUMING &&
     (_bluecherry_opdata.ota_state != BLUECHERRY_OTA_STATE_DOWNLOADING ||
      _bluecherry_opdata.ota_progress + _bluecherry_opdata.ota_buffer_pos == 0)) {
    return false;
  }

  _bluecherry_opdata.ota_buffer_pos = 0;
  _bluecherry_opdata.ota_state = BLUECHERRY_OTA_STATE_RESUMING;
  _bluecherry_opdata.ota_resume_due = true;

  ESP_LOGI(TAG, "OTA: resuming firmware v%d at %lu bytes", _bluecherry_opdata.ota_target_version,
           (unsigned long) _bluecherry_opdata.ota_progress);
  return true;
}

/**
 * @brief Tell the cloud how much of the interrupted download is already written.
 *
 * ota_resume_due stays set until the cloud acknowledges the RESUME.
 *
 * @return None.
 */
static void _bluecherry_ota_queue_resume(void)
{
  const uint32_t offset = _bluecherry_opdata.ota_progress;
  uint8_t payload[6];
  size_t n = 0;

  payload[n++] = BLUECHERRY_EVENT_TYPE_OTA_RESUME;
  payload[n++] = (uint8_t) _bluecherry_opdata.ota_target_version;
  payload[n++] = offset & 0xFF;
  payload[n++] = (offset >> 8) & 0xFF;
  payload[n++] = (offset >> 16) & 0xFF;
  payload[n++] = (offset >> 24) & 0xFF;

  _bluecherry_publish_event(payload, (uint8_t) n);
}

/**
 * @brief Carry out whatever otaStart or otaAbort asked for.
 *
 * Run before the send step, so the resulting event goes out in the same cycle.
 *
 * @return None.
 */
static void _bluecherry_ota_service_requests(void)
{
  if(_ota_start_req) {
    _ota_start_req = false;
    _bluecherry_ota_begin();
  }

  if(_ota_abort_req) {
    _ota_abort_req = false;
    if(_bluecherry_opdata.ota_state != BLUECHERRY_OTA_STATE_IDLE) {
      _bluecherry_ota_fail(_ota_abort_code);
    }
  }
}

/**
 * @brief Finish the image and report it verified.
 *
 * Ordering is load-bearing, not incidental:
 *
 *   1. write the withheld 16 byte header  -> image complete, still NOT bootable
 *   2. hash the partition and compare     -> mismatch: report and give up
 *   3. send VERIFIED                      -> committed only once acked
 *
 * The boot partition is NOT set here but in _bluecherry_ota_commit, once the server has
 * acknowledged the VERIFIED, so an unexpected reset in between boots the OLD image and the server
 * simply retries. Committing first is what lets a device reboot into firmware the cloud never
 * learned about.
 *
 * The hash is read back from flash rather than accumulated over the arriving bytes, so it attests
 * to what is actually stored, and it is exactly the SHA-256 ESP-IDF appends to the image - the
 * same value the cloud records as the fingerprint of that build.
 *
 * @return None.
 */
static void _bluecherry_ota_verify(void)
{
  /* 1. Enable the partition: write the stashed first bytes. */
  if(esp_partition_write(_bluecherry_opdata.ota_partition, 0, _bluecherry_opdata.ota_skip_buffer,
                         ENCRYPTED_BLOCK_SIZE) != ESP_OK) {
    ESP_LOGE(TAG, "OTA: could not write the image header");
    _bluecherry_ota_fail(BLUECHERRY_OTA_ERR_WRITE_FAILED);
    return;
  }

  uint8_t header[ENCRYPTED_BLOCK_SIZE];
  if(esp_partition_read(_bluecherry_opdata.ota_partition, 0, header, ENCRYPTED_BLOCK_SIZE) !=
     ESP_OK) {
    ESP_LOGE(TAG, "OTA: could not read back the image header");
    _bluecherry_ota_fail(BLUECHERRY_OTA_ERR_WRITE_FAILED);
    return;
  }
  if(header[0] != ESP_IMAGE_HEADER_MAGIC) {
    ESP_LOGE(TAG, "OTA: magic header missing on the partition");
    _bluecherry_ota_fail(BLUECHERRY_OTA_ERR_BAD_MAGIC);
    return;
  }

  /* 2. Hash what is actually in flash. */
  uint8_t actual[BLUECHERRY_PARTITION_HASH_LEN];
  if(esp_partition_get_sha256(_bluecherry_opdata.ota_partition, actual) != ESP_OK) {
    ESP_LOGE(TAG, "OTA: could not hash the written partition");
    _bluecherry_ota_fail(BLUECHERRY_OTA_ERR_HASH_MISMATCH);
    return;
  }

  if(!_bluecherry_opdata.ota_unverified &&
     memcmp(actual, _bluecherry_opdata.ota_expected_hash, BLUECHERRY_PARTITION_HASH_LEN) != 0) {
    ESP_LOGE(TAG, "OTA: hash mismatch, the image in flash is not the one announced");
    _bluecherry_ota_fail(BLUECHERRY_OTA_ERR_HASH_MISMATCH);
    return;
  }

  /* 3. Report it. In unverified mode there was nothing to compare against, but the hash is still
   * sent: it is the value an operator needs to record as the build's fingerprint and turn this
   * into a verified update. */
  uint8_t payload[2 + BLUECHERRY_PARTITION_HASH_LEN];
  payload[0] = BLUECHERRY_EVENT_TYPE_OTA_VERIFIED;
  payload[1] = (uint8_t) _bluecherry_opdata.ota_target_version;
  memcpy(payload + 2, actual, BLUECHERRY_PARTITION_HASH_LEN);

  if(_bluecherry_publish_event(payload, sizeof(payload)) != ESP_OK) {
    _bluecherry_ota_fail(BLUECHERRY_OTA_ERR_WRITE_FAILED);
    return;
  }

  _bluecherry_opdata.ota_state = BLUECHERRY_OTA_STATE_AWAITING_VERIFIED;
  ESP_LOGI(TAG, "OTA: image verified, reporting to the cloud before committing");
}

/**
 * @brief Commit the new image once the cloud has acknowledged the VERIFIED.
 *
 * Called from the send step, which knows the message was acknowledged because _bluecherry_coap_rxtx
 * only returns ESP_OK on an ACK. This is the single irreversible step in the whole flow.
 *
 * @return None.
 */
static void _bluecherry_ota_commit(void)
{
  if(esp_ota_set_boot_partition(_bluecherry_opdata.ota_partition) != ESP_OK) {
    ESP_LOGE(TAG, "OTA: could not set the boot partition");
    _bluecherry_ota_fail(BLUECHERRY_OTA_ERR_SET_BOOT_FAILED);
    return;
  }

  _bluecherry_opdata.ota_state = BLUECHERRY_OTA_STATE_COMPLETE;
  ESP_LOGI(TAG, "OTA: firmware v%d installed and acknowledged; boot partition set",
           _bluecherry_opdata.ota_target_version);

  /* Nobody watching, or watching without taking the decision: reboot now. An application that
   * says it owns the moment keeps running the old firmware until it reboots, which is the point
   * of saying so. */
  if(!_bluecherry_ota_notify(BLUECHERRY_OTA_EVENT_COMPLETE, 0)) {
    ESP_LOGI(TAG, "OTA: rebooting into the new firmware");
    esp_restart();
  }
}

/**
 * @brief Map a partition to a platform neutral OTA slot index.
 *
 * ESP subtypes OTA_0..OTA_15 become 0..15; a factory or test partition, or no partition at all,
 * becomes BLUECHERRY_OTA_SLOT_NONE. The wire carries the index rather than the subtype, so
 * reporting it does not require the reader to understand an ESP specific encoding.
 *
 * @param part The partition to map, or NULL.
 *
 * @return The slot index, or BLUECHERRY_OTA_SLOT_NONE.
 */
static uint8_t _bluecherry_ota_slot_index(const esp_partition_t* part)
{
  if(part == NULL || part->subtype < ESP_PARTITION_SUBTYPE_APP_OTA_MIN ||
     part->subtype >= ESP_PARTITION_SUBTYPE_APP_OTA_MAX) {
    return BLUECHERRY_OTA_SLOT_NONE;
  }
  return (uint8_t) (part->subtype - ESP_PARTITION_SUBTYPE_APP_OTA_MIN);
}

/**
 * @brief Why the device last booted, as a string.
 *
 * A string rather than the raw esp_reset_reason_t, so the report is self describing and the cloud
 * needs no lookup table for it - the numeric values behind these names are not portable.
 *
 * @return The reset reason.
 */
static const char* _bluecherry_reset_reason_str(void)
{
  switch(esp_reset_reason()) {
  case ESP_RST_POWERON:
    return "poweron";
  case ESP_RST_EXT:
    return "ext";
  case ESP_RST_SW:
    return "sw";
  case ESP_RST_PANIC:
    return "panic";
  case ESP_RST_INT_WDT:
    return "int_wdt";
  case ESP_RST_TASK_WDT:
    return "task_wdt";
  case ESP_RST_WDT:
    return "wdt";
  case ESP_RST_DEEPSLEEP:
    return "deepsleep";
  case ESP_RST_BROWNOUT:
    return "brownout";
  case ESP_RST_SDIO:
    return "sdio";
  default:
    return "unknown";
  }
}

/**
 * @brief Append a [1B length][UTF-8] INIT_INFO field, or drop it if it will not fit.
 *
 * Dropping is the safe failure: a cleared bit is something the server renders correctly, whereas a
 * half written string would misalign every field after it - the parser locates fields positionally
 * and cannot resync.
 *
 * @param buf The payload buffer.
 * @param cap The capacity of the payload buffer.
 * @param n The current write position, advanced on success.
 * @param s The string to append.
 *
 * @return True when the field was written, so the caller can set its presence bit only then.
 */
static bool _bluecherry_info_add_str(uint8_t* buf, size_t cap, size_t* n, const char* s)
{
  size_t len = strnlen(s, BLUECHERRY_INFO_STR_MAX);
  if(*n + 1 + len > cap) {
    ESP_LOGW(TAG, "INIT_INFO field \"%s\" dropped: only %uB left", s, (unsigned) (cap - *n));
    return false;
  }
  buf[(*n)++] = (uint8_t) len;
  memcpy(buf + *n, s, len);
  *n += len;
  return true;
}

/**
 * @brief Queue INIT_INFO: the running partition hash plus optional details.
 *
 * Queued once per boot, as the first thing a session carries: the contents cannot change while
 * the device runs, and it is how the server learns which image a device came up on.
 *
 * Fields MUST be written in ascending presence bit order: the server decodes positionally and
 * cannot recover from a field out of place.
 *
 * @return None.
 */
static void _bluecherry_send_init_info(void)
{
  const esp_partition_t* running = esp_ota_get_running_partition();
  if(running == NULL) {
    ESP_LOGW(TAG, "No running partition; skipping INIT_INFO");
    return;
  }

  /* Sized from the priority slot's payload budget rather than by adding the fields up. Only the
   * STRING fields are bounds checked, by _bluecherry_info_add_str; every fixed width field is
   * written straight in, and the two ota_slot bytes land after two strings that may each be
   * BLUECHERRY_INFO_STR_MAX long - so what has to hold unconditionally is that the whole unchecked
   * path fits even at those maxima. Adding a third string BEFORE ota_slot breaks the assumption,
   * not just the total. */
  uint8_t payload[BLUECHERRY_EVENT_PAYLOAD_MAX];
  static_assert(1 + 1 + BLUECHERRY_PARTITION_HASH_LEN + 2 /* event, schema, hash, bitmap */
                        + 1 + 3 + 4 + 4 + 4 /* platform, version, slot size, uptime, heap */
                        + 2                 /* ota_slot, written after the strings */
                        + 2 * (1 + BLUECHERRY_INFO_STR_MAX) /* lib_name, mcu */
                    <= BLUECHERRY_EVENT_PAYLOAD_MAX,
                "INIT_INFO's unchecked writes must fit the event payload budget even when "
                "every preceding string field is at its maximum length");
  size_t n = 0;

  payload[n++] = BLUECHERRY_EVENT_TYPE_INIT_INFO;
  payload[n++] = BLUECHERRY_INIT_INFO_SCHEMA;

  if(esp_partition_get_sha256(running, payload + n) != ESP_OK) {
    ESP_LOGW(TAG, "Could not hash the running partition; skipping INIT_INFO");
    return;
  }
  n += BLUECHERRY_PARTITION_HASH_LEN;

  /* The bitmap has to be written before the fields it describes, so the strings are measured
   * first and their bits only set once they are known to fit. */
  const size_t bitmap_at = n;
  n += 2;

  uint16_t present = BLUECHERRY_INFO_BIT_PLATFORM | BLUECHERRY_INFO_BIT_LIB_VERSION |
                     BLUECHERRY_INFO_BIT_OTA_SLOT_SIZE | BLUECHERRY_INFO_BIT_UPTIME |
                     BLUECHERRY_INFO_BIT_TOTAL_HEAP | BLUECHERRY_INFO_BIT_OTA_SLOT;

  /* Optional fields, in ascending bit order. */
#ifdef ARDUINO
  payload[n++] = BLUECHERRY_PLATFORM_ARDUINO;
#else
  payload[n++] = BLUECHERRY_PLATFORM_ESP_IDF;
#endif

  payload[n++] = BLUECHERRY_LIB_VERSION_MAJOR;
  payload[n++] = BLUECHERRY_LIB_VERSION_MINOR;
  payload[n++] = BLUECHERRY_LIB_VERSION_PATCH;

  const esp_partition_t* slot = esp_ota_get_next_update_partition(NULL);
  const uint32_t slot_size = slot ? slot->size : 0;
  payload[n++] = slot_size & 0xFF;
  payload[n++] = (slot_size >> 8) & 0xFF;
  payload[n++] = (slot_size >> 16) & 0xFF;
  payload[n++] = (slot_size >> 24) & 0xFF;

  const uint32_t uptime = (uint32_t) (esp_timer_get_time() / 1000000);
  payload[n++] = uptime & 0xFF;
  payload[n++] = (uptime >> 8) & 0xFF;
  payload[n++] = (uptime >> 16) & 0xFF;
  payload[n++] = (uptime >> 24) & 0xFF;

  /* Total, not free: free heap moves with whatever the application has allocated, so it cannot be
   * compared between devices or over time. Total is the chip's internal RAM capacity, which is
   * what tells you how much room there ever was. */
  const uint32_t heap = (uint32_t) heap_caps_get_total_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  payload[n++] = heap & 0xFF;
  payload[n++] = (heap >> 8) & 0xFF;
  payload[n++] = (heap >> 16) & 0xFF;
  payload[n++] = (heap >> 24) & 0xFF;

  if(_bluecherry_info_add_str(payload, sizeof(payload), &n, BLUECHERRY_LIB_NAME)) {
    present |= BLUECHERRY_INFO_BIT_LIB_NAME;
  }

  if(_bluecherry_info_add_str(payload, sizeof(payload), &n, BLUECHERRY_MCU)) {
    present |= BLUECHERRY_INFO_BIT_MCU;
  }

  payload[n++] = _bluecherry_ota_slot_index(running);
  payload[n++] = _bluecherry_ota_slot_index(slot);

  if(_bluecherry_info_add_str(payload, sizeof(payload), &n, _bluecherry_reset_reason_str())) {
    present |= BLUECHERRY_INFO_BIT_RESET_REASON;
  }

  payload[bitmap_at] = present & 0xFF;
  payload[bitmap_at + 1] = (present >> 8) & 0xFF;

  if(_bluecherry_publish_event(payload, (uint8_t) n) == ESP_OK) {
    ESP_LOGD(TAG, "INIT_INFO queued (%uB)", (unsigned) n);
  }
}

/**
 * @brief Process an OTA initialize event: an update is on offer.
 *
 * Payload: [version(1)][size(4, LE)][sha256(32)][chunk size(1)] = 38 bytes after the event type
 * byte.
 *
 * @param data The event data.
 * @param len The length of the event data.
 *
 * @return None.
 */
static void _bluecherry_ota_process_initialize(uint8_t* data, uint16_t len)
{
  if(len != 38) {
    ESP_LOGE(TAG, "OTA: initialize expected 38B, got %uB", len);
    return;
  }

  if(_bluecherry_opdata.ota_state == BLUECHERRY_OTA_STATE_RESUMING) {
    ESP_LOGI(TAG, "OTA: the cloud restarted the update");
    _bluecherry_opdata.ota_resume_due = false;
  } else if(_bluecherry_opdata.ota_state != BLUECHERRY_OTA_STATE_IDLE &&
            _bluecherry_opdata.ota_state != BLUECHERRY_OTA_STATE_OFFERED) {
    ESP_LOGW(TAG, "OTA: already busy, ignoring re-offer");
    return;
  }

  _bluecherry_opdata.ota_partition = esp_ota_get_next_update_partition(NULL);
  if(!_bluecherry_opdata.ota_partition) {
    _bluecherry_opdata.ota_target_version = (int8_t) data[0];
    _bluecherry_ota_fail(BLUECHERRY_OTA_ERR_NO_PARTITION);
    return;
  }

  _bluecherry_opdata.ota_target_version = (int8_t) data[0];
  _bluecherry_opdata.ota_size = ((uint32_t) data[1]) | ((uint32_t) data[2] << 8) |
                                ((uint32_t) data[3] << 16) | ((uint32_t) data[4] << 24);
  memcpy(_bluecherry_opdata.ota_expected_hash, data + 5, BLUECHERRY_PARTITION_HASH_LEN);

  /* An all zero expected hash means the cloud has no fingerprint on record, so there is nothing to
   * check the image against. The hash is still computed and reported, since that is the value an
   * operator needs to record one. */
  _bluecherry_opdata.ota_unverified = true;
  for(size_t i = 0; i < BLUECHERRY_PARTITION_HASH_LEN; ++i) {
    if(_bluecherry_opdata.ota_expected_hash[i] != 0) {
      _bluecherry_opdata.ota_unverified = false;
      break;
    }
  }

  if(_bluecherry_opdata.ota_size == 0 ||
     _bluecherry_opdata.ota_size > _bluecherry_opdata.ota_partition->size) {
    ESP_LOGE(TAG, "OTA: %lu bytes will not fit a %lu byte slot",
             (unsigned long) _bluecherry_opdata.ota_size,
             (unsigned long) _bluecherry_opdata.ota_partition->size);
    _bluecherry_ota_fail(BLUECHERRY_OTA_ERR_TOO_LARGE);
    return;
  }

  _bluecherry_opdata.ota_progress = 0;
  _bluecherry_opdata.ota_buffer_pos = 0;
  _bluecherry_opdata.ota_state = BLUECHERRY_OTA_STATE_OFFERED;

  ESP_LOGI(TAG, "OTA: firmware v%d offered, %lu bytes, %s", _bluecherry_opdata.ota_target_version,
           (unsigned long) _bluecherry_opdata.ota_size,
           _bluecherry_opdata.ota_unverified ? "UNVERIFIED (no fingerprint)" : "verified");

  /* Nobody watching, or watching without taking the decision: start now. An application only gets
   * to choose the moment by saying so, so forgetting to decide cannot leave a device sitting on an
   * update forever. */
  if(!_bluecherry_ota_notify(BLUECHERRY_OTA_EVENT_AVAILABLE, 0)) {
    WalterBlueCherry::otaStart();
  }
}

/**
 * @brief Process an OTA chunk: stage it, and flush a sector at a time.
 *
 * The DOWNLOADING guard is what protects a finished image: once the last chunk has been staged and
 * hashed the state moves to AWAITING_VERIFIED, so a chunk arriving after it - a duplicated frame
 * on a lossy link - is dropped here instead of tripping the overrun check below, which would
 * overwrite the queued VERIFIED in the single priority slot with an error and discard a correctly
 * written image.
 *
 * @param data The chunk data.
 * @param len The length of the chunk data.
 *
 * @return None.
 */
static void _bluecherry_ota_process_chunk(uint8_t* data, uint16_t len)
{
  if(_bluecherry_opdata.ota_state == BLUECHERRY_OTA_STATE_RESUMING) {
    /* Chunks the cloud sent before it got the RESUME belong to another offset. */
    if(_bluecherry_opdata.ota_resume_due) {
      ESP_LOGD(TAG, "OTA: chunk from before the resume, ignoring");
      return;
    }
    ESP_LOGI(TAG, "OTA: download resumed at %lu bytes",
             (unsigned long) _bluecherry_opdata.ota_progress);
    _bluecherry_opdata.ota_state = BLUECHERRY_OTA_STATE_DOWNLOADING;
  }

  if(_bluecherry_opdata.ota_state != BLUECHERRY_OTA_STATE_DOWNLOADING) {
    ESP_LOGW(TAG, "OTA: chunk outside a download, ignoring");
    return;
  }

  uint8_t* ota_buffer = _bluecherry_opdata.ota_buffer;
  uint32_t& ota_size = _bluecherry_opdata.ota_size;
  uint32_t& ota_progress = _bluecherry_opdata.ota_progress;

  if(len == 0 || ota_progress + _bluecherry_opdata.ota_buffer_pos + len > ota_size) {
    ESP_LOGE(TAG, "OTA: chunk empty or beyond the announced size");
    _bluecherry_ota_fail(BLUECHERRY_OTA_ERR_CHUNK_OVERRUN);
    return;
  }

  size_t left = len;

  while((_bluecherry_opdata.ota_buffer_pos + left) > SPI_FLASH_SEC_SIZE) {
    size_t to_buff = SPI_FLASH_SEC_SIZE - _bluecherry_opdata.ota_buffer_pos;

    memcpy(ota_buffer + _bluecherry_opdata.ota_buffer_pos, data + (len - left), to_buff);
    _bluecherry_opdata.ota_buffer_pos += to_buff;

    if(!_bluecherry_ota_flush()) {
      _bluecherry_ota_fail(BLUECHERRY_OTA_ERR_WRITE_FAILED);
      return;
    }

    left -= to_buff;
  }

  memcpy(ota_buffer + _bluecherry_opdata.ota_buffer_pos, data + (len - left), left);
  _bluecherry_opdata.ota_buffer_pos += left;

  /* The last sector is short of the flush threshold above, so the final flush is triggered by the
   * byte count instead - and it is the one that reports 100%. No progress event is emitted for a
   * chunk that only staged bytes: nothing observable changed. */
  if(ota_progress + _bluecherry_opdata.ota_buffer_pos == ota_size) {
    if(!_bluecherry_ota_flush()) {
      _bluecherry_ota_fail(BLUECHERRY_OTA_ERR_WRITE_FAILED);
      return;
    }
    ESP_LOGI(TAG, "OTA: %lu bytes received, verifying", (unsigned long) ota_progress);
    _bluecherry_ota_verify();
  }
}

/**
 * @brief Process an incoming BlueCherry event.
 *
 * Called for every record that arrives on topic byte 0x00, BlueCherry's internal channel.
 *
 * Events 1, 2 and 3 are OTA messages this client does not implement. Event 1 is answered with the
 * [4][0xB2] probe reply and nothing else; 2 and 3 are discarded outright. Discarding them is not
 * laziness - a handful can still be in flight in the window before the cloud's switch takes
 * effect, and writing them would corrupt the partition.
 *
 * @param data The event data.
 * @param len The length of the data block.
 *
 * @return None.
 */
static void _bluecherry_process_event(uint8_t* data, uint8_t len)
{
  if(len == 0) {
    ESP_LOGW(TAG, "Empty BlueCherry event, ignoring");
    return;
  }

  switch(data[0]) {
  case BLUECHERRY_EVENT_TYPE_OTA_PROBE: {
    /* While resuming, the RESUME is the answer. */
    if(_bluecherry_opdata.ota_state == BLUECHERRY_OTA_STATE_RESUMING) {
      ESP_LOGD(TAG, "OTA probe from the cloud, answered with a resume");
      _bluecherry_opdata.ota_resume_due = true;
      _bluecherry_ota_queue_resume();
      break;
    }

    /* Answer it and touch no OTA state: the cloud has not yet been told what we speak, and
     * treating this as an offer would clobber a transfer that may already be running. */
    ESP_LOGD(TAG, "OTA probe from the cloud, answered");
    uint8_t reply[2] = { BLUECHERRY_EVENT_TYPE_ERROR, BLUECHERRY_OTA_ERROR_UNSUPPORTED_PROTOCOL };
    _bluecherry_publish_event(reply, sizeof(reply));
    break;
  }

  case BLUECHERRY_EVENT_TYPE_OTA_UNSUPPORTED_CHUNK:
  case BLUECHERRY_EVENT_TYPE_OTA_UNSUPPORTED_FINISH:
    /* Leftovers from the probe window. Discard them. */
    ESP_LOGD(TAG, "Ignoring unsupported OTA event 0x%x", data[0]);
    break;

#if CONFIG_WALTER_MODEM_ENABLE_MOTA
  /* The modem firmware update, which is unique to Walter and belongs to the modem driver. */
  case BLUECHERRY_EVENT_TYPE_MOTA_INITIALIZE:
  case BLUECHERRY_EVENT_TYPE_MOTA_CHUNK:
  case BLUECHERRY_EVENT_TYPE_MOTA_FINISH:
    if(WalterBlueCherry::_motaDispatch(data[0], data + 1, len - 1)) {
      _bluecherry_mota_fail();
    }
    break;
#endif

  case BLUECHERRY_EVENT_TYPE_OTA_INITIALIZE:
    _bluecherry_ota_process_initialize(data + 1, len - 1);
    break;

  case BLUECHERRY_EVENT_TYPE_OTA_CHUNK:
    _bluecherry_ota_process_chunk(data + 1, len - 1);
    break;

  default:
    /* Benign on purpose. An unknown event must never abort a running update. */
    ESP_LOGW(TAG, "Ignoring unknown BlueCherry event type 0x%x from cloud server", data[0]);
    break;
  }
}

#pragma endregion
#pragma region TRANSPORT

/**
 * @brief Close the modem socket carrying the session, if there is one.
 *
 * @return None.
 */
static void _bluecherry_cleanup_network(void)
{
  if(_bluecherry_opdata.sock > 0) {
    WalterModem::socketClose(_bluecherry_opdata.sock);
    _bluecherry_opdata.sock = -1;
  }
}

/**
 * @brief Drop the session and everything describing it, ready for a fresh connect.
 *
 * @return None.
 */
static void _bluecherry_cleanup_session(void)
{
  _bluecherry_cleanup_network();

  if(_bc_rx_buf != NULL) {
    xMessageBufferReset(_bc_rx_buf);
  }
  _bc_peer_closed = false;
  _bluecherry_opdata.in_buf_len = 0;
}

/**
 * @brief Read one datagram from the BlueCherry socket.
 *
 * Stands in for the Mbed TLS read of the reference client, and reports the same three outcomes so
 * that the retransmission loops above can be used unchanged: a positive byte count,
 * MBEDTLS_ERR_SSL_TIMEOUT when nothing arrived, and any other negative value for a hard failure.
 *
 * The event task reads every datagram out of the modem the moment it is announced, so this only
 * takes the next one from the receive buffer, the way recv() takes it from a socket's on WiFi.
 *
 * @param buf The buffer to read into.
 * @param len The capacity of the buffer, at least BLUECHERRY_MAX_INCOMING_MESSAGE_LEN: a datagram
 * that does not fit is left where it is and never handed over.
 *
 * @return The number of bytes read, or a negative Mbed TLS error code.
 */
static int _bluecherry_mbed_dtls_read(unsigned char* buf, size_t len)
{
  _bluecherry_park_for_sleep();
  _bluecherry_tickle_watchdog();

  if(_bc_peer_closed) {
    _bc_peer_closed = false;
    ESP_LOGE(TAG, "The BlueCherry socket was closed by the modem");
    return MBEDTLS_ERR_NET_RECV_FAILED;
  }

  if(_bc_rx_buf == NULL || _bluecherry_opdata.sock <= 0) {
    return MBEDTLS_ERR_NET_RECV_FAILED;
  }

  /* The buffer's own wait replaces the poll loop of the reference client. The timeout is the same
   * one its Mbed TLS configuration used, so the caller's retransmit accounting is unchanged. */
  size_t got =
      xMessageBufferReceive(_bc_rx_buf, buf, len, pdMS_TO_TICKS(BLUECHERRY_SSL_READ_TIMEOUT));
  return got > 0 ? (int) got : MBEDTLS_ERR_SSL_TIMEOUT;
}

/**
 * @brief Write one datagram to the modem.
 *
 * The poll loop of the reference client collapses here: a socket send is a blocking AT command
 * that either got its OK or did not.
 *
 * @param buf The bytes to send.
 * @param len The number of bytes to send.
 *
 * @return The number of bytes written, or a negative Mbed TLS error code.
 */
static int _bluecherry_mbed_dtls_write(const unsigned char* buf, size_t len)
{
  _bluecherry_park_for_sleep();
  _bluecherry_tickle_watchdog();

  if(_bluecherry_opdata.sock <= 0) {
    return MBEDTLS_ERR_NET_SEND_FAILED;
  }

  /* Every frame carries its CoAP message id in bytes 2 and 3. */
  if(len >= 4) {
    _bluecherry_opdata.sent_message_id = (uint16_t) ((buf[2] << 8) | buf[3]);
  }

  if(!WalterModem::socketSend(_bluecherry_opdata.sock, (uint8_t*) buf, (uint16_t) len,
                              WALTER_MODEM_RAI_NO_INFO)) {
    ESP_LOGE(TAG, "Could not write to the BlueCherry cloud connection");
    return MBEDTLS_ERR_NET_SEND_FAILED;
  }

  return (int) len;
}

/**
 * @brief Open a DTLS secured UDP socket to the BlueCherry cloud.
 *
 * The modem performs the name resolution, the UDP bind and the DTLS handshake inside the dial, so
 * there is no separate handshake step and nothing to poll. DTLS rather than TLS follows from the
 * socket being dialled UDP; the security itself is the per socket flag and the TLS profile, which
 * names the NVM slots holding the credentials rather than carrying them.
 *
 * The handshake budget is passed as the socket connection timeout, so the same number is enforced
 * one layer down instead of by a deadline here.
 *
 * @param host The hostname of the server.
 * @param port The UDP port to dial.
 *
 * @return True on success, false on error.
 */
static bool _bluecherry_dtls_connect(const char* host, uint16_t port)
{
  _bluecherry_cleanup_session();

  int sock_id = WalterBlueCherry::_reserveSocket();
  if(sock_id < 0) {
    ESP_LOGE(TAG, "No modem socket is available for BlueCherry");
    return false;
  }
  _bluecherry_opdata.sock = sock_id;

  _bluecherry_tickle_watchdog();

  if(!WalterModem::socketConfig(sock_id, _bluecherry_opdata.pdp_ctx_id, BLUECHERRY_SOCKET_MTU,
                                BLUECHERRY_SOCKET_EXCHANGE_TIMEOUT,
                                BLUECHERRY_HANDSHAKE_TIMEOUT_SEC,
                                BLUECHERRY_SOCKET_SEND_DELAY_MS)) {
    ESP_LOGE(TAG, "Could not configure the BlueCherry socket");
    goto fail;
  }

  /* The ring mode has to stay on the default, which reports how many bytes arrived: without the
   * length the ring notification says nothing the receive path can act on. */
  if(!WalterModem::socketConfigExtended(sock_id)) {
    ESP_LOGE(TAG, "Could not configure the BlueCherry socket ring mode");
    goto fail;
  }

  if(!WalterModem::socketConfigSecure(sock_id, true, _bluecherry_opdata.tls_profile_id)) {
    ESP_LOGE(TAG, "Could not enable DTLS on the BlueCherry socket");
    goto fail;
  }

  _bluecherry_tickle_watchdog();

  if(!WalterModem::socketDial(sock_id, WALTER_MODEM_SOCKET_PROTO_UDP, port, host)) {
    ESP_LOGE(TAG, "Could not dial %s:%u", host, port);
    goto fail;
  }

  _bluecherry_tickle_watchdog();

  xMessageBufferReset(_bc_rx_buf);
  _bc_peer_closed = false;

  ESP_LOGI(TAG, "Connected to %s:%u on socket %d", host, port, sock_id);
  return true;

fail:
  _bluecherry_cleanup_session();
  return false;
}

#pragma endregion
#pragma region COAP

/**
 * @brief Parse the type and message ID from a received CoAP packet.
 *
 * @param buf Pointer to the input packet buffer.
 * @param len Number of bytes in the packet buffer.
 * @param type Output pointer for the packet type.
 * @param msg_id Output pointer for the parsed message ID.
 *
 * @return ESP_OK if parsing succeeded.
 */
static esp_err_t _bluecherry_parse_ack_meta(const uint8_t* buf, size_t len, uint8_t* type,
                                            uint16_t* msg_id)
{
  if(len < 4) {
    return ESP_ERR_INVALID_SIZE;
  }

  size_t offset = 0;
  uint8_t header = buf[offset++];
  uint8_t version = (header >> 6) & 0x03;
  if(version != 1) {
    return ESP_ERR_INVALID_VERSION;
  }

  *type = (header >> 4) & 0x03;
  uint8_t token_len = header & 0x0F;

  if(len < (size_t) (4 + token_len)) {
    return ESP_ERR_INVALID_SIZE;
  }

  offset += token_len;
  offset++; // code

  *msg_id = buf[offset++];
  *msg_id <<= 8;
  *msg_id |= buf[offset++];

  return ESP_OK;
}

/**
 * @brief Perform a CoAP transmit and receive round trip with the BlueCherry cloud.
 *
 * Fills in the CoAP header the message reserved room for and runs the confirmable exchange,
 * retransmitting on the BLUECHERRY_ACK_TIMEOUT schedule until the acknowledgement for this exact
 * message id arrives.
 *
 * A packet that is not that acknowledgement is read out and discarded without being parsed. Per
 * RFC 7252 an acknowledgement repeating a message id already handled within the session is a
 * retransmission, or the answer to one, and both ends treat "same message id, same payload" as
 * binding - so discarding it is correct, and reading it out is what keeps it from being handed
 * over again in place of the next exchange's answer.
 *
 * @param msg The message to send, or NULL to send an empty sync packet.
 *
 * @return ESP_OK on success.
 */
static esp_err_t _bluecherry_coap_rxtx(_bluecherry_msg_t* msg)
{
  uint8_t no_payload_hdr[BLUECHERRY_COAP_HEADER_SIZE];
  uint8_t* data = msg == NULL ? no_payload_hdr : msg->data;
  size_t data_len = msg == NULL ? BLUECHERRY_COAP_HEADER_SIZE : msg->len;

  if(data_len < BLUECHERRY_COAP_HEADER_SIZE) {
    ESP_LOGE(TAG, "Cannot send CoAP message smaller than %uB", BLUECHERRY_COAP_HEADER_SIZE);
    return ESP_ERR_NO_MEM;
  }

  uint16_t tx_message_id = _bluecherry_opdata.cur_message_id + 1;
  if(tx_message_id == 0) {
    tx_message_id = 1;
  }

  uint8_t missed_msg_count =
      (uint8_t) (tx_message_id - _bluecherry_opdata.last_acked_message_id - 1);

  /* Two deliberate departures from CoAP, both of which the server depends on:
   *
   *   - byte 1 is the CoAP Code, repurposed as this device's lost message counter. The server
   *     reads it as nr_lost to decide whether to replay the previous frame or pop new messages,
   *     and closes the session at 250.
   *   - byte 4 is the 0xFF payload marker, written even when there is no payload. Real CoAP omits
   *     it; this framing always expects it. */
  data[0] = 0x40; /* CON, TKL=0 */
  data[1] = missed_msg_count;
  data[2] = tx_message_id >> 8;
  data[3] = tx_message_id & 0xFF;
  data[4] = 0xFF;

  double timeout = BLUECHERRY_ACK_TIMEOUT *
                   (1 + (esp_random() / 4294967296.0) * (BLUECHERRY_ACK_RANDOM_FACTOR - 1));

  for(uint8_t attempt = 1; attempt <= BLUECHERRY_MAX_RETRANSMITS; ++attempt) {
    /* Monotonic, so that a clock step mid exchange cannot make the deadline expire instantly or
     * never. */
    _bluecherry_opdata.last_tx_us = esp_timer_get_time();
    _bluecherry_tickle_watchdog();

    /* Entered before the write rather than after it. The write is an AT command and takes long
     * enough for an application polling the state to catch the exchange still looking idle. */
    _bluecherry_set_state(BLUECHERRY_STATE_AWAITING_RESPONSE);

    if(_bluecherry_mbed_dtls_write(data, data_len) < 0) {
      return ESP_FAIL;
    }

    while(true) {
      int ret =
          _bluecherry_mbed_dtls_read(_bluecherry_opdata.in_buf, sizeof(_bluecherry_opdata.in_buf));
      if(ret > 0) {
        _bluecherry_opdata.in_buf_len = ret;

        uint8_t rsp_type = 0;
        uint16_t rsp_message_id = 0;
        esp_err_t perr = _bluecherry_parse_ack_meta(
            _bluecherry_opdata.in_buf, _bluecherry_opdata.in_buf_len, &rsp_type, &rsp_message_id);
        if(perr == ESP_ERR_INVALID_VERSION) {
          ESP_LOGW(TAG, "Ignoring CoAP packet with invalid version while awaiting ACK");
          continue;
        }
        if(perr != ESP_OK) {
          ESP_LOGW(TAG, "Ignoring malformed CoAP packet while awaiting ACK");
          continue;
        }

        if(rsp_type != BLUECHERRY_COAP_TYPE_ACK) {
          ESP_LOGW(TAG, "Ignoring non-ACK CoAP packet while awaiting ACK");
          continue;
        }

        if(rsp_message_id != tx_message_id) {
          ESP_LOGD(TAG, "Received ACK with mismatching message ID %u while awaiting %u",
                   rsp_message_id, tx_message_id);
          continue;
        }

        _bluecherry_opdata.cur_message_id = tx_message_id;
        /* Left at AWAITING_RESPONSE: the response has not been walked, so IDLE would tell an
         * application waiting to sleep that nothing is outstanding before CONTINUE was read. */
        return ESP_OK;
      } else if(ret != MBEDTLS_ERR_SSL_TIMEOUT) {
        return ESP_FAIL;
      }

      if((esp_timer_get_time() - _bluecherry_opdata.last_tx_us) >= (int64_t) (timeout * 1000000)) {
        break;
      }
    }

    timeout *= 2;
  }

  /* Left at AWAITING_RESPONSE for the same reason; the caller moves it to AWAIT_CONNECTION. */
  return ESP_ERR_TIMEOUT;
}

/**
 * @brief Common CoAP transmit and receive function for provisioning operations.
 *
 * The response is not parsed: a fixed header length is skipped and the remainder is handed back
 * verbatim. rx_cap is therefore the only thing standing between a malformed or hostile response
 * and the caller's buffer, so an oversized response is rejected rather than truncated.
 *
 * @param tx_buf Pointer to the buffer containing the payload to transmit.
 * @param tx_len Length of the payload to transmit.
 * @param rx_buf Pointer to the buffer where the received data will be stored.
 * @param rx_cap Capacity of rx_buf in bytes.
 * @param rx_len Pointer to a variable where the length of the received data will be stored.
 * @param header Pointer to the CoAP header to be used for the message.
 * @param header_len Length of the CoAP header.
 *
 * @return True if the transmission and reception were successful, false otherwise.
 */
static bool _bluecherry_ztp_coap_rxtx_common(uint8_t* tx_buf, uint16_t tx_len, uint8_t* rx_buf,
                                             size_t rx_cap, uint16_t* rx_len, const uint8_t* header,
                                             size_t header_len)
{
  _bluecherry_opdata.cur_message_id += 1;
  if(_bluecherry_opdata.cur_message_id == 0) {
    _bluecherry_opdata.cur_message_id = 1;
  }

  size_t data_len = header_len;
  uint8_t data[BLUECHERRY_ZTP_TX_BUF_SIZE];

  if(header_len + 1 + (size_t) tx_len > sizeof(data)) {
    ESP_LOGE(TAG, "ZTP request of %u bytes does not fit the transmit buffer",
             (unsigned) (header_len + 1 + (size_t) tx_len));
    return false;
  }

  memcpy(data, header, header_len);

  if(tx_len > 0) {
    data[header_len] = 0xFF;
    memcpy(data + header_len + 1, tx_buf, tx_len);
    data_len = header_len + 1 + tx_len;
  }

  double timeout = BLUECHERRY_ACK_TIMEOUT *
                   (1 + (esp_random() / 4294967296.0) * (BLUECHERRY_ACK_RANDOM_FACTOR - 1));

  /* Receive into the session buffer instead of a second kilobyte of stack. Provisioning only runs
   * from BLUECHERRY_STATE_NOT_PROVISIONED, before any CoAP session exists, so in_buf is idle and
   * no other path can be reading it. */
  uint8_t* rx_scratch = _bluecherry_opdata.in_buf;
  const size_t rx_scratch_cap = sizeof(_bluecherry_opdata.in_buf);

  for(uint8_t attempt = 1; attempt <= BLUECHERRY_MAX_RETRANSMITS; ++attempt) {
    int64_t last_tx_us = esp_timer_get_time();
    _bluecherry_tickle_watchdog();

    if(_bluecherry_mbed_dtls_write(data, data_len) < 0) {
      return false;
    }

    while(true) {
      int ret = _bluecherry_mbed_dtls_read(rx_scratch, rx_scratch_cap);

      if(ret > 0) {
        if(ret > BLUECHERRY_ZTP_RSP_HEADER_LEN) {
          size_t payload_len = (size_t) ret - BLUECHERRY_ZTP_RSP_HEADER_LEN;
          if(payload_len > rx_cap) {
            ESP_LOGE(TAG, "ZTP response payload of %u bytes exceeds the %u byte buffer",
                     (unsigned) payload_len, (unsigned) rx_cap);
            return false;
          }
          memcpy(rx_buf, rx_scratch + BLUECHERRY_ZTP_RSP_HEADER_LEN, payload_len);
          *rx_len = (uint16_t) payload_len;
        } else {
          *rx_len = 0;
        }
        return true;
      } else if(ret != MBEDTLS_ERR_SSL_TIMEOUT) {
        return false;
      }

      if((esp_timer_get_time() - last_tx_us) >= (int64_t) (timeout * 1000000)) {
        break;
      }
    }

    timeout *= 2;
  }

  return false;
}

/**
 * @brief CoAP transmit and receive function for requesting a device ID.
 *
 * @param tx_buf Pointer to the buffer containing the payload to transmit.
 * @param tx_len Length of the payload to transmit.
 * @param rx_buf Pointer to the buffer where the received data will be stored.
 * @param rx_cap Capacity of rx_buf in bytes.
 * @param rx_len Pointer to a variable where the length of the received data will be stored.
 *
 * @return True if the transmission and reception were successful, false otherwise.
 */
static bool _bluecherry_ztp_coap_rxtx_devid(uint8_t* tx_buf, uint16_t tx_len, uint8_t* rx_buf,
                                            size_t rx_cap, uint16_t* rx_len)
{
  const uint8_t header[] = { 0x40,
                             0x01,
                             (uint8_t) (_bluecherry_opdata.cur_message_id >> 8),
                             (uint8_t) (_bluecherry_opdata.cur_message_id & 0xFF),
                             0xB2,
                             0x76,
                             0x31,
                             0x05,
                             0x64,
                             0x65,
                             0x76,
                             0x69,
                             0x64 };

  return _bluecherry_ztp_coap_rxtx_common(tx_buf, tx_len, rx_buf, rx_cap, rx_len, header,
                                          sizeof(header));
}

/**
 * @brief CoAP transmit and receive function for signing a certificate request.
 *
 * @param tx_buf Pointer to the buffer containing the payload to transmit.
 * @param tx_len Length of the payload to transmit.
 * @param rx_buf Pointer to the buffer where the received data will be stored.
 * @param rx_cap Capacity of rx_buf in bytes.
 * @param rx_len Pointer to a variable where the length of the received data will be stored.
 *
 * @return True if the transmission and reception were successful, false otherwise.
 */
static bool _bluecherry_ztp_coap_rxtx_sign(uint8_t* tx_buf, uint16_t tx_len, uint8_t* rx_buf,
                                           size_t rx_cap, uint16_t* rx_len)
{
  const uint8_t header[] = { 0x40,
                             0x01,
                             (uint8_t) (_bluecherry_opdata.cur_message_id >> 8),
                             (uint8_t) (_bluecherry_opdata.cur_message_id & 0xFF),
                             0xB2,
                             0x76,
                             0x31,
                             0x04,
                             0x73,
                             0x69,
                             0x67,
                             0x6E };

  return _bluecherry_ztp_coap_rxtx_common(tx_buf, tx_len, rx_buf, rx_cap, rx_len, header,
                                          sizeof(header));
}

#pragma endregion
#pragma region ZTP

/**
 * @brief Initializes the CBOR context.
 *
 * @param cbor CBOR context to initialize.
 * @param buffer Output buffer to use.
 * @param capacity Maximum size of the buffer.
 *
 * @return 0 on success, non-zero on failure.
 */
static int _ztp_cbor_init(_ztp_cbor_t* cbor, uint8_t* buffer, size_t capacity)
{
  if(buffer == NULL || capacity == 0) {
    return -1;
  }

  cbor->buffer = buffer;
  cbor->capacity = capacity;
  cbor->position = 0;

  return 0;
}

/**
 * @brief Returns the size of encoded data.
 *
 * @param cbor CBOR context.
 *
 * @return Size of encoded data.
 */
static size_t _ztp_cbor_size(const _ztp_cbor_t* cbor)
{
  return cbor->position;
}

/**
 * @brief Writes a single byte to the CBOR buffer.
 *
 * @param cbor CBOR context.
 * @param byte Byte to write.
 *
 * @return 0 on success, non-zero on failure.
 */
static int _ztp_cbor_write_byte(_ztp_cbor_t* cbor, uint8_t byte)
{
  if(cbor->position < cbor->capacity) {
    cbor->buffer[cbor->position++] = byte;
    return 0;
  }
  return -1;
}

/**
 * @brief Writes a byte array to the CBOR buffer.
 *
 * @param cbor CBOR context.
 * @param data Data to write.
 * @param length Length of data to write.
 *
 * @return 0 on success, non-zero on failure.
 */
static int _ztp_cbor_write_bytes(_ztp_cbor_t* cbor, const uint8_t* data, size_t length)
{
  if(cbor->position + length <= cbor->capacity) {
    memcpy(&cbor->buffer[cbor->position], data, length);
    cbor->position += length;
    return 0;
  }
  return -1;
}

/**
 * @brief Encodes the type and value into CBOR format.
 *
 * @param cbor CBOR context.
 * @param major_type Major type of the CBOR data.
 * @param value Value to encode.
 *
 * @return 0 on success, non-zero on failure.
 */
static int _ztp_cbor_encode_type_and_value(_ztp_cbor_t* cbor, uint8_t major_type, size_t value)
{
  if(value < 24) {
    return _ztp_cbor_write_byte(cbor, (major_type << 5) | value);
  } else if(value < 256) {
    if(_ztp_cbor_write_byte(cbor, (major_type << 5) | 0x18) < 0)
      return -1;
    return _ztp_cbor_write_byte(cbor, (uint8_t) value);
  } else if(value < 65536) {
    if(_ztp_cbor_write_byte(cbor, (major_type << 5) | 0x19) < 0)
      return -1;
    uint8_t bytes[] = { (uint8_t) (value >> 8), (uint8_t) value };
    return _ztp_cbor_write_bytes(cbor, bytes, 2);
  }
  return -1;
}

/**
 * @brief Encodes a byte string into CBOR format.
 *
 * @param cbor CBOR context.
 * @param data Data to encode.
 * @param length Length of data to encode.
 *
 * @return 0 on success, non-zero on failure.
 */
static int _ztp_cbor_encode_bytes(_ztp_cbor_t* cbor, const uint8_t* data, size_t length)
{
  if(_ztp_cbor_encode_type_and_value(cbor, 2, length) < 0)
    return -1;
  return _ztp_cbor_write_bytes(cbor, data, length);
}

/**
 * @brief Encodes a string into CBOR format.
 *
 * @param cbor CBOR context.
 * @param str String to encode.
 *
 * @return 0 on success, non-zero on failure.
 */
static int _ztp_cbor_encode_string(_ztp_cbor_t* cbor, const char* str)
{
  size_t len = strlen(str);
  if(_ztp_cbor_encode_type_and_value(cbor, 3, len) < 0)
    return -1;
  return _ztp_cbor_write_bytes(cbor, (const uint8_t*) str, len);
}

/**
 * @brief Encodes a 64-bit unsigned integer into CBOR format.
 *
 * @param cbor CBOR context.
 * @param value Value to encode.
 *
 * @return 0 on success, non-zero on failure.
 */
static int _ztp_cbor_encode_uint64(_ztp_cbor_t* cbor, uint64_t value)
{
  if(_ztp_cbor_encode_type_and_value(cbor, 2, 8) < 0)
    return -1;

  uint8_t bytes[] = { (uint8_t) (value >> 56), (uint8_t) (value >> 48), (uint8_t) (value >> 40),
                      (uint8_t) (value >> 32), (uint8_t) (value >> 24), (uint8_t) (value >> 16),
                      (uint8_t) (value >> 8),  (uint8_t) value };

  return _ztp_cbor_write_bytes(cbor, bytes, 8);
}

/**
 * @brief Encodes a signed integer into CBOR format.
 *
 * @param cbor The CBOR context.
 * @param value Value to encode.
 *
 * @return 0 on success, non-zero on failure.
 */
static int _ztp_cbor_encode_int(_ztp_cbor_t* cbor, int value)
{
  if(value >= 0) {
    return _ztp_cbor_encode_type_and_value(cbor, 0, (size_t) value);
  } else {
    return _ztp_cbor_encode_type_and_value(cbor, 1, (size_t) (-value - 1));
  }
}

/**
 * @brief Starts encoding an array into CBOR format.
 *
 * @param cbor The CBOR context.
 * @param size Expected size of the array.
 *
 * @return 0 on success, non-zero on failure.
 */
static int _ztp_cbor_start_array(_ztp_cbor_t* cbor, size_t size)
{
  return _ztp_cbor_encode_type_and_value(cbor, 4, size);
}

/**
 * @brief Starts encoding a map into CBOR format.
 *
 * @param cbor The CBOR context.
 * @param size Expected size of the map.
 *
 * @return 0 on success, non-zero on failure.
 */
static int _ztp_cbor_start_map(_ztp_cbor_t* cbor, size_t size)
{
  return _ztp_cbor_encode_type_and_value(cbor, 5, size);
}

/**
 * @brief Decodes a device ID from CBOR data.
 *
 * @param cbor_data CBOR data to decode.
 * @param cbor_size Size of CBOR data.
 * @param decoded_str Buffer to store decoded device ID.
 * @param decoded_size Size of decoded device ID buffer.
 *
 * @return 0 on success, non-zero on failure.
 */
static int _ztp_cbor_decode_device_id(const uint8_t* cbor_data, size_t cbor_size, char* decoded_str,
                                      size_t decoded_size)
{
  if(cbor_size < 1 || !cbor_data) {
    return -1;
  }

  uint8_t initial_byte = cbor_data[0];
  if((initial_byte >> 5) != 3) {
    return -2;
  }

  size_t length = 0;
  uint8_t additional_info = initial_byte & 0x1F;

  if(additional_info > 23) {
    return -3;
  }

  length = additional_info;
  cbor_data++;
  cbor_size--;

  if(length > cbor_size) {
    return -4;
  }

  if(length >= decoded_size) {
    return -5;
  }

  memcpy(decoded_str, cbor_data, length);
  decoded_str[length] = '\0';

  return 0;
}

/**
 * @brief Decodes a signed certificate from CBOR data.
 *
 * @param cbor_data CBOR data to decode.
 * @param cbor_size Size of CBOR data.
 * @param decoded_data Buffer to store decoded certificate.
 * @param decoded_len Pointer to store size of decoded certificate.
 *
 * @return 0 on success, non-zero on failure.
 */
static int _ztp_cbor_decode_certificate(const uint8_t* cbor_data, size_t cbor_size,
                                        unsigned char* decoded_data, size_t* decoded_len)
{
  if(cbor_size < 1 || !cbor_data) {
    return -1;
  }

  uint8_t initial_byte = cbor_data[0];
  if((initial_byte >> 5) != 2) {
    return -2;
  }

  size_t length = 0;
  size_t offset = 1;
  uint8_t additional_info = initial_byte & 0x1F;

  if(additional_info < 24) {
    length = additional_info;
  } else if(additional_info == 24) {
    length = cbor_data[offset++];
  } else if(additional_info == 25) {
    length = (cbor_data[offset] << 8) | cbor_data[offset + 1];
    offset += 2;
  } else if(additional_info == 26) {
    length = (cbor_data[offset] << 24) | (cbor_data[offset + 1] << 16) |
             (cbor_data[offset + 2] << 8) | cbor_data[offset + 3];
    offset += 4;
  } else {
    return -3;
  }

  if(offset + length > cbor_size) {
    return -4;
  }

  memcpy(decoded_data, cbor_data + offset, length);
  *decoded_len = length;

  return 0;
}

/**
 * @brief Gather entropy from the hardware random number generator.
 *
 * @param data Unused context pointer.
 * @param output The buffer to fill.
 * @param len The number of bytes requested.
 * @param olen Filled in with the number of bytes produced.
 *
 * @return 0 on success.
 */
static int _bluecherry_ztp_hardware_random_entropy(void* data, unsigned char* output, size_t len)
{
  (void) data;
  esp_fill_random(output, len);
  return 0;
}

/**
 * @brief Seed the deterministic random bit generator used to sign the CSR.
 *
 * The reference client borrows the generator its TLS stack already owns. Here the modem performs
 * the TLS, so the only thing on this side that needs randomness is the key generation, and it gets
 * a generator of its own seeded from the hardware source.
 *
 * @return True on success, false on error.
 */
static bool _bluecherry_ztp_seed_random(void)
{
  mbedtls_entropy_init(&_bluecherry_opdata.entropy);
  mbedtls_ctr_drbg_init(&_bluecherry_opdata.ctr_drbg);

  bootloader_random_enable();
  int ret =
      mbedtls_ctr_drbg_seed(&_bluecherry_opdata.ctr_drbg, _bluecherry_ztp_hardware_random_entropy,
                            &_bluecherry_opdata.entropy, NULL, 0);
  bootloader_random_disable();

  return ret == 0;
}

/**
 * @brief Release everything the CSR generation reserved.
 *
 * @param result The result to hand back.
 *
 * @return The result it was given, unchanged, so callers can return _ztp_finish_csr_gen(false).
 */
static bool _ztp_finish_csr_gen(bool result)
{
  mbedtls_pk_free(&_bluecherry_opdata.devkey);
  mbedtls_x509write_csr_free(&_bluecherry_opdata.ztp_mb_csr);
  mbedtls_ctr_drbg_free(&_bluecherry_opdata.ctr_drbg);
  mbedtls_entropy_free(&_bluecherry_opdata.entropy);

  if(!result) {
    ztp_pkey_buf[0] = '\0';
    ztp_cert_buf[0] = '\0';
  }

  return result;
}

/**
 * @brief Add a device ID parameter of blob type.
 *
 * @param type The type of the device ID parameter.
 * @param blob The blob value of the device ID parameter.
 *
 * @return True if the parameter was added successfully, false otherwise.
 */
static bool _ztp_add_device_id_parameter_blob(BlueCherryZtpDeviceIdType type,
                                              const unsigned char* blob)
{
  if(blob == NULL ||
     _bluecherry_opdata.ztp_dev_id_params.count >= BLUECHERRY_ZTP_MAX_DEVICE_ID_PARAMS) {
    return false;
  }

  _bluecherry_ztp_device_id_param_t* param =
      &_bluecherry_opdata.ztp_dev_id_params.param[_bluecherry_opdata.ztp_dev_id_params.count];

  switch(type) {
  case BLUECHERRY_ZTP_DEVICE_ID_TYPE_MAC:
    param->type = BLUECHERRY_ZTP_DEVICE_ID_TYPE_MAC;
    memcpy(param->value.mac, blob, BLUECHERRY_ZTP_MAC_LEN);
    _bluecherry_opdata.ztp_dev_id_params.count += 1;
    break;

  /* Walter has an IMEI and the reference client does not, so this arm is an addition rather than
   * a port. The encoder already handles the type. */
  case BLUECHERRY_ZTP_DEVICE_ID_TYPE_IMEI:
    param->type = BLUECHERRY_ZTP_DEVICE_ID_TYPE_IMEI;
    memset(param->value.imei, 0, sizeof(param->value.imei));
    strncpy(param->value.imei, (const char*) blob, BLUECHERRY_ZTP_IMEI_LEN);
    _bluecherry_opdata.ztp_dev_id_params.count += 1;
    break;

  default:
    return false;
  }

  return true;
}

/**
 * @brief Request the device ID from the BlueCherry provisioning server.
 *
 * @return True if the device ID was successfully requested and decoded, false otherwise.
 */
static bool _ztp_request_device_id(void)
{
  int ret;
  uint8_t cbor_buf[256];
  _ztp_cbor_t cbor;

  if(_ztp_cbor_init(&cbor, cbor_buf, sizeof(cbor_buf)) < 0) {
    ESP_LOGE(TAG, "Failed to init CBOR buffer");
    return false;
  }

  if(_ztp_cbor_start_array(&cbor, 2) < 0) {
    ESP_LOGE(TAG, "Failed to start CBOR array");
    return false;
  }

  if(_ztp_cbor_encode_string(&cbor, bc_type_id) < 0) {
    ESP_LOGE(TAG, "Failed to encode typeId value");
    return false;
  }

  if(_ztp_cbor_start_map(&cbor, _bluecherry_opdata.ztp_dev_id_params.count) < 0) {
    ESP_LOGE(TAG, "Failed to start CBOR map");
    return false;
  }

  for(int i = 0; i < _bluecherry_opdata.ztp_dev_id_params.count; i++) {
    int type = (int) _bluecherry_opdata.ztp_dev_id_params.param[i].type;
    if(_ztp_cbor_encode_int(&cbor, type) < 0) {
      ESP_LOGE(TAG, "Failed to encode param type (%u)", type);
      return false;
    }

    switch(_bluecherry_opdata.ztp_dev_id_params.param[i].type) {
    case BLUECHERRY_ZTP_DEVICE_ID_TYPE_IMEI: {
      uint64_t imei = strtoull(_bluecherry_opdata.ztp_dev_id_params.param[i].value.imei, NULL, 10);
      if(_ztp_cbor_encode_uint64(&cbor, imei) < 0) {
        ESP_LOGE(TAG, "Failed to encode IMEI number");
        return false;
      }
    } break;

    case BLUECHERRY_ZTP_DEVICE_ID_TYPE_MAC: {
      if(_ztp_cbor_encode_bytes(&cbor, _bluecherry_opdata.ztp_dev_id_params.param[i].value.mac,
                                BLUECHERRY_ZTP_MAC_LEN) < 0) {
        ESP_LOGE(TAG, "Failed to encode MAC address");
        return false;
      }
    } break;

    case BLUECHERRY_ZTP_DEVICE_ID_TYPE_OOB_CHALLENGE: {
      uint64_t oob_challenge = _bluecherry_opdata.ztp_dev_id_params.param[i].value.oob_challenge;
      if(_ztp_cbor_encode_uint64(&cbor, oob_challenge) < 0) {
        ESP_LOGE(TAG, "Failed to encode OOB challenge");
        return false;
      }
    } break;

    default:
      break;
    }
  }

  uint8_t rx_buf[16];
  uint16_t rx_len = 0;
  if(!_bluecherry_ztp_coap_rxtx_devid(cbor_buf, _ztp_cbor_size(&cbor), rx_buf, sizeof(rx_buf),
                                      &rx_len)) {
    ESP_LOGE(TAG, "Failed to sync with the ZTP CoAP server");
    return false;
  }

  ret = _ztp_cbor_decode_device_id(rx_buf, rx_len, ztp_bc_dev_id, sizeof(ztp_bc_dev_id));
  if(ret < 0) {
    ESP_LOGD(TAG, "Failed to decode device id: %d", ret);
    return false;
  }

  return true;
}

/**
 * @brief Generate a key pair and CSR for provisioning.
 *
 * The subject is "C=BE,CN=<typeId>.<deviceId>", which the server splits on the first dot to get
 * the type and device identifiers, so the format is load-bearing at both ends.
 *
 * @return True if the key pair and CSR were generated successfully, false otherwise.
 */
static bool _ztp_generate_key_and_csr(void)
{
  int ret;
  uint8_t csr_buf[BLUECHERRY_ZTP_CERT_BUF_SIZE];

  if(bc_type_id == NULL || strlen(bc_type_id) != BLUECHERRY_ZTP_ID_LEN ||
     strlen(ztp_bc_dev_id) != BLUECHERRY_ZTP_ID_LEN) {
    return false;
  }

  mbedtls_pk_init(&_bluecherry_opdata.devkey);
  mbedtls_x509write_csr_init(&_bluecherry_opdata.ztp_mb_csr);

  if(!_bluecherry_ztp_seed_random()) {
    ESP_LOGE(TAG, "Could not seed the random number generator");
    return _ztp_finish_csr_gen(false);
  }

  if(mbedtls_pk_setup(&_bluecherry_opdata.devkey, mbedtls_pk_info_from_type(MBEDTLS_PK_ECKEY)) !=
     0) {
    return _ztp_finish_csr_gen(false);
  }

  if(mbedtls_ecp_gen_key(MBEDTLS_ECP_DP_SECP256R1, mbedtls_pk_ec(_bluecherry_opdata.devkey),
                         mbedtls_ctr_drbg_random, &_bluecherry_opdata.ctr_drbg) != 0) {
    return _ztp_finish_csr_gen(false);
  }

  if(mbedtls_pk_write_key_pem(&_bluecherry_opdata.devkey, (unsigned char*) ztp_pkey_buf,
                              BLUECHERRY_ZTP_PKEY_BUF_SIZE) != 0) {
    return _ztp_finish_csr_gen(false);
  }

  mbedtls_x509write_csr_set_md_alg(&_bluecherry_opdata.ztp_mb_csr, MBEDTLS_MD_SHA256);
  mbedtls_x509write_csr_set_key(&_bluecherry_opdata.ztp_mb_csr, &_bluecherry_opdata.devkey);

  snprintf(ztp_subj_buf, BLUECHERRY_ZTP_SUBJ_BUF_SIZE, "C=BE,CN=%s.%s", bc_type_id, ztp_bc_dev_id);
  if(mbedtls_x509write_csr_set_subject_name(&_bluecherry_opdata.ztp_mb_csr, ztp_subj_buf) != 0) {
    return _ztp_finish_csr_gen(false);
  }

  ret = mbedtls_x509write_csr_der(&_bluecherry_opdata.ztp_mb_csr, csr_buf,
                                  BLUECHERRY_ZTP_CERT_BUF_SIZE, mbedtls_ctr_drbg_random,
                                  &_bluecherry_opdata.ctr_drbg);
  if(ret < 0) {
    ESP_LOGE(TAG, "Failed to write CSR DER: -0x%04X", -ret);
    return _ztp_finish_csr_gen(false);
  }

  /* The DER is written to the END of the buffer, so it has to be moved to the front. */
  size_t offset = BLUECHERRY_ZTP_CERT_BUF_SIZE - ret;
  _bluecherry_opdata.ztp_csr.length = ret;
  memcpy(_bluecherry_opdata.ztp_csr.buffer, csr_buf + offset, _bluecherry_opdata.ztp_csr.length);

  return _ztp_finish_csr_gen(true);
}

/**
 * @brief Request a signed certificate from the BlueCherry provisioning server.
 *
 * @return True if the signed certificate was successfully requested and stored, false otherwise.
 */
static bool _ztp_request_signed_certificate(void)
{
  int ret;
  uint8_t cbor_buf[BLUECHERRY_ZTP_CERT_BUF_SIZE];
  uint8_t coap_data[BLUECHERRY_ZTP_CERT_BUF_SIZE];
  _ztp_cbor_t cbor;

  _ztp_cbor_init(&cbor, cbor_buf, BLUECHERRY_ZTP_CERT_BUF_SIZE);
  mbedtls_x509_crt_init(&_bluecherry_opdata.devcert);

  if(_ztp_cbor_encode_bytes(&cbor, _bluecherry_opdata.ztp_csr.buffer,
                            _bluecherry_opdata.ztp_csr.length) < 0) {
    ESP_LOGE(TAG, "Failed to encode CSR");
    return false;
  }

  uint16_t rx_len = 0;
  if(!_bluecherry_ztp_coap_rxtx_sign(cbor_buf, _ztp_cbor_size(&cbor), coap_data, sizeof(coap_data),
                                     &rx_len)) {
    ESP_LOGE(TAG, "Failed to receive response from the ZTP CoAP server");
    return false;
  }

  size_t decoded_size;
  ret = _ztp_cbor_decode_certificate(coap_data, rx_len, cbor_buf, &decoded_size);
  if(ret < 0) {
    ESP_LOGE(TAG, "Failed to decode certificate: %d", ret);
    return false;
  }

  ret = mbedtls_x509_crt_parse_der(&_bluecherry_opdata.devcert, cbor_buf, decoded_size);
  if(ret < 0) {
    ESP_LOGE(TAG, "Failed to parse DER certificate, error code: -0x%x", -ret);
    mbedtls_x509_crt_free(&_bluecherry_opdata.devcert);
    return false;
  }

  size_t pem_len;
  ret =
      mbedtls_pem_write_buffer("-----BEGIN CERTIFICATE-----\n", "-----END CERTIFICATE-----\n",
                               _bluecherry_opdata.devcert.raw.p, _bluecherry_opdata.devcert.raw.len,
                               cbor_buf, BLUECHERRY_ZTP_CERT_BUF_SIZE, &pem_len);
  if(ret < 0) {
    ESP_LOGE(TAG, "Failed to write PEM: -0x%04X", -ret);
    mbedtls_x509_crt_free(&_bluecherry_opdata.devcert);
    return false;
  }

  /* pem_len counts the terminating NUL, so the copy is already terminated. */
  memcpy(ztp_cert_buf, cbor_buf, pem_len);

  mbedtls_x509_crt_free(&_bluecherry_opdata.devcert);
  return true;
}

/**
 * @brief Point the TLS profile at the credentials in modem NVM.
 *
 * Mutual DTLS: the server is checked against the CA and the hostname, and the device presents the
 * certificate and key the modem holds. Note the argument order - the DEVICE certificate goes in
 * the client CA slot, which is what the modem expects.
 *
 * @return True on success, false on error.
 */
static bool _bluecherry_configure_own_cert(void)
{
  return WalterModem::tlsConfigProfile(_bluecherry_opdata.tls_profile_id,
                                       WALTER_MODEM_TLS_VALIDATION_URL_AND_CA,
                                       WALTER_MODEM_TLS_VERSION_12, BLUECHERRY_SLOT_CA,
                                       BLUECHERRY_SLOT_DEVCERT, BLUECHERRY_SLOT_PRIVKEY);
}

/**
 * @brief Obtain the device credentials if the modem does not hold them yet.
 *
 * Runs from the synchronisation cycle rather than from init, so that reserving memory and reaching
 * the network stay separate concerns and a device with no cloud in sight still initialises.
 *
 * Unlike the reference client there is no storage handler: the modem's own NVM is the store, so
 * the credentials are written there and never touch ESP32 flash, and the presence check is a read
 * of the three slots.
 *
 * @return True once credentials are installed and the connection can be attempted.
 */
static bool _bluecherry_provision(void)
{
  if(!WalterBlueCherry::isProvisioned()) {
    ESP_LOGI(TAG, "Device is not provisioned for BlueCherry communication, starting ZTP...");

    if(bc_type_id == NULL) {
      ESP_LOGE(TAG, "(ZTP) No device type was supplied, cannot provision");
      return false;
    }

    /* The profile is about to be rewritten, and a socket bound to it would not survive that. */
    _bluecherry_cleanup_session();

    /* The provisioning service is authenticated against the same CA, but this device has no
     * certificate of its own to present yet, so the session is one sided. */
    if(!WalterModem::tlsWriteCredential(false, BLUECHERRY_SLOT_CA, BLUECHERRY_CA)) {
      ESP_LOGE(TAG, "(ZTP) Could not write the CA to the modem");
      goto fail;
    }

    if(!WalterModem::tlsConfigProfile(_bluecherry_opdata.tls_profile_id,
                                      WALTER_MODEM_TLS_VALIDATION_CA, WALTER_MODEM_TLS_VERSION_12,
                                      BLUECHERRY_SLOT_CA)) {
      ESP_LOGE(TAG, "(ZTP) Could not configure the provisioning TLS profile");
      goto fail;
    }

    if(!_bluecherry_dtls_connect(BLUECHERRY_HOST, BLUECHERRY_ZTP_PORT)) {
      ESP_LOGE(TAG, "(ZTP) Could not connect to the provisioning server");
      goto fail;
    }

    ESP_LOGI(TAG, "(ZTP) Connected");

    {
      uint8_t mac[8] = { 0 };
      if(esp_read_mac(mac, ESP_MAC_WIFI_STA) == ESP_OK) {
        _ztp_add_device_id_parameter_blob(BLUECHERRY_ZTP_DEVICE_ID_TYPE_MAC, mac);
      }

      WalterModemRsp identity = {};
      if(WalterModem::getIdentity(&identity)) {
        _ztp_add_device_id_parameter_blob(BLUECHERRY_ZTP_DEVICE_ID_TYPE_IMEI,
                                          (const unsigned char*) identity.data.identity.imei);
      }

      if(_bluecherry_opdata.ztp_dev_id_params.count == 0) {
        ESP_LOGE(TAG, "(ZTP) Could not identify this device to the provisioning server");
        goto fail;
      }
    }

    if(!_ztp_request_device_id()) {
      ESP_LOGD(TAG, "(ZTP) Could not request device ID");
      ESP_LOGE(TAG, "(ZTP) This device might not exist- or is not set to WAIT-PROVISION on the "
                    "BlueCherry platform.");
      goto fail;
    }

    if(!_ztp_generate_key_and_csr()) {
      ESP_LOGE(TAG, "(ZTP) Could not generate private key");
      goto fail;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    if(!_ztp_request_signed_certificate()) {
      ESP_LOGE(TAG, "(ZTP) Could not request signed certificate");
      goto fail;
    }

    /* The provisioning session authenticated only the server, and the profile is about to be
     * rewritten, so drop it before installing anything. */
    _bluecherry_cleanup_session();

    if(!WalterBlueCherry::provision(ztp_cert_buf, ztp_pkey_buf, BLUECHERRY_CA)) {
      ESP_LOGE(TAG, "(ZTP) Could not write the issued credentials to the modem");
      goto fail;
    }

    /* The key exists in exactly one other place now, the modem's NVM, and never reaches flash. */
    memset(ztp_pkey_buf, 0, sizeof(ztp_pkey_buf));
    memset(ztp_cert_buf, 0, sizeof(ztp_cert_buf));

    ESP_LOGI(TAG, "(ZTP) Provisioned successfully");
  }

  if(!_bluecherry_configure_own_cert()) {
    ESP_LOGE(TAG, "Could not configure device credentials");
    goto fail;
  }

  _bluecherry_opdata.ztp_dev_id_params.count = 0;
  return true;

fail:
  /* The specific reason is logged where it was detected; the caller reports the failure itself,
   * so that the message can name the retry delay it is about to apply. */
  _bluecherry_cleanup_session();
  _bluecherry_opdata.ztp_dev_id_params.count = 0;
  return false;
}

#pragma endregion
#pragma region SYNC_CYCLE

/**
 * @brief Run one synchronisation cycle: provision, connect, send one message, dispatch the reply.
 *
 * Runs only on the synchronisation task, one at a time, which is what lets it touch the shared
 * connection state without locking. Timing is the caller's concern, not its own.
 *
 * @return BLUECHERRY_SYNC_CONTINUE when work is still outstanding and another cycle should follow
 * immediately, ESP_OK when everything settled, or an error for the round that failed.
 */
static esp_err_t _bluecherry_sync_once(void)
{
  static int64_t last_retry_time_us = 0;
  static uint32_t retry_interval_ms = 100;
  static uint32_t provision_interval_ms = BLUECHERRY_PROVISION_RETRY_MS;

  /* Obtain credentials before anything else. This is backoff gated on its own timer rather than
   * sharing the connect one, so a provisioning service that is down does not also throttle the
   * reconnects of a device that is already provisioned.
   *
   * A deadline rather than an elapsed time comparison, so that _next_provision_us == 0 means "due
   * now" and the first attempt after boot is not itself delayed by the backoff. */
  if(_bluecherry_opdata.state == BLUECHERRY_STATE_NOT_PROVISIONED) {
    if(esp_timer_get_time() < _next_provision_us) {
      return ESP_ERR_NOT_FINISHED;
    }

    if(!_bluecherry_provision()) {
      ESP_LOGE(TAG, "(ZTP) Provisioning failed, retrying in %lu s",
               (unsigned long) (provision_interval_ms / 1000));
      _next_provision_us = esp_timer_get_time() + (int64_t) provision_interval_ms * 1000;

      provision_interval_ms *= 2;
      if(provision_interval_ms > BLUECHERRY_PROVISION_RETRY_MAX_MS) {
        provision_interval_ms = BLUECHERRY_PROVISION_RETRY_MAX_MS;
      }
      return ESP_ERR_NOT_FINISHED;
    }

    _next_provision_us = 0;
    provision_interval_ms = BLUECHERRY_PROVISION_RETRY_MS;
    _bluecherry_set_state(BLUECHERRY_STATE_AWAIT_CONNECTION);
  }

  /* (re)connect if needed with exponential backoff (non-blocking) */
  if(_bluecherry_opdata.state == BLUECHERRY_STATE_AWAIT_CONNECTION) {
    /* Nothing to dial into. The modem reports this on every +CEREG, polled or unsolicited, so it
     * clears itself the moment the network is back; until then a dial would spend 20 s of RF to
     * fail. The backoff is reset rather than advanced: the wait ends with the outage, not one
     * interval later. */
    if(!WalterBlueCherry::_networkUp()) {
      retry_interval_ms = 100;
      return ESP_ERR_NOT_FINISHED;
    }

    int64_t now_us = esp_timer_get_time();
    int64_t elapsed_ms = (now_us - last_retry_time_us) / 1000;
    if(elapsed_ms >= retry_interval_ms) {
      last_retry_time_us = now_us;
      if(!_bluecherry_dtls_connect(BLUECHERRY_HOST, BLUECHERRY_PORT)) {
        ESP_LOGE(TAG, "Could not connect to BlueCherry server");
        retry_interval_ms = (retry_interval_ms < 30000) ? retry_interval_ms * 2 : 30000;
        return ESP_ERR_NOT_FINISHED;
      }

      _bluecherry_opdata.cur_message_id = 0;
      _bluecherry_opdata.last_acked_message_id = 0;
      _bluecherry_opdata.sent_message_id = 0;

      /* A download the cloud already sent chunks for is resumed, anything else in progress is
       * dropped. A protocol reply from the dead session is meaningless, so the priority slot goes
       * too. A deep sleep resume does not come through here: the session survived. */
      if(!_bluecherry_ota_prepare_resume()) {
        _bluecherry_ota_reset();
      }
      _bluecherry_opdata.pending_event_len = 0;

      /* Not IDLE: the new session still has to run this cycle's exchange. */
      _bluecherry_set_state(BLUECHERRY_STATE_PENDING_MESSAGES);
      retry_interval_ms = 100;

      /* Once per boot: the running image cannot change without a reset. */
      if(!_bluecherry_rtc.init_info_acked) {
        _bluecherry_send_init_info();
      }

      /* The RESUME goes out once the slot is free. */
      if(_bluecherry_opdata.ota_resume_due && _bluecherry_opdata.pending_event_len == 0) {
        _bluecherry_ota_queue_resume();
      }
    } else {
      return ESP_ERR_NOT_FINISHED;
    }
  }

  if(_bluecherry_opdata.state == BLUECHERRY_STATE_UNINITIALIZED) {
    ESP_LOGE(TAG, "Cannot sync in the current state");
    return ESP_ERR_INVALID_STATE;
  }

  _bluecherry_msg_t out_msg;

  /* Internal channel protocol replies jump the application queue. Only one message goes out per
   * sync, so a probe reply left to queue behind a full publish buffer would be that many syncs
   * away - long enough for the server to push a whole image in a form this client cannot
   * accept. */
  if(_bluecherry_opdata.pending_event_len > 0) {
    _bluecherry_msg_t ev = { _bluecherry_opdata.pending_event_len,
                             _bluecherry_opdata.pending_event };
    if(_bluecherry_coap_rxtx(&ev) != ESP_OK) {
      ESP_LOGE(TAG, "Could not sync internal event with cloud");
      _bluecherry_set_state(BLUECHERRY_STATE_AWAIT_CONNECTION);
      return ESP_ERR_NOT_FINISHED;
    }

    /* ESP_OK is the ACK, so the server now has this boot's INIT_INFO. */
    if(ev.data[BLUECHERRY_COAP_HEADER_SIZE + BLUECHERRY_MQTT_HEADER_SIZE] ==
       BLUECHERRY_EVENT_TYPE_INIT_INFO) {
      _bluecherry_rtc.init_info_acked = true;
    }

    _bluecherry_opdata.pending_event_len = 0;

    /* A RESUME is done once acknowledged. Anything else sent first leaves it due. */
    if(ev.data[BLUECHERRY_COAP_HEADER_SIZE + BLUECHERRY_MQTT_HEADER_SIZE] ==
       BLUECHERRY_EVENT_TYPE_OTA_RESUME) {
      _bluecherry_opdata.ota_resume_due = false;
    } else if(_bluecherry_opdata.ota_resume_due) {
      _bluecherry_ota_queue_resume();
    }

    /* The round trip only returns ESP_OK once the ACK is in, so this is where a VERIFIED is known
     * to have landed - and therefore the only safe point to make the new image the boot target. */
    if(_bluecherry_opdata.ota_state == BLUECHERRY_OTA_STATE_AWAITING_VERIFIED) {
      _bluecherry_ota_commit();
    }
  }
  /* Peeked, not popped: the message stays in the buffer until its ACK is in, so a failed sync
   * retries it rather than dropping it. */
  else if(_bluecherry_ring_peek(&out_msg)) {
    if(_bluecherry_coap_rxtx(&out_msg) == ESP_OK) {
      _bluecherry_ring_pop();
    } else {
      ESP_LOGE(TAG, "Could not sync payload with cloud");
      _bluecherry_set_state(BLUECHERRY_STATE_AWAIT_CONNECTION);
      return ESP_ERR_NOT_FINISHED;
    }
  } else {
    /* Nothing to send, but an empty sync is still the only thing that lets the server deliver
     * downlink to a device that never publishes. */
    if(_bluecherry_coap_rxtx(NULL) != ESP_OK) {
      ESP_LOGE(TAG, "Could not sync with cloud");
      _bluecherry_set_state(BLUECHERRY_STATE_AWAIT_CONNECTION);
      return ESP_ERR_NOT_FINISHED;
    }
  }

  bool want_resync = false;

  if(_bluecherry_opdata.in_buf_len < BLUECHERRY_COAP_HEADER_SIZE) {
    ESP_LOGE(TAG, "Received CoAP packet too small: %u", (unsigned) _bluecherry_opdata.in_buf_len);
    return ESP_ERR_INVALID_SIZE;
  }

  uint16_t offset = 0;

  uint8_t header = _bluecherry_opdata.in_buf[offset++];
  uint8_t version = (header >> 6) & 0x03;
  if(version != 1) {
    ESP_LOGE(TAG, "Received CoAP packet with version %d, expected 1", version);
    return ESP_ERR_INVALID_VERSION;
  }

  uint8_t type = (header >> 4) & 0x03;
  uint8_t token_len = header & 0x0F;

  size_t min_header_len = (size_t) (1 + token_len + 1 + 2 + 1);
  if(_bluecherry_opdata.in_buf_len < min_header_len) {
    ESP_LOGE(TAG, "Received CoAP packet with invalid length %u for token length %u",
             (unsigned) _bluecherry_opdata.in_buf_len, token_len);
    return ESP_ERR_INVALID_SIZE;
  }

  offset += token_len;
  uint8_t code = _bluecherry_opdata.in_buf[offset++];
  uint16_t msg_id = _bluecherry_opdata.in_buf[offset++];
  msg_id <<= 8;
  msg_id |= _bluecherry_opdata.in_buf[offset++];
  if(_bluecherry_opdata.in_buf[offset++] != 0xFF) {
    ESP_LOGE(TAG, "Received CoAP packet without payload marker");
    return ESP_ERR_INVALID_RESPONSE;
  }

  if(type == BLUECHERRY_COAP_TYPE_ACK) {
    if(msg_id != _bluecherry_opdata.cur_message_id) {
      ESP_LOGE(TAG, "Received ACK for %u instead of %u", msg_id, _bluecherry_opdata.cur_message_id);
      return ESP_ERR_INVALID_STATE;
    }

    _bluecherry_opdata.last_acked_message_id = msg_id;
  }

  switch(code) {
  case BLUECHERRY_COAP_RSP_VALID:
    want_resync = false;
    break;

  case BLUECHERRY_COAP_RSP_CONTINUE:
    want_resync = true;
    break;

  default:
    ESP_LOGE(TAG, "Received invalid CoAP code %02X", code);
    return ESP_ERR_INVALID_RESPONSE;
  }

  while(offset < _bluecherry_opdata.in_buf_len) {
    /* Both header bytes have to be there before either is read: a single trailing byte would
     * otherwise take the length from past the payload, and from past the buffer itself on a frame
     * that filled it. */
    if(offset + 2 > _bluecherry_opdata.in_buf_len) {
      ESP_LOGE(TAG, "Received truncated payload header");
      return ESP_ERR_INVALID_SIZE;
    }

    uint8_t topic = _bluecherry_opdata.in_buf[offset++];
    uint8_t data_len = _bluecherry_opdata.in_buf[offset++];

    if(offset + data_len > _bluecherry_opdata.in_buf_len) {
      ESP_LOGE(TAG, "Received malformed payload length");
      return ESP_ERR_INVALID_SIZE;
    }

    if(topic == 0x00) {
      want_resync = true;

      _bluecherry_process_event(_bluecherry_opdata.in_buf + offset, data_len);
    } else if(_bluecherry_opdata.msg_handler != NULL) {
      _bluecherry_opdata.msg_handler(topic, data_len, _bluecherry_opdata.in_buf + offset,
                                     _bluecherry_opdata.msg_handler_args);
    }

    offset += data_len;
  }

  ESP_LOGD(TAG, "Synchronized messages with cloud");

  /* An application is entitled to sleep on IDLE, so the server having more queued is only one way
   * this can be unsettled - the outgoing queue and a just queued reply count too. */
  if(_bluecherry_settle_to(_bluecherry_work_pending(want_resync)) ==
     BLUECHERRY_STATE_PENDING_MESSAGES) {
    return (esp_err_t) BLUECHERRY_SYNC_CONTINUE;
  }

  return ESP_OK;
}

#pragma endregion
#pragma region SLEEP

/**
 * @brief Read out and drop whatever the modem buffered while the ESP32 slept.
 *
 * An ACK that arrives once the exchange has settled is announced by a +SQNSRING with nobody awake
 * to hear it. The datagram then stays in the modem and is handed back in answer to the next
 * message instead, leaving every following cycle one buffer behind.
 *
 * This is the one place that asks the modem for bytes it has not announced. An empty socket, and
 * equally a closed one, answers +CME ERROR, which is not retried and so costs a single round trip.
 *
 * Dropping is unconditional: the server only ever sends ACKs, and an ACK that outlived its
 * exchange was answered before the device slept.
 *
 * @return True when the socket is empty, false when it would not empty.
 */
static bool _bluecherry_drain_socket(void)
{
  for(uint8_t i = 0; i < BLUECHERRY_RESUME_DRAIN_MAX; ++i) {
    WalterModemRsp rsp = {};

    if(!WalterModem::socketReceive(_bluecherry_opdata.sock, _bluecherry_opdata.in_buf,
                                   sizeof(_bluecherry_opdata.in_buf), &rsp)) {
      return true;
    }

    ESP_LOGD(TAG, "Dropped %uB the modem held across the sleep",
             (unsigned) rsp.data.socketResponse.bytesReceived);
  }

  return false;
}

/**
 * @brief Take the session back out of RTC memory after a deep sleep.
 *
 * The modem stays powered across deep sleep, so the socket, the PDP context and the DTLS session
 * are all still there and the message id sequence the server is holding is still valid. Resuming
 * therefore deliberately does NOT do what a reconnect does: the message ids and the OTA state are
 * kept exactly as they were.
 *
 * Nothing here proves the session is alive, and nothing can: the modem's socket state report does
 * not cover the socket BlueCherry is normally given. So the session is resumed optimistically and
 * the first exchange decides - a dead one shows up as a failed send or an unanswered retransmit,
 * which the synchronisation cycle already turns into a reconnect. The cost of being wrong is one
 * round trip.
 *
 * @return True when a session was resumed, false when one has to be built.
 */
static bool _bluecherry_session_resume(void)
{
  if(_bluecherry_rtc.magic != BLUECHERRY_RTC_MAGIC || _bluecherry_rtc.sock <= 0) {
    return false;
  }

  _bluecherry_opdata.sock = _bluecherry_rtc.sock;
  _bluecherry_opdata.tls_profile_id = _bluecherry_rtc.tls_profile_id;
  _bluecherry_opdata.pdp_ctx_id = _bluecherry_rtc.pdp_ctx_id;
  _bluecherry_opdata.cur_message_id = _bluecherry_rtc.cur_message_id;
  _bluecherry_opdata.last_acked_message_id = _bluecherry_rtc.last_acked_message_id;
  _bluecherry_opdata.sent_message_id = _bluecherry_rtc.cur_message_id;

  _bluecherry_opdata.ota_state = (_bluecherry_ota_state) _bluecherry_rtc.ota_state;
  _bluecherry_opdata.ota_size = _bluecherry_rtc.ota_size;
  _bluecherry_opdata.ota_progress = _bluecherry_rtc.ota_progress;
  _bluecherry_opdata.ota_target_version = _bluecherry_rtc.ota_target_version;
  _bluecherry_opdata.ota_unverified = _bluecherry_rtc.ota_unverified;
  memcpy(_bluecherry_opdata.ota_expected_hash, _bluecherry_rtc.ota_expected_hash,
         BLUECHERRY_PARTITION_HASH_LEN);
  memcpy(_bluecherry_opdata.ota_skip_buffer, _bluecherry_rtc.ota_skip_buffer, ENCRYPTED_BLOCK_SIZE);
  _bluecherry_opdata.ota_resume_due = _bluecherry_rtc.ota_resume_due;

  _bluecherry_opdata.pending_event_len = _bluecherry_rtc.pending_event_len;
  memcpy(_bluecherry_opdata.pending_event, _bluecherry_rtc.pending_event,
         BLUECHERRY_PENDING_EVENT_SIZE);

  /* The partition is remembered as a slot index rather than as a pointer, because a pointer into
   * the partition table is meaningless after the application has been replaced. */
  _bluecherry_opdata.ota_partition = esp_ota_get_next_update_partition(NULL);
  _bluecherry_opdata.ota_buffer_pos = 0;

  if(_bluecherry_opdata.ota_state != BLUECHERRY_OTA_STATE_IDLE &&
     (_bluecherry_opdata.ota_partition == NULL ||
      _bluecherry_ota_slot_index(_bluecherry_opdata.ota_partition) !=
          _bluecherry_rtc.ota_partition_slot)) {
    ESP_LOGW(TAG, "OTA: the target slot moved across the sleep, abandoning the update");
    _bluecherry_ota_reset();
  }

  /* Empty the modem's socket buffer before the first exchange runs, so that nothing left over
   * from before the sleep answers it. Closing the socket is the only way to make the modem drop
   * what a drain could not; the session then goes with it and the next cycle dials a new one. */
  if(!_bluecherry_drain_socket()) {
    ESP_LOGW(TAG, "Socket %d would not empty, closing it", _bluecherry_opdata.sock);
    _bluecherry_cleanup_session();
    _bluecherry_rtc.magic = 0;
    return false;
  }

  ESP_LOGI(TAG, "Resumed the BlueCherry session on socket %d at message %u",
           _bluecherry_opdata.sock, _bluecherry_opdata.cur_message_id);

  /* The RESUME goes out once the slot is free. */
  if(_bluecherry_opdata.ota_resume_due && _bluecherry_opdata.pending_event_len == 0) {
    _bluecherry_ota_queue_resume();
  }

  _bluecherry_set_state(BLUECHERRY_STATE_IDLE);
  return true;
}

void WalterBlueCherry::_sleepPrepare()
{
  if(_bluecherry_opdata.state == BLUECHERRY_STATE_UNINITIALIZED) {
    _bluecherry_rtc.magic = 0;
    return;
  }

  /* Stop the synchronisation task first, so nothing is sent or changed after the snapshot. */
  portENTER_CRITICAL(&_bluecherry_state_lock);
  _sleep_requested = true;
  portEXIT_CRITICAL(&_bluecherry_state_lock);

  bool parked = _sync_task == NULL;
  if(!parked) {
    xTaskNotifyGive(_sync_task);
  }

  for(int waited_ms = 0; !parked && waited_ms < BLUECHERRY_SLEEP_WAIT_MS; waited_ms += 10) {
    portENTER_CRITICAL(&_bluecherry_state_lock);
    parked = _sync_parked;
    portEXIT_CRITICAL(&_bluecherry_state_lock);
    if(parked) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  if(!parked) {
    ESP_LOGW(TAG, "BlueCherry is still busy, the next boot starts a new session");
    _bluecherry_rtc.magic = 0;
    return;
  }

  if(_bluecherry_opdata.state != BLUECHERRY_STATE_IDLE) {
    ESP_LOGW(TAG, "Sleeping while BlueCherry is not idle; queued data may be lost. Wait for "
                  "BLUECHERRY_STATE_IDLE before sleeping.");
  }

  /* The staging buffer does not survive deep sleep, so an interrupted download resumes from what
   * is on flash after the wake. */
  _bluecherry_ota_prepare_resume();

  _bluecherry_rtc.sock = (int8_t) _bluecherry_opdata.sock;
  _bluecherry_rtc.tls_profile_id = _bluecherry_opdata.tls_profile_id;
  _bluecherry_rtc.pdp_ctx_id = (uint8_t) _bluecherry_opdata.pdp_ctx_id;

  /* The last frame sent counts as delivered, acknowledged or not: the first message after the wake
   * then carries an id the server has not seen, and reports nothing lost, so the server answers it
   * with new data instead of replaying its last frame. */
  _bluecherry_rtc.cur_message_id = _bluecherry_opdata.sent_message_id;
  _bluecherry_rtc.last_acked_message_id = _bluecherry_opdata.sent_message_id;

  _bluecherry_rtc.ota_state = (uint8_t) _bluecherry_opdata.ota_state;
  _bluecherry_rtc.ota_size = _bluecherry_opdata.ota_size;
  _bluecherry_rtc.ota_progress = _bluecherry_opdata.ota_progress;
  _bluecherry_rtc.ota_target_version = _bluecherry_opdata.ota_target_version;
  _bluecherry_rtc.ota_unverified = _bluecherry_opdata.ota_unverified;
  _bluecherry_rtc.ota_partition_slot = _bluecherry_ota_slot_index(_bluecherry_opdata.ota_partition);
  memcpy(_bluecherry_rtc.ota_expected_hash, _bluecherry_opdata.ota_expected_hash,
         BLUECHERRY_PARTITION_HASH_LEN);
  memcpy(_bluecherry_rtc.ota_skip_buffer, _bluecherry_opdata.ota_skip_buffer, ENCRYPTED_BLOCK_SIZE);
  _bluecherry_rtc.ota_resume_due = _bluecherry_opdata.ota_resume_due;

  _bluecherry_rtc.pending_event_len = (uint16_t) _bluecherry_opdata.pending_event_len;
  memcpy(_bluecherry_rtc.pending_event, _bluecherry_opdata.pending_event,
         BLUECHERRY_PENDING_EVENT_SIZE);

  _bluecherry_rtc.magic = BLUECHERRY_RTC_MAGIC;
}

#pragma endregion
#pragma region PRIVATE_BRIDGES

bool WalterBlueCherry::_networkUp()
{
  return WalterModem::_networkAttached;
}

int WalterBlueCherry::_reserveSocket()
{
  WalterModemSocket* sock = WalterModem::_socketReserve();
  return sock == NULL ? -1 : sock->id;
}

uint8_t* WalterBlueCherry::_otaBuffer()
{
  return _bluecherry_opdata.ota_buffer;
}

bool WalterBlueCherry::_motaDispatch(uint8_t event, uint8_t* data, uint16_t len)
{
#if CONFIG_WALTER_MODEM_ENABLE_MOTA
  switch(event) {
  case BLUECHERRY_EVENT_TYPE_MOTA_INITIALIZE:
    return WalterModem::_processMotaInitializeEvent(data, len);
  case BLUECHERRY_EVENT_TYPE_MOTA_CHUNK:
    return WalterModem::_processMotaChunkEvent(data, len);
  case BLUECHERRY_EVENT_TYPE_MOTA_FINISH:
    return WalterModem::_processMotaFinishEvent();
  default:
    break;
  }
#endif
  return false;
}

int WalterBlueCherry::_socketId()
{
  return _bluecherry_opdata.sock;
}

void WalterBlueCherry::_handleSocketEvent(WMSocketEventType event, uint16_t data_len)
{
  switch(event) {
  case WALTER_MODEM_SOCKET_EVENT_RING: {
    /* Read out here, the moment it is announced, the way a socket's receive buffer fills on WiFi.
     * A datagram left in the modem is handed over in answer to a later read instead, where it
     * answers the wrong exchange. */
    if(data_len == 0 || _bc_rx_buf == NULL || _bluecherry_opdata.sock <= 0) {
      break;
    }

    uint16_t want = data_len;
    if(want > sizeof(_bc_rx_scratch)) {
      ESP_LOGW(TAG, "A %uB datagram does not fit the %uB receive buffer and will be truncated",
               data_len, (unsigned) sizeof(_bc_rx_scratch));
      want = (uint16_t) sizeof(_bc_rx_scratch);
    }

    WalterModemRsp rsp = {};
    if(!WalterModem::socketReceive(_bluecherry_opdata.sock, _bc_rx_scratch, want, &rsp)) {
      ESP_LOGW(TAG, "Could not read the announced %uB from the BlueCherry socket", data_len);
      break;
    }

    size_t got = rsp.data.socketResponse.bytesReceived;
    if(got > want) {
      got = want;
    }

    if(got > 0 && xMessageBufferSend(_bc_rx_buf, _bc_rx_scratch, got, 0) != got) {
      ESP_LOGW(TAG, "Dropping a %uB BlueCherry datagram, the receive buffer is full",
               (unsigned) got);
    }
    break;
  }

  case WALTER_MODEM_SOCKET_EVENT_DISCONNECTED:
    _bc_peer_closed = true;
    break;

  default:
    break;
  }
}

#pragma endregion
#pragma region PUBLIC

bool WalterBlueCherry::init(uint8_t tls_profile_id, const char* device_type_id,
                            const BlueCherryPublishBuffer* publish_buffer)
{
  /* First, because this call is not passive: isProvisioned below issues AT commands, and the
   * command queue they go into is created by WalterModem::begin. Without this the send lands on a
   * NULL queue handle and aborts inside FreeRTOS instead of failing here. */
  if(!WalterModem::_initialized) {
    ESP_LOGE(TAG, "The modem must be started before BlueCherry can be initialized");
    return false;
  }

  if(device_type_id != NULL && strlen(device_type_id) != BLUECHERRY_ZTP_ID_LEN) {
    ESP_LOGE(TAG, "The BlueCherry device type must be %u characters", BLUECHERRY_ZTP_ID_LEN);
    return false;
  }

  /* The device type is re-supplied on every boot, including after a deep sleep, because it cannot
   * be carried across one. Handlers are re-installed the same way, through their own setters. */
  bc_type_id = device_type_id;

  if(_bluecherry_opdata.state != BLUECHERRY_STATE_UNINITIALIZED) {
    return true;
  }

  _bluecherry_opdata.tls_profile_id = tls_profile_id;
  _bluecherry_opdata.pdp_ctx_id = 1;
  _bluecherry_opdata.sock = -1;

  if(_bluecherry_ring_init(publish_buffer) != ESP_OK) {
    return false;
  }

  if(_bc_rx_buf == NULL) {
    _bc_rx_buf =
        xMessageBufferCreateStatic(sizeof(_bc_rx_buf_mem), _bc_rx_buf_mem, &_bc_rx_buf_struct);
    if(_bc_rx_buf == NULL) {
      ESP_LOGE(TAG, "Could not create the BlueCherry receive buffer");
      _bluecherry_ring_deinit();
      return false;
    }
  }

  /* Before the task exists, so it never runs a cycle against a budget a dial would trip. */
  _bluecherry_widen_watchdog(WalterModem::_watchdogTimeout);

  /* Started unconditionally: a sync request is a trigger, so the task has to exist even when the
   * application drives synchronisation itself. Guarded on the handle so a retried init cannot
   * leave two running. The stack has to carry the deepest operation, which is provisioning: a
   * DTLS handshake, the CBOR buffers and an EC key generation. */
  if(_sync_task == NULL) {
    BaseType_t ret = xTaskCreate(_bluecherry_sync_task, "bc_sync", BLUECHERRY_SYNC_TASK_STACK_SIZE,
                                 NULL, BLUECHERRY_SP, &_sync_task);
    if(ret != pdPASS) {
      _sync_task = NULL;
      ESP_LOGE(TAG, "Could not start the synchronisation task");
      _bluecherry_ring_deinit();
      return false;
    }
  }

  /* A session that survived a deep sleep is picked up here rather than in the modem's own wake
   * path, because that runs before the application has re-supplied any of the above. */
  if(_bluecherry_session_resume()) {
    /* Deliberately without a first cycle. The session is already up, so there is nothing left to
     * establish, and an exchange the application did not ask for costs it data and raises a state
     * change it has no reason to expect. A device that wakes every few seconds to read a sensor
     * and publishes once a day would spend its entire budget confirming a connection it was not
     * about to use. Downlink still arrives on the next requested sync, or on the auto sync
     * interval where one is set. */
    return true;
  }

  _bluecherry_set_state(isProvisioned() ? BLUECHERRY_STATE_AWAIT_CONNECTION
                                        : BLUECHERRY_STATE_NOT_PROVISIONED);

  /* A cold start has a connection or a provisioning run still ahead of it, so start the first
   * cycle rather than wait it out. */
  xTaskNotifyGive(_sync_task);
  return true;
}

bool WalterBlueCherry::provision(const char* cert_pem, const char* priv_key_pem,
                                 const char* ca_cert)
{
  if(cert_pem == NULL || priv_key_pem == NULL || ca_cert == NULL) {
    return false;
  }

  if(!WalterModem::tlsWriteCredential(false, BLUECHERRY_SLOT_DEVCERT, cert_pem)) {
    ESP_LOGE(TAG, "Could not write the device certificate to the modem");
    return false;
  }

  if(!WalterModem::tlsWriteCredential(true, BLUECHERRY_SLOT_PRIVKEY, priv_key_pem)) {
    ESP_LOGE(TAG, "Could not write the device private key to the modem");
    return false;
  }

  if(!WalterModem::tlsWriteCredential(false, BLUECHERRY_SLOT_CA, ca_cert)) {
    ESP_LOGE(TAG, "Could not write the CA chain to the modem");
    return false;
  }

  return true;
}

bool WalterBlueCherry::isProvisioned()
{
  return WalterModem::_tlsIsCredentialPresent(false, BLUECHERRY_SLOT_DEVCERT) &&
         WalterModem::_tlsIsCredentialPresent(false, BLUECHERRY_SLOT_CA) &&
         WalterModem::_tlsIsCredentialPresent(true, BLUECHERRY_SLOT_PRIVKEY);
}

bool WalterBlueCherry::publish(uint8_t topic, uint16_t len, const uint8_t* data)
{
  ESP_LOGD(TAG, "Scheduling publish on topic 0x%02X with %uB of data", topic, len);

  if(data == NULL && len > 0) {
    return false;
  }

  /* The wire length field is a single byte. The reference client accepts anything that fits a
   * frame and writes len & 0xFF, which produces a frame the server misparses with no error raised
   * on either side; refusing the call turns that into a visible failure. */
  if(len > BLUECHERRY_MAX_PUBLISH_LEN) {
    ESP_LOGE(TAG, "A message may carry at most %uB, not %uB", BLUECHERRY_MAX_PUBLISH_LEN, len);
    return false;
  }

  if(_bluecherry_ring_push(topic, len, data) != ESP_OK) {
    return false;
  }

  /* Reported before this returns rather than when the task gets round to it. An application
   * waiting for BLUECHERRY_STATE_IDLE before it sleeps would otherwise see the idle left over from
   * the previous cycle, conclude that nothing is outstanding, and sleep on top of the message it
   * just queued. */
  _bluecherry_request_sync();

  /* Something to send is reason enough to run: the interval exists to force an empty sync when
   * there is nothing queued, not to hold queued messages back. With automatic synchronisation off
   * there is no timer at all, so the message waits for the application to ask for a sync. */
  if(_auto_sync_interval_sec > 0 && _sync_task != NULL) {
    xTaskNotifyGive(_sync_task);
  }

  return true;
}

bool WalterBlueCherry::sync()
{
  if(_sync_task == NULL || _bluecherry_opdata.state == BLUECHERRY_STATE_UNINITIALIZED) {
    return false;
  }

  /* Same reason as in publish: the caller has asked for an exchange, so the state has to say one
   * is outstanding before this returns. Waiting for the task to say so leaves a window in which
   * the previous cycle's idle is still showing. A cycle with nothing to do settles straight back
   * to idle. */
  _bluecherry_request_sync();

  xTaskNotifyGive(_sync_task);
  return true;
}

bool WalterBlueCherry::setAutoSync(uint32_t interval_sec)
{
  if(_sync_task == NULL) {
    return false;
  }

  _auto_sync_interval_sec = interval_sec;
  _bluecherry_arm_auto_sync();

  if(interval_sec > 0) {
    /* Only when the new interval leaves one already overdue. Setting a schedule is not the same
     * as asking for a synchronisation, and anything still in the future is picked up by the
     * task's next wake anyway. */
    if(esp_timer_get_time() >= _next_auto_sync_us) {
      xTaskNotifyGive(_sync_task);
    }
    ESP_LOGI(TAG, "Automatic synchronisation every %lu s", (unsigned long) interval_sec);
  } else {
    ESP_LOGI(TAG, "Automatic synchronisation disabled");
  }

  return true;
}

BlueCherryState WalterBlueCherry::getState()
{
  return _bluecherry_opdata.state;
}

bool WalterBlueCherry::setStateHandler(blueCherryStateHandler handler, void* args)
{
  _bluecherry_opdata.state_handler_args = args;
  _bluecherry_opdata.state_handler = handler;
  return true;
}

bool WalterBlueCherry::setMsgHandler(blueCherryMsgHandler handler, void* args)
{
  _bluecherry_opdata.msg_handler = handler;
  _bluecherry_opdata.msg_handler_args = args;
  return true;
}

bool WalterBlueCherry::setOtaHandler(blueCherryOtaHandler handler, void* args)
{
  _bluecherry_opdata.ota_handler = handler;
  _bluecherry_opdata.ota_handler_args = args;
  return true;
}

bool WalterBlueCherry::otaStart()
{
  /* Advisory: the application may be on any task, and the offer could be withdrawn between this
   * check and the request being serviced. The task re-checks before acting. */
  if(_bluecherry_opdata.ota_state != BLUECHERRY_OTA_STATE_OFFERED) {
    ESP_LOGW(TAG, "otaStart: no update is on offer");
    return false;
  }

  _ota_start_req = true;
  return sync();
}

bool WalterBlueCherry::otaAbort(uint8_t error_code)
{
  if(_bluecherry_opdata.ota_state == BLUECHERRY_OTA_STATE_IDLE) {
    return false;
  }

  _ota_abort_code = error_code;
  _ota_abort_req = true;
  return sync();
}

bool WalterBlueCherry::close()
{
  _bluecherry_cleanup_session();
  _bluecherry_rtc.magic = 0;

  if(_bluecherry_opdata.state != BLUECHERRY_STATE_UNINITIALIZED &&
     _bluecherry_opdata.state != BLUECHERRY_STATE_NOT_PROVISIONED) {
    _bluecherry_set_state(BLUECHERRY_STATE_AWAIT_CONNECTION);
  }

  return true;
}

size_t WalterBlueCherry::getOtaProgressPercentage()
{
  return (size_t) _bluecherry_ota_progress_percent();
}

size_t WalterBlueCherry::getOtaProgressBytes()
{
  return (size_t) _bluecherry_opdata.ota_progress;
}

size_t WalterBlueCherry::getOtaSize()
{
  return (size_t) _bluecherry_opdata.ota_size;
}

#pragma endregion
#pragma region DEPRECATED

/* The BlueCherry API moved off WalterModem onto WalterBlueCherry. These forwarders keep the calls
 * that did not change meaning compiling, and say so at runtime as well as at build time.
 *
 * blueCherryInit and blueCherrySync are deliberately absent rather than forwarded: a sync is now a
 * trigger instead of a blocking exchange, and incoming messages arrive through a handler instead
 * of a response structure, so a forwarder would compile and then silently deliver nothing. Failing
 * to compile at the exact call site is the honest outcome. */

#define BLUECHERRY_DEPRECATED_WARN(replacement)                                                    \
  ESP_LOGW(TAG, "%s is deprecated, use %s instead", __func__, replacement)

bool WalterModem::blueCherryProvision(const char* cert_pem, const char* priv_key_pem,
                                      const char* ca_cert, WalterModemRsp* rsp, walterModemCb cb,
                                      void* args)
{
  BLUECHERRY_DEPRECATED_WARN("WalterBlueCherry::provision");
  return WalterBlueCherry::provision(cert_pem, priv_key_pem, ca_cert);
}

bool WalterModem::blueCherryIsProvisioned()
{
  BLUECHERRY_DEPRECATED_WARN("WalterBlueCherry::isProvisioned");
  return WalterBlueCherry::isProvisioned();
}

bool WalterModem::blueCherryPublish(uint8_t topic, uint8_t len, uint8_t* data)
{
  BLUECHERRY_DEPRECATED_WARN("WalterBlueCherry::publish");
  return WalterBlueCherry::publish(topic, len, data);
}

bool WalterModem::blueCherryClose(WalterModemRsp* rsp, walterModemCb cb, void* args)
{
  BLUECHERRY_DEPRECATED_WARN("WalterBlueCherry::close");
  return WalterBlueCherry::close();
}

size_t WalterModem::blueCherryGetOtaProgressPercentage()
{
  BLUECHERRY_DEPRECATED_WARN("WalterBlueCherry::getOtaProgressPercentage");
  return WalterBlueCherry::getOtaProgressPercentage();
}

size_t WalterModem::blueCherryGetOtaProgressBytes()
{
  BLUECHERRY_DEPRECATED_WARN("WalterBlueCherry::getOtaProgressBytes");
  return WalterBlueCherry::getOtaProgressBytes();
}

size_t WalterModem::blueCherryGetOtaSize()
{
  BLUECHERRY_DEPRECATED_WARN("WalterBlueCherry::getOtaSize");
  return WalterBlueCherry::getOtaSize();
}

#pragma endregion

#endif
