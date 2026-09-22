/**
 * @file WalterBlueCherry.h
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
 * The BlueCherry cloud client. BlueCherry speaks CoAP over DTLS while presenting an MQTT shaped
 * API of publish and subscribe on single byte topics, and carries firmware updates for both the
 * ESP32 and the modem.
 *
 * It sits on top of WalterModem rather than inside it: the modem provides the DTLS secured UDP
 * socket and holds the credentials in its own NVM, and everything above that - the CoAP framing,
 * the message queue, the OTA state machine and Zero-Touch Provisioning - is implemented here.
 *
 * An application includes this header instead of WalterModem.h, which it pulls in itself:
 *
 *     WalterModem modem;
 *     WalterBlueCherry blueCherry;
 *
 *     modem.begin(UART_NUM_1);
 *     blueCherry.init(BC_TLS_PROFILE, BC_DEVICE_TYPE);
 *     blueCherry.setMsgHandler(msgHandler);
 *     blueCherry.publish(0x84, len, data);
 *     blueCherry.sync();
 *     while(blueCherry.getState() != BLUECHERRY_STATE_IDLE) {
 *       vTaskDelay(pdMS_TO_TICKS(100));
 *     }
 *     modem.sleep(300);
 *
 * The enumerator names below are deliberately the BLUECHERRY_* ones used by the bluecherry-esp-idf
 * client rather than WalterModem-prefixed ones. The protocol logic is shared between the two
 * implementations line for line, so keeping the names identical is what lets a fix on one side be
 * carried to the other without rewriting it. The type names are BlueCherry* rather than
 * WalterModem*: BlueCherry is its own client now, not a WalterModem abstraction, and the
 * reference's snake_case bluecherry_*_t names leave the CamelCase spelling free.
 */

#ifndef WALTER_BLUECHERRY_H
#define WALTER_BLUECHERRY_H

#include "WalterModem.h"

#if CONFIG_BLUECHERRY_ENABLE

#pragma region CONFIGURATION

/* Each of these is a Kconfig symbol under menu "BlueCherry". Under ESP-IDF the value arrives in
 * sdkconfig.h; Arduino has no Kconfig and no sdkconfig.h, so the literal below is what it takes.
 * The symbols are BlueCherry's own and are read without regard to WALTER_MODEM_KCONFIG, which
 * configures the modem driver. */

/**
 * @brief The default hostname for Bluecherry.
 */
#ifndef CONFIG_BLUECHERRY_HOSTNAME
#define CONFIG_BLUECHERRY_HOSTNAME "coap.bluecherry.io"
#endif
static constexpr const char* BLUECHERRY_HOSTNAME = CONFIG_BLUECHERRY_HOSTNAME;

/**
 * @brief The default port for Bluecherry CoAP.
 */
#ifndef CONFIG_BLUECHERRY_PORT
#define CONFIG_BLUECHERRY_PORT 5684
#endif
static constexpr uint16_t BLUECHERRY_PORT = CONFIG_BLUECHERRY_PORT;

/**
 * @brief The port of the BlueCherry Zero-Touch Provisioning server. Shares the host with
 * BLUECHERRY_HOSTNAME.
 */
#ifndef CONFIG_BLUECHERRY_ZTP_PORT
#define CONFIG_BLUECHERRY_ZTP_PORT 5688
#endif
static constexpr uint16_t BLUECHERRY_ZTP_PORT = CONFIG_BLUECHERRY_ZTP_PORT;

/**
 * @brief The size of the buffer holding messages waiting to be published, in bytes.
 *
 * Used only when the application passes no buffer of its own to WalterBlueCherry::init. Each
 * queued message costs its payload plus 9 bytes.
 */
#ifndef CONFIG_BLUECHERRY_PUBLISH_BUFFER_SIZE
#define CONFIG_BLUECHERRY_PUBLISH_BUFFER_SIZE 4096
#endif
static constexpr int BLUECHERRY_PUBLISH_BUFFER_SIZE = CONFIG_BLUECHERRY_PUBLISH_BUFFER_SIZE;

/**
 * @brief The stack of the BlueCherry synchronisation task, in bytes.
 *
 * The task runs every network operation BlueCherry performs. The deepest path is Zero-Touch
 * Provisioning: a DTLS handshake, then CBOR exchanges holding a few kilobytes in nested frames,
 * then a SECP256R1 key generation. 4096 is not enough for that and overflows.
 */
#ifndef CONFIG_BLUECHERRY_SYNC_TASK_STACK_SIZE
#define CONFIG_BLUECHERRY_SYNC_TASK_STACK_SIZE 8192
#endif
static constexpr int BLUECHERRY_SYNC_TASK_STACK_SIZE = CONFIG_BLUECHERRY_SYNC_TASK_STACK_SIZE;

/**
 * @brief The task watchdog budget the BlueCherry synchronisation task needs, in seconds.
 *
 * The watchdog has one timeout shared by every subscribed task, so BlueCherry widens the whole
 * timer to this when it is currently narrower, and never narrows it. A cycle feeds the watchdog
 * around each blocking step, but socketDial cannot be broken up: the DTLS handshake runs inside
 * AT+SQNSD and is allowed 20 seconds on its own. Arduino has no Kconfig, so it takes this literal.
 */
#ifndef CONFIG_BLUECHERRY_WDT_TIMEOUT_S
#define CONFIG_BLUECHERRY_WDT_TIMEOUT_S 60
#endif
static constexpr int BLUECHERRY_WDT_TIMEOUT_S = CONFIG_BLUECHERRY_WDT_TIMEOUT_S;

#pragma endregion
#pragma region CONSTANTS

/**
 * @brief The length of a partition SHA-256 digest.
 */
#define BLUECHERRY_PARTITION_HASH_LEN 32

/**
 * @brief The maximum size of an incoming BlueCherry message payload.
 */
constexpr uint16_t BLUECHERRY_MAX_INCOMING_MESSAGE_LEN = 1220;

/**
 * @brief The size of the firmware staging buffer.
 *
 * One flash sector, because that is the granularity esp_partition_erase_range works in. The modem
 * firmware upgrade borrows the same buffer and caps its STP transfer blocks to this.
 */
#define BLUECHERRY_OTA_BUFFER_SIZE SPI_FLASH_SEC_SIZE

/**
 * @brief The smallest publish buffer that can be passed to init.
 */
#define BLUECHERRY_MIN_PUBLISH_BUFFER 256

/**
 * @brief The largest payload a single publish call accepts.
 *
 * The wire length field is a single byte, so this is 255 and not the 1017 bytes that would
 * otherwise fit in a frame. The reference client accepts the larger value and writes len & 0xFF,
 * which produces a frame the server misparses with no error raised on either side.
 */
#define BLUECHERRY_MAX_PUBLISH_LEN 255

/**
 * @brief The number of characters in a BlueCherry Type ID or Device ID.
 */
#define BLUECHERRY_ZTP_ID_LEN 8

#pragma endregion
#pragma region ENUMS

/**
 * @brief The states the BlueCherry connection can be in.
 *
 * Read with getState or subscribe with setStateHandler. BLUECHERRY_STATE_IDLE is the only state in
 * which nothing is outstanding in either direction, and therefore the point at which a sleepy
 * device can call WalterModem::sleep without stranding data. sync only starts the work, so waiting
 * for it to return is not the same thing.
 */
typedef enum {
  /**
   * @brief Nothing has been allocated yet.
   */
  BLUECHERRY_STATE_UNINITIALIZED = 0,

  /**
   * @brief Allocated, but the modem holds no device credentials yet.
   *
   * The next sync runs Zero-Touch Provisioning before connecting.
   */
  BLUECHERRY_STATE_NOT_PROVISIONED,

  /**
   * @brief Credentials are present but there is no live session.
   */
  BLUECHERRY_STATE_AWAIT_CONNECTION,

  /**
   * @brief Session up and nothing outstanding in either direction. Safe to sleep.
   */
  BLUECHERRY_STATE_IDLE,

  /**
   * @brief A confirmable message is on the wire.
   *
   * Transient and entered once per transmitted message, so a state handler sees it often.
   */
  BLUECHERRY_STATE_AWAITING_RESPONSE,

  /**
   * @brief The server has data queued, an internal reply is waiting to go out, or the outgoing
   * queue is not empty. The synchronisation task keeps cycling until this clears.
   */
  BLUECHERRY_STATE_PENDING_MESSAGES
} BlueCherryState;

/**
 * @brief The device identifier types the ZTP server accepts.
 */
typedef enum {
  BLUECHERRY_ZTP_DEVICE_ID_TYPE_MAC = 0,
  BLUECHERRY_ZTP_DEVICE_ID_TYPE_IMEI,
  BLUECHERRY_ZTP_DEVICE_ID_TYPE_OOB_CHALLENGE
} BlueCherryZtpDeviceIdType;

/**
 * @brief OTA error codes reported to the cloud.
 */
typedef enum {
  BLUECHERRY_OTA_ERR_NO_PARTITION = 1,
  BLUECHERRY_OTA_ERR_TOO_LARGE = 2,
  BLUECHERRY_OTA_ERR_ERASE_FAILED = 3,
  BLUECHERRY_OTA_ERR_WRITE_FAILED = 4,
  BLUECHERRY_OTA_ERR_HASH_MISMATCH = 5,
  BLUECHERRY_OTA_ERR_BAD_MAGIC = 6,
  BLUECHERRY_OTA_ERR_SET_BOOT_FAILED = 7,
  BLUECHERRY_OTA_ERR_APP_ABORTED = 8,
  BLUECHERRY_OTA_ERR_CHUNK_OVERRUN = 9
} BlueCherryOtaError;

/**
 * @brief OTA events reported to the application.
 */
typedef enum {
  /**
   * @brief An update is available, details are in BlueCherryOtaInfo.
   *
   * Carries a decision: the download. Return true and nothing happens until otaStart is called,
   * with no deadline. Raised again on every reconnect while the update is still on offer.
   */
  BLUECHERRY_OTA_EVENT_AVAILABLE,

  /**
   * @brief The download has begun. Carries no decision.
   */
  BLUECHERRY_OTA_EVENT_STARTED,

  /**
   * @brief bytes_received of size written so far. Carries no decision.
   *
   * Emitted once per batch that reaches flash, not once per received chunk.
   */
  BLUECHERRY_OTA_EVENT_PROGRESS,

  /**
   * @brief The image is written, hashed, acknowledged by the server and the boot partition is
   * set. Walter keeps running the old firmware until it restarts.
   *
   * Carries a decision: the reboot. Return true and it is yours to schedule with esp_restart.
   */
  BLUECHERRY_OTA_EVENT_COMPLETE,

  /**
   * @brief The update failed, error_code says why. Carries no decision.
   */
  BLUECHERRY_OTA_EVENT_FAILED
} BlueCherryOtaEvent;

#pragma endregion
#pragma region STRUCTS

/**
 * @brief Details accompanying a BlueCherryOtaEvent.
 */
typedef struct {
  /**
   * @brief The BlueCherry firmware version being offered or installed.
   */
  int8_t version;

  /**
   * @brief The total image size in bytes.
   */
  uint32_t size;

  /**
   * @brief The expected image SHA-256, or all zeroes.
   *
   * All zeroes means the cloud holds no fingerprint for this build, so the download cannot be
   * checked against one. The hash is computed and reported either way, and is the value an
   * operator uses to populate it.
   */
  uint8_t sha256[BLUECHERRY_PARTITION_HASH_LEN];

  /**
   * @brief Bytes written to flash so far, for BLUECHERRY_OTA_EVENT_PROGRESS.
   */
  uint32_t bytes_received;

  /**
   * @brief A BlueCherryOtaError, for BLUECHERRY_OTA_EVENT_FAILED.
   */
  uint8_t error_code;
} BlueCherryOtaInfo;

/**
 * @brief Where the queue of messages waiting to be published lives.
 *
 * Passed to init, or NULL to let the library allocate
 * CONFIG_BLUECHERRY_PUBLISH_BUFFER_SIZE bytes in internal RAM. Supplying a buffer is
 * how an application decides both the size of the queue and the memory it comes out of; PSRAM and
 * static arrays both work.
 *
 * The buffer does not survive deep sleep unless the application placed it in RTC memory itself.
 * That is not normally a concern, because the contract is to sync until the state is
 * BLUECHERRY_STATE_IDLE and only then sleep, at which point the queue is empty.
 */
typedef struct {
  /**
   * @brief The buffer, which the library borrows and never frees.
   *
   * It must stay valid until the application stops using BlueCherry.
   */
  uint8_t* buffer;

  /**
   * @brief The size of buffer in bytes, at least BLUECHERRY_MIN_PUBLISH_BUFFER.
   */
  size_t size;
} BlueCherryPublishBuffer;

#pragma endregion
#pragma region HANDLERS

/**
 * @brief Handler for incoming messages on any topic other than the internal 0x00 channel.
 *
 * Called on the synchronisation task with a pointer into the receive buffer, so it must copy
 * anything it keeps and must not block.
 *
 * @param topic The single byte topic index the cloud maps to an MQTT topic.
 * @param len The number of bytes in data.
 * @param data The payload, valid only for the duration of the call.
 * @param args The argument given to setMsgHandler.
 *
 * @return None.
 */
typedef void (*blueCherryMsgHandler)(uint8_t topic, uint16_t len, const uint8_t* data, void* args);

/**
 * @brief Handler notified of every connection state change.
 *
 * Called on every transition and must not block. Usually that is the synchronisation task, but a
 * publish or a sync request reports the work it just created before returning, so it can also run
 * on whichever task made that call.
 *
 * @param state The state just entered.
 * @param args The argument given to setStateHandler.
 *
 * @return None.
 */
typedef void (*blueCherryStateHandler)(BlueCherryState state, void* args);

/**
 * @brief Handler for OTA events.
 *
 * Return true when this call took the decision the event carries, false to leave it to the
 * library. Two events carry one: BLUECHERRY_OTA_EVENT_AVAILABLE (start the download) and
 * BLUECHERRY_OTA_EVENT_COMPLETE (reboot). For the other three the return value is ignored.
 *
 * A handler that returns false everywhere is therefore equivalent to registering none at all, so
 * watching an update cannot accidentally stop one.
 *
 * @param event The event that occurred.
 * @param info Details for the event, valid only for the duration of the call.
 * @param args The argument given to setOtaHandler.
 *
 * @return True if the application took this event's decision, false to apply the default.
 */
typedef bool (*blueCherryOtaHandler)(BlueCherryOtaEvent event, const BlueCherryOtaInfo* info,
                                     void* args);

#pragma endregion

#pragma region CLASS

/**
 * @brief The BlueCherry cloud client.
 *
 * Like WalterModem this class has only static members, so instantiating it is cosmetic and there
 * is exactly one BlueCherry connection per process.
 */
class WalterBlueCherry
{
  friend class WalterModem;

public:
  /**
   * @brief Initialize BlueCherry and start the synchronisation task.
   *
   * Performs no network I/O and cannot fail because the cloud is unreachable, so it needs no
   * retry loop. It does talk to the modem, so WalterModem::begin must have succeeded first. It
   * must be called on every boot, including after deep sleep: it is what resumes a session that
   * survived the sleep, which does not persist on its own.
   *
   * Handlers are installed separately once this returns, with setMsgHandler, setOtaHandler and
   * setStateHandler. They do not survive a deep sleep either, so they are re-installed on the
   * same path.
   *
   * When the modem holds no device credentials the state becomes BLUECHERRY_STATE_NOT_PROVISIONED
   * and the first sync runs Zero-Touch Provisioning, which requires device_type_id.
   *
   * Firmware updates are accepted by default. An application that does not want them installs an
   * OTA handler with setOtaHandler and, on BLUECHERRY_OTA_EVENT_AVAILABLE, either returns true
   * without ever calling otaStart - which defers the update indefinitely - or calls otaAbort to
   * refuse it outright.
   *
   * @param tls_profile_id The modem TLS profile to use. BlueCherry owns NVM slots 0, 5 and 6.
   * @param device_type_id The 8 character BlueCherry Type ID, required for provisioning only.
   * @param publish_buffer Where to queue outgoing messages, or NULL to allocate the Kconfig
   * default in internal RAM.
   *
   * @return True on success, false on error.
   */
  static bool init(uint8_t tls_profile_id, const char* device_type_id = NULL,
                   const BlueCherryPublishBuffer* publish_buffer = NULL);

  /**
   * @brief Upload BlueCherry credentials to the modem.
   *
   * Writes the device certificate to NVM slot 5, the private key to slot 0 and the CA to slot 6.
   * Only needed to provision Walter by hand; Zero-Touch Provisioning calls this itself.
   *
   * @param cert_pem The device certificate in PEM format.
   * @param priv_key_pem The device private key in PEM format.
   * @param ca_cert The BlueCherry CA chain in PEM format.
   *
   * @return True on success, false on error.
   */
  static bool provision(const char* cert_pem, const char* priv_key_pem, const char* ca_cert);

  /**
   * @brief Check whether the modem holds BlueCherry credentials.
   *
   * Reads NVM slots 5, 6 and 0. The key itself is never readable back out of the modem.
   *
   * @return True when all three are present, false otherwise.
   */
  static bool isProvisioned();

  /**
   * @brief Queue a message for publication.
   *
   * Never touches the network: the message is framed into the publish buffer and goes out on the
   * next synchronisation. The bound is bytes rather than messages, and a full buffer fails the
   * call without dropping anything already queued.
   *
   * @param topic The single byte topic index the cloud maps to an MQTT topic. 0x00 is reserved.
   * @param len The number of bytes in data, at most BLUECHERRY_MAX_PUBLISH_LEN.
   * @param data The payload, which is copied.
   *
   * @return True when queued, false on error.
   */
  static bool publish(uint8_t topic, uint16_t len, const uint8_t* data);

  /**
   * @brief Ask the synchronisation task to run a cycle.
   *
   * Returns as soon as the task is signalled, not when the exchange completes. Wait for getState
   * to report BLUECHERRY_STATE_IDLE, or use a state handler, to know that everything queued has
   * gone out and everything queued at the cloud has come in.
   *
   * The state leaves BLUECHERRY_STATE_IDLE before this returns, so a caller may poll it straight
   * away without racing the task.
   *
   * @return True when the task was signalled, false when BlueCherry is not initialized.
   */
  static bool sync();

  /**
   * @brief Synchronise on a timer in addition to on demand.
   *
   * The interval is the worst case latency for downlink delivery on a device that stays awake. A
   * sleepy device normally leaves this off and calls sync itself, so that it controls how many
   * cycles run before it sleeps.
   *
   * @param interval_sec The interval in seconds, or 0 to disable.
   *
   * @return True on success, false when BlueCherry is not initialized.
   */
  static bool setAutoSync(uint32_t interval_sec);

  /**
   * @brief Get the current connection state.
   *
   * @return The current BlueCherryState.
   */
  static BlueCherryState getState();

  /**
   * @brief Install a handler notified of every connection state change.
   *
   * @param handler The handler, or NULL to remove the current one.
   * @param args Optional user arguments for the handler.
   *
   * @return True on success, false on error.
   */
  static bool setStateHandler(blueCherryStateHandler handler, void* args = NULL);

  /**
   * @brief Install a handler for incoming messages.
   *
   * Optional, but recommended: it is the only way an application sees its downlink. There is no
   * default one to fall back on, because the payloads are application data the library cannot
   * interpret - unlike OTA, which it drives itself when no handler is installed. Without one an
   * incoming message is acknowledged like any other and then discarded.
   *
   * @param handler The handler, or NULL to ignore incoming messages.
   * @param args Optional user arguments for the handler.
   *
   * @return True on success, false on error.
   */
  static bool setMsgHandler(blueCherryMsgHandler handler, void* args = NULL);

  /**
   * @brief Install a handler for OTA events.
   *
   * Optional. Without one the library downloads an update as soon as it is offered and reboots
   * when it is installed, and a handler that returns false everywhere behaves identically.
   *
   * @param handler The handler, or NULL to remove the current one.
   * @param args Optional user arguments for the handler.
   *
   * @return True on success, false on error.
   */
  static bool setOtaHandler(blueCherryOtaHandler handler, void* args = NULL);

  /**
   * @brief Accept an offered firmware update.
   *
   * Only needed when an OTA handler took the BLUECHERRY_OTA_EVENT_AVAILABLE decision. There is no
   * deadline, so an application may defer the download to a convenient moment.
   *
   * @return True when the download was requested, false when no update is on offer.
   */
  static bool otaStart();

  /**
   * @brief Abandon the firmware update in progress and tell the cloud why.
   *
   * The cloud allows three attempts before it stops offering the update.
   *
   * @param error_code A BlueCherryOtaError reported to the cloud.
   *
   * @return True when the abort was queued, false when no update is in progress.
   */
  static bool otaAbort(uint8_t error_code = BLUECHERRY_OTA_ERR_APP_ABORTED);

  /**
   * @brief Close the BlueCherry session and release the modem socket.
   *
   * Only needed to drop a session deliberately. Deep sleep keeps the modem powered, so a session
   * normally survives it and is resumed by the next init without a new handshake.
   *
   * @return True on success, false on error.
   */
  static bool close();

  /**
   * @brief Get the progress of the firmware update in progress, as a percentage.
   *
   * @return The percentage of the image written to flash.
   */
  static size_t getOtaProgressPercentage();

  /**
   * @brief Get the progress of the firmware update in progress, in bytes.
   *
   * @return The number of bytes written to flash.
   */
  static size_t getOtaProgressBytes();

  /**
   * @brief Get the total size of the firmware update in progress.
   *
   * @return The announced image size in bytes, or 0 when no update is running.
   */
  static size_t getOtaSize();

  /**
   * =============================================================================================
   * Bridges across the modem driver boundary. Not part of the API. Most run outward, and exist
   * because the implementation is written as file static helpers rather than as members, and the
   * friendship that lets this class reach WalterModem's internals does not extend to those.
   * _otaBuffer runs the other way: the op data is a file static, so the modem driver needs a
   * handle to borrow the staging buffer for an STP transfer.
   * =============================================================================================
   */

  /**
   * @brief Reserve a modem socket for the BlueCherry session.
   *
   * @return The reserved socket id, or -1 when the pool is exhausted.
   */
  static int _reserveSocket();

  /**
   * @brief Whether the modem last reported an attached network.
   *
   * @return True when a dial has something to dial into.
   */
  static bool _networkUp();

  /**
   * @brief Lend the staging buffer to the modem firmware upgrade.
   *
   * The ESP32 update reaches it as an ordinary member; this exists only because the STP transfer
   * lives in the modem driver, on the other side of the op data's translation unit.
   *
   * @return One flash sector, borrowable only while no OTA chunk can arrive.
   */
  static uint8_t* _otaBuffer();

  /**
   * @brief Hand a modem firmware update event to the modem driver.
   *
   * Events 5 to 8 belong to the modem rather than to the application processor, so BlueCherry only
   * routes them. The handlers report whether an error should be sent back to the cloud, so the
   * sense of the result is inverted compared with everything else here.
   *
   * @param event The BlueCherry event type, which must be one of the modem firmware events.
   * @param data The event payload, without the event type byte.
   * @param len The length of the payload.
   *
   * @return True when the cloud should be told the update failed.
   */
  static bool _motaDispatch(uint8_t event, uint8_t* data, uint16_t len);

private:
  /**
   * @brief The modem socket id carrying the BlueCherry session, or -1 when there is none.
   *
   * Called by WalterModem's event processing task to recognise socket events belonging to
   * BlueCherry before they are dispatched to the application's socket event handler.
   *
   * @return The socket id, or -1.
   */
  static int _socketId();

  /**
   * @brief Record a socket event for the BlueCherry socket.
   *
   * Called from WalterModem's event processing task. It only notes the event, since issuing an AT
   * command there would stall every other event behind it; the synchronisation task performs the
   * actual AT+SQNSRECV.
   *
   * @param event The socket event that occurred.
   * @param data_len The number of bytes the modem reports as available, for a ring event.
   *
   * @return None.
   */
  static void _handleSocketEvent(WMSocketEventType event, uint16_t data_len);

  /**
   * @brief Snapshot the resumable BlueCherry state into RTC memory before deep sleep.
   *
   * Called from WalterModem::_sleepPrepare. Also flushes a partially staged OTA sector to flash,
   * since the staging buffer lives in regular RAM which deep sleep does not preserve.
   *
   * @return None.
   */
  static void _sleepPrepare();
};

#pragma endregion

#endif
#endif
