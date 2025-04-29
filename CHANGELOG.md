# WalterModem ChangeLog

This is the Changelog for WalterModem

## [V1.1.2](https://github.com/QuickSpot/walter-esp-idf/releases/tag/v1.1.2)

### features

- Added support for light sleeep.

### Bug Fixes

- Changed the `CORE_DEBUG_LEVEL` macro to the `ARDUINO` macro to distinguish between ESP-IDF and Arduino.

## [V1.1.3](https://github.com/QuickSpot/walter-esp-idf/releases/tag/v1.1.3)

### Features

- Update component file with extra metadata.
- Updated examples to use component registry for ESP-IDF.

### Bug Fixes

- Fixed compile errors for `ESP-IDF V5.4.0`


## [V1.2.0](https://github.com/QuickSpot/walter-esp-idf/releases/tag/v1.1.2)

### Features

- Reworked MQTT implementation

#### event system

- Added System Event Handlers (Registration, System, AT)
- Added MQTT event handler
- Added HTTP event handler
- Added CoAP event handler
- Added GNSS event handler

#### restructuring

- Seperated Socket implementation (proto/WalterSocket.cpp)
- Seperated MQTT implementation (proto/WalterMQTT.cpp)
- Seperated HTTP implementation (proto/WalterHTTP.cpp)
- Seperated GNSS implementation (proto/WalterGNSS.cpp)
- Seperated BlueCherry implementation (proto/WalterBlueCherry.cpp)

- Regionised the Code Base for easier navigation.

- Seperated Common Defines (WalterDefines.h)

#### CONFIG

- added support for [`KCONFIG`](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/kconfig.html) (ESP-IDF)
- added support for disabeling unused protocols (ESP-IDF,ARDUINO)
- added CONFIG macro.

#### Breaking changes

1) createPDPContext => definePDPContext
2) authenticatePDPContext => setAuthParams
3) socketCreate => socketConfig
4) configSocket => socketConfigExtended
5) connectSocket => socketDial
6) clock is now a struct wich contains the `epochTime` and the `timeZoneOffset`

`all examples have been reworked.`
