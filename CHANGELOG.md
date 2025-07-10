# WalterModem ChangeLog

This is the Changelog for WalterModem

## [V1.1.2](https://github.com/QuickSpot/walter-arduino/releases/tag/v1.1.2)

### features

- Added support for light sleep.

### Bug Fixes

- Changed the `CORE_DEBUG_LEVEL` macro to the `ARDUINO` macro to distinguish between ESP-IDF and Arduino.

## [V1.1.3](https://github.com/QuickSpot/walter-arduino/releases/tag/v1.1.3)

### Features

- Update component file with extra metadata.
- Updated examples to use component registry for ESP-IDF.

### Bug Fixes

- Fixed compile errors for `ESP-IDF V5.4.0`

## [V1.2.0](https://github.com/QuickSpot/walter-arduino/releases/tag/v1.1.2)

### Features

- Reworked MQTT implementation

#### event system

- Added System Event Handlers (Registration, System, AT)
- Added MQTT event handler
- Added HTTP event handler
- Added CoAP event handler
- Added GNSS event handler

#### restructuring

- Separated Socket implementation (proto/WalterSocket.cpp)
- Separated MQTT implementation (proto/WalterMQTT.cpp)
- Separated HTTP implementation (proto/WalterHTTP.cpp)
- Separated GNSS implementation (proto/WalterGNSS.cpp)
- Separated BlueCherry implementation (proto/WalterBlueCherry.cpp)

- Regionised the Code Base for easier navigation. (search by region)

- Separated Common Defines (WalterDefines.h)

#### CONFIG

- added support for [`KCONFIG`](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/kconfig.html) (ESP-IDF)
- added support for disabling unused protocols (ESP-IDF,ARDUINO)
- added CONFIG macro.

#### Breaking changes

1) createPDPContext => definePDPContext
2) authenticatePDPContext => setAuthParams
3) socketCreate => socketConfig
4) configSocket => socketConfigExtended
5) connectSocket => socketDial
6) clock is now a struct which contains the `epochTime` and the `timeZoneOffset`

`all examples have been reworked.`

## [v1.2.1](https://github.com/QuickSpot/walter-arduino/releases/tag/v1.2.1)

### Bug Fixes

- fixed `out of bounds` compile error in release mode ([#46](https://github.com/QuickSpot/walter-esp-idf/issues/46))
- fixed mqttConfig wrong defaults ([#47](https://github.com/QuickSpot/walter-esp-idf/issues/47))
- fixed bug in socketConfig where there was a redundant _pdpContextGet ([#51](https://github.com/QuickSpot/walter-esp-idf/issues/51))
- fixed bug where numbers where not converted correctly in socketListen ([#52](https://github.com/QuickSpot/walter-esp-idf/issues/52))
- fixed bug where the queueProcessingTask could get stuck due to having the same task priority (IDLE) ([#53](https://github.com/QuickSpot/walter-esp-idf/issues/53))
- fixed issue where socketListen did not have the option to pass the protocol ([#54](https://github.com/QuickSpot/walter-esp-idf/issues/54))
- fixed bug where the socketRing handler has the wrong starting offset ([#55](https://github.com/QuickSpot/walter-esp-idf/issues/55))
- fixed bug where numbers where not converted correctly in socketReceive ([#56](https://github.com/QuickSpot/walter-esp-idf/issues/56))
- fixed bug where tlsWriteCredential is private ([#66](https://github.com/QuickSpot/walter-esp-idf/issues/66))

### Docs

- updated documentation for definePDPContext([#50](https://github.com/QuickSpot/walter-esp-idf/issues/50))

## [v1.3.0](https://github.com/QuickSpot/walter-arduino/releases/tag/v1.3.0)

### Features

- completely reworked the parsing logic from scratch to make the parser more streamlined and performant ([#60](https://github.com/QuickSpot/walter-esp-idf/commit/96cf8a11f4b11a48cdcb1ac65b50a8466612b610))


### Bug Fixes

- fixed bug in the positioning example where the modem clock could have been set but not the gnss clock which would cause the example to crash ([#40](https://github.com/QuickSpot/walter-arduino/commit/1186de955351ab41da8b128c8cec31d735bc385d)).
- fixed issues with socketReceive ([#60](https://github.com/QuickSpot/walter-esp-idf/commit/96cf8a11f4b11a48cdcb1ac65b50a8466612b610)).
- fixed httpConfigProfile timeout bug which would cause crashes in some cases ([#20](https://github.com/QuickSpot/walter-esp-idf/commit/d3a7da0e66fbaf555d4ab2940043ca290a0c9177)).
- added delay to BlueCherry example, as it would otherwise continuously poll the server ([#72](https://github.com/QuickSpot/walter-esp-idf/commit/3d0f4e5ca57c1aac28dbc922682d8ec83f39f4f9)).

## [v1.3.1](https://github.com/QuickSpot/walter-arduino/releases/tag/v1.3.1)

### Bug Fixes

- fixed bug where tlsWriteCredential is private, `again`

## [v1.3.2](https://github.com/QuickSpot/walter-arduino/releases/tag/v1.3.2)

### Features

- feat(socket): add datareceived param to socketReceive. ([#86](https://github.com/QuickSpot/walter-esp-idf/pull/86))
- feat(socket): add receive example + (fix required bugs) ([#88](https://github.com/QuickSpot/walter-esp-idf/pull/88))
- feat(socket): add arduino example ([#49](https://github.com/QuickSpot/walter-arduino/pull/49))



### Bug Fixes

- fix(MQTT): mqttConfig function, prevent trailing commas when no TLS profile is used ([#78](https://github.com/QuickSpot/walter-esp-idf/pull/78))

- fix(modem) get network reg state ([#79](https://github.com/QuickSpot/walter-esp-idf/pull/79))
- fix(modem) compilation warnings ([#81](https://github.com/QuickSpot/walter-esp-idf/pull/81))
- fix(modem) define should be constexpr ([#82](https://github.com/QuickSpot/walter-esp-idf/pull/82))
- fix(socket): issue memcpy in socket urc handler ([#84](https://github.com/QuickSpot/walter-esp-idf/pull/84))
- fix(socket): bug where didRIng flag was not reset ([#90](https://github.com/QuickSpot/walter-esp-idf/pull/90))
- fix(socket): didRing flag not reset when reading using dataview mode ([#92](https://github.com/QuickSpot/walter-esp-idf/pull/92))
- fix(parser): update uart_config flow control treshhold ([#93](https://github.com/QuickSpot/walter-esp-idf/pull/93))
- fix(parser): buffer was not null on reset ([#97](https://github.com/QuickSpot/walter-esp-idf/pull/97))
- fix(parser): update params ([#48](https://github.com/QuickSpot/walter-arduino/pull/48))
- fix(parser): buffer was not null on reset ([#97](https://github.com/QuickSpot/walter-esp-idf/pull/97))

## [v1.3.3](https://github.com/QuickSpot/walter-arduino/releases/tag/v1.3.3)

### Bug Fixes

- fix(nvm): _tlsIsCredentialPresent ([#99](https://github.com/QuickSpot/walter-esp-idf/pull/99))
- fix(modem): socket receive + general bug fixes ([#102](https://github.com/QuickSpot/walter-esp-idf/pull/102))
- fix(socket): example updated begin function ([#58](https://github.com/QuickSpot/walter-arduino/pull/58))
- fix(walterfeels): updated walterfeels example ([#52](https://github.com/QuickSpot/walter-arduino/pull/52))
- fix(http): wrong int type in httpring handler ([#54](https://github.com/QuickSpot/walter-arduino/pull/54))
- fix(parser): dataPrompt bug where modem could hang

## [v1.4.0](https://github.com/QuickSpot/walter-arduino/releases/tag/v1.4.0)

### Features

- feat(http): added support for extra header line ([#106](https://github.com/QuickSpot/walter-esp-idf/pull/106))
- feat(bluecherry psm example): BlueCherry example update with deepsleep and PSM ([#109](https://github.com/QuickSpot/walter-esp-idf/pull/109))
feat(bluecherry-coap-restructure): bluecherry coap messages via udp modem sockets ([#62](https://github.com/QuickSpot/walter-arduino/pull/62))


### Bug Fixes

- fix(socket): added NO CARRIER support for socketDial ([#108](https://github.com/QuickSpot/walter-esp-idf/pull/108))
- fix(modem): fixed NO CARRIER bug _finishQueueCMd ([#110](https://github.com/QuickSpot/walter-esp-idf/pull/110))
- fix(examples): add socket config tls ([#111](https://github.com/QuickSpot/walter-esp-idf/pull/111))
- fix(socket-tls-examples): updated examples to disable tls by default ([#63](https://github.com/QuickSpot/walter-arduino/pull/63))
- fix(bluecherry): bluecherry port was not properly set (replaced by CONFIG macro)