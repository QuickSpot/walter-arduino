# Walter modem library for Arduino

[![arduino-library-badge](https://www.ardu-badge.com/badge/WalterModem.svg?)](https://www.ardu-badge.com/WalterModem)


## Introduction

Walter is a board designed by [DPTechnics](https://www.dptechnics.com) which 
combines an [ESP32-S3](https://www.espressif.com/en/products/socs/esp32-s3) and
a [Sequans Monarch 2](https://www.sequans.com/products/monarch-2-gm02sp) in a
small form factor IoT module. This gives Walter a vast amount of wireless
connectivity options such as:
- Bluetooth Low Energy 5.0
- 1T1R WiFi b/g/n
- LTE Cat-M1 (LTE-M)
- LTE Cat-NB1 (NB-IoT rel. 13)
- LTE Cat-NB2 (NB-IoT rel. 14), ready for rel. 15
- GNSS receiver (GPS and Galileo)

Besides these you get all the goodies from the ESP32-S3 chip such as  SPI, I2S,
I2C, PWM, RMT, ADC, UART, SD/MMC host and TWAI. 

We design and manufacture Walter in Belgium and guarantee that the board will be
available for a minimum of 10 years. This makes Walter a solid choice to design
your next LPWAN IoT product with.

The Walter modem library makes it easy to interface with the Sequans Monarch 2
modem on the Arduino platform. The library allows for UDP and TCP communication
over NB-IoT and LTE-M networks and also supports the GNSS functionality. 

This library is designed to consume as little energy as possible by making use
of the FreeRTOS locking mechanisms and the hardware UART. There are not active
wait situations which consume useless CPU cycles. Besides that the library
does not allocate dynamic heap memory. All RAM is determined at compiled time.
This makes debugging easier and mitigates unexpected out-of-memory situations.

All Walter libraries and software are written with the same design strategies:
 - Keep it simple
 - Be as efficient as possible
 - Keep the code documented
 - Do not change underlying frameworks

This means that Walter is very easy to use in the Arduino ecosystem as you can
use the `ESP32S3 Dev Module` board from the standard 
[Arduino core for the ESP32-S3](https://github.com/espressif/arduino-esp32).
More information about how to set this up can be found in the 
[documentation repository](https://github.com/QuickSpot/walter-documentation).

## Set up the library

The library is designed to be used with the Arduino IDE or any variant such as
the VSCode plugins. You need to start by installing the ESP32 core for Arduino
using the [official documentation](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html#installing-using-arduino-ide).
After installation of the most recent ESP32 core you must select the following
settings in de IDE:
 - Board: DPTechnics Walter
 - Port: the serial port on which Walter is connected
 - CPU Frequency: any frequency you like, the slower the lower power you consume
 - Core Debug Level: set this to "Debug" if you want to see the modem AT commands 
 - USB DFU On Boot: Disabled
 - Erase Asll Flash Before Sketch Upload: Enabled
   (must be Enabled to make modem firmware updates possible)
 - Events Run On: Core 1
 - Flash Mode: QIO 80MHz
 - Flash Size: 16MB (128Mb)
 - JTAG Adapter: Disabled
 - Arduino Runs On: Core 1
 - USB Firmware MSC On Boot: Disabled
 - Partition Scheme: 16M Flash (2MB APP/12.5MB FATFS)
   (must be this partition scheme to make modem firmware updates possible)
 - PSRAM: QSPI PSRAM
 - Upload Mode: UART0 / Hardware CDC
 - Upload Speed: 921600
 - USB Mode: Hardware CDC and JTAG

Now you need to clone this repository in the `libraries` directory of your 
Arduino IDE. The location of this folder depends on your OS and installation,
typical locations are:
 - Windows: `%USERPROFILE%\Documents\Arduino\libraries`
 - Mac OSX: `~/Documents/Arduino/libraries`
 - Linux: `~/sketchbook/libraries`

That's it, you can now run any of the examples and connect to the internet.

## Contributions

We welcome all contributions to the software via github pull requests. Please
take the design strategies in mind when contributing. 

## License

The library is published under the 'DPTechnics 5 clause' license. This is 
essentially the same as the `BSD-3-Clause` license with the addition that
binaries of which the source code is not open should run on a Walter board from
DPTechnics bv.
