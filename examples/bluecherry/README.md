# Walter Bluecherry example

## Purpose

This sketch sends and receives mqtt data using the DPTechnics BlueCherry cloud platform.
It also supports OTA updates which are scheduled through the BlueCherry
[web interface](https://blueapp.bluecherry.io).

Provisioning is zero-touch and handled by the library: the sketch only supplies the 8-character
device type (`BC_DEVICE_TYPE`) and the modem TLS profile BlueCherry may use (`BC_TLS_PROFILE`).
There are no certificates to paste in and no extra source files to add to the sketch folder.

## Required Hardware

To run this example you will need the following items:

- Walter
- An LTE antenna
- A SIM card
- USB-C cable to flash Walter

## Required Software

Please follow the instructions from the [documentation](https://www.quickspot.io/index.html)
to [install](https://www.quickspot.io/documentation.html#/walter-modem/setup/arduino) the modem library and [setup](https://www.quickspot.io/documentation.html#/developer-toolchains/arduino) the Arduino IDE for use with Walter.

In the Tools menu, make sure `PSRAM` is set to `QSPI PSRAM`. The sketch places the publish buffer
there and falls back to internal RAM with a warning when it is unavailable.

## Run the example

Make sure to connect the LTE antenna to Walter. Running the example without the
antenna connected could damage the radio frontend of the modem. Also insert the
SIM card before starting the sketch.
