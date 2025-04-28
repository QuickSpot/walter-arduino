# Walter modem passthrough example

## Purpose

This example will use the ESP32 in Walter as a passthrough device and connect
the UART on the USB-C connector directly to the UART of the Sequans Monarch 2
chipset. Every time this sketch starts the modem will be physically reset.

## Required hardware

To run this example you will need the following items:

- Walter
- An LTE antenna (in case you are going to connect to a network)
- A SIM card (in case you are going to connect to a network)
- USB-C cable to flash Walter

## Required software

Please follow the instructions from the [documentation](https://www.quickspot.io/index.html)
to [install](https://www.quickspot.io/documentation.html#/walter-modem/setup/arduino) the modem library and [setup](https://www.quickspot.io/documentation.html#/developer-toolchains/arduino) the Arduino IDE for use with Walter.

## Run the example

Make sure to connect the LTE antenna to Walter. Enabling the transceiver inside
the modem without the antenna connected could damage the radio frontend of the
modem.

You can now use a terminal program such as PuTTY or miniterm to easily type in
commands. Make sure to set the baudrate to 115200@8N1 and enable local echo if
you want to see what your are typing in. You can check communication by entering
`AT\r\n`, you should get an `OK` back from the modem.
