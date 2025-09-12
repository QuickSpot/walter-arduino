# Walter UDP example

## Purpose

This example will make Walter construct two data packets and send it to our demo
[server](http://walterdemo.quickspot.io/) every 30 seconds.

## Required hardware

To run this example you will need the following items:

- Walter
- An LTE antenna
- A SIM card
- USB-C cable to flash Walter

## Required software

1. Follow the instructions in the [documentation](https://www.quickspot.io/index.html) to:

   * [Install](https://www.quickspot.io/documentation.html#/walter-modem/setup/arduino) the modem library
   * [Setup](https://www.quickspot.io/documentation.html#/developer-toolchains/arduino) the Arduino IDE for Walter

## Running the example

1. Connect the LTE antenna to Walter.
   **Warning:** Running without the antenna connected may damage the radio frontend.

2. Insert the SIM card.

3. Flash the example sketch to Walter.

4. You should see requests being performed, and the responses being logged.
