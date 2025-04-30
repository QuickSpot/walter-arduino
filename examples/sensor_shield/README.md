# Walter sensor shield example

## Purpose

This example requires Walter to be plugged in on a sensor shield. Walter will
now make use of the sensors and send a measurement to the demo
[server](http://walterdemo.quickspot.io/) every 60 seconds. Without modification
this example is a high resolution remote weather station solution.

## Required hardware

To run this example you will need the following items:

- Walter
- An LTE antenna
- The Walter Sensor Shield
- A SIM card
- USB-C cable to flash Walter

## Required software

Please follow the instructions from the [documentation](https://www.quickspot.io/index.html)
to [install](https://www.quickspot.io/documentation.html#/walter-modem/setup/arduino) the modem library and [setup](https://www.quickspot.io/documentation.html#/developer-toolchains/arduino) the Arduino IDE for use with Walter.
This example also requires the following publicly available Arduino libraries:

- [Adafruit BusIO](https://github.com/adafruit/Adafruit_BusIO)
- [Adafruit MPL3115A2](https://github.com/adafruit/Adafruit_MPL3115A2_Library)
- [Adafruit LTR329](https://github.com/adafruit/Adafruit_LTR329_LTR303)

These can be installed via the Arduino IDE library manager.

## Run the example

Make sure to connect the LTE antenna to Walter. Running the example without the
antenna connected could damage the radio frontend of the modem. Also insert the
SIM card before starting the sketch.

You should now be able to see your measurements pop up on the
[Walter Demo](http://walterdemo.quickspot.io/) website. Walter identifies itself
to the demo server using his MAC address, this is how you can identify the graph
to look at.
