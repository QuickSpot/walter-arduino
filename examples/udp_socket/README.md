# Walter modem udp socket example

## Purpose

This example will make Walter count and send the counter value to our demo
[server](http://walterdemo.quickspot.io/) every 10 seconds. It allows you to
test connectivity without creating a motherboard to plug Walter in.

## Required hardware

To run this example you will need the following items:

- Walter
- An LTE antenna
- A SIM card
- USB-C cable to flash Walter

## Required software

Please follow the instructions from the [documentation](https://www.quickspot.io/index.html)
to [install](https://www.quickspot.io/documentation.html#/walter-modem/setup/arduino) the modem library and [setup](https://www.quickspot.io/documentation.html#/developer-toolchains/arduino) the Arduino IDE for use with Walter.

## Run the example

Make sure to connect the LTE antenna to Walter. Running the example without the
antenna connected could damage the radio frontend of the modem. Also insert the
SIM card before starting the sketch.

You should now be able to see your Walter pop up on the
[Walter Demo](http://walterdemo.quickspot.io/) website. Walter identifies itself
to the demo server using his MAC address. You should see the last sent ping
counter value update every time Walter sends out a message.
