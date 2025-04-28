# Walter Feels example

## Purpose

Walter Feels is an open source carrier board reference design for the Walter 
module. This example is a demonstration that reads out the internal power 
management and sensors. The data is than transmitted to the demo 
[server](http://walterdemo.quickspot.io/) as fast as possible.

## Required hardware

To run this example you will need the following items:

- Walter
- Walter Feels
- An LTE antenna
- A SIM card
- USB-C cable to flash Walter
- A power supply or battery to power Walter Feels

**Important: never connect the power and the USB-C cable at the same time. This
would lead to both power supplies to be connected. For development you can power
Walter Feels via the USB-C on Walter or use a USB-C cable with a cut +5V lead.**

## Required software

Please follow the instructions from the [documentation](https://www.quickspot.io/index.html)
to [install](https://www.quickspot.io/documentation.html#/walter-modem/setup/arduino) the modem library and [setup](https://www.quickspot.io/documentation.html#/developer-toolchains/arduino) the Arduino IDE for use with Walter.

## Run the example

Make sure to connect the LTE antenna to Walter. Running the example without the
antenna connected could damage the radio frontend of the modem. Also insert the
SIM card before starting the sketch.

Update the code to contain the correct APN settings for the SIM card that you
are using. The example is set up to make use of the Soracom SIM card that is
included in the Walter development kit.

You should now be able to see your Walter Feels pop up on the
[Walter Demo](http://walterdemo.quickspot.io/) website. Walter identifies itself
to the demo server using his MAC address.