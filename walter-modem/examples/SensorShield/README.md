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

Please follow the instructions from the main [README.md](../../README.md) file 
to install the modem library and set-up the Arduino IDE for use with Walter.
This example also requires the following publicly available Arduino libraries:
 - [Adafruit BusIO](https://github.com/adafruit/Adafruit_BusIO)
 - [Adafruit MPL3115A2](https://github.com/adafruit/Adafruit_MPL3115A2_Library)
 - [Adafruit LTR329](https://github.com/adafruit/Adafruit_LTR329_LTR303)

These can be installed via the Arduino IDE library manager.

## Run the example

Make sure to connect the LTE antenna to Walter. Running the example without the
antenna connected could damage the radio frontend of the modem. Also insert the
SIM card before starting the sketch.

Update the code to contain the correct APN settings for the SIM card that you
are using. The example is set up to make use of the Soracom SIM card that is 
included in the Walter development kit.

You should now be able to see your measurements pop up on the 
[Walter Demo](http://walterdemo.quickspot.io/) website. Walter identifies itself
to the demo server using his MAC address, this is how you can identify the graph
to look at.

## Data format

This example uploads data using a very simple UDP protocol which is ideal for 
NB-IoT and LTE-M but it could be that a measurement packet is lost. Currently we
are working on a more advanced protocol which offers 100% reliable data transfer
but without loosing the advantages of UDP.

Currently the system defines the following sensor types:
 - 0x01: Humidity
 - 0x02: Temperature
 - 0x03: Lux level
 - 0x04: Pressure

 The `Satcount` field is also used to indicate if a fix could be obtained or 
 not. When no valid fix could be obtained this field is set to 0xFF.

The protocol is very simple and supports sending measurements, GNSS only and 
measurements together with GNSS data. The type of packet is determined based on 
the datagram length. The folowing datagrams are supported:

### Ping packet (8B)

| MAC      | Counter  |
|----------|----------|
| 6B       | 2B       |
| Uint8[6] | Uint16BE |

### Minimal sensor packet (9B)

| MAC      | Sensor type | Value    |
|----------|-------------|----------|
| 6B       | 1B          | 2B       |
| Uint8[6] | Uint16BE    | Uint16BE |

### Sensor data packet (14B)

| MAC      | Humidity | Temperature | Lux      | Pressure |
|----------|----------|-------------|----------|----------|
| 6B       | 2B       | 2B          | 2B       | 2B       |
| Uint8[6] | Uint16BE | Uint16BE    | Uint16BE | Uint16BE |

### GNSS packet (15B)

| MAC      | Satcount | Latitude | Longitude |
|----------|----------|----------|-----------|
| 6B       | 1B       | 4B       | 4B        |
| Uint8[6] | Uint8    | IEEE 754 | IEEE 754  |

### Minimal sensor + GNSS packet (18B)

| MAC      | Sensor type | Value    | Satcount | Latitude | Longitude |
|----------|-------------|----------|----------|----------|-----------|
| 6B       | 1B          | 2B       | 1B       | 4B       | 4B        |
| Uint8[6] | Uint16BE    | Uint16BE | Uint8    | IEEE 754 | IEEE 754  |

### Sensor data + GNSS packet (23B)

| MAC      | Humidity | Temperature | Lux      | Pressure | Satcount | Latitude | Longitude |
|----------|----------|-------------|----------|----------|----------|----------|-----------|
| 6B       | 2B       | 2B          | 2B       | 2B       | 1B       | 4B       | 4B        |
| Uint8[6] | Uint16BE | Uint16BE    | Uint16BE | Uint16BE | Uint8    | IEEE 754 | IEEE 754  |