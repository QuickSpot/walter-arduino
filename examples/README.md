# Examples

Get started right away with Walter and his LTE-M/NB-IoT connectivity together
with the GNSS functionality by using the examples. Also found in the examples 
directory are some sketches that are handy for diagnostic reasons. All examples
are GPLv3 licensed.

The included examples are:
 - [passtrough](passthrough/): by flashing this sketch onto Walter
   you are able to talk directly to the Sequans Monarch 2 chipset using AT
   commands.

 - [udp_socket](udp_socket/): this sketch makes Walter connect to a network and
   send out UDP messages to our testserver. This sketch is perfect to test a
   connection without having Walter connected to any shield.

 - [coap](coap/): this sketch makes Walter connect to a network and
   send out COAP messages to a public COAP test server.

 - [http_querry](http_querry/): this sketch makes Walter connect to a network and
   perform a HTTP GET to a test server.

 - [http_send](http_send/): this sketch makes Walter connect to a network and
   perform a HTTP POST to a test server.

 - [mqtt](mqtt/): this sketch makes Walter connect to a network and
   send and receive MQTT messages using the MQTT commands in the modem.

 - [bluecherry](bluecherry/): this sketch makes Walter connect to a network and
   send and receive MQTT messages and an ota update using the DPTechnics COAP to MQTT bridge.

 - [sensor_shield](sensor_shield/): this sketch requires Walter to be placed onto
   a board which contains various sensors. It will read out the sensors and
   transmit them to the demo server every 60 seconds.

 - [positioning](positioning/): this sketch requires Walter to be placed onto
   a board which contains various sensors. It will read out the sensors and
   transmit them to the demo server every 60 seconds.

 - [mota](mota/): this sketch updates the modem firmware.
   Please read the comments at the top for info on how to put the
   firmware image on the ESP32 flash before running the sketch.
