# Examples

Get started right away with Walter and his LTE-M/NB-IoT connectivity together
with the GNSS functionality by using the examples. Also found in the examples 
directory are some sketches that are handy for diagnostic reasons. All examples
are GPLv3 licensed.

The included examples are:
 - [ModemPassthrough](ModemPassthrough/): by flashing this sketch onto Walter
   you are able to talk directly to the Sequans Monarch 2 chipset using AT
   commands.

 - [ModemSocketTest](ModemSocketTest/): this sketch makes Walter connect to a network and
   send out UDP messages to our testserver. This sketch is perfect to test a
   connection without having Walter connected to any shield.

 - [ModemCoapTest](ModemCoapTest/): this sketch makes Walter connect to a network and
   send out COAP messages to a public COAP test server.

 - [ModemHttpTest](ModemHttpTest/): this sketch makes Walter connect to a network and
   perform a HTTP GET to a test server.

 - [ModemHttpSendTest](ModemHttpSendTest/): this sketch makes Walter connect to a network and
   perform a HTTP POST to a test server.

 - [ModemMqttTest](ModemMqttTest/): this sketch makes Walter connect to a network and
   send and receive MQTT messages using the MQTT commands in the modem.

 - [ModemBlueCherryTest](ModemBlueCherryTest/): this sketch makes Walter connect to a network and
   send and receive MQTT messages and an ota update using the DPTechnics COAP to MQTT bridge.

 - [SensorShield](SensorShield/): this sketch requires Walter to be placed onto
   a board which contains various sensors. It will read out the sensors and
   transmit them to the demo server every 60 seconds.

 - [Positioning](Positioning/): this sketch requires Walter to be placed onto
   a board which contains various sensors. It will read out the sensors and
   transmit them to the demo server every 60 seconds.
