# Examples

Get started right away with Walter and his LTE-M/NB-IoT connectivity together
with the GNSS functionality by using the examples. Also found in the examples 
directory are some sketches that are handy for diagnostic reasons. All examples
are GPLv3 licensed.

The included examples are:
 - [ModemPassthrough](ModemPassthrough/): by flashing this sketch onto Walter
   you are able to talk directly to the Sequans Monarch 2 chipset using AT
   commands.

 - [ModemTest](ModemTest/): this sketch makes Walter connect to a network and
   send out ping messages to our testserver. This sketch is perfect to test a
   connection without having Walter connected to any shield.

 - [SensorShield](SensorShield/): this sketch requires Walter to be placed onto
   a board which contains various sensors. It will read out the sensors and
   transmit them to the demo server every 60 seconds.

 - [Positioning](Positioning/): this sketch requires Walter to be placed onto
   a board which contains various sensors. It will read out the sensors and
   transmit them to the demo server every 60 seconds.