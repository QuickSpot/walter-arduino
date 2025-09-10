# Walter MQTTS Example

## Purpose

This example demonstrates how Walter can securely connect using **MQTT with TLS certificates**.
Walter will publish a counter value along with its MAC address to the MQTT broker on the topic.

## Required Hardware

To run this example you will need the following items:

* Walter
* An LTE antenna
* A SIM card with data plan
* USB-C cable to flash Walter

## Required Software

1. Follow the instructions in the [documentation](https://www.quickspot.io/index.html) to:

   * [Install](https://www.quickspot.io/documentation.html#/walter-modem/setup/arduino) the modem library
   * [Setup](https://www.quickspot.io/documentation.html#/developer-toolchains/arduino) the Arduino IDE for Walter

## Configuration

Before flashing the example, configure the TLS certificates and credentials:

* In the example sketch, update the following:

  ```cpp
   #define MQTTS_PORT 8883
   #define MQTTS_HOST 
   #define MQTTS_TOPIC "walter-tls-test-topic"
   #define MQTTS_CLIENT_ID "walter-client"
   #define MQTTS_USERNAME
   #define MQTTS_PASSWORD

  const char ca_cert[] PROGMEM             = R"EOF(-----BEGIN CERTIFICATE-----\n...\n-----END CERTIFICATE-----)EOF";
  const char client_cert[] PROGMEM         = R"EOF(-----BEGIN CERTIFICATE-----\n...\n-----END CERTIFICATE-----)EOF";
  const char client_private_key[] PROGMEM  = R"EOF(-----BEGIN RSA PRIVATE KEY-----\n...\n-----END RSA PRIVATE KEY-----)EOF";
  ```

## Running the Example

1. Connect the LTE antenna to Walter.
   **Warning:** Running without the antenna connected may damage the radio frontend.

2. Insert the SIM card.

3. Flash the example sketch to Walter.

4. Walter will connect to the MQTT broker over TLS (port 8883).

5. You should see messages with Walter’s MAC address and counter values appearing on the topic.
    You can monitor these messages on a subscribed client.
