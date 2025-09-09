# Walter AWS MQTT Example

## Purpose

This example demonstrates how Walter can securely connect to **AWS IoT Core** over **MQTT with TLS certificates**.
Walter will publish a counter value along with its MAC address to your AWS IoT Core endpoint on the topic.

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

2. Set up your **AWS IoT Core environment**:

   * Create an IoT Thing
   * Attach a policy allowing `iot:Connect`, `iot:Publish`, `iot:Subscribe`, `iot:Receive`
   * Download the following:

     * Device certificate (`.crt`)
     * Private key (`.key`)
     * Root CA (`.pem`)
   * Note your **AWS IoT Core endpoint** (found under *Settings* in the AWS IoT Core console).

## Configuration

Before flashing the example, configure the TLS certificates and endpoint:

* In the example sketch, update the following:

  ```cpp
  #define AWS_MQTTS_PORT PORT (typically 8883)
  #define AWS_MQTTS_ENDPOINT "AWS ENDPOINT"
  #define AWS_MQTTS_TOPIC "AWS TOPIC"
  #define AWS_MQTTS_CLIENT_ID "AWS CLIENT ID"

  const char ca_cert[] PROGMEM             = R"EOF(-----BEGIN CERTIFICATE-----\n...\n-----END CERTIFICATE-----)EOF";
  const char client_cert[] PROGMEM         = R"EOF(-----BEGIN CERTIFICATE-----\n...\n-----END CERTIFICATE-----)EOF";
  const char client_private_key[] PROGMEM  = R"EOF(-----BEGIN RSA PRIVATE KEY-----\n...\n-----END RSA PRIVATE KEY-----)EOF";
  ```

## Running the Example

1. Connect the LTE antenna to Walter.
   **Warning:** Running without the antenna connected may damage the radio frontend.

2. Insert the SIM card.

3. Flash the example sketch to Walter.

4. Walter will connect to the AWS IoT Core MQTT broker over TLS (port 8883).

5. You should see messages with Walter’s MAC address and counter values appearing on the topic.
    You can monitor these messages in the AWS IoT MQTT test client.
