# Walter HTTP example

## Purpose

This example demonstrates how Walter can send and receive requests using **HTTP**.
Walter will make HTTP GET and POST requests to [httpbin.org](https://httpbin.org), including a GET of
a 3.7 KB multi-line HTML page.

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

Before flashing the example, configure the routes and credentials:

* In the example sketch, update the following:

  ```cpp
  #define HTTP_PORT 80
  #define HTTP_HOST "httpbin.org"
  #define HTTP_GET_ENDPOINT "/get"
  #define HTTP_POST_ENDPOINT "/post"
  #define HTTP_LARGE_ENDPOINT "/html"
  ```

* Responses are received straight into `in_buf`, so make it large enough for the biggest response
  you expect. `httpReceive()` must be given the size reported by the ring event.

## Running the example

1. Connect the LTE antenna to Walter.
   **Warning:** Running without the antenna connected may damage the radio frontend.

2. Insert the SIM card.

3. Flash the example sketch to Walter.

4. You should see requests being performed, and the responses being logged.
