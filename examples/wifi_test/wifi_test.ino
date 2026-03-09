/**
 * @file wifi_test.ino
 * @brief Simple WiFi connectivity test for the Walter board (ESP32-S3).
 *
 * Connects to WiFi and performs an HTTP GET to httpbin.org/get
 * to verify internet connectivity. Prints the response to Serial.
 *
 * Setup:
 *   1. Copy config.h.example to private/config.h
 *   2. Fill in your WiFi SSID and password in private/config.h
 *   3. Flash and open Serial Monitor at 115200 baud
 */

#include <WiFi.h>
#include <HTTPClient.h>
#include "private/config.h"

void setup()
{
  Serial.begin(115200);
  delay(2000);

  Serial.println("\r\n=== Walter WiFi Connectivity Test ===\r\n");

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.printf("Connecting to WiFi \"%s\"", WIFI_SSID);
  int timeout = 0;
  while(WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
    timeout++;
    if(timeout > 40) {
      Serial.println("\r\nFailed to connect to WiFi. Check credentials.");
      return;
    }
  }

  Serial.println();
  Serial.printf("Connected! IP: %s\r\n\r\n", WiFi.localIP().toString().c_str());

  /* Perform HTTP GET */
  HTTPClient http;
  http.begin("http://httpbin.org/get");

  Serial.println("Sending HTTP GET to httpbin.org/get ...");
  int httpCode = http.GET();

  if(httpCode > 0) {
    Serial.printf("HTTP response code: %d\r\n", httpCode);
    Serial.println(http.getString());
  } else {
    Serial.printf("HTTP GET failed: %s\r\n", http.errorToString(httpCode).c_str());
  }

  http.end();
}

void loop()
{
  delay(30000);

  if(WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected.");
    return;
  }

  /* Repeat the GET every 30s so you can watch it in Serial Monitor */
  HTTPClient http;
  http.begin("http://httpbin.org/get");

  Serial.println("Sending HTTP GET to httpbin.org/get ...");
  int httpCode = http.GET();

  if(httpCode > 0) {
    Serial.printf("HTTP response code: %d\r\n", httpCode);
    Serial.println(http.getString());
  } else {
    Serial.printf("HTTP GET failed: %s\r\n", http.errorToString(httpCode).c_str());
  }

  http.end();
}
