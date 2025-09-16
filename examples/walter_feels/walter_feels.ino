#include <Arduino.h>
#include <HardwareSerial.h>
#include <WalterModem.h>
#include <Wire.h>
#include "esp_mac.h"
#include "esp_sleep.h"
#include "hdc1080.h"
#include "lps22hb.h"
#include "ltc4015.h"
#include "scd30.h"
#include "WalterFeels.h"

#define CO2_EN_PIN 13
#define CO2_SDA_PIN 12
#define CO2_SCL_PIN 11

/**
 * @brief The address of the server to upload the data to.
 */
#define SERV_ADDR "walterdemo.quickspot.io"

/**
 * @brief The UDP port on which the server is listening.
 */
#define SERV_PORT 1999

/**
 * @brief The size in bytes of a minimal sensor + GNSS packet.
 */
#define PACKET_SIZE 39

/**
 * @brief All fixes with a confidence below this number are considered ok.
 */
#define MAX_GNSS_CONFIDENCE 200.0

/**
 * @brief The first RAT to try to connect to.
 */
#define PREFERRED_RAT WALTER_MODEM_RAT_LTEM

/**
 * @brief Flag used to signal when a fix is received.
 */
volatile bool fixRcvd = false;

/**
 * @brief The modem instance.
 */
WalterModem modem;

/**
 * @brief The buffer to transmit to the UDP server. The first 6 bytes will be
 * the MAC address of the Walter this code is running on.
 */
uint8_t dataBuf[PACKET_SIZE] = { 0 };

/**
 * @brief The last received GNSS fix.
 */
WalterModemGNSSFix posFix = {};

/**
 * @brief The HDC1080 temperature/humidity sensor instance.
 */
// HDC1080 hdc1080;

/**
 * @brief The LPS22HB barometric pressure sensor instance.
 */
LPS22HB lps22hb(Wire);

/**
 * @brief The SCD30 CO2 sensor instance.
 */
// SCD30 scd30;

/**
 * @brief Use UART1 for modem communication.
 */
HardwareSerial ModemSerial(1);

/**
 * @brief User UART2 for communication with modbus sensors.
 */
HardwareSerial SensorSerial(2);

/**
 * @brief Time duration in seconds indicating how long the ESP should stay in
 * deep-sleep.
 */
const uint32_t SLEEP_DURATION = 300;

/**
 * @brief The binary configuration settings for eDRX.
 */
const char* edrxCycle = "0010";
const char* edrxPTW = "0000";

/**
 * @brief The binary configuration settings for PSM.
 * These can be calculated using e.g.
 * https://www.soracom.io/psm-calculation-tool/
 */
const char* psmActive = "00000001";
const char* psmTAU = "00000110";

/**
 * @brief This function checks if we are connected to the LTE network
 *
 * @return true when connected, false otherwise
 */
bool lteConnected()
{
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  return (regState == WALTER_MODEM_NETWORK_REG_REGISTERED_HOME ||
          regState == WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING);
}

/**
 * @brief This function waits for the modem to be connected to the LTE network.
 *
 * @param timeout_sec The amount of seconds to wait before returning a time-out.
 *
 * @return true if connected, false on time-out.
 */
bool waitForNetwork(int timeout_sec = 300)
{
  Serial.print("Connecting to the network...");
  int time = 0;
  while(!lteConnected()) {
    Serial.print(".");
    delay(1000);
    time++;
    if(time > timeout_sec)
      return false;
  }
  Serial.println();
  Serial.println("Connected to the network");
  return true;
}

/**
 * @brief Disconnect from the LTE network.
 *
 * This function will disconnect the modem from the LTE network and block until
 * the network is actually disconnected. After the network is disconnected the
 * GNSS subsystem can be used.
 *
 * @return true on success, false on error.
 */
bool lteDisconnect()
{
  /* Set the operational state to minimum */
  if(modem.setOpState(WALTER_MODEM_OPSTATE_MINIMUM)) {
    Serial.println("Successfully set operational state to MINIMUM");
  } else {
    Serial.println("Error: Could not set operational state to MINIMUM");
    return false;
  }

  /* Wait for the network to become available */
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  while(regState != WALTER_MODEM_NETWORK_REG_NOT_SEARCHING) {
    delay(100);
    regState = modem.getNetworkRegState();
  }

  Serial.println("Disconnected from the network");
  return true;
}

/**
 * @brief This function tries to connect the modem to the cellular network.
 *
 * @return true on success, false on error.
 */
bool lteConnect()
{
  /* Set the operational state to NO RF */
  if(modem.setOpState(WALTER_MODEM_OPSTATE_NO_RF)) {
    Serial.println("Successfully set operational state to NO RF");
  } else {
    Serial.println("Error: Could not set operational state to NO RF");
    return false;
  }

  /* Create PDP context */
  if(modem.definePDPContext()) {
    Serial.println("Created PDP context");
  } else {
    Serial.println("Error: Could not create PDP context");
    return false;
  }

  /* Set the operational state to full */
  if(modem.setOpState(WALTER_MODEM_OPSTATE_FULL)) {
    Serial.println("Successfully set operational state to FULL");
  } else {
    Serial.println("Error: Could not set operational state to FULL");
    return false;
  }

  /* Set the network operator selection to automatic */
  if(modem.setNetworkSelectionMode(WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC)) {
    Serial.println("Network selection mode was set to automatic");
  } else {
    Serial.println("Error: Could not set the network selection mode to automatic");
    return false;
  }

  return waitForNetwork();
}

/**
 * @brief Check the assistance data in the modem response.
 *
 * This function checks the availability of assistance data in the modem's
 * response. This function also sets a flag if any of the assistance databases
 * should be updated.
 *
 * @param rsp The modem response to check.
 * @param updateAlmanac Pointer to the flag to set when the almanac should be
 * updated.
 * @param updateEphemeris Pointer to the flag to set when ephemeris should be
 * updated.
 *
 * @return None.
 */
void checkAssistanceData(WalterModemRsp* rsp, bool* updateAlmanac = NULL,
                         bool* updateEphemeris = NULL)
{
  if(updateAlmanac != NULL) {
    *updateAlmanac = false;
  }

  if(updateEphemeris != NULL) {
    *updateEphemeris = false;
  }

  Serial.print("Almanac data is ");
  if(rsp->data.gnssAssistance.almanac.available) {
    Serial.printf("available and should be updated within %ds\r\n",
                  rsp->data.gnssAssistance.almanac.timeToUpdate);
    if(updateAlmanac != NULL) {
      *updateAlmanac = rsp->data.gnssAssistance.almanac.timeToUpdate <= 0;
    }
  } else {
    Serial.print("not available.\r\n");
    if(updateAlmanac != NULL) {
      *updateAlmanac = true;
    }
  }

  Serial.print("Real-time ephemeris data is ");
  if(rsp->data.gnssAssistance.realtimeEphemeris.available) {
    Serial.printf("available and should be updated within %ds\r\n",
                  rsp->data.gnssAssistance.realtimeEphemeris.timeToUpdate);
    if(updateEphemeris != NULL) {
      *updateEphemeris = rsp->data.gnssAssistance.realtimeEphemeris.timeToUpdate <= 0;
    }
  } else {
    Serial.print("not available.\r\n");
    if(updateEphemeris != NULL) {
      *updateEphemeris = true;
    }
  }
}

/**
 * @brief Update GNSS assistance data if needed.
 *
 * Checks the GNSS almanac and real-time ephemeris validity and, if required,
 * connects to the LTE network to download fresh assistance data. Ensures the
 * system clock is synchronized before updating.
 *
 * @return True on success, false on error (e.g., LTE connection or modem
 *         command failed).
 */
bool updateGNSSAssistance()
{
  WalterModemRsp rsp = {};

  lteDisconnect();

  /* Even with valid assistance data the system clock could be invalid */
  modem.gnssGetUTCTime(&rsp);

  if(rsp.data.clock.epochTime <= 4) {
    /* The system clock is invalid, connect to LTE network to sync time */
    if(!lteConnect()) {
      Serial.print("Could not connect to LTE network\r\n");
      return false;
    }

    /*
     * Wait for the modem to synchronize time with the LTE network, try 5 times
     * with a delay of 500ms.
     */
    for(int i = 0; i < 5; ++i) {
      modem.gnssGetUTCTime(&rsp);

      if(rsp.data.clock.epochTime > 4) {
        Serial.printf("Synchronized clock with network: %" PRIi64 "\r\n", rsp.data.clock.epochTime);
        break;
      } else if(i == 4) {
        Serial.print("Could not sync time with network\r\n");
        return false;
      }

      delay(500);
    }
  }

  /* Check the availability of assistance data */
  if(!modem.gnssGetAssistanceStatus(&rsp) ||
     rsp.type != WALTER_MODEM_RSP_DATA_TYPE_GNSS_ASSISTANCE_DATA) {
    Serial.print("Could not request GNSS assistance status\r\n");
    return false;
  }

  bool updateAlmanac = false;
  bool updateEphemeris = false;
  checkAssistanceData(&rsp, &updateAlmanac, &updateEphemeris);

  if(!(updateAlmanac || updateEphemeris)) {
    if(lteConnected()) {
      if(!lteDisconnect()) {
        Serial.print("Could not disconnect from the LTE network\r\n");
        return false;
      }
    }

    return true;
  }

  if(!lteConnected()) {
    if(!lteConnect()) {
      Serial.print("Could not connect to LTE network\r\n");
      return false;
    }
  }

  if(updateAlmanac) {
    if(!modem.gnssUpdateAssistance(WALTER_MODEM_GNSS_ASSISTANCE_TYPE_ALMANAC)) {
      Serial.print("Could not update almanac data\r\n");
      return false;
    }
  }

  if(updateEphemeris) {
    if(!modem.gnssUpdateAssistance(WALTER_MODEM_GNSS_ASSISTANCE_TYPE_REALTIME_EPHEMERIS)) {
      Serial.print("Could not update real-time ephemeris data\r\n");
      return false;
    }
  }

  if(!modem.gnssGetAssistanceStatus(&rsp) ||
     rsp.type != WALTER_MODEM_RSP_DATA_TYPE_GNSS_ASSISTANCE_DATA) {
    Serial.print("Could not request GNSS assistance status\r\n");
    return false;
  }

  checkAssistanceData(&rsp);

  if(!lteDisconnect()) {
    Serial.print("Could not disconnect from the LTE network\r\n");
    return false;
  }

  return true;
}

/**
 * @brief Connect to an UDP socket.
 *
 * This function will set-up the modem and connect an UDP socket. The LTE
 * connection must be active before this function can be called.
 *
 * @param ip The IP address of the server to connect to.
 * @param port The port to connect to.
 *
 * @return True on success, false on error.
 */
bool socketConnect(const char* ip, uint16_t port)
{
  /* Construct a socket */
  if(modem.socketConfig()) {
    Serial.print("Created a new socket\r\n");
  } else {
    Serial.print("Could not create a new socket\r\n");
    return false;
  }

  /* disable socket tls as the demo server does not use it */
  if(modem.socketConfigSecure(false)) {
    Serial.print("Configured TLS\r\n");
  } else {
    Serial.print("Could not configure TLS\r\n");
    return false;
  }

  /* Connect to the UDP test server */
  if(modem.socketDial(ip, port)) {
    Serial.printf("Connected to UDP server %s:%d\r\n", ip, port);
  } else {
    Serial.print("Could not connect UDP socket\r\n");
    return false;
  }

  return true;
}

void performGnssFix()
{
  /* Check clock and assistance data, update if required */
  if(!updateGNSSAssistance()) {
    Serial.print("Could not update GNSS assistance data\r\n");
    delay(1000);
    ESP.restart();
    return;
  }

  /* Try up to 5 times to get a good fix */
  for(int i = 0; i < 5; ++i) {
    fixRcvd = false;
    if(!WalterModem::gnssPerformAction()) {
      Serial.print("Could not request GNSS fix\r\n");
      delay(1000);
      ESP.restart();
      return;
    }
    Serial.print("Started GNSS fix\r\n");

    int j = 0;
    while(!fixRcvd) {
      Serial.print(".");
      if(j >= 300) {
        Serial.println("");
        Serial.println("Timed out while waiting for GNSS fix");
        delay(1000);
        ESP.restart();
        break;
      }
      j++;
      delay(500);
    }

    Serial.println("");

    if(posFix.estimatedConfidence > MAX_GNSS_CONFIDENCE) {
      posFix.satCount = 0;
      posFix.latitude = 0.0;
      posFix.longitude = 0.0;
      Serial.print("Could not get a valid fix\r\n");
    }
  }

  uint8_t abovedBTreshold = 0;
  for(int i = 0; i < posFix.satCount; ++i) {
    if(posFix.sats[i].signalStrength >= 30) {
      abovedBTreshold += 1;
    }
  }

  Serial.printf("GNSS fix attempt finished:\r\n"
                "  Confidence: %.02f\r\n"
                "  Latitude: %.06f\r\n"
                "  Longitude: %.06f\r\n"
                "  Satcount: %d\r\n"
                "  Good sats: %d\r\n",
                posFix.estimatedConfidence, posFix.latitude, posFix.longitude, posFix.satCount,
                abovedBTreshold);
}

/**
 * @brief This function is called when a fix attempt finished.
 *
 * This function is called by Walter's modem library as soon as a fix attempt
 * has finished. This function should be handled as an interrupt and should be
 * as short as possible as it is called within the modem data thread.
 *
 * @param fix The fix data.
 * @param args Optional arguments, a NULL pointer in this case.
 *
 * @return None.
 */
void fixHandler(const WalterModemGNSSFix* fix, void* args)
{
  memcpy(&posFix, fix, sizeof(WalterModemGNSSFix));
  fixRcvd = true;
}

void setup_charger()
{
  LTC4015::initialize(3, 4);
  LTC4015::suspend_charging();
  LTC4015::enable_force_telemetry();
  delay(1000);
  LTC4015::start_charging();
  LTC4015::enable_mppt();
  LTC4015::enable_coulomb_counter();
}

void setup()
{
  Serial.begin(115200);
  delay(5000);

  Serial.print("Walter Feels\r\n");

  WalterModemRsp rsp = {};

  /* Get the MAC address for board validation */
  esp_read_mac(dataBuf, ESP_MAC_WIFI_STA);
  Serial.printf("Walter's MAC is: %02X:%02X:%02X:%02X:%02X:%02X\r\n", dataBuf[0], dataBuf[1],
                dataBuf[2], dataBuf[3], dataBuf[4], dataBuf[5]);

  WalterFeels::set3v3(true);
  WalterFeels::setI2cBusPower(true);

  Serial.println("MAIN: Configuring I2C...");

  // Initialize I2C with same pins and frequency
  if(!Wire.begin(WFEELS_PIN_I2C_SDA, WFEELS_PIN_I2C_SCL, 100000)) {
    Serial.println("MAIN: I2C config failed");
    return;
  }

  // Wire1.begin(CO2_SDA_PIN, CO2_SCL_PIN);
  // Serial.println("Waiting for CO2 sensor to boot");
  // delay(100);

  // bool co2_sensor_installed = scd30.begin(Wire1);
  // for(int i = 0; i < 10 && !co2_sensor_installed; ++i) {
  //   delay(500);
  //   co2_sensor_installed = scd30.begin(Wire1);
  // }

  // if(!co2_sensor_installed) {
  //   digitalWrite(CO2_EN_PIN, HIGH);
  //   Serial.println("Warning: No CO2 sensor is installed");
  // } else {
  //   Serial.println("Sensirion SCD30 CO2 sensor is installed");
  // }

  Serial.println("MAIN: I2C configured");

  delay(2000);

  setup_charger();
  // hdc1080.begin();

  delay(2000);
  lps22hb.begin();

  /* Initialize the LTE modem library */
  if(WalterModem::begin(&ModemSerial)) {
    Serial.println("Modem initialization success");
  } else {
    Serial.println("Error: Modem initialization fault");
    delay(1000);
    ESP.restart();
    return;
  }

  // Connect to cellular network
  if(!lteConnect()) {
    Serial.println("Error: Unable to connect to cellular network, restarting Walter "
                   "in 10 seconds");
    delay(10000);
    ESP.restart();
  }

  WalterModem::configPSM(WALTER_MODEM_PSM_ENABLE, psmTAU, psmActive);

  if(!WalterModem::getCellInformation(WALTER_MODEM_SQNMONI_REPORTS_SERVING_CELL, &rsp)) {
    Serial.println("Could not request cell information");
  } else {
    Serial.printf("Connected on band %u using operator %s (%u%02u)", rsp.data.cellInformation.band,
                  rsp.data.cellInformation.netName, rsp.data.cellInformation.cc,
                  rsp.data.cellInformation.nc);
    Serial.printf(" and cell ID %u.\r\n", rsp.data.cellInformation.cid);
    Serial.printf("Signal strength: RSRP: %.2f, RSRQ: %.2f.\r\n", rsp.data.cellInformation.rsrp,
                  rsp.data.cellInformation.rsrq);
  }

  if(WalterModem::getIdentity(&rsp)) {
    Serial.print("Modem identity:\r\n");
    Serial.printf(" -IMEI: %s\r\n", rsp.data.identity.imei);
    Serial.printf(" -IMEISV: %s\r\n", rsp.data.identity.imeisv);
    Serial.printf(" -SVN: %s\r\n", rsp.data.identity.svn);
  }

  delay(500);

  if(WalterModem::getSIMCardID(&rsp)) {
    Serial.print("SIM card identity:\r\n");
    Serial.printf(" -ICCID: %s\r\n", rsp.data.simCardID.iccid);
    Serial.printf(" -eUICCID: %s\r\n", rsp.data.simCardID.euiccid);
  }

  if(WalterModem::getSIMCardIMSI(&rsp)) {
    Serial.printf("Active IMSI: %s\r\n", rsp.data.imsi);
  }

  if(!WalterModem::gnssConfig()) {
    Serial.println("Could not configure the GNSS subsystem");
    delay(1000);
    ESP.restart();
    return;
  }

  WalterModem::gnssSetEventHandler(fixHandler);

  /* Init modem and charger on initial boot */
  // if(esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED) {
  //   WalterFeels::setI2cBusPower(false);
  //   LTC4015::initialize();
  //   LTC4015::enable_coulomb_counter();
  //   WalterFeels::setI2cBusPower(false);
  // }

  /* Initialize the sensors and read values */

  float temp = 0; // hdc1080.readTemperature();
  float hum = 0;  // hdc1080.readHumidity();
  float pressure = lps22hb.readPressure();
  uint16_t co2ppm = 0; // co2_sensor_installed ? scd30.getCO2() : 0;

  Serial.printf("Sensor data, temp: %.02fC, hum: %.02f%%, press: %.02fhPa, co2: %d\r\n", temp, hum,
                pressure, co2ppm);

  uint16_t chargeStatus = LTC4015::read_word(LTC4015_REG_CHARGE_STATUS);
  uint16_t chargerState = LTC4015::read_word(LTC4015_REG_CHARGER_STATE);
  uint16_t inputVoltage = LTC4015::get_input_voltage() * 1000;
  uint16_t inputCurrent = LTC4015::get_input_current() * 1000;
  uint16_t systemVoltage = LTC4015::get_system_voltage() * 1000;
  uint16_t batteryVoltage = LTC4015::get_battery_voltage() * 1000;
  uint16_t chargeCurrent = LTC4015::get_charge_current() * 1000;
  uint16_t chargeCount = LTC4015::get_qcount();

  /* Disable 3.3V and I2C bus power */
  WalterFeels::set3v3(false);
  WalterFeels::setI2cBusPower(false);

  performGnssFix();

  /* Construct UDP packet and transmit */
  uint16_t rawTemperature = temp * 100;
  uint16_t rawHumidity = hum * 100;
  uint16_t rawPressure = pressure * 100;
  float batteryCharge = (float) chargeCount * 100 / 65535;
  uint16_t rawBatteryCharge = batteryCharge * 100;

  dataBuf[6] = rawTemperature >> 8;
  dataBuf[7] = rawTemperature & 0xFF;
  dataBuf[8] = rawHumidity >> 8;
  dataBuf[9] = rawHumidity & 0xFF;
  dataBuf[10] = rawPressure >> 8;
  dataBuf[11] = rawPressure & 0xFF;
  dataBuf[12] = co2ppm >> 8;
  dataBuf[13] = co2ppm & 0xFF;
  dataBuf[14] = chargeStatus >> 8;
  dataBuf[15] = chargeStatus & 0xFF;
  dataBuf[16] = chargerState >> 8;
  dataBuf[17] = chargerState & 0xFF;
  dataBuf[18] = inputVoltage >> 8;
  dataBuf[19] = inputVoltage & 0xFF;
  dataBuf[20] = inputCurrent >> 8;
  dataBuf[21] = inputCurrent & 0xFF;
  dataBuf[22] = systemVoltage >> 8;
  dataBuf[23] = systemVoltage & 0xFF;
  dataBuf[24] = batteryVoltage >> 8;
  dataBuf[25] = batteryVoltage & 0xFF;
  dataBuf[26] = chargeCurrent >> 8;
  dataBuf[27] = chargeCurrent & 0xFF;
  dataBuf[28] = rawBatteryCharge >> 8;
  dataBuf[29] = rawBatteryCharge & 0xFF;
  dataBuf[30] = posFix.satCount;
  memcpy(dataBuf + 31, &posFix.latitude, 4);
  memcpy(dataBuf + 35, &posFix.longitude, 4);

  Serial.println("Transmitting Walter Feels data packet");

  if(!socketConnect(SERV_ADDR, SERV_PORT)) {
    Serial.print("Could not connect to UDP server socket\r\n");
    delay(1000);
    ESP.restart();
    return;
  }

  if(!modem.socketSend(dataBuf, PACKET_SIZE)) {
    Serial.print("Could not transmit data\r\n");
    delay(1000);
    ESP.restart();
    return;
  }

  delay(5000);

  if(!modem.socketClose()) {
    Serial.print("Could not close the socket\r\n");
    delay(1000);
    ESP.restart();
    return;
  }

  WalterFeels::prepareDeepSleep();

  Serial.println("I'm tired, I'm going to deep sleep now for 300 seconds");
  Serial.flush();
  modem.sleep(SLEEP_DURATION);
}

void loop()
{
  // Never reaches here
}