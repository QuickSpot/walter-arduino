#include <Arduino.h>
#include <HardwareSerial.h>
#include <WalterModem.h>
#include <Wire.h>


#include "esp_mac.h"
#include "hdc1080.h"
#include "lps22hb.h"
#include "scd30.h"


#include "LTC4015Registers.h"
#include "WalterFeels.h"


/**
 * @brief Pin definitions
 */
#define PWR_3V3_EN_PIN 0
#define PWR_12V_EN_PIN 43
#define I2C_BUS_PWR_EN_PIN 1
#define CAN_EN_PIN 44
#define SDI12_TX_EN_PIN 10
#define SDI12_RX_EN_PIN 9
#define RS232_TX_EN_PIN 17
#define RS232_RX_EN_PIN 16
#define RS485_TX_EN_PIN 18
#define RS485_RX_EN_PIN 8
#define CO2_EN_PIN 13

#define I2C_SDA_PIN 42
#define I2C_SCL_PIN 2
#define SD_CMD_PIN 6
#define SD_CLK_PIN 5
#define SD_DAT0_PIN 4
#define GPIO_A_PIN 39
#define GPIO_B_PIN 38
#define SER_RX_PIN 41
#define SER_TX_PIN 40
#define CAN_RX_PIN 7
#define CAN_TX_PIN 15
#define CO2_SDA_PIN 12
#define CO2_SCL_PIN 11

/**
 * @brief The address of the server to upload the data to.
 */
#define SERV_ADDR "64.225.64.140"

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
uint8_t dataBuf[PACKET_SIZE] = {0};

/**
 * @brief The last received GNSS fix.
 */
WalterModemGNSSFix posFix = {};

/**
 * @brief The HDC1080 temperature/humidity sensor instance.
 */
HDC1080 hdc1080;

/**
 * @brief The LPS22HB barometric pressure sensor instance.
 */
LPS22HB lps22hb(Wire);

/**
 * @brief The SCD30 CO2 sensor instance.
 */
SCD30 scd30;

/**
 * @brief The charger instance.
 */
LTC4015 charger(3, 4);

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
const char *edrxCycle = "0010";
const char *edrxPTW = "0000";

/**
 * @brief The binary configuration settings for PSM.
 * These can be calculated using e.g.
 * https://www.soracom.io/psm-calculation-tool/
 */
const char *psmActive = "00000001";
const char *psmTAU = "00000110";

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

void fixHandler(const WalterModemGNSSFix *fix, void *args) {
  memcpy(&posFix, fix, sizeof(WalterModemGNSSFix));
  fixRcvd = true;
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
void checkAssistanceData(WalterModemRsp *rsp, bool *updateAlmanac = NULL,
                         bool *updateEphemeris = NULL) {
  if (updateAlmanac != NULL) {
    *updateAlmanac = false;
  }

  if (updateEphemeris != NULL) {
    *updateEphemeris = false;
  }

  Serial.print("Almanac data is ");
  if (rsp->data.gnssAssistance.almanac.available) {
    Serial.printf("available and should be updated within %ds\n",
                  rsp->data.gnssAssistance.almanac.timeToUpdate);
    if (updateAlmanac != NULL) {
      *updateAlmanac = rsp->data.gnssAssistance.almanac.timeToUpdate <= 0;
    }
  } else {
    Serial.println("not available.");
    if (updateAlmanac != NULL) {
      *updateAlmanac = true;
    }
  }

  Serial.print("Real-time ephemeris data is ");
  if (rsp->data.gnssAssistance.realtimeEphemeris.available) {
    Serial.printf("available and should be updated within %ds\n",
                  rsp->data.gnssAssistance.realtimeEphemeris.timeToUpdate);
    if (updateEphemeris != NULL) {
      *updateEphemeris =
          rsp->data.gnssAssistance.realtimeEphemeris.timeToUpdate <= 0;
    }
  } else {
    Serial.println("not available.\n");
    if (updateEphemeris != NULL) {
      *updateEphemeris = true;
    }
  }
}

/**
 * @brief This function will update GNSS assistance data when needed.
 *
 * This function will check if the current real-time ephemeris data is good
 * enough to get a fast GNSS fix. If not the function will attach to the LTE
 * network to download newer assistance data.
 *
 * @return True on success, false on error.
 */
bool updateGNSSAssistance() {
  Serial.println("Updating GNSS assistance data...");
  WalterModemRsp rsp = {};

  /* Even with valid assistance data the system clock could be invalid */
  if (!modem.getClock(&rsp)) {
    Serial.println("Error: Could not check the modem time");
    return false;
  }

  if (rsp.data.clock.epochTime <= 0) {
    /*
     * Wait for the modem to synchronize time with the LTE network, try 5 times
     * with a delay of 500ms.
     */
    for (int i = 0; i < 5; ++i) {
      if (!modem.getClock(&rsp)) {
        Serial.println("Error: Could not check the modem time");
        return false;
      }

      if (rsp.data.clock.epochTime > 0) {
        Serial.printf("Got time from LTE: %" PRIi64 "\n", rsp.data.clock.epochTime);
        break;
      } else if (i == 4) {
        Serial.println("Error: Could not sync time with network");
        return false;
      }

      delay(500);
    }
  }

  /* Check the availability of assistance data */
  if (!modem.getGNSSAssistanceStatus(&rsp) ||
      rsp.type != WALTER_MODEM_RSP_DATA_TYPE_GNSS_ASSISTANCE_DATA) {
    Serial.println("Error: Could not request GNSS assistance status");
    return false;
  }

  bool updateAlmanac = false;
  bool updateEphemeris = false;
  checkAssistanceData(&rsp, &updateAlmanac, &updateEphemeris);

  if (!(updateAlmanac || updateEphemeris)) {
    Serial.println("GNSS assistance data is still up-to-date");
    return true;
  }

  if (updateAlmanac) {
    if (!modem.updateGNSSAssistance(
            WALTER_MODEM_GNSS_ASSISTANCE_TYPE_ALMANAC)) {
      Serial.println("Error: Could not update almanac data");
      return false;
    }
  }

  if (updateEphemeris) {
    if (!modem.updateGNSSAssistance(
            WALTER_MODEM_GNSS_ASSISTANCE_TYPE_REALTIME_EPHEMERIS)) {
      Serial.println("Error: Could not update real-time ephemeris data");
      return false;
    }
  }

  if (!modem.getGNSSAssistanceStatus(&rsp) ||
      rsp.type != WALTER_MODEM_RSP_DATA_TYPE_GNSS_ASSISTANCE_DATA) {
    Serial.println("Error: Could not request GNSS assistance status");
    return false;
  }

  checkAssistanceData(&rsp);
  Serial.println("GNSS assistance data successfully updated");
  return true;
}

/**
 * @brief Setting up the modem communication.
 */
void modem_setup() {
  WalterModemRAT rat = WALTER_MODEM_RAT_UNKNOWN;

  WalterModemRsp rsp = {};
  if (modem.getRAT(&rsp)) {
    rat = rsp.data.rat;
  } else {
    Serial.println("Error: Could not retrieve radio access technology");
    delay(1000);
    ESP.restart();
    return;
  }

  if (rat != PREFERRED_RAT) {
    rat = rat == WALTER_MODEM_RAT_LTEM ? WALTER_MODEM_RAT_NBIOT
                                       : WALTER_MODEM_RAT_LTEM;

    if (!modem.setRAT(rat)) {
      Serial.println("Could not switch radio access technology");
      delay(1000);
      ESP.restart();
      return;
    }
    Serial.println("Switched modem radio technology");
  }

  /* Set operational state to MINIMUM */
  if (modem.setOpState(WALTER_MODEM_OPSTATE_MINIMUM)) {
    Serial.println("Successfully set operational state to MINIMUM\r\n");
  } else {
    Serial.println("Could not set operational state to MINIMUM\r\n");
    return;
  }

  if (!modem.configGNSS()) {
    Serial.println("Could not configure the GNSS subsystem\n");
    delay(1000);
    ESP.restart();
    return;
  }

  /* Create PDP context */
  if (modem.definePDPContext()) {
    Serial.println("Created PDP context");
  } else {
    Serial.println("Error: Could not create PDP context");
    return;
  }

  /* Enable / disable PSM */
  if (modem.configPSM(WALTER_MODEM_PSM_ENABLE, psmTAU, psmActive)) {
    Serial.println("Configured PSM");
  } else {
    Serial.println("Error: Could not configure PSM");
    return;
  }

  if (modem.configCEREGReports(
          WALTER_MODEM_CEREG_REPORTS_ENABLED_UE_PSM_WITH_LOCATION_EMM_CAUSE)) {
    Serial.println(
        "Configured CEREG to receive PSM result allocated by the network \r\n");
  } else {
    Serial.println("Error: Could not configure CEREG to receive PSM result "
                   "allocated by the network");
  }

  /* Enable /disable eDRX */
  if (modem.configEDRX(WALTER_MODEM_EDRX_DISABLE)) {
    Serial.println("Configured eDRX");
  } else {
    Serial.println("Error: Could not configure eDRX");
  }

  if (modem.setOpState(WALTER_MODEM_OPSTATE_FULL)) {
    Serial.println("Successfully set operational state to FULL");
  } else {
    Serial.println("Error: Could not set operational state to FULL");
    return;
  }

  /* Set the network operator selection to automatic */
  if (!modem.setNetworkSelectionMode(WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC)) {
    Serial.println(
        "Error: Could not set the network selection mode to automatic");
    return;
  }

  Serial.printf("Connecting to the %s network\n",
                rat == WALTER_MODEM_RAT_LTEM ? "LTE-M" : "NB-IoT");

  int count = 0;
  WalterModemNetworkRegState regState = modem.getNetworkRegState();
  while (!(regState == WALTER_MODEM_NETWORK_REG_REGISTERED_HOME ||
           regState == WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING)) {
    if (count >= 1800) {
      Serial.println(
          "Error: Could not connect to the network, going to to try other RAT");
      break;
    }

    count += 1;
    delay(1000);
    regState = modem.getNetworkRegState();
  }

  if (!(regState == WALTER_MODEM_NETWORK_REG_REGISTERED_HOME ||
        regState == WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING)) {
    /* Set the operational state to minimum */
    if (!modem.setOpState(WALTER_MODEM_OPSTATE_MINIMUM)) {
      Serial.println("Error: Could not set operational state to MINIMUM");
      return;
    }

    delay(1000);

    rat = rat == WALTER_MODEM_RAT_LTEM ? WALTER_MODEM_RAT_NBIOT
                                       : WALTER_MODEM_RAT_LTEM;

    if (!modem.setRAT(rat)) {
      Serial.println("Error: Could not switch radio access technology");
      delay(1000);
      ESP.restart();
      return;
    }
    Serial.println("Switched modem radio technology");

    /* Set the operational state to full */
    if (!modem.setOpState(WALTER_MODEM_OPSTATE_FULL)) {
      Serial.println("Error: Could not set operational state to FULL");
      return;
    }

    /* Set the network operator selection to automatic */
    if (!modem.setNetworkSelectionMode(
            WALTER_MODEM_NETWORK_SEL_MODE_AUTOMATIC)) {
      Serial.println("Error: Could not set the network selection mode to automatic");
      return;
    }

    Serial.printf("Connecting to %s network\n",
                  rat == WALTER_MODEM_RAT_LTEM ? "LTE-M" : "NB-IoT");

    /* Wait for the network to become available (max 30 minutes) */
    count = 0;
    regState = modem.getNetworkRegState();
    while (!(regState == WALTER_MODEM_NETWORK_REG_REGISTERED_HOME ||
             regState == WALTER_MODEM_NETWORK_REG_REGISTERED_ROAMING)) {
      if (count >= 1800) {
        Serial.println("Error: Could not connect to the network, going to reboot");
        delay(1000);
        ESP.restart();
        return;
      }

      count += 1;
      delay(1000);
      regState = modem.getNetworkRegState();
    }
  }

  Serial.println("Error: Connected to the network");
}

void modem_transmit() {
  if (modem.socketConfig()) {
    Serial.println("Created a new socket");
  } else {
    Serial.println("Error: Could not create a new socket");
  }

  if (modem.socketDial(SERV_ADDR, SERV_PORT)) {
    Serial.printf("Connected to UDP server %s:%d\r\n", SERV_ADDR, SERV_PORT);
  } else {
    Serial.println("Error: Could not connect UDP socket");
  }

  if (modem.socketSend(dataBuf, PACKET_SIZE)) {
    Serial.printf("Transmitted data");
  } else {
    Serial.println("Error: Could not transmit data");
    delay(1000);
    ESP.restart();
  }

  if (modem.socketClose()) {
    Serial.println("Successfully closed the socket");
  } else {
    Serial.println("Error: Could not close the socket");
  }
}

void setup_charger() {
  charger.initialize();
  charger.suspend_charging();
  charger.enable_force_telemetry();
  delay(100);
  charger.disable_force_telemetry();
  charger.start_charging();
  charger.enable_mppt();
}

/**
 * @brief Configure the debug serial port.
 *
 * This function will initialize the serial debug port at 115200 baud. The
 * function will block up to 5 seconds, waiting for the USB Serial to be opened.
 * If the port is not opened, the function will continue.
 */
void setup_serial() {
  Serial.begin(115200);

  unsigned long startMillis = millis();
  while (!Serial && (millis() - startMillis < 5000)) {
  }
}

void setup() {
  setup_serial();
  Serial.println("Walter feels demo firmware V2.2");

  /* Disable output holds */
  gpio_hold_dis((gpio_num_t)PWR_3V3_EN_PIN);
  gpio_hold_dis((gpio_num_t)PWR_12V_EN_PIN);
  gpio_hold_dis((gpio_num_t)I2C_BUS_PWR_EN_PIN);
  gpio_hold_dis((gpio_num_t)CAN_EN_PIN);
  gpio_hold_dis((gpio_num_t)SDI12_TX_EN_PIN);
  gpio_hold_dis((gpio_num_t)SDI12_RX_EN_PIN);
  gpio_hold_dis((gpio_num_t)RS232_TX_EN_PIN);
  gpio_hold_dis((gpio_num_t)RS232_RX_EN_PIN);
  gpio_hold_dis((gpio_num_t)RS485_TX_EN_PIN);
  gpio_hold_dis((gpio_num_t)RS485_RX_EN_PIN);
  gpio_hold_dis((gpio_num_t)CO2_EN_PIN);

  /* Configure IO pins */
  pinMode(PWR_3V3_EN_PIN, OUTPUT);
  pinMode(PWR_12V_EN_PIN, OUTPUT);
  pinMode(I2C_BUS_PWR_EN_PIN, OUTPUT);
  pinMode(CAN_EN_PIN, OUTPUT);
  pinMode(SDI12_TX_EN_PIN, OUTPUT);
  pinMode(SDI12_RX_EN_PIN, OUTPUT);
  pinMode(RS232_TX_EN_PIN, OUTPUT);
  pinMode(RS232_RX_EN_PIN, OUTPUT);
  pinMode(RS485_TX_EN_PIN, OUTPUT);
  pinMode(RS485_RX_EN_PIN, OUTPUT);
  pinMode(CO2_EN_PIN, OUTPUT);
  pinMode(I2C_SDA_PIN, INPUT);
  pinMode(I2C_SCL_PIN, INPUT);
  pinMode(SD_CMD_PIN, INPUT);
  pinMode(SD_CLK_PIN, INPUT);
  pinMode(SD_DAT0_PIN, INPUT);
  pinMode(GPIO_A_PIN, INPUT);
  pinMode(GPIO_B_PIN, INPUT);
  pinMode(SER_RX_PIN, INPUT);
  pinMode(SER_TX_PIN, INPUT);
  pinMode(CAN_RX_PIN, INPUT);
  pinMode(CAN_TX_PIN, INPUT);
  pinMode(CO2_SDA_PIN, INPUT);
  pinMode(CO2_SCL_PIN, OUTPUT);

  /* Disable all peripherals*/
  digitalWrite(PWR_3V3_EN_PIN, HIGH);
  digitalWrite(PWR_12V_EN_PIN, LOW);
  digitalWrite(I2C_BUS_PWR_EN_PIN, LOW);
  digitalWrite(CAN_EN_PIN, HIGH);
  digitalWrite(SDI12_TX_EN_PIN, LOW);
  digitalWrite(SDI12_RX_EN_PIN, LOW);
  digitalWrite(RS232_TX_EN_PIN, LOW);
  digitalWrite(RS232_RX_EN_PIN, HIGH);
  digitalWrite(RS485_TX_EN_PIN, LOW);
  digitalWrite(RS485_RX_EN_PIN, HIGH);
  digitalWrite(CO2_EN_PIN, HIGH);

  /* Initialize the LTE modem library */
  if (WalterModem::begin(&ModemSerial)) {
    Serial.println("Modem initialization success");
  } else {
    Serial.println("Error: Modem initialization fault");
    delay(1000);
    ESP.restart();
    return;
  }

  /* Initialize I2C masters */
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  /* Initialize CO2 sensor if installed */
  digitalWrite(PWR_3V3_EN_PIN, LOW);
  digitalWrite(CO2_EN_PIN, LOW);
  Wire1.begin(CO2_SDA_PIN, CO2_SCL_PIN);
  Serial.println("Waiting for CO2 sensor to boot");

  bool co2_sensor_installed = scd30.begin(Wire1);
  for (int i = 0; i < 10 && !co2_sensor_installed; ++i) {
    delay(500);
    co2_sensor_installed = scd30.begin(Wire1);
  }

  if (!co2_sensor_installed) {
    digitalWrite(CO2_EN_PIN, HIGH);
    Serial.println("No CO2 sensor is installed");
  } else {
    Serial.println("Sensirion SCD30 CO2 sensor is installed");
  }

  /* Init modem and charger on initial boot */
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED) {
    digitalWrite(I2C_BUS_PWR_EN_PIN, HIGH);
    charger.initialize();
    charger.enable_coulomb_counter();
    digitalWrite(I2C_BUS_PWR_EN_PIN, LOW);

    modem_setup();
  }

  /* Check clock and assistance data, update if required */
  if (!updateGNSSAssistance()) {
    Serial.println("Error: Could not update GNSS assistance data.");
    delay(1000);
    ESP.restart();
    return;
  }

  /* Perform GNSS positioning */
  modem.setGNSSEventHandler(fixHandler);
  fixRcvd = false;
  bool gnssSearching = false;
  for (int i = 0; i < 5; ++i) {
    if (modem.performGNSSAction()) {
      gnssSearching = true;
      break;
    }

    Serial.println("Waiting with GNSS fix because radio is busy");
    delay(5000);
  }

  if (gnssSearching) {
    Serial.println("Started GNSS fix");

    int j = 0;
    while (!fixRcvd) {
      Serial.println(".");
      if (j >= 240) {
        Serial.println("Error: Timed out while waiting for GNSS fix");
        delay(1000);
        ESP.restart();
        break;
      }
      j++;
      delay(500);
    }
  }

  uint8_t aboveThreshold = 0;
  for (int i = 0; i < posFix.satCount; ++i) {
    if (posFix.sats[i].signalStrength >= 30) {
      aboveThreshold += 1;
    }
  }

  float lat = posFix.latitude;
  float lon = posFix.longitude;

  if (posFix.estimatedConfidence > MAX_GNSS_CONFIDENCE) {
    posFix.satCount = 0xFF;
    lat = 0.0;
    lon = 0.0;
    Serial.println("Error: Could not get a valid fix");
  } else {
    Serial.printf("GNSS fix attempt finished:\n"
                  "  Confidence: %.02f\n"
                  "  Latitude: %.06f\n"
                  "  Longitude: %.06f\n"
                  "  Satcount: %d\n"
                  "  Good sats: %d\n",
                  posFix.estimatedConfidence, posFix.latitude, posFix.longitude,
                  posFix.satCount, aboveThreshold);
  }

  /* Enable 3.3V and I2C bus power, wait for sensors to boot */
  digitalWrite(PWR_3V3_EN_PIN, LOW);
  digitalWrite(I2C_BUS_PWR_EN_PIN, HIGH);
  delay(50);

  /* Initialize the sensors and read values */
  hdc1080.begin();
  lps22hb.begin();
  setup_charger();

  float temp = hdc1080.readTemperature();
  float hum = hdc1080.readHumidity();
  float pressure = lps22hb.readPressure();
  uint16_t co2ppm = co2_sensor_installed ? scd30.getCO2() : 0;

  Serial.printf(
      "Sensor data, temp: %.02fC, hum: %.02f%%, press: %.02fhPa, co2: %d\r\n",
      temp, hum, pressure, co2ppm);

  uint16_t chargeStatus = charger.read_word(CHARGE_STATUS);
  uint16_t chargerState = charger.read_word(CHARGER_STATE);
  uint16_t inputVoltage = charger.get_input_voltage() * 1000;
  uint16_t inputCurrent = charger.get_input_current() * 1000;
  uint16_t systemVoltage = charger.get_system_voltage() * 1000;
  uint16_t batteryVoltage = charger.get_battery_voltage() * 1000;
  uint16_t chargeCurrent = charger.get_charge_current() * 1000;
  uint16_t chargeCount = charger.get_qcount();

  /* Disable 3.3V and I2C bus power */
  digitalWrite(PWR_3V3_EN_PIN, HIGH);
  digitalWrite(I2C_BUS_PWR_EN_PIN, LOW);

  /* Construct UDP packet and transmit */
  uint16_t rawTemperature = temp * 100;
  uint16_t rawHumidity = hum * 100;
  uint16_t rawPressure = pressure * 100;
  float batteryCharge = (float)chargeCount * 100 / 65535;
  uint16_t rawBatteryCharge = batteryCharge * 100;

  /* Construct a Walter Feels packet */
  esp_read_mac(dataBuf, ESP_MAC_WIFI_STA);
  Serial.printf("Walter's MAC is: %02X:%02X:%02X:%02X:%02X:%02X\n", dataBuf[0],
                dataBuf[1], dataBuf[2], dataBuf[3], dataBuf[4], dataBuf[5]);

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
  memcpy(dataBuf + 31, &lat, 4);
  memcpy(dataBuf + 35, &lon, 4);

  Serial.println("Transmitting Walter Feels data packet");
  modem_transmit();

  /* Disable all peripherals*/
  digitalWrite(PWR_3V3_EN_PIN, HIGH);
  digitalWrite(PWR_12V_EN_PIN, LOW);
  digitalWrite(I2C_BUS_PWR_EN_PIN, LOW);
  digitalWrite(CAN_EN_PIN, HIGH);
  digitalWrite(SDI12_TX_EN_PIN, LOW);
  digitalWrite(SDI12_RX_EN_PIN, LOW);
  digitalWrite(RS232_TX_EN_PIN, LOW);
  digitalWrite(RS232_RX_EN_PIN, HIGH);
  digitalWrite(RS485_TX_EN_PIN, LOW);
  digitalWrite(RS485_RX_EN_PIN, HIGH);
  digitalWrite(CO2_EN_PIN, HIGH);

  /* Configure PIN holding states for low power deep sleep */
  gpio_hold_en((gpio_num_t)PWR_3V3_EN_PIN);
  gpio_hold_en((gpio_num_t)PWR_12V_EN_PIN);
  gpio_hold_en((gpio_num_t)I2C_BUS_PWR_EN_PIN);
  gpio_hold_en((gpio_num_t)CAN_EN_PIN);
  gpio_hold_en((gpio_num_t)SDI12_TX_EN_PIN);
  gpio_hold_en((gpio_num_t)SDI12_RX_EN_PIN);
  gpio_hold_en((gpio_num_t)RS232_TX_EN_PIN);
  gpio_hold_en((gpio_num_t)RS232_RX_EN_PIN);
  gpio_hold_en((gpio_num_t)RS485_TX_EN_PIN);
  gpio_hold_en((gpio_num_t)RS485_RX_EN_PIN);
  gpio_hold_en((gpio_num_t)CO2_EN_PIN);
  gpio_deep_sleep_hold_en();

  Serial.println("I'm tired, I'm going to deep sleep now for 300 seconds");
  Serial.flush();
  modem.sleep(SLEEP_DURATION);
}

void loop() {
  // Never reaches here
}