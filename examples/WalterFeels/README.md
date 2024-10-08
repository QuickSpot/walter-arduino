# Walter feels arduino sketch

## HDC1080
The used [lib](https://github.com/closedcube/ClosedCube_HDC1080_Arduino) for the hdc1080.

```
#include <arduino.h>
#include <ClosedCube_HDC1080.h>

/**
 * @brief The i2c address of the hdc1080 sensor.
 */
#define HDC1080_I2C_ADDRESS 0x40

/**
 * @brief The I2C sda pin to use.
 */
#define I2C_SCA_PIN 42

/**
 * @brief The I2C scl pin to use.
 */
#define I2C_SCL_PIN 2

/**
 * @brief The I2C enable pin for the pull up resistors.
 */
#define I2C_BUS_POWER_PIN 1

/**
 * @brief The HDC1080 sensor instants
 */
ClosedCube_HDC1080 hdc1080;

/**
 * @brief Set the wire for all other sensors.
 */
void sensor_set_wire()
{
    pinMode(I2C_BUS_POWER_PIN, OUTPUT);
    digitalWrite(I2C_BUS_POWER_PIN, HIGH);
    Wire.begin(I2C_SCA_PIN, I2C_SCL_PIN);
}

void setup()
{
  Serial.begin(115200);
	Serial.printf("Walter feel test v0.0.1\r\n");
  delay(500);

  sensor_set_wire();
  hdc1080.begin(HDC1080_I2C_ADDRESS);
  hdc1080.setResolution(HDC1080_RESOLUTION_14BIT, HDC1080_RESOLUTION_14BIT);
}

void hdc1080_get_values()
{
  Serial.printf("The HDC1080 values\r\n");
	Serial.printf("Temperature= %.2fC\r\nRelative Humidity= %.2f%\r\n",
    hdc1080.readTemperature(),hdc1080.readHumidity());
}

void loop()
{
  hdc1080_get_values();
  delay(2000);
}

```

## LPS22HB
The used [lib](https://github.com/arduino-libraries/Arduino_LPS22HB) for LPS22HB.<br>
I removed the wire.begin function from the lib.<br>
To init the I2C in the setup function.<br>
```

#include <arduino.h>
#include <Arduino_LPS22HB.h>

/**
 * @brief The I2C sda pin to use.
 */
#define I2C_SCA_PIN 42

/**
 * @brief The I2C scl pin to use.
 */
#define I2C_SCL_PIN 2


/**
 * @brief The I2C enable pin for the pull up resistors.
 */
#define I2C_BUS_POWER_PIN 1

/**
 * @brief Setting up the barometric sensor.
 */
void sensor_setup_lps22hb()
{
  if (!BARO.begin()) {
    Serial.println("Failed to initialize pressure sensor!");
    while (1);
  }
}


void lps22hd_get_values()
{
  float pressure = BARO.readPressure();
  Serial.printf("Pressure = %.2fkPa\n\r",pressure);

  float temperature = BARO.readTemperature();
  Serial.print("Temperature = %.2f C\n\r");
}

/**
 * @brief Initialize the Walter feels system.
 * 
 * This function will initialize the walter feels system and check communication
 * with all peripherals and sensors.
 * 
 * @return None.
 */
void setup()
{
    Serial.begin(115200);
    delay(500);

    pinMode(ENABLE_3_3V_PIN, OUTPUT);
    digitalWrite(ENABLE_3_3V_PIN, LOW);

    sensor_set_wire();
    sensor_setup_lps22hb();
    
    delay(200);
}


void loop()
{
  
  lps22hd_get_values();
  delay(5000);
}

```

## SCD30 test
The used [lib](https://github.com/sparkfun/SparkFun_SCD30_Arduino_Library) for the SDC30.

The example code:<br>
```

#include <arduino.h>
#include "SparkFun_SCD30_Arduino_Library.h"

/**
 * @brief The I2C sda pin to use for the Co2 sensor.
 */
#define I2C_CO2_SCA_PIN 12

/**
 * @brief The I2C scl pin to use for the Co2 sensor.
 */
#define I2C_CO2_SCL_PIN 11

/**
 * @brief The pin to enable power to the Co2 pin.
 */
#define I2C_CO2_ENABLE_PIN 13

/**
 * @brief The I2C enable pin for the pull up resistors.
 */
#define I2C_BUS_POWER_PIN 1

/**
 * @brief The instance of the SDC30 sensor class.
 */
SCD30 airSensor;

/**
 * @brief Setup for the co2 sensor.
 */
void co2_sensor_setup()
{
  pinMode(I2C_BUS_POWER_PIN, OUTPUT);
  digitalWrite(I2C_BUS_POWER_PIN, HIGH);

  pinMode(I2C_CO2_ENABLE_PIN, OUTPUT);
  digitalWrite(I2C_CO2_ENABLE_PIN, LOW);
  Wire1.begin(I2C_CO2_SCA_PIN, I2C_CO2_SCL_PIN);

  while(!airSensor.begin(Wire1))
  {
    Serial.println("Air sensor not detected.");
    delay(500);
  }
}

void setup()
{
    Serial.begin(115200);
	  Serial.printf("Walter feel test v0.0.1\r\n");
    delay(500);
    sensor_set_wire();
}

void loop()
{
    
  if(airSensor.dataAvailable())
  {
    Serial.print("co2(ppm): ");
    Serial.print(airSensor.getCO2());

    Serial.print(" temp(C): ");
    Serial.print(airSensor.getTemperature(), 1);

    Serial.print(" humidity(%): ");
    Serial.print(airSensor.getHumidity(), 1);

    Serial.println();
  } else {
    Serial.println("Waiting for new data");
  } 
  delay(2000);
}

```

## 1 wire sensor test
The used [lib](https://github.com/milesburton/Arduino-Temperature-Control-Library) for the 1 wire temperature sensor DS18B20.

The test code:<br>
```

#include <OneWire.h>
#include <arduino.h>
#include <DallasTemperature.h>

/**
 * @brief The pin used for the one wire bus.
 */
#define ONE_WIRE_BUS_PIN 13

/**
 * @brief The one wire instance for the one wire sensor.
 */
OneWire oneWire(ONE_WIRE_BUS_PIN);	

DallasTemperature sensors(&oneWire);

void setup()
{
    Serial.begin(115200);
	  Serial.printf("Walter feel test v0.0.1\r\n");
    delay(500);
    sensors.begin();

}

void loop()
{
  sensors.requestTemperatures(); // Send the command to get temperatures

  // You can use multiple sensors if you have them connected
  // The function returns the temperature in degrees Celsius
  float temperature = sensors.getTempCByIndex(0);

  // You can also get temperature in Fahrenheit using sensors.getTempFByIndex(0);

  if (temperature > 0) {
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println("°C");
  } else {
    Serial.println("Error: Unable to read temperature data.");
  }

  delay(2000);
}
```


## SD card reader test
This [link](https://dr-mntn.net/2021/02/using-the-sd-card-in-1-bit-mode-on-the-esp32-cam-from-ai-thinker) was used for the test program.<br>

```
#include <arduino.h>
#include "FS.h"
#include "SPI.h"
#include "SD_MMC.h"

/**
 * @brief The pin number of SD data 0 pin.
 */
#define SD_DATA_0_PIN 40

/**
 * @brief The pin number for the SD clk pin
 */
#define SD_CLK_PIN 39

/**
 * @brief The pin number for the SD CMD pin.
 */
#define SD_CMD_PIN 38

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.path(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void createDir(fs::FS &fs, const char * path){
    Serial.printf("Creating Dir: %s\n", path);
    if(fs.mkdir(path)){
        Serial.println("Dir created");
    } else {
        Serial.println("mkdir failed");
    }
}

void removeDir(fs::FS &fs, const char * path){
    Serial.printf("Removing Dir: %s\n", path);
    if(fs.rmdir(path)){
        Serial.println("Dir removed");
    } else {
        Serial.println("rmdir failed");
    }
}

void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while(file.available()){
        Serial.write(file.read());
    }
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
    Serial.printf("Renaming file %s to %s\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("File renamed");
    } else {
        Serial.println("Rename failed");
    }
}

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}

void testFileIO(fs::FS &fs, const char * path){
    File file = fs.open(path);
    static uint8_t buf[512];
    size_t len = 0;
    uint32_t start = millis();
    uint32_t end = start;
    if(file){
        len = file.size();
        size_t flen = len;
        start = millis();
        while(len){
            size_t toRead = len;
            if(toRead > 512){
                toRead = 512;
            }
            file.read(buf, toRead);
            len -= toRead;
        }
        end = millis() - start;
        Serial.printf("%u bytes read for %u ms\n", flen, end);
        file.close();
    } else {
        Serial.println("Failed to open file for reading");
    }


    file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }

    size_t i;
    start = millis();
    for(i=0; i<2048; i++){
        file.write(buf, 512);
    }
    end = millis() - start;
    Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
    file.close();
}

void setup()
{
  if(!SD_MMC.setPins(SD_CLK_PIN, SD_CMD_PIN, SD_DATA_0_PIN)) {
    Serial.printf("Unable to set the pins for the sd card.\n");
  }
  Serial.printf("Pins set for the sd card.\n");


  SD_MMC.begin("/sdcard", true);

  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE){
      Serial.println("No SD card attached");
      return;
  }

  Serial.print("SD_MMC Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }

    uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
    Serial.printf("SD_MMC Card Size: %lluMB\n", cardSize);

    listDir(SD_MMC, "/", 0);
    createDir(SD_MMC, "/mydir");
    listDir(SD_MMC, "/", 0);
    removeDir(SD_MMC, "/mydir");
    listDir(SD_MMC, "/", 2);
    writeFile(SD_MMC, "/hello.txt", "Hello ");
    appendFile(SD_MMC, "/hello.txt", "World!\n");
    readFile(SD_MMC, "/hello.txt");
    deleteFile(SD_MMC, "/foo.txt");
    renameFile(SD_MMC, "/hello.txt", "/foo.txt");
    readFile(SD_MMC, "/foo.txt");
    testFileIO(SD_MMC, "/test.txt");
    Serial.printf("Total space: %lluMB\n", SD_MMC.totalBytes() / (1024 * 1024));
    Serial.printf("Used space: %lluMB\n", SD_MMC.usedBytes() / (1024 * 1024));if(!SD_MMC.setPins(SD_CLK_PIN, SD_CMD_PIN, SD_DATA_0_PIN)) {
    Serial.printf("Unable to set the pins for the sd card.\n");
  }
}

void loop()
{

}

```

## RS485 test

```
#include <arduino.h>

/**
 * @brief The serial RX pin to communicate.
 */
#define SERIAL_RX_PIN 7

/**
 * @brief The serial TX pin to communicate. 
 */
#define SERIAL_TX_PIN 15

/**
 * @brief The RX enable pin for RS485.
 */
#define RS485_RX_ENABLE_PIN 10

/**
 * @brief The RX enable pin for RS485.
 */
#define RS485_TX_ENABLE_PIN 43

/**
 * @brief The RX enable pin for RS232.
 */
#define RS232_RX_ENABLE_PIN 11

/**
 * @brief The TX enable pin for RS232.
 */
#define RS232_TX_ENABLE_PIN 39

/**
 * @brief User UART2 for communication with modbus sensors.
 */
HardwareSerial SensorSerial(2);

/**
 * @brief Setup of the rs485 connection. 
 */
void setup_rs485_connection()
{
  /* Configure RS485 transceiver to enable RX output.*/
  digitalWrite(RS232_RX_ENABLE_PIN, LOW);
  digitalWrite(RS232_TX_ENABLE_PIN, HIGH);
  digitalWrite(RS232_TX_ENABLE_PIN, LOW);
  digitalWrite(RS485_RX_ENABLE_PIN, LOW);

  /* Configure serial for the modbus. */
  SensorSerial.begin(9600, SERIAL_8N1, SERIAL_RX_PIN, SERIAL_TX_PIN);
}


void setup()
{
    pinMode(RS232_RX_ENABLE_PIN, OUTPUT);
    pinMode(RS232_TX_ENABLE_PIN, OUTPUT);
    pinMode(RS485_RX_ENABLE_PIN, OUTPUT);
    pinMode(RS485_TX_ENABLE_PIN, OUTPUT);
    setup_rs485_connection();
  
}

void loop()
{
  digitalWrite(RS485_RX_ENABLE_PIN, LOW);
  digitalWrite(RS485_TX_ENABLE_PIN, LOW);

  int rxBufSize = SensorSerial.available();
  while (rxBufSize--) {
    Serial.printf("%c",SensorSerial.read());
  }
}
```

## RS232 test

```
#include <arduino.h>

/**
 * @brief The serial RX pin to communicate.
 */
#define SERIAL_RX_PIN 7

/**
 * @brief The serial TX pin to communicate. 
 */
#define SERIAL_TX_PIN 15

/**
 * @brief The RX enable pin for RS485.
 */
#define RS485_RX_ENABLE_PIN 10

/**
 * @brief The RX enable pin for RS485.
 */
#define RS485_TX_ENABLE_PIN 43

/**
 * @brief The RX enable pin for RS232.
 */
#define RS232_RX_ENABLE_PIN 11

/**
 * @brief The TX enable pin for RS232.
 */
#define RS232_TX_ENABLE_PIN 39

/**
 * @brief User UART2 for communication with modbus sensors.
 */
HardwareSerial SensorSerial(2);

/**
 * @brief Setup od the rs232 connection.
 */
void setup_rs232_connection()
{
  /** Configure RS232 transceiver to enable RX output.*/
  digitalWrite(RS485_RX_ENABLE_PIN, HIGH);
  digitalWrite(RS485_TX_ENABLE_PIN, LOW);
  digitalWrite(RS232_RX_ENABLE_PIN, LOW);
  digitalWrite(RS232_TX_ENABLE_PIN, HIGH);

  /* Configure serial for RS232. */
  SensorSerial.begin(9600, SERIAL_8N1, SERIAL_RX_PIN, SERIAL_TX_PIN);
  // SensorSerial.onReceive(_handleRxData);
}



void setup()
{
    pinMode(RS232_RX_ENABLE_PIN, OUTPUT);
    pinMode(RS232_TX_ENABLE_PIN, OUTPUT);
    pinMode(RS485_RX_ENABLE_PIN, OUTPUT);
    pinMode(RS485_TX_ENABLE_PIN, OUTPUT);
    setup_rs232_connection();
  
}

void loop()
{
  int rxBufSize = SensorSerial.available();
  while (rxBufSize--) {
    Serial.printf("%c",SensorSerial.read());
  }
}
```

## LSD6SDM test
Used this [lib](https://github.com/deneyapkart/deneyap-6-eksen-ataletsel-olcum-birimi-arduino-library) for the test.<br>

I adjusted the address of the device from 0x6B to 0x6A in the lib
```
// #define LSM6DSM_ADDRESS (0x6B)
#define LSM6DSM_ADDRESS (0x6A)
```

The test code:<br>
```

#include <arduino.h>
#include <Deneyap_6EksenAtaletselOlcumBirimi.h>   

/**
 * @brief The I2C sda pin to use.
 */
#define I2C_SCA_PIN 42

/**
 * @brief The I2C scl pin to use.
 */
#define I2C_SCL_PIN 2

/**
 * @brief The I2C enable pin for the pull up resistors.
 */
#define I2C_BUS_POWER_PIN 1

/**
 * @brief The instance of the gyro and acc meter class.
 */
LSM6DSM AccGyro;

/**
 * @brief Set the wire for all other sensors.
 */
void sensor_set_wire()
{
    pinMode(I2C_BUS_POWER_PIN, OUTPUT);
    digitalWrite(I2C_BUS_POWER_PIN, HIGH);
    Wire.begin(I2C_SCA_PIN, I2C_SCL_PIN);
}

void setup()
{
  Serial.begin(115200);
	  Serial.printf("Walter feel test v0.0.3\r\n");
    delay(500);

    // modem_setup();

    pinMode(ENABLE_3_3V_PIN, OUTPUT);
    digitalWrite(ENABLE_3_3V_PIN, LOW);

    sensor_set_wire();
    
    if (AccGyro.begin() != IMU_SUCCESS) {
        delay(2500);
        Serial.println("Unable to begin connection with gyro");
    }

  delay(200);
}

void loop()
{
  Serial.printf("Acc data, X degrees: %.2f, Y degrees: %.2f, Z degrees: %.2f\r\n", 
    AccGyro.readFloatAccelX(), AccGyro.readFloatAccelY(), AccGyro.readFloatAccelZ());
  delay(500);

  Serial.printf("Gyro data ,X degrees: %.2f,Y degrees: %.2fZ degrees: %.2f\r\n",
    AccGyro.readFloatGyroX(), AccGyro.readFloatGyroY(), AccGyro.readFloatGyroZ());
  delay(500);

  Serial.printf("\nThe temperature is :  %.2fC\r\n", AccGyro.readTempC());
  
  delay(2000);
}

```
