
#include <Wire.h>
#include  <Chrono.h>

String sensorId = "";

//##### server ota
#include <HTTPClient.h>
#include <ESP32httpUpdate.h>

#define USE_SERIAL Serial

String serverAddress = "***************";
const String sVersion = "0.2/";


//#### save sensor states
// include library to read and write from flash memory
#include <EEPROM.h>
// define the number of bytes you want to access
#define EEPROM_SIZE 2
// Variables will change:
int state = 1;         // the current state of the output pin

// Instanciate a Chrono object.
Chrono myChrono;
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */

//#### Wifi config

#include <ArduinoJson.h>
#include <HTTPClient.h>


const char *ssid = "mokkula_789340";
char *password = "1Y8BNB7R939";
const String host = "http://192.168.8.200:8080";

//##### BME280
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
/*#include <SPI.h>
  #define BME_SCK 18
  #define BME_MISO 19
  #define BME_MOSI 23
  #define BME_CS 5*/

#define I2C_SDA_FOR_BME 21
#define I2C_SCL_FOR_BME 22

#define SEALEVELPRESSURE_HPA (1013.25)

TwoWire I2CBME = TwoWire(0);
Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI
unsigned long delayTime;

//##### APDS9960
#include <SparkFun_APDS9960.h>

// Global Variables
SparkFun_APDS9960 apds = SparkFun_APDS9960(); // 22->scl 23->sda

uint16_t ambient_light = 0;
uint16_t red_light = 0;
uint16_t green_light = 0;
uint16_t blue_light = 0;


//#### CO2
#include <Arduino.h>

#define TXD2_CO2 18
#define RXD2_CO2 19

byte CO2req[] = {0xFE, 0X04, 0X00, 0X03, 0X00, 0X01, 0XD5, 0XC5};
byte Response[20];
uint16_t crc_02;
int ASCII_WERT;
int int01, int02, int03;
unsigned long ReadCRC;      // CRC Control Return Code

// Ausselesen ABC-Periode / FirmWare / SensorID
byte ABCreq[] = {0xFE, 0X03, 0X00, 0X1F, 0X00, 0X01, 0XA1, 0XC3};   // 1f in Hex -> 31 dezimal
byte FWreq[] = {0xFE, 0x04, 0x00, 0x1C, 0x00, 0x01, 0xE4, 0x03};    // FW      ->       334       1 / 78
byte ID_Hi[] = {0xFE, 0x04, 0x00, 0x1D, 0x00, 0x01, 0xB5, 0xC3};    // Sensor ID hi    1821       7 / 29
byte ID_Lo[] = {0xFE, 0x04, 0x00, 0x1E, 0x00, 0x01, 0x45, 0xC3};    // Sensor ID lo   49124     191 / 228 e.g. in Hex 071dbfe4
//byte Tstreq[] = {0xFE, 0x03, 0x00, 0x01, 0x00, 0x01, 0xE4, 0x03};   // Read the holding register
//byte Tstreq[] = {0xFE, 0x04, 0x00, 0x01, 0x00, 0x01, 0xE4, 0x03}; // Read the input Register
byte Tstreq[] = {0xFE, 0x44, 0X00, 0X01, 0X02, 0X9F, 0X25};       // undocumented function 44
int Test_len = 7;               // length of Testrequest in case of function 44 only 7 otherwise 8


//###### MLX90614
#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();
#define I2C_SDA_FOR_XML 16
#define I2C_SCL_FOR_XML 17


#define SLAVEADDR 0x5A


void sendDataToPi(String jsonData) {
  HTTPClient http;
  http.begin(host);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  auto httpCode = http.POST(jsonData);
  Serial.println(httpCode); //Print HTTP return code
  String payload = http.getString();
  Serial.println(payload); //Print request response payload

  if (httpCode == 200) {
    //send the ESP to deep sleep (to be woken up by pulling rst low)
    Serial.println();
    Serial.println("closing connection");
    http.end(); //Close connection Serial.println();
    Serial.println("Data send to core system (pi) successfully ");
    ESP.restart();
  } else {
    ESP.restart();
  }
}

void wifiConnection() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while ((!(WiFi.status() == WL_CONNECTED)))
  {
    delay(300);
    Serial.print(".");
    if (myChrono.hasPassed(60000))
    {
      Serial.println("go to sleep. cant connect to Wifi");
      esp_sleep_enable_timer_wakeup(300 * uS_TO_S_FACTOR);
      esp_deep_sleep_start();
    }
  }
  Serial.println("Connected");
  Serial.println((WiFi.localIP().toString()));
}

void send_Request (byte * Request, int Re_len)
{
  while (!Serial1.available())
  {
    Serial1.write(Request, Re_len);   // Send request to S8-Sensor
    delay(50);
  }
}

void read_Response (int RS_len)
{
  int01 = 0;
  while (Serial1.available() < 7 )
  {
    int01++;
    if (int01 > 10)
    {
      while (Serial1.available())
        Serial1.read();
      break;
    }
    delay(50);
  }
  for (int02 = 0; int02 < RS_len; int02++)    // Empfangsbytes
  {
    Response[int02] = Serial1.read();
  }
}

unsigned short int ModBus_CRC(unsigned char * buf, int len)
{
  unsigned short int crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++) {
    crc ^= (unsigned short int)buf[pos];   // XOR byte into least sig. byte of crc
    for (int i = 8; i != 0; i--) {         // Loop over each bit
      if ((crc & 0x0001) != 0) {           // If the LSB is set
        crc >>= 1;                         // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else                            // else LSB is not set
        crc >>= 1;                    // Just shift right
    }
  }  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return crc;
}

unsigned long get_Value(int RS_len)
/* Extract the Value from the response (byte 3 and 4)
    and check the CRC if the response is compleat and correct */
{
  /* Loop only for test and developtment purpose */
  //  for (int i=0; i<RS_len-2; i++)
  //  {
  //    int03 = Response[i];
  //    Serial.printf("Wert[%i] : %i / ", i, int03);
  //  }

  // Check the CRC //
  ReadCRC = (uint16_t)Response[RS_len - 1] * 256 + (uint16_t)Response[RS_len - 2];
  if (ModBus_CRC(Response, RS_len - 2) == ReadCRC) {
    // Read the Value //
    unsigned long val = (uint16_t)Response[3] * 256 + (uint16_t)Response[4];
    return val * 1;       // S8 = 1. K-30 3% = 3, K-33 ICB = 10
  }
  else {
    Serial.print("Error");
    return 99;
  }
}

void setup() {
  Serial.begin(9600);

  wifiConnection();
  //####  sensor states




  // initialize EEPROM with predefined size
  EEPROM.begin(EEPROM_SIZE);
  // read the last state from flash memory
  state = EEPROM.read(1);

  Serial.println("state ---->" + state);
  //  if (state == 0) {
  //    //##### APDS9960
  //
  //    // Initialize APDS-9960 (configure I2C and initial values)
  //    if ( apds.init() ) {
  //      Serial.println(F("APDS-9960 initialization complete"));
  //    } else {
  //      Serial.println(F("Something went wrong during APDS-9960 init!"));
  //    }
  //
  //    // Start running the APDS-9960 light sensor (no interrupts)
  //    if ( apds.enableLightSensor(false) ) {
  //      Serial.println(F("Light sensor is now running"));
  //    } else {
  //      Serial.println(F("Something went wrong during light sensor init!"));
  //    }
  //
  //
  //
  //    EEPROM.write(1, 1);
  //    EEPROM.commit();
  //    Serial.println("1");
  //  } else



  if (state == 1) {

    
    Wire.begin();
    pinMode(I2C_SCL_FOR_XML, OUTPUT);       // Clock goes from master to peripherical
    pinMode(I2C_SDA_FOR_XML, INPUT_PULLUP); // Data should be internally pulled up (or 4.7K pullup to 3.3v)
    Wire.begin(I2C_SDA_FOR_XML, I2C_SCL_FOR_XML, 10000);
    delay(1000);
    Wire.beginTransmission(SLAVEADDR);
    Wire.write((uint8_t)0);
    delay(1000);
    uint8_t err = Wire.endTransmission(false);
    if (err != 7) {
      //Serial.printf("Setting Eeprom read to address 0 failed =%d(%s).\n", Wire.lastError(), Wire.getErrorText(Wire.lastError)));
    }
    uint8_t count = Wire.requestFrom(SLAVEADDR, 64); // 32 address 16bits each so 64 bytes
    if ( count != 64 ) {
      //Serial.printf("Reading Eeprom data failed =%d(%s).  Only read %bytes.\n", Wire.lastError(), Wire.getErrorText(Wire.lastError), count);
    }
    uint16_t a = 0; // eeprom address register
    while (Wire.available()) {
      uint16_t b, c;
      b = Wire.read();
      c = Wire.read();
      b = b | (c << 8);
      Serial.printf("0x%02: 0x%02X\n", a++, b);
    }
    Serial.println(" all done");


    //delay(5000);

    mlx.begin();
    delay(1000);

    EEPROM.write(1, 2);
    EEPROM.commit();
    Serial.println("2" );
  } else if (state == 2) {

    //##### CO2
    Serial1.begin(9600, SERIAL_8N1, RXD2_CO2, TXD2_CO2);      // UART to Sensair CO2 Sensor

    Serial.println();
    Serial.print("ABC-Tage: ");
    send_Request(ABCreq, 8);                     // Request ABC-Information from the Sensor
    read_Response(7);

    Serial.printf("%02ld", get_Value(7));
    Serial.println("");

    Serial.print("Sensor ID : ");                // 071dbfe4
    send_Request(ID_Hi, 8);
    read_Response(7);
    Serial.printf("%02x%02x", Response[3], Response[4]);
    send_Request(ID_Lo, 8);
    read_Response(7);
    Serial.printf("%02x%02x", Response[3], Response[4]);
    Serial.println("");

    Serial.print("FirmWare  : ");
    send_Request(FWreq, 8);                // Send Request for Firmwar to the Sensor
    read_Response(7);                   // get Response (Firmware 334) from the Sensor
    Serial.printf("%02d%02d", Response[3], Response[4]);
    Serial.println("");

    // Read all Register
    for (int i = 1; i <= 31; i++)
    {
      Tstreq[3] = i;
      crc_02 = ModBus_CRC(Tstreq, Test_len - 2);  // calculate CRC for request
      Tstreq[Test_len - 1] = (uint8_t)((crc_02 >> 8) & 0xFF);
      Tstreq[Test_len - 2] = (uint8_t)(crc_02 & 0xFF);
      delay(100);
      //      Serial.print("Registerwert : ");
      //      Serial.printf("%02x", Tstreq[0]);
      //      Serial.printf("%02x", Tstreq[1]);
      //      Serial.printf("%02x", Tstreq[2]);
      //      Serial.print(" ");
      //      Serial.printf("%02x", Tstreq[3]);
      //      Serial.print(" ");
      //      Serial.printf("%02x", Tstreq[Test_len - 3]);
      //      Serial.printf("%02x", Tstreq[Test_len - 2]);
      //      Serial.printf("%02x", Tstreq[Test_len - 1]);

      send_Request(Tstreq, Test_len);
      read_Response(7);
      // Check if CRC is OK
      ReadCRC = (uint16_t)Response[6] * 256 + (uint16_t)Response[5];
      if (ModBus_CRC(Response, Test_len - 2) == ReadCRC) {
        //        Serial.printf(" -> (d) %03d %03d", Response[3], Response[4]);
        //        Serial.printf(" -> (x) %02x %02x", Response[3], Response[4]);
        //        Serial.printf(" ->(Wert) %05d", (uint16_t)Response[3] * 256 + (uint16_t)Response[4]);
        ASCII_WERT = (int)Response[3];
        //  Serial.print(" -> (t) ");
        if ((ASCII_WERT <= 128) and (ASCII_WERT >= 47))
        {
          Serial.print((char)Response[3]);
          Serial.print(" ");
          ASCII_WERT = (int)Response[4];
          if ((ASCII_WERT <= 128) and (ASCII_WERT >= 47))
          {
            Serial.print((char)Response[4]);
          }
          else
          {
            Serial.print(" ");
          }
        }
        else
        {
          Serial.print("   ");
        }
        if ((Response[1] == 68) and (Tstreq[3] == 8))
          Serial.print("   CO2");
        //Serial.println("  Ende ");
      }
      else {
        Serial.print("CRC-Error");
        // if we have err we reset for next state
        EEPROM.write(1, 3);
        EEPROM.commit();
        ESP.restart();
      }

    }


    EEPROM.write(1, 3);
    EEPROM.commit();
    Serial.println("3" );
  }
  if (state == 3) {
    //##### BME280
    Serial.println(F("BME280 test"));
    I2CBME.begin(I2C_SDA_FOR_BME, I2C_SCL_FOR_BME, 100000);
    bool status;
    // default settings
    // (you can also pass in a Wire library object like &Wire2)
    status = bme.begin(0x76, &I2CBME);
    if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
      // if we have err we reset for next state
      EEPROM.write(1, 2);
      EEPROM.commit();
      ESP.restart();

      while (1);
    }

    Serial.println();

    


    EEPROM.write(1, 4);
    EEPROM.commit();
    Serial.println("3" );

  }
  if (state == 4) {
    Serial.println(" ");
    Serial.println("version " + sVersion );
    serverAddress += sVersion;


    USE_SERIAL.println();
    USE_SERIAL.println();
    USE_SERIAL.println();

    for (uint8_t t = 4; t > 0; t--) {
      USE_SERIAL.printf("[SETUP] WAIT %d...\n", t);
      USE_SERIAL.flush();
      delay(1000);
    }

    EEPROM.write(1, 5);
    EEPROM.commit();
  }
  if (state == 5) {
    EEPROM.write(1, 1);
    EEPROM.commit();
    Serial.println("Sleep for 5 min" );

    esp_sleep_enable_timer_wakeup(300 * uS_TO_S_FACTOR);
    esp_deep_sleep_start();

  }



  delayTime = 5000;

  // Wait for initialization and calibration to finish
  delay(500);


}

void loop() {
  if (state == 0) {
    //getAPDS9960Values();
  } else if (state == 1) {
    getMLX90614Value();
  } else if (state == 2) {
    getCo2Valeu();
  } else if (state == 3) {
     getBMEValues();
   
    Serial.println("wait 5 min to next loop");
  } else if (state == 4) {
    doUpdate();
  }


  delay(1000);
}

void doUpdate() {
  if ((WiFi.status() == WL_CONNECTED)) {
    t_httpUpdate_return ret = ESPhttpUpdate.update(serverAddress);

    switch (ret) {
      case HTTP_UPDATE_FAILED:
        USE_SERIAL.printf("HTTP_UPDATE_FAILD Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
        ESP.restart();
        break;

      case HTTP_UPDATE_NO_UPDATES:
        USE_SERIAL.println("HTTP_UPDATE_NO_UPDATES");
        ESP.restart();
        break;

      case HTTP_UPDATE_OK:
        USE_SERIAL.println("HTTP_UPDATE_OK");
        ESP.restart();
        break;
    }
  }
}

int countLeafTValue = 0 ;
int leafTDivideBy = 5;
float totalLeafT = 0.0;

void getMLX90614Value() {

  Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempC());
  Serial.print("*C\tObject = "); Serial.print(mlx.readObjectTempC()); Serial.println("*C");

  countLeafTValue++;
  totalLeafT += mlx.readObjectTempC();

  if (countLeafTValue == leafTDivideBy) {

    StaticJsonDocument<300> doc;
    doc["type"] = "LeafT";
    doc["sensorId"] = "mb7zHvjR8p";
    doc["productType"] = "Tomato";
    doc["area"] = 1;
    doc["node"] = "node1";
    doc["volt"] = 0;
    doc["ambient"] = mlx.readAmbientTempC();
    doc["object"] = totalLeafT / leafTDivideBy;


    String jsonOutput;
    serializeJson(doc, jsonOutput);

    sendDataToPi(jsonOutput);
  }
}

void getAPDS9960Values() {

  // Read the light levels (ambient, red, green, blue)
  if (  !apds.readAmbientLight(ambient_light) ||
        !apds.readRedLight(red_light) ||
        !apds.readGreenLight(green_light) ||
        !apds.readBlueLight(blue_light) ) {
    Serial.println("Error reading light values");
  } else {
    Serial.print("Ambient: ");
    Serial.print(ambient_light);
    Serial.print(" Red: ");
    Serial.print(red_light);
    Serial.print(" Green: ");
    Serial.print(green_light);
    Serial.print(" Blue: ");
    Serial.println(blue_light);

    StaticJsonDocument<300> doc;
    doc["type"] = "light";
    doc["sensorId"] = "eC4dYeAz4m";
    doc["productType"] = "Tomato";
    doc["area"] = 1;
    doc["node"] = "node1";
    doc["volt"] = 0;
    doc["ambient"] = ambient_light;
    doc["red"] = red_light;
    doc["green"] = green_light;
    doc["blue"] = blue_light;

    String jsonOutput;
    serializeJson(doc, jsonOutput);

    sendDataToPi(jsonOutput);
  }
}

void getBMEValues() {

  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");

  // Convert temperature to Fahrenheit
  /*Serial.print("Temperature = ");
    Serial.print(1.8 * bme.readTemperature() + 32);
    Serial.println(" *F");*/

  Serial.print("Pressure = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println();

  StaticJsonDocument<300> doc;
  doc["type"] = "TH";
  doc["sensorId"] = "sUgKh4c3kQ";
  doc["productType"] = "Tomato";
  doc["area"] = 1;
  doc["node"] = "node1";
  doc["volt"] = 0;
  doc["temperature"] = bme.readTemperature();
  doc["humidity"] = bme.readHumidity();
  doc["pressure"] = bme.readPressure() / 100.0F;
  doc["altitude"] = bme.readAltitude(SEALEVELPRESSURE_HPA);

  String jsonOutput;
  serializeJson(doc, jsonOutput);

  sendDataToPi(jsonOutput);
}

void getCo2Valeu() {
  send_Request(CO2req, 8);               // send request for CO2-Data to the Sensor
  read_Response(7);                      // receive the response from the Sensor
  unsigned long CO2 = get_Value(7);
  //delay(10000);
  String CO2s = String(CO2);
  Serial.println(CO2s);

  StaticJsonDocument<300> doc;
  doc["type"] = "Co2";
  doc["sensorId"] = "A5zteFfwWq";
  doc["productType"] = "Tomato";
  doc["area"] = 1;
  doc["node"] = "node1";
  doc["volt"] = 0;
  doc["ppm"] = CO2s.toInt();

  String jsonOutput;
  serializeJson(doc, jsonOutput);

  sendDataToPi(jsonOutput);
}
