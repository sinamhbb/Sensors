
#include <Wire.h>
#include  <Chrono.h>

//##### server ota
#include <HTTPClient.h>
#include <ESP32httpUpdate.h>

#define USE_SERIAL Serial

String serverAddress = "http://51.195.19.158:8000/fermentrackMlx/";
const String sVersion = "0.3/";

bool isTimeToUpdate = true;
//#### Wifi config

#include <ArduinoJson.h>


#include <Preferences.h>

Preferences preferences;
//
const char *ssid = "mobile_EXT";
char *password = "abcd1234";
const String host = "http://192.168.43.200:8080";
// Instanciate a Chrono object.
Chrono myChrono;
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */


//###### MLX90614
#include <Adafruit_MLX90614.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();
#define I2C_SDA_FOR_XML 16
#define I2C_SCL_FOR_XML 17

#define SLAVEADDR 0x5A

int countLeafTValue = 0 ;
int leafTDivideBy = 1;
float totalLeafT = 0.0;



float objectTemp = 0;
float ambientTemp = 0;
bool isCorrectValue = false;
bool isSearchingForCorrectValue = true;

unsigned int state;

void sendDataToPi(String jsonData) {
  HTTPClient http;
  http.begin(host);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  auto httpCode = http.POST(jsonData);
  Serial.println("http code "); //Print HTTP return code
  Serial.println(httpCode); //Print HTTP return code
  String payload = http.getString();
  Serial.println(payload); //Print request response payload

  if (httpCode == 200) {
    //send the ESP to deep sleep (to be woken up by pulling rst low)
    Serial.println();
    Serial.println("closing connection");
    http.end(); //Close connection Serial.println();
    Serial.println("Data send to core system (pi) successfully ");
    digitalWrite(I2C_SDA_FOR_XML, LOW);
    digitalWrite(I2C_SCL_FOR_XML,  LOW);
    isCorrectValue = false;

    preferences.putUInt("state", 2);
    preferences.end();
    ESP.restart();


  }
  else {
    countLeafTValue = 0;
    totalLeafT = 0.0;
    if (myChrono.hasPassed(120000))
    {
      Serial.println("go to sleep. cant send data");
      digitalWrite(I2C_SDA_FOR_XML, LOW);
      digitalWrite(I2C_SCL_FOR_XML,  LOW);

      preferences.putUInt("state", 2);
      preferences.end();
      ESP.restart();

    }
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



void setup() {
  Serial.begin(9600);

  wifiConnection();

  preferences.begin("my-app", false);

  state = preferences.getUInt("state", 1);
  if (state == 1) {
    Wire.begin();

    pinMode(I2C_SCL_FOR_XML, OUTPUT);       // Clock goes from master to peripherical
    pinMode(I2C_SDA_FOR_XML, INPUT_PULLUP); // Data should be internally pulled up (or 4.7K pullup to 3.3v)
    delay(1000);
    Wire.begin(I2C_SDA_FOR_XML, I2C_SCL_FOR_XML, 50000);
    delay(1000);
    Wire.beginTransmission(SLAVEADDR);
   // Wire.write(highByte(SLAVEADDR));
   // Wire.write(lowByte(SLAVEADDR));
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
    delay(5000);
    while (Wire.available()) {
      delay(100);
      uint16_t b, c;
      b = Wire.read();
      c = Wire.read();
      b = b | (c << 8);
      Serial.printf("0x%02: 0x%02X\n", a++, b);
    }
    Serial.println(" all done");
    delay(2000);

    //delay(5000);

    mlx.begin();

  } else if (state == 2) {

    //OTA
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

    preferences.putUInt("state", 3);
    preferences.end();

  } else if (state == 3) {
    preferences.putUInt("state", 1);
    preferences.end();
    Serial.println("Sleep for 5 min" );

    esp_sleep_enable_timer_wakeup(60 * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
  }


}

void loop() {
  if (state == 1) {
    if (isSearchingForCorrectValue) {
      getMLX90614Value();
      delay(1000);

    } else if (isCorrectValue) {
      SendCorrectValue() ;
      delay(1000);
    }
  } else if (state == 2) {
    doUpdate();
  }



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
      default :
        ESP.restart();
        break;
    }
  } else {
    ESP.restart();
  }
}



void getMLX90614Value() {
  objectTemp = mlx.readObjectTempC();
  ambientTemp = mlx.readAmbientTempC();
  Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempC());
  Serial.print("*C\tObject = "); Serial.print(objectTemp); Serial.println("*C");

  if (objectTemp > 100 || objectTemp < -10) {
    digitalWrite(I2C_SDA_FOR_XML, LOW);
    digitalWrite(I2C_SCL_FOR_XML,  LOW);
    delay(3000);
    ESP.restart();
  } else {
    isCorrectValue = true;
    isSearchingForCorrectValue = false;
  }

  //  countLeafTValue++;
  //  totalLeafT += objectTemp;
  //  if (countLeafTValue == leafTDivideBy) {
  //
  //    StaticJsonDocument<300> doc;
  //    doc["type"] = "LeafT";
  //    doc["sensorId"] = "mb7zHvjR8p";
  //    doc["productType"] = "Tomato";
  //    doc["area"] = 1;
  //    doc["node"] = "node1";
  //    doc["volt"] = 0;
  //    doc["ambient"] = roundf( objectTemp * 100) / 100;
  //    doc["object"] = roundf((totalLeafT / leafTDivideBy) * 100) / 100;
  //    Serial.println(roundf( objectTemp * 100) / 100);
  //    Serial.println(roundf((totalLeafT / leafTDivideBy) * 100) / 100);
  //
  //    String jsonOutput;
  //    serializeJson(doc, jsonOutput);
  //    delay(1000);
  //    sendDataToPi(jsonOutput);
  //  }
}

void SendCorrectValue() {
  StaticJsonDocument<300> doc;
  doc["type"] = "LeafT";
  doc["sensorId"] = "mb7zHvjR8p";
  doc["productType"] = "Tomato";
  doc["area"] = 1;
  doc["node"] = "node1";
  doc["volt"] = 0;
  doc["ambient"] = roundf( ambientTemp * 100) / 100;
  doc["object"] = roundf((objectTemp) * 100) / 100;
  Serial.println(roundf(ambientTemp * 100) / 100);
  Serial.println(roundf((objectTemp) * 100) / 100);

  String jsonOutput;
  serializeJson(doc, jsonOutput);
  delay(1000);
  sendDataToPi(jsonOutput);
}
