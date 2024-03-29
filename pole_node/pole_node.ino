/*
This example will receive multiple universes via Artnet and control a strip of ws2811 leds via 
Adafruit's NeoPixel library: https://github.com/adafruit/Adafruit_NeoPixel
This example may be copied under the terms of the MIT license, see the LICENSE file for details
*/
#include <ArtnetWifi.h>
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
 #include "Adafruit_VL53L0X.h"
#include <WiFi.h>

//Wifi settings
const char* ssid = "Athenaeum";
const char* password = "community";

// Neopixel settings
const int numLeds = 30; // change for your setup
const int numberOfChannels = numLeds * 3; // Total number of channels you want to receive (1 led = 3 channels)
const byte dataPin = 0;
Adafruit_NeoPixel leds = Adafruit_NeoPixel(numLeds, dataPin, NEO_GRB + NEO_KHZ800);

// sensor settings
const int num_sensors = 0;
const int pin_address = 1;
int node_address = 0;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Artnet settings
ArtnetWifi artnet;
const int startUniverse = 0;
const char host[] = "10.0.0.54";


// Check if we got all universes
const int maxUniverses = numberOfChannels / 512 + ((numberOfChannels % 512) ? 1 : 0);
bool universesReceived[maxUniverses];
bool sendFrame = 1;
int previousDataLength = 0;

// connect to wifi – returns true if successful or false if not
bool ConnectWifi(void)
{
  bool state = true;
  int i = 0;

  WiFi.begin(ssid, password);
  Serial.println("");
  Serial.println("Connecting to WiFi");
  
  // Wait for connection
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (i > 20){
      state = false;
      break;
    }
    i++;
  }
  if (state){
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("");
    Serial.println("Connection failed.");
  }
  
  return state;
}

void initTest()
{
  for (int i = 0 ; i < numLeds ; i++)
    leds.setPixelColor(i, 30, 0, 0);
  leds.show();
  delay(100);
  for (int i = 0 ; i < numLeds ; i++)
    leds.setPixelColor(i, 0, 30, 0);
  leds.show();
  delay(100);
  for (int i = 0 ; i < numLeds ; i++)
    leds.setPixelColor(i, 0, 0, 30);
  leds.show();
  delay(100);
  for (int i = 0 ; i < numLeds ; i++)
    leds.setPixelColor(i, 0, 0, 0);
  leds.show();
}

void onDmxFrame(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data)
{
  Serial.print("frame");
  sendFrame = 1;
  // set brightness of the whole strip 
  if (universe == 15)
  {
    leds.setBrightness(data[0]);
    leds.show();
  }

  // Store which universe has got in
  if ((universe - startUniverse) < maxUniverses)
    universesReceived[universe - startUniverse] = 1;

  for (int i = 0 ; i < maxUniverses ; i++)
  {
    if (universesReceived[i] == 0)
    {
      //Serial.println("Broke");
      sendFrame = 0;
      break;
    }
  }

  // read universe and put into the right part of the display buffer
  for (int i = 0; i < length / 3; i++)
  {
    int led = i + (universe - startUniverse) * (previousDataLength / 3);
    if (led < numLeds)
      leds.setPixelColor(led, data[i * 3], data[i * 3 + 1], data[i * 3 + 2]);
  }
  previousDataLength = length;     
  
  if (sendFrame)
  {
    leds.show();
    // Reset universeReceived to 0
    memset(universesReceived, 0, maxUniverses);
  }
}

void setup() {
    Serial.begin(115200);

    // which node am I?
    pinMode(pin_address, INPUT);
    node_address = digitalRead(pin_address);
    Serial.print("Node address: ");
    Serial.println(node_address);

    // setup sensors
    if (num_sensors > 0) {
        Wire.begin(2, 3);
        Serial.println("Adafruit VL53L0X test");
        if (!lox.begin()) {
            Serial.println(F("Failed to boot VL53L0X"));
            while (1);
        }
    } else {
        Serial.println("No sensors");
    }

  ConnectWifi();
  artnet.begin(host);
  
//  artnet_tx.begin(host);
//  artnet_tx.setLength(numLeds * 3);
//  artnet_tx.setUniverse(startUniverse);
  
  leds.begin();
  initTest();

  // this will be called for each packet received
  artnet.setArtDmxCallback(onDmxFrame);
}

void loop()
{
  // Receive
  artnet.read();


  // Transmit
  artnet.setLength(num_sensors + 1);  
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  int data;  
  artnet.setByte(0, node_address);

  for (uint8_t sensor = 0; sensor < num_sensors; sensor++) {
    if (sensor == 0) {
      if (measure.RangeStatus != 4) {  // phase failures have incorrect data
        data = measure.RangeMilliMeter;
      } else {
        data = 0;
      }
    } else {
      data = sensor;
    }
      artnet.setByte(sensor + 1, data);
  }
  artnet.write();

//  Serial.println("read");
}
