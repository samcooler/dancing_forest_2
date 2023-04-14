#include "Adafruit_VL53L1X.h"
#include "Adafruit_NeoPixel.h"
#include "SparkFunLSM6DS3.h"

#define SDA_PIN 21
#define SCL_PIN 22

#define PIN_led_rgb 9
#define PIN_led_side 8
#define PIN_led_rgbw 7
#define PIN_led_ext 6
#define PIN_led_enable 10

#define PIN_current_measure 1
#define PIN_battery_measure 2

// general setup
uint32_t loop_count = 0;

// power setup
float power_current = 0;
float power_battery = 0;

// lidar setup
const int numSensors = 3;
Adafruit_VL53L1X *sensors[numSensors];
const uint8_t sensorAddresses[3] = { 0x29, 0x2A, 0x2B };
const uint8_t sensorXShutPins[3] = { 3, 4, 5};
int16_t distance[3];

// led setup
Adafruit_NeoPixel led_rgb(60, PIN_led_rgb, NEO_GRB + NEO_KHZ800);
// Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRBW + NEO_KHZ800);
// Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRBW + NEO_KHZ800);
// Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRBW + NEO_KHZ800);

// accel setup
LSM6DS3 accel;
float acceleration[3];
float gyroscopic[3];
float temperature;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println(F("Dancing Forest"));

  // LED start
  led_rgb.begin();
  led_rgb.show();
  led_rgb.setBrightness(50);

  // I2C start
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println(F("Wire is started"));

  // accel start
  accel.begin();
  Serial.println(F("Accelerometer is started"));

  // lidar start
  for (int i = 0; i < numSensors; i++) {
    pinMode(sensorXShutPins[i], OUTPUT);
    digitalWrite(sensorXShutPins[i], LOW);
  }
  for (int i = 0; i < numSensors; i++) {
    sensors[i] = new Adafruit_VL53L1X();
    digitalWrite(sensorXShutPins[i], HIGH);
    delay(100);

    if (!sensors[i]->begin(0x29, &Wire)) {
      Serial.print(F("Error on init of VL sensor: "));
      Serial.println(sensors[i]->vl_status);
      while (1) delay(10);
    }
    sensors[i]->VL53L1X_SetI2CAddress(sensorAddresses[i]);
    Serial.print("lidar ");
    Serial.print(i);
    Serial.print(" initialized on i2c address ");
    Serial.println(sensorAddresses[i]);
    if (!sensors[i]->startRanging()) {
      Serial.print(F("Couldn't start ranging: "));
      Serial.println(sensors[i]->vl_status);
      while (1) delay(10);
    }
    Serial.println(F("Ranging started"));
    // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
    sensors[i]->setTimingBudget(50);
  }
}

void read_power() {
  power_current = analogRead(PIN_current_measure);
  power_battery = analogRead(PIN_battery_measure);

  Serial.println(F("Power: current measurement"));
  Serial.println(power_current,4);
  Serial.println(F("Power: battery measurement"));
  Serial.println(power_battery, 4);
}

void read_sensors() {
  // read accel
  acceleration[0] = accel.readFloatAccelX();
  acceleration[1] = accel.readFloatAccelY();
  acceleration[2] = accel.readFloatAccelZ();
  gyroscopic[0] = accel.readFloatGyroY();
  gyroscopic[1] = accel.readFloatGyroX();
  gyroscopic[2] = accel.readFloatGyroZ();
  temperature = accel.readTempF();

  // read lidar
  for (int i = 0; i < numSensors; i++) {
    if (sensors[i]->dataReady()) {
      distance[i] = sensors[i]->distance();
      sensors[i]->clearInterrupt();
    }
  }
}

void print_sensor_values() {
  Serial.println(F("acceleration: "));
  Serial.println(acceleration[0], 4);
  Serial.println(acceleration[1], 4);
  Serial.println(acceleration[2], 4);
  Serial.println(gyroscopic[0], 4);
  Serial.println(gyroscopic[1], 4);
  Serial.println(gyroscopic[2], 4);
  
  for (int i = 0; i < numSensors; i++) {
      Serial.print(F("dist"));
      Serial.print(i);
      Serial.print(": ");
      Serial.print(distance[i]);
      Serial.print(", ");
  }
  Serial.println();
}

void loop() {
  Serial.println(F("Loop start"));
  if(loop_count % 10 == 0) {
    read_power();
  }

  read_sensors();
  print_sensor_values();

  Serial.println();
  delay(100);

  loop_count ++;
}



