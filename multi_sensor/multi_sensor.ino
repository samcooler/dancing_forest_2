#include "Adafruit_VL53L1X.h"
#include "Adafruit_NeoPixel.h"
#include "Arduino_LSM6DS3.h"

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
const uint8_t sensorAddresses[3] = { 0x2A, 0x2B, 0x2C };  // 0x29 is default
const uint8_t sensorXShutPins[3] = { 3, 4, 5 };
int16_t distance[3];

// setup LEDs
Adafruit_NeoPixel leds_rgb(60, 9, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel leds_side(12, 8, NEO_RGB + NEO_KHZ800);
Adafruit_NeoPixel leds_rgbw(16, 7, NEO_RGBW + NEO_KHZ800);

// accel setup
// float acceleration[3];
// float gyroscopic[3];
// float temperature;
float x, y, z, gx, gy, gz;

void setup() {
  Serial.begin(9600);
  while (!Serial) delay(10);
  delay(1000);
  Serial.println(F("Dancing Forest"));

  // enable LED power
  pinMode(PIN_led_enable, OUTPUT);
  digitalWrite(PIN_led_enable, HIGH);

  // LED start
  leds_rgb.begin();  // INITIALIZE NeoPixel strip object (REQUIRED)
  leds_rgb.show();   // Turn OFF all pixels ASAP

  leds_side.begin();  // INITIALIZE NeoPixel strip object (REQUIRED)
  leds_side.show();   // Turn OFF all pixels ASAP

  leds_rgbw.begin();  // INITIALIZE NeoPixel strip object (REQUIRED)
  leds_rgbw.show();   // Turn OFF all pixels ASAP

  // I2C start
  Wire.begin(12, 11);
  Serial.println(F("Wire is started"));

  // accel start
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }
  Serial.println(F("Accelerometer is started"));

  // lidar start
  for (int i = 0; i < numSensors; i++) {
    Serial.print("shutdown sensor ");
    Serial.println(i);
    pinMode(sensorXShutPins[i], OUTPUT);
    digitalWrite(sensorXShutPins[i], LOW);
  }
  for (int i = 1; i < numSensors; i++) {
    sensors[i] = new Adafruit_VL53L1X();
    Serial.print("bring up sensor ");
    Serial.println(i);
    digitalWrite(sensorXShutPins[i], HIGH);
    delay(300);

    if (!sensors[i]->begin(0x29, &Wire, 1)) {
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
    sensors[i]->setTimingBudget(100);
  }
}

void read_power() {
  power_current = analogRead(PIN_current_measure);
  power_battery = analogRead(PIN_battery_measure);

  Serial.println(F("Power: current measurement"));
  Serial.println(power_current, 4);
  Serial.println(F("Power: battery measurement"));
  Serial.println(power_battery, 4);
}

void read_sensors() {
  // read accel
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    IMU.readGyroscope(gx, gy, gz);
    // temperature = accel.readTempF();
  }
  // read lidar
  for (int i = 1; i < numSensors; i++) {
    if (sensors[i]->dataReady()) {
      distance[i] = sensors[i]->distance();
      sensors[i]->clearInterrupt();
    }
  }
}

void print_sensor_values() {
  Serial.println(F("acceleration: "));
  // Serial.println(acceleration[0], 4);
  // Serial.println(acceleration[1], 4);
  // Serial.println(acceleration[2], 4);
  // Serial.println(gyroscopic[0], 4);
  // Serial.println(gyroscopic[1], 4);
  // Serial.println(gyroscopic[2], 4);

  Serial.print(x);
  Serial.print('\t');
  Serial.print(y);
  Serial.print('\t');
  Serial.print(z);
  Serial.print('\t');
  Serial.print(gx);
  Serial.print('\t');
  Serial.print(gy);
  Serial.print('\t');
  Serial.println(gz);

  for (int i = 0; i < numSensors; i++) {
    Serial.print(F("dist"));
    Serial.print(i);
    Serial.print(": ");
    Serial.print(distance[i]);
    Serial.print(", ");
  }
  Serial.println();
}

void simple_led_colors() {
  int a, b, c;
  for (int i = 0; i < 60; i++) {
    a = int(abs(float(distance[1]) / 10.0)) % 255;
    b = int(abs(float(distance[2]) / 10.0)) % 255;
    c = int(abs(float(x + y + z)) * 40) % 255;
    // leds_rgb.rainbow(firstPixelHue, 2, 255, 100, 1);
    leds_rgb.setPixelColor(i, a, b, c);
  }
  Serial.println(a);
  Serial.println(b);
  Serial.println(c);

  // firstPixelHue = int(float(x + y + z) * 120) % 255;
  // leds_rgbw.rainbow(firstPixelHue, 2, 255, 100, 1);
  // Serial.println(firstPixelHue);

  // firstPixelHue = int(float(gx + gy + gz) * 120.0) % 255;
  // leds_side.rainbow(firstPixelHue, 2, 255, 100, 1);
  // Serial.println(firstPixelHue);

  leds_rgb.show();
  // leds_rgbw.show();
  // leds_side.show();
  // }
}

void loop() {
  Serial.println(F("Loop start"));
  if (loop_count % 10 == 0) {
    read_power();
  }

  read_sensors();
  print_sensor_values();
  simple_led_colors();

  Serial.println();
  delay(20);

  loop_count++;
}
