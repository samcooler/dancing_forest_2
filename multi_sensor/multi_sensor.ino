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
#define led_count_rgbw 16
#define led_count_rgb_single 24
#define led_count_upper 12
#define led_count_rgb 60 
#define led_count_side 12
#define strip_rgbw 0 // used for identity in functions
#define strip_side 1
#define strip_rgb_inner 2
#define strip_rgb_outer 3
#define strip_rgb_upper 4

Adafruit_NeoPixel leds_rgb(led_count_rgb, 9, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel leds_side(led_count_side, 8, NEO_RGB + NEO_KHZ800);
Adafruit_NeoPixel leds_rgbw(led_count_rgbw, 7, NEO_RGBW + NEO_KHZ800);


// accel setup
// float acceleration[3];
// float gyroscopic[3];
// float temperature;
float ax, ay, az, gx, gy, gz;

void setup() {
  Serial.begin(115200);
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
    delay(200);

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
  Serial.println("Startup complete, playing ring");
  boot_animation();
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
    IMU.readAcceleration(ax, ay, az);
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

float convert_xy_to_angle(float x, float y) {
  float angleDegrees = atan2(-1*y, x) * (180.0 / PI);
  if (angleDegrees < 0) { angleDegrees += 360; }
  float normalizedAngle = angleDegrees / 360.0;
  return normalizedAngle;
}
float convert_angle_to_led(float angle, int strip_id) {
  float output;
  int start_offset;
  switch (strip_id) {
    case strip_rgbw:
      output = (angle + 0.375) * led_count_rgbw;
      while(output > led_count_rgbw) { output -= led_count_rgbw; }
      while(output < 0) { output += led_count_rgbw; }
      return output;
    case strip_side:
      output = (angle + .675) * led_count_side;
      while(output > led_count_side) { output -= led_count_side; }
      while(output < 0) { output += led_count_side; }
      return output;
    case strip_rgb_inner:
      start_offset = 5;
      output = (angle + 0.02) * led_count_rgb_single;
      while(output > led_count_rgb_single) { output -= led_count_rgb_single; }
      while(output < 0) { output += led_count_rgb_single; }
      return output + start_offset;
    case strip_rgb_outer:
      start_offset = 29;
      output = (angle + 0) * led_count_rgb_single;
      while(output > led_count_rgb_single) { output -= led_count_rgb_single; }
      while(output < 0) { output += led_count_rgb_single; }
      return output + start_offset;
    case strip_rgb_upper:
      int start_offset = 53;
      output = angle * led_count_upper;
      while(output > led_count_upper) { output -= led_count_upper; }
      while(output < 0) { output += led_count_upper; }
      output += start_offset;
      if (output > led_count_rgb) { output -= led_count_rgb; }
      return output;
  }
}

void print_sensor_values() {
  Serial.println(F("acceleration: "));
  Serial.print(ax);Serial.print('\t');Serial.print(ay);Serial.print('\t');
  Serial.print(az);Serial.print('\t');Serial.print(gx);Serial.print('\t');
  Serial.print(gy);Serial.print('\t');Serial.println(gz);
  for (int i = 0; i < numSensors; i++) {
    Serial.print(F("dist"));Serial.print(i);Serial.print(": ");Serial.print(distance[i]);
    Serial.print(", ");
  }
  Serial.println();
}

void boot_animation() {
  uint8_t color;
  for(int active = 0; active < led_count_rgbw; active++) {
    for (int i = 0; i < led_count_rgbw; i++) {
      if(i == active) {
        color = 50;
      } else {
        color = 0;
      }
      leds_rgbw.setPixelColor(i, 0, 0, 0, color);
    } 
    leds_rgbw.show();
    delay(30);
  }
}

void simple_led_colors() {
  // int a, b, c;
  // for (int i = 0; i < 60; i++) {
  //   a = int(abs(float(distance[1]) / 10.0)) % 255;
  //   b = int(abs(float(distance[2]) / 10.0)) % 255;
  //   c = int(abs(float(x + y + z)) * 40) % 255;
  //   // leds_rgb.rainbow(firstPixelHue, 2, 255, 100, 1);
  //   leds_rgb.setPixelColor(i, a, b, c);
  // }
  // Serial.println(a);
  // Serial.println(b);
  // Serial.println(c);
  float direction = convert_xy_to_angle(ax, ay);

  // RGB W
  float led = convert_angle_to_led(direction, strip_rgbw);
  int led_active = int(floor(led));
  // Serial.println(led);
  // Serial.println(led_active);
  int color = 0;
  for (int i = 0; i < led_count_rgbw; i++) {
    if(i == led_active) { color = 100;} else { color = 0; }
    leds_rgbw.setPixelColor(i, color, 0, 0);
  }

  // SIDE
  led = convert_angle_to_led(direction, strip_side);
  led_active = int(floor(led));
  // Serial.println(led);
  // Serial.println(led_active);
  for (int i = 0; i < led_count_side; i++) {
    if(i == led_active) { color = 100;} else { color = 0; }
    leds_side.setPixelColor(i, 0, color, 0);
  }

  // rgb
  led = convert_angle_to_led(direction, strip_rgb_inner);
  int led_active_inner = int(floor(led));
  led = convert_angle_to_led(direction, strip_rgb_outer);
  int led_active_outer = int(floor(led));
  led = convert_angle_to_led(direction, strip_rgb_upper);
  int led_active_upper = int(floor(led));  

  Serial.println(led);
  // Serial.println(led_active_inner);
  // Serial.println(led_active_outer);
  Serial.println(led_active_upper);
  for (int i = 0; i < led_count_rgb; i++) {
    if(i == led_active_inner or i == led_active_outer or i == led_active_upper) { color = 100;} else { color = 0; }
    leds_rgb.setPixelColor(i, 0, 0, color);
  }

  leds_rgb.show();
  leds_rgbw.show();
  leds_side.show();
}

void loop() {
  // Serial.println(F("Loop start"));
  // if (loop_count % 10 == 0) {
  //   read_power();
  // }

  read_sensors();
  // print_sensor_values();
  simple_led_colors();

  Serial.println();
  // delay(5);

  loop_count++;
}
