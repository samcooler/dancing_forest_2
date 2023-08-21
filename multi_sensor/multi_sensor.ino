#include "Adafruit_VL53L1X.h"
#include "Adafruit_NeoPixel.h"
#include "Arduino_LSM6DS3.h"
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>

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
const float battery_voltage_threshold = 3.3;

// lidar setup
#define use_lidar 0
const int numSensors = 3;
Adafruit_VL53L1X *sensors[numSensors];
const uint8_t sensorAddresses[3] = { 0x2A, 0x2B, 0x2C };  // 0x29 is default
const uint8_t sensorXShutPins[3] = { 3, 4, 5 };
int16_t distance[3];

// setup LEDs
#define led_count_rgbw 16
#define led_count_rgb_single 24
#define led_count_upper 24
#define led_count_rgb 72
#define led_count_side 24
#define strip_rgbw 0  // used for identity in functions
#define strip_side 1
#define strip_rgb_inner 2
#define strip_rgb_outer 3
#define strip_rgb_upper 4
#define strip_rgb_lower 5
#define BRIGHTNESS 255

Adafruit_NeoPixel leds_rgb(led_count_rgb, 9, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel leds_side(led_count_side, 8, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel leds_rgbw(led_count_rgbw, 7, NEO_GRBW + NEO_KHZ800);
int sleep_luminance = 0;
int main_luminance = 1000;
const int sleep_start = 12000;
const int sleep_fully = 20000;
float sleep_oscillation_period = 1000;
const float sleep_luminance_baseline = 100;
const float sleep_oscillation_magnitude = 30;
const int sensor_threshold = 10;

// accel setup
float ax, ay, az, gx, gy, gz;
uint32_t last_motion_time;
const int BUFFER_SIZE = 20;  // Set the buffer size for acceleration smoothing

float buffer_ax[BUFFER_SIZE] = { 0 };
float buffer_ay[BUFFER_SIZE] = { 0 };
int buffer_index_ax = 0;
int buffer_index_ay = 0;
float buffer_sum_ax = 0;
float buffer_sum_ay = 0;
int sample_count = 0;

#define USE_MAGNETOMETER false
Adafruit_LIS3MDL magnetometer;
bool magnetometer_found = false;
sensors_event_t event;

void setup() {
  randomSeed(analogRead(PIN_battery_measure));

  Serial.begin(115200);
  while (!Serial) delay(10);
  delay(1000);
  Serial.println(F("Dancing Forest"));

  // I2C start
  Wire.begin(12, 11);
  Serial.println(F("Wire is started"));
  delay(500);

  if(USE_MAGNETOMETER)
  {
    magnetometer_found = magnetometer.begin_I2C(0x1C, &Wire);
    if(magnetometer_found)
    {
      Serial.println("LIS3MDL Found!");
      magnetometer.setPerformanceMode(LIS3MDL_MEDIUMMODE);
      magnetometer.setOperationMode(LIS3MDL_CONTINUOUSMODE);
      magnetometer.setDataRate(LIS3MDL_DATARATE_40_HZ);
      magnetometer.setRange(LIS3MDL_RANGE_4_GAUSS);
      magnetometer.setIntThreshold(500);
      magnetometer.setIntThreshold(500);
      magnetometer.configInterrupt(false, false, true, // enable z axis
                              true, // polarity
                              false, // don't latch
                              true); // enabled!
    }else
    {
      Serial.println("LIS3MDL NOT Found!");
    }
  }


  // accel start
  while (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    delay(500);
  }
  Serial.println(F("Accelerometer is started"));
  last_motion_time = millis();

  // LED start
  sleep_oscillation_period += random(2000);
  leds_rgb.begin();
  leds_rgb.setBrightness(BRIGHTNESS);
  leds_rgb.show();
  leds_side.begin();
  leds_side.setBrightness(BRIGHTNESS);
  leds_side.show();
  leds_rgbw.begin();
  leds_rgbw.setBrightness(BRIGHTNESS);
  leds_rgbw.show();
  // enable LED power
  pinMode(PIN_led_enable, OUTPUT);
  digitalWrite(PIN_led_enable, HIGH);


  // lidar start
  if (use_lidar) {
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
  }
  Serial.println("Startup complete, playing ring");
  boot_animation();
}

void read_power() {
  power_current = analogRead(PIN_current_measure);
  power_battery = analogRead(PIN_battery_measure) * 0.001611;

  Serial.print(F("I: "));
  Serial.println(power_current, 2);
  Serial.print(F("V: "));
  Serial.println(power_battery, 2);
}

void read_sensors() {
  // read accel
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    // temperature = accel.readTempF();

    if (abs(gx) + abs(gy) > sensor_threshold) {
      last_motion_time = millis();
    }
  }
  // read lidar
  if (use_lidar) {
    for (int i = 1; i < numSensors; i++) {
      if (sensors[i]->dataReady()) {
        distance[i] = sensors[i]->distance();
        sensors[i]->clearInterrupt();
      }
    }
  }

  if (magnetometer_found)
  {
    magnetometer.getEvent(&event);
  }
}

float convert_xy_to_angle(float x, float y) {
  float angleDegrees = atan2(-1 * y, x) * (180.0 / PI);
  if (angleDegrees < 0) { angleDegrees += 360; }
  float normalizedAngle = angleDegrees / 360.0;
  return normalizedAngle;
}
float convert_xy_to_magnitude(float x, float y) {
  return sqrt(x * x + y * y);
}
float convert_angle_to_led(float angle, int strip_id) {
  float output;
  int start_offset;
  switch (strip_id) {
    case strip_rgbw:
      output = (angle + 0.375) * led_count_rgbw;
      while (output > led_count_rgbw) { output -= led_count_rgbw; }
      while (output < 0) { output += led_count_rgbw; }
      return output;
    case strip_side:
      output = (angle + .675) * led_count_side;
      while (output > led_count_side) { output -= led_count_side; }
      while (output < 0) { output += led_count_side; }
      return output;
    case strip_rgb_inner:
      start_offset = 5;
      output = (angle + 0.02) * led_count_rgb_single;
      while (output > led_count_rgb_single) { output -= led_count_rgb_single; }
      while (output < 0) { output += led_count_rgb_single; }
      return output + start_offset;
    case strip_rgb_outer:
      start_offset = 29;
      output = (angle + 0) * led_count_rgb_single;
      while (output > led_count_rgb_single) { output -= led_count_rgb_single; }
      while (output < 0) { output += led_count_rgb_single; }
      return output + start_offset;
    case strip_rgb_upper:
      int start_offset = 53;
      output = angle * led_count_upper;
      while (output > led_count_upper) { output -= led_count_upper; }
      while (output < 0) { output += led_count_upper; }
      output += start_offset;
      if (output > led_count_rgb) { output -= led_count_rgb; }
      return output;
  }
}

void print_sensor_values() {
  Serial.println(F("acceleration: "));
  Serial.print(ax);
  Serial.print('\t');
  Serial.print(ay);
  Serial.print('\t');
  Serial.print(az);
  Serial.print('\t');
  Serial.print(gx);
  Serial.print('\t');
  Serial.print(gy);
  Serial.print('\t');
  Serial.println(gz);
  if (use_lidar) {
    for (int i = 0; i < numSensors; i++) {
      Serial.print(F("dist"));
      Serial.print(i);
      Serial.print(": ");
      Serial.print(distance[i]);
      Serial.print(", ");
    }
  }
  /* Display the results (magnetic field is measured in uTesla) */
  Serial.print("\tX: "); Serial.print(event.magnetic.x);
  Serial.print(" \tY: "); Serial.print(event.magnetic.y);
  Serial.print(" \tZ: "); Serial.print(event.magnetic.z);
  Serial.println(" uTesla ");
  // Serial.println();
}

void boot_animation() {
  uint8_t color;
  for (int active = 0; active < led_count_rgbw; active++) {
    for (int i = 0; i < led_count_rgbw; i++) {
      if (i == active) {
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

float calculate_smoothed_mean(float new_value, float buffer[], int &buffer_index, float &buffer_sum) {
  buffer_sum -= buffer[buffer_index];
  buffer[buffer_index] = new_value;
  buffer_sum += new_value;

  if (sample_count < BUFFER_SIZE) { sample_count++; }
  buffer_index = (buffer_index + 1) % BUFFER_SIZE;
  return buffer_sum / sample_count;
}

void sleep_colors() {
  uint32_t now = millis();

  float time_since_motion = now - last_motion_time;
  if (time_since_motion > sleep_fully) {
    // sleep mode
    sleep_luminance = int(sin((time_since_motion - sleep_fully) / sleep_oscillation_period) * sleep_oscillation_magnitude + sleep_luminance_baseline);
    main_luminance = 0;
  } else if (time_since_motion > sleep_start and time_since_motion <= sleep_fully) {
    // fade into sleep over 4 seconds, after 1 second of inactivity
    sleep_luminance = int((time_since_motion - sleep_start) * sleep_luminance_baseline / (sleep_fully - sleep_start));
    main_luminance -= 1;
    if (main_luminance < 0) { main_luminance = 0;}

  } else if (time_since_motion < 1000 and sleep_luminance > 0) {
    // fade out of sleep slowly
    if(loop_count % 3 == 0) {
      sleep_luminance -= 1;
    }
    main_luminance += 5;
    if( main_luminance > 1000) {
      main_luminance = 1000;
    }    
  } else {
    sleep_luminance = 0;
    main_luminance += 5;
    if( main_luminance > 1000) {
      main_luminance = 1000;
    }
  }
  // Serial.println(sleep_luminance);
  int sleep_luminance_out = sleep_luminance;
  for (int i = 0; i < led_count_rgbw; i++) {
    // if(random(100) <= 1) {
      // sleep_luminance_out = constrain(sleep_luminance_out + 100, 0, 255);
    // }
    leds_rgbw.setPixelColor(i,leds_rgbw.gamma32(leds_rgbw.Color(0,0,0,sleep_luminance_out)));
  }
  leds_rgbw.show();
}

int which_rgb_strip(int i) {
  if (i < 5 or i > 52) {
    return strip_rgb_upper;
  } else {
    return strip_rgb_lower;
  }
}

void tilt_colors() {
  float mean_ax = calculate_smoothed_mean(ax, buffer_ax, buffer_index_ax, buffer_sum_ax);
  float mean_ay = calculate_smoothed_mean(ay, buffer_ay, buffer_index_ay, buffer_sum_ay);
  float direction = convert_xy_to_angle(mean_ax, mean_ay);
  direction *= 2;
  if(direction > 1.0) {
    direction -= 1.0;
  }
  // Serial.println(direction);
  float distance = convert_xy_to_magnitude(mean_ax, mean_ay);

  int distance_saturation = int(255 * 3 * distance);
  int distance_value = int(200 * distance * main_luminance / 1000);
  // int distance_value_brighter = int(400 * distance);
  uint32_t base_color = leds_rgb.ColorHSV(int(65536 * direction), constrain(distance_saturation, 0, 255), constrain(distance_value, 0, 175));
  // uint32_t brighter = leds_rgb.ColorHSV(int(65536 * direction), constrain(distance_saturation, 0, 255), constrain(distance_value_brighter, 0, 255));

  // Serial.println(direction);
  // Serial.println(distance);
  // Serial.print(distance_value);
  // Serial.print(' ');
  // Serial.println(main_luminance);

  for (int i = 0; i < led_count_rgb; i++) {
    leds_rgb.setPixelColor(i, leds_rgb.gamma32(base_color));
  }
  leds_rgb.show();

  // for (int i = 0; i < led_count_side; i++) {
  //   leds_side.setPixelColor(i, leds_side.gamma32(base_color));
  // }
  // leds_side.show();

}

void direction_test_colors() {
  float direction = convert_xy_to_angle(ax, ay);

  // RGB W
  float led = convert_angle_to_led(direction, strip_rgbw);
  int led_active = int(floor(led));
  // Serial.println(led);
  // Serial.println(led_active);
  int color = 0;
  for (int i = 0; i < led_count_rgbw; i++) {
    if (i == led_active) {
      color = 100;
    } else {
      color = 0;
    }
    leds_rgbw.setPixelColor(i, color, 0, 0);
  }

  // SIDE
  led = convert_angle_to_led(direction, strip_side);
  led_active = int(floor(led));
  // Serial.println(led);
  // Serial.println(led_active);
  for (int i = 0; i < led_count_side; i++) {
    if (i == led_active) {
      color = 100;
    } else {
      color = 0;
    }
    leds_side.setPixelColor(i, 0, color, 0);
  }

  // rgb
  led = convert_angle_to_led(direction, strip_rgb_inner);
  int led_active_inner = int(floor(led));
  led = convert_angle_to_led(direction, strip_rgb_outer);
  int led_active_outer = int(floor(led));
  led = convert_angle_to_led(direction, strip_rgb_upper);
  int led_active_upper = int(floor(led));

  // Serial.println(led);
  // Serial.println(led_active_inner);
  // Serial.println(led_active_outer);
  // Serial.println(led_active_upper);
  for (int i = 0; i < led_count_rgb; i++) {
    if (i == led_active_inner or i == led_active_outer or i == led_active_upper) {
      color = 100;
    } else {
      color = 0;
    }
    leds_rgb.setPixelColor(i, 0, 0, color);
  }

  leds_rgb.show();
  // leds_rgbw.show();
  // leds_side.show();
}

void loop() {
  // Serial.println(F("Loop start"));
  if (loop_count % 100 == 0) {
    Serial.println(F("Loop 100th start"));
    read_power();
  }

  read_sensors();
  print_sensor_values();
  // simple_led_colors();
  sleep_colors();
  tilt_colors();


  loop_count++;
}
