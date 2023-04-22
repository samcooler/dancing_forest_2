
#include <Adafruit_NeoPixel.h>
#include <Arduino_LSM6DS3.h>

// setup LEDs
Adafruit_NeoPixel leds_lower_side(60, 9, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel leds_upper(12, 8, NEO_RGB + NEO_KHZ800);
Adafruit_NeoPixel leds_lower_center(16, 7, NEO_RGBW + NEO_KHZ800);

float x, y, z, gx, gy, gz;


void setup() {

  Serial.begin(9600);
  while (!Serial)
    ;

  // setup IMU
  Wire.begin(12, 11);
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }

  // enable LED power
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);

  leds_lower_side.begin();  // INITIALIZE NeoPixel strip object (REQUIRED)
  leds_lower_side.show();   // Turn OFF all pixels ASAP

  leds_upper.begin();  // INITIALIZE NeoPixel strip object (REQUIRED)
  leds_upper.show();   // Turn OFF all pixels ASAP

  leds_lower_center.begin();  // INITIALIZE NeoPixel strip object (REQUIRED)
  leds_lower_center.show();   // Turn OFF all pixels ASAP
}

void loop() {
  for (long firstPixelHue = 0; firstPixelHue < 5 * 65536; firstPixelHue += 256) {
    leds_lower_side.rainbow(firstPixelHue, 2, 255, 100, 1);
    leds_upper.rainbow(firstPixelHue, 2, 255, 100, 1);
    leds_lower_center.rainbow(firstPixelHue, 2, 255, 100, 1);

    leds_lower_side.show();    // Update strip with new contents
    leds_upper.show();         // Update strip with new contents
    leds_lower_center.show();  // Update strip with new contents
    delay(2);               // Pause for a moment

    if (firstPixelHue % 20 == 0 & IMU.accelerationAvailable()) {
      IMU.readAcceleration(x, y, z);
      IMU.readGyroscope(gx, gy, gz);

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
    }
  }
}


// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(int wait) {

  for (long firstPixelHue = 0; firstPixelHue < 5 * 65536; firstPixelHue += 256) {
    leds_lower_side.rainbow(firstPixelHue, 2, 255, 100, 1);
    leds_upper.rainbow(firstPixelHue, 2, 255, 100, 1);
    leds_lower_center.rainbow(firstPixelHue, 2, 255, 100, 1);

    leds_lower_side.show();    // Update strip with new contents
    leds_upper.show();         // Update strip with new contents
    leds_lower_center.show();  // Update strip with new contents
    delay(wait);               // Pause for a moment
  }
}
