// standtest for the dancing forest

#include <Adafruit_NeoPixel.h>


// Declare our NeoPixel strip object:
Adafruit_NeoPixel leds_lower_side(72, 9, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel leds_upper(24, 8, NEO_RGB + NEO_KHZ800);
Adafruit_NeoPixel leds_lower_center(16, 7, NEO_RGBW + NEO_KHZ800);

void setup() {
  // enable LED power
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);

  leds_lower_side.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  leds_lower_side.show();            // Turn OFF all pixels ASAP
  
  leds_upper.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  leds_upper.show();            // Turn OFF all pixels ASAP
  
  leds_lower_center.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  leds_lower_center.show();            // Turn OFF all pixels ASAP
}

void loop() {
  rainbow(10);

  
}


// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(int wait) {

  for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {
    leds_lower_side.rainbow(firstPixelHue, 2, 255, 100, 1);
    leds_upper.rainbow(firstPixelHue, 2, 255, 100, 1);
    leds_lower_center.rainbow(firstPixelHue, 2, 255, 100, 1);

    leds_lower_side.show(); // Update strip with new contents
    leds_upper.show(); // Update strip with new contents
    leds_lower_center.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
  }
}
