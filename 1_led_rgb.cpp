#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel LED_RGB(1, 48, NEO_GRBW + NEO_KHZ800); 

void setup() {
  Serial.begin(115200);
  LED_RGB.begin();
  LED_RGB.setBrightness(5);
}

void loop() {
  Serial.println("LOOP");
  LED_RGB.setPixelColor(0, LED_RGB.Color(255, 0, 0)); // Red
  LED_RGB.show();
  delay(1000);
  
  LED_RGB.setPixelColor(0, LED_RGB.Color(0, 255, 0)); // Green
  LED_RGB.show();
  delay(1000);
  
  LED_RGB.setPixelColor(0, LED_RGB.Color(0, 0, 255)); // Blue
  LED_RGB.show();
  delay(1000);
}