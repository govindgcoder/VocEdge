#include <Arduino.h>
#include <FastLED.h>

#define LED_PIN     48
#define NUM_LEDS    1
#define BRIGHTNESS  50  // Keep it low to protect eyes/power limits

CRGB leds[NUM_LEDS];

void setup() {
    Serial.begin(115200);
    
    // Initialize FastLED for the onboard WS2812B / NeoPixel
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(BRIGHTNESS);
}

void loop() {
    // Turn LED Green
    leds[0] = CRGB::Green;
    FastLED.show();
    delay(1000);

    // Turn LED Off
    leds[0] = CRGB::Black;
    FastLED.show();
    delay(1000);
}