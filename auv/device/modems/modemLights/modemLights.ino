#include <Adafruit_NeoPixel.h>
#define PIN        6 
#define NUMPIXELS 8 
#define NUM_FLASHES 1
#define FLASHING_DELAY 150

#define RX_PIN A1
#define TX_PIN A2

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRBW + NEO_KHZ800);

void flash(uint8_t r, uint8_t g, uint8_t b);

void setup() {

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, INPUT);

  Serial.begin(9600);

  pixels.clear(); // Set all pixel colors to 'off'

}

void loop() {

    Serial.println(analogRead(RX_PIN));
    if (analogRead(RX_PIN)>=500) { //rx
      flash(255,130,0);
    }
    else if (analogRead(TX_PIN)>=500) { //tx
      flash(0, 255, 0);
    }
    else {
      pixels.clear();
    }
  delay(50);
}


void flash(uint8_t r, uint8_t g, uint8_t b) {

  for (int x = 0; x < NUM_FLASHES; x++) {
    for (int a = 0; a < NUMPIXELS; a++) {
      pixels.setPixelColor(a, r, g, b);
    }
    pixels.show();   // Send the updated pixel colors to the hardware.
    delay(FLASHING_DELAY);
    pixels.clear(); // Set all pixel colors to 'off'
    pixels.show();   // Send the updated pixel colors to the hardware.
    delay(50);
  }

  pixels.clear(); // Set all pixel colors to 'off'

}
