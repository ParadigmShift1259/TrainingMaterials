#include <Adafruit_NeoPixel.h>
// Which pin on the Arduino is connected to the NeoPixels?
#define LED_PIN   6


// How many NeoPixels are attached to the Arduino?
#define LED_COUNT 9

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
// Arg 1 = # of pixels in the strip
// Arg 2 = Arduino pin # (most are valid)
// Arg 3 = Pixel type flags, add together as needed
//  NEO_KHZ800  800 KHz Bitstream (most NeoPixel products)
//  NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)

void setup() {
  // put your setup code here, to run once:
  strip.begin();            // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();             // Turn OFF all pixels ASAP
  strip.setBrightness(20);  // Set BRIGHTNESS low (max = 255)
}

void loop() {
  // put your main code here, to run repeatedly:
  // Fill along the length of the strip in various colors...
  // colorWipe(strip.Color(255,   0,   0), 50); // Red
  // colorWipe(strip.Color(  0, 255,   0), 50); // Green
  // colorWipe(strip.Color(  0,   0, 255), 50); // Blue

  // Do a theater marquee effect in various colors...
  // theaterChase(strip.Color(127, 127, 127), 50); // White, half brightness
  // theaterChase(strip.Color(127,   0,   0), 50); // Red, half brightness
  // theaterChase(strip.Color(  0,   0, 127), 50); // Blue, half brightness

  // rainbow(10);             // Flowing rainbow cycle along the whole strip
  theaterChaseRainbow(50); // Rainbow-enhanced theaterChase variant
}

void colorWipe(uint32_t color, int wait)
{
  for(int i=0; i<strip.numPixels(); i++)
  {
    strip.setPixelColor(i, color);
    strip.show();
    delay(wait);
  }
}

void theaterChase(uint32_t color, int wait)
{
  for(int a=0; a<10; a++) {
    for(int b=0; b<3; b++) {
      strip.clear();

      for(int c=b; c<strip.numPixels(); c += 3) {
        strip.setPixelColor(c, color);
      }
      strip.show();
      delay(wait);
    }
  }  
}

void rainbow(int wait)
{
  for(long firstPixelHue=0; firstPixelHue < 5*65536; firstPixelHue += 256) {
    strip.rainbow(firstPixelHue);
    strip.show();
    delay(wait);
  }
}

void theaterChaseRainbow(int wait)
{
  int firstPixelHue = 0;
  for(int a=0; a<30; a++) {
    for(int b=0; b<3; b++) {
      strip.clear();
      for(int c=b; c<strip.numPixels(); c+= 3) {
        int       hue   = firstPixelHue + c * 65536L / strip.numPixels();
        uint32_t  color = strip.gamma32(strip.ColorHSV(hue));
        strip.setPixelColor(c, color);
      }
      strip.show();
      delay(wait);
      firstPixelHue += 65536 / 90;
    }
  }
}