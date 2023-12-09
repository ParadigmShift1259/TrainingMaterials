#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Unneeded
#endif



#define LED_PIN 6


#define LED_COUNT 9


Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);



void setup() {

  strip.begin();
  strip.show();
  strip.setBrightness(20);

}

void loop() {
  
  //colorWipe(strip.Color(255,  0,  0),  50);
  //colorWipe(strip.Color(0,  255,  0), 50);
  //colorWipe(strip.Color(0,  0,  255), 50);




  //theaterChase(strip.Color(127,  127,  127),  50);
  //theaterChase(strip.Color(127,  0,  0), 50);
  //theaterChase(strip.Color(0,  127,  0), 50);

  rainbow(10);
  theaterChaseRainbow(50);

}


void colorWipe(uint32_t color, int wait)
{

  for(int i=0; i<strip.numPixels(); i++){
    strip.setPixelColor(i, color);
    strip.show();
    delay(wait);
  }

}

void theaterChase(uint32_t color, int wait)
{

  for(int a=0; a<10; a++){
    for(int b=0; b<3; b++) {
      strip.clear();

      for(int c=b; c<strip.numPixels(); c += 3){
        strip.setPixelColor(c, color);
      }
      strip.show();
      delay(wait);

    }

  }

}

void rainbow(int wait){
  for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue +=256){
    strip.rainbow(firstPixelHue);
    strip.show();
    delay(wait);
  }


}

void theaterChaseRainbow(int wait) {
  int firstPixelHue =0;
  for(int a = 0; a <30; a++){
    for(int b=0; b<3; b++){
      strip.clear();
      for(int c =b; c<strip.numPixels(); c +=3){
        int   hue     = firstPixelHue + c *65536L / strip.numPixels();
        uint32_t color = strip.gamma32(strip.ColorHSV(hue));
        strip.setPixelColor(c, color);
      }
      strip.show();
      delay(wait);
      firstPixelHue += 65536 / 90;
    }
  }
}


