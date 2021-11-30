/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "c:/Users/boyd/Documents/IoT/IoT_Capstone-IvanBoyd/U2_FairyLights/src/U2_FairyLights.ino"
/*
 * Project U2_FairyLights
 * Description:
 * Author:
 * Date:
 */

#include "neopixel.h"
// #define PIXEL_TYPE WS2812B 
void setup();
void loop();
#line 10 "c:/Users/boyd/Documents/IoT/IoT_Capstone-IvanBoyd/U2_FairyLights/src/U2_FairyLights.ino"
#define PIXEL_TYPE SK6812RGBW

const int NEOPIXPIN     = D8;
const int NEOPIXEL_NUM  = 100;
int  low = 1, med = 50, high = 145;         // NeoPix brightness 0-155, 145 bout high enuf 
int i, j;

Adafruit_NeoPixel fairyNP(NEOPIXEL_NUM, NEOPIXPIN, PIXEL_TYPE);
SYSTEM_MODE(SEMI_AUTOMATIC); //Using BLE and not Wifi

void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected, 5000);  
  Serial.printf("Starting up the fairy lights \n");
  delay(2000);
  fairyNP.begin();
  fairyNP.clear();
  fairyNP.setBrightness(100);
  for (i = 0; i < 100; i++) {
    fairyNP.setPixelColor(i, random(0,255),  random(0,255),  random(0,255));
    fairyNP.show();
    delay(100);
  }
  fairyNP.show();
  delay(2000);
  fairyNP.clear();
  fairyNP.show();
}


void loop() {
  fairyNP.begin();
  fairyNP.clear();
  for (i = 0; i < 200; i++) {
    fairyNP.setBrightness(random(1,100));
    fairyNP.setPixelColor(random(0,100), random(0,100), random(0,100), random(0,100));
    fairyNP.show();
    delay(30);
  }
  fairyNP.show();
  delay(4000);
  fairyNP.clear();
  fairyNP.show();
  delay(1000);
}