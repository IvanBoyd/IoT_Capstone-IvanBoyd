/*
 * Project        U2_FairyLights
 * Description:   Basic Fairy Light Test, Randomly show all 100 NeoPixel Fairy lights
 *                at random colors and random intensity
 *                    - Has been incorporated under U2_PIR-MotionSensor.ino
 * Author:        Ivan Boyd
 * Date:          11/29/2021
 */

#include "neopixel.h"
// #define PIXEL_TYPE WS2812B 
#define PIXEL_TYPE SK6812RGBW

const int NEOPIXPIN     = D8;
const int NEOPIXEL_NUM  = 100;
int  low = 1, med = 50, high = 145;         // NeoPix brightness 0-155, 145 bout high enuf 
int i, j;
// int new_Pix_P = 0;

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

  // //        E N C O D E R    S E T U P
  // position = myEnc.read();
  // last_Pos    =  -999;
  //   enc_P     = 0;    //assign enc to Pix Ring vars for Map function
  //   enc_Low   = 0;    // enc_P, enc_Low, enc_High, pix_Low, pix_High, new_Pix_P
  //   enc_High  = 95;
  //   pix_Low   = 0;
  //   pix_High  = 11;
  //   new_Pix_P = 0;
  //    // enc_P, enc_Low, enc_High, pix_Low, pix_High, new_Pix_P
  // if (position>enc_High) {
  //   myEnc.write(enc_High);
  // }
  // if (position<enc_Low) {
  //   myEnc.write(enc_Low);
  // }

  // // new_Pix_P = map(myEnc.read(), );
  // new_Pix_P = map(myEnc.read(), enc_Low, enc_High, pix_Low, pix_High);
   
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