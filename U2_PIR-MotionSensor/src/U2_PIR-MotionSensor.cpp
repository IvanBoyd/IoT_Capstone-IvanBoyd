/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "c:/Users/boyd/Documents/IoT/IoT_Capstone-IvanBoyd/U2_PIR-MotionSensor/src/U2_PIR-MotionSensor.ino"
/*
 * Project      U2 PIR Motion Sensor triggers the 100 NeoPixel Fairy Lights
 * Description: This module uses an interrupt based on motion to trigger lighting the
 *        NeoPixel fairy Lights. 
 *        Next step: to program Encoder to set the intensity of the fairy lights
 *  Problems: - uses delay to control random display of lights
 * Author:      Ivan Boyd
 * Date:        11/30/21
 * 
 */

//======================================
#include "Encoder.h"
void setup();
void loop();
void runPRTchk();
void detectsMovement();
void flSetUp();
void flSub();
void encLedSetUp();
#line 14 "c:/Users/boyd/Documents/IoT/IoT_Capstone-IvanBoyd/U2_PIR-MotionSensor/src/U2_PIR-MotionSensor.ino"
int timeSeconds = 10;
#include "neopixel.h"
// #define PIXEL_TYPE WS2812B 
#define PIXEL_TYPE SK6812RGBW

const int FLPIXPIN     = D8;                // Fairy Light Pix Pin 
const int FLPIXEL_NUM  = 100;               // Fairy Light Pix Number 
int   FLlow = 1, FLmed = 50, FLhigh = 145;   // NeoPix brightness 0-255, 145 is too bright for me 
int   FLi, FLj;
// const int NEOPIXPIN     = D8;
// const int NEOPIXEL_NUM  = 100;
// int  low = 1, med = 50, high = 145;         // NeoPix brightness 0-255, 145 is too bright for me 
// int i, j;


Adafruit_NeoPixel fairyNP(FLPIXEL_NUM, FLPIXPIN, PIXEL_TYPE);
bool fairyLightsOn = false;

// Set GPIOs for LED and PIR Motion Sensor
const int led = 16;
const int motionSensor = D7;

// Timer: Auxiliary variables
// unsigned long now = millis();
// unsigned long lastTrigger = 0;
// boolean startTimer = false;

unsigned long FLnow         = millis();
unsigned long FLlastTrigger = 0;
bool       FLstartTimer  = false;

// Encoder Header 
int  pinA     =  A5,          pinB      = A4,
     position = 0,            last_Pos  = 0
     ; 
int enc_P   = 0,              enc_Low = 0,          enc_High  = 0,       //set enc to Pix Ring vars for Map function
    pix_Low   = 0,            pix_High  = 0,        new_Pix_P = 0        // enc = encoder, pix = pixel ring
    ;                                        

const int SWITCHPIN = A0,     GRNPIN    = A2,       REDPIN    = A1,      T    = 1000;
              //pinmodes  22, input, 21 & 20 output to pixels,  21  output to pixels   20 output to pixels
              // PIXPIN    = A3,     // Neopixels neeeds a lib but no pin mode
                
Encoder myEnc(pinB,pinA);
 
SYSTEM_MODE(SEMI_AUTOMATIC); //Using BLE and not Wifi
// Checks if motion was detected, sets LED HIGH and starts a timer

void setup() {
  // Serial port for debugging purposes
  Serial.begin(9600);
  runPRTchk();        // Start print to monitor
  FLlastTrigger = millis();

  // PIR Motion Sensor mode INPUT_PULLUP
  pinMode(motionSensor, INPUT);
  // Set motionSensor pin as interrupt, assign interrupt function and set RISING mode
  attachInterrupt(motionSensor, detectsMovement, RISING);

  // Set LED to LOW
  encLedSetUp();       // Set Up for Encoder LED's
  flSetUp();       // Set Up for Fairy Lights

  //                    E N C O D E R    S E T U P
  position = myEnc.read();
  last_Pos    =  -999;
    enc_P     = 0;    //assign enc to Pix Ring vars for Map function
    enc_Low   = 0;    // enc_P, enc_Low, enc_High, pix_Low, pix_High, new_Pix_P
    enc_High  = 95;
    pix_Low   = 2;
    pix_High  = 100;
    new_Pix_P = 0;
     // enc_P, enc_Low, enc_High, pix_Low, pix_High, new_Pix_P
  if (position>enc_High) {
    myEnc.write(enc_High);
  }
  if (position<enc_Low) {
    myEnc.write(enc_Low);
  }

  // new_Pix_P = map(myEnc.read(), );
  new_Pix_P = map(myEnc.read(), enc_Low, enc_High, pix_Low, pix_High);
  last_Pos = myEnc.read();
}

void loop() {
//             N E W    E N C O D E R     S T U F F
position = myEnc.read();  
// Serial.printf("Encoder position: %i \n",position);
  //      myEnc.write(GRNPIN);
  if (position == last_Pos) {    //dial has not moved
      //  Serial.printf("NO Movement: Im in pos = %i last_Pos part of if: %i \n", position, last_Pos);
      //  delay(T);
  }
  else {                 // dial has moved so  change NeoPix pos & light itprint to screen
    // Serial.printf("YES!! Movement: I was in last_pos/: %i  Now Pos is: %i \n", last_Pos, position);
    // Serial.printf("Imagine Neo Pixel light up: %i \n",position);
    last_Pos = myEnc.read();
      //  delay(T);
    // enc_P, enc_Low, enc_High, pix_Low, pix_High, new_Pix_P
    if (position>enc_High) {
      myEnc.write(enc_High);
    }
    if (position<enc_Low) {
      myEnc.write(enc_Low);
    }
        last_Pos = myEnc.read();
    //    delay(T);
    // enc_P, enc_Low, enc_High, pix_Low, pix_High, new_Pix_P
    if (position>enc_High) {
      myEnc.write(enc_High);
    }
    if (position<enc_Low) {
      myEnc.write(enc_Low);
    }
  }
  new_Pix_P = map(myEnc.read(), enc_Low, enc_High, pix_Low, pix_High);
  // Serial.printf("mapping Neo pix's:  enc_Low: %i  enc_High %i pix_Low %i  pix_High %i\n",enc_Low, enc_High, pix_Low, pix_High);
  // Serial.printf("mapping Neo pix's: Position %i maps to Neo Pixel: %i \n",position, new_Pix_P);
  //         end    N E W    E N C O D E R     S T U F F  end

  flSub();        // turns on FL's when PIR sensor is activated
}

// Start print to monitor
void runPRTchk() {               
  Serial.begin(9600);
  // while(!Serial);
  waitFor(Serial.isConnected, 15000); 
  Serial.printf("Printer initialized\n"); 
  delay(1000);
}

void detectsMovement() {
//  Serial.println("MOTION DETECTED!!!");
 digitalWrite(REDPIN, HIGH);
 // Turn on Fairy Lights
 fairyLightsOn    = true;
 FLstartTimer     = true;
 FLlastTrigger    = millis();
}

// Set Up for Fairy Lights
void flSetUp()  {
  Serial.printf("Starting up the fairy lights \n");
  // delay(500);
  fairyNP.begin();
  fairyNP.clear();
  fairyNP.setBrightness(30);
  for (FLi = 0; FLi < 100; FLi++) {
    fairyNP.setPixelColor(FLi, random(0,255),  random(0,255),  random(0,255));
    fairyNP.show();
    delay(30);
  }
  // fairyNP.show();
  // delay(2000);
  fairyNP.clear();
  fairyNP.show();
}

// turns on FL's (fairy lights) when PIR sensor is activated
void flSub() {
    // fairyNP.begin();
  FLnow = millis();               // Current time
  // Turn off the LED after the number of seconds defined in the timeSeconds variable
  if(FLstartTimer && (FLnow - FLlastTrigger > (timeSeconds*1000))) {
    // Serial.println("Motion stopped...");
    // digitalWrite(led, LOW);
    digitalWrite(REDPIN, LOW);
    digitalWrite(GRNPIN, HIGH);
    FLstartTimer = false;
    // turn off Fairy Lights
    fairyNP.clear();
    fairyNP.show();
    fairyLightsOn = false;
  }
  if (fairyLightsOn)  {
    digitalWrite(GRNPIN, LOW);
    // fairyNP.setBrightness(random(1,4));
    fairyNP.setBrightness(random(1,new_Pix_P));
    fairyNP.setPixelColor(random(0,100), random(0,255), random(0,255), random(0,255));
    fairyNP.show();
    delay(random(50,100));        // *** needs to be embedded in timer function
  }
}
  // Set LED to LOW
  void encLedSetUp() {
    pinMode(led, OUTPUT);
    pinMode(GRNPIN, OUTPUT);
    pinMode(REDPIN, OUTPUT);
    digitalWrite(led, LOW);
    digitalWrite(GRNPIN, HIGH);
    digitalWrite(REDPIN, LOW);
  }