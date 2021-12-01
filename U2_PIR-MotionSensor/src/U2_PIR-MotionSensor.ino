/*
 * Project      U2 PIR Motion Sensor triggers the 100 NeoPixel Fairy Lights
 * Description: This module uses an interrupt based on motion to trigger lighting the
 *        NeoPixel fairy Lights. 
 *        Next step: to program Encoder to set the intensity of the fairy lights
 *  Problems: - uses delay to control random display of lights
 * Author:      Ivan Boyd
 * Date:        11/30/21
 * 
 * STATUS: Encoder not moving based on enc_read, even tho the value changes its not goint
 * into the else loop based on a change of value
 */

//======================================
#include "Encoder.h"
int timeSeconds = 10;
#include "neopixel.h"
// #define PIXEL_TYPE WS2812B 
#define PIXEL_TYPE SK6812RGBW

const int NEOPIXPIN     = D8;
const int NEOPIXEL_NUM  = 100;
int  low = 1, med = 50, high = 145;         // NeoPix brightness 0-155, 145 bout high enuf 
int i, j;

Adafruit_NeoPixel fairyNP(NEOPIXEL_NUM, NEOPIXPIN, PIXEL_TYPE);
bool fairyLightsOn = false;
// Set GPIOs for LED and PIR Motion Sensor
const int led = 16;
const int motionSensor = D7;

// Timer: Auxiliary variables
unsigned long now = millis();
unsigned long lastTrigger = 0;
boolean startTimer = false;

// Encoder Header 
int  pinA =  A5,              pinB  = A4,
     position = 0,            last_Pos = 0
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
  lastTrigger = millis();

  
  // PIR Motion Sensor mode INPUT_PULLUP
  pinMode(motionSensor, INPUT);
  // Set motionSensor pin as interrupt, assign interrupt function and set RISING mode
  attachInterrupt(motionSensor, detectsMovement, RISING);

  // Set LED to LOW
  pinMode(led, OUTPUT);
  pinMode(GRNPIN, OUTPUT);
  pinMode(REDPIN, OUTPUT);
  digitalWrite(led, LOW);
  digitalWrite(GRNPIN, HIGH);
  digitalWrite(REDPIN, LOW);

  // Set Up for Fairy Lights
  Serial.printf("Starting up the fairy lights \n");
  delay(2000);
  fairyNP.begin();
  fairyNP.clear();
  fairyNP.setBrightness(100);
  for (i = 0; i < 100; i++) {
    fairyNP.setPixelColor(i, random(0,255),  random(0,255),  random(0,255));
    fairyNP.show();
    delay(40);
  }
  fairyNP.show();
  delay(2000);
  fairyNP.clear();
  fairyNP.show();
  //                    E N C O D E R    S E T U P
  position = myEnc.read();
  last_Pos    =  -999;
    enc_P     = 0;    //assign enc to Pix Ring vars for Map function
    enc_Low   = 0;    // enc_P, enc_Low, enc_High, pix_Low, pix_High, new_Pix_P
    enc_High  = 95;
    pix_Low   = 0;
    pix_High  = 11;
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
  
}

void loop() {
//             N E W    E N C O D E R     S T U F F
position = myEnc.read();  
Serial.printf("Encoder position: %i \n",position);
  //      myEnc.write(GRNPIN);
  if (position == last_Pos) {    //dial has not moved
       Serial.printf("NO Movement: Im in pos = %i last_Pos part of if: %i \n", position, last_Pos);
       delay(T);
  }
  else {                 // dial has moved so  change NeoPix pos & light itprint to screen
    Serial.printf("YES!! Movement: I was in last_pos/: %i  Now Pos is: %i \n", last_Pos, position);
    Serial.printf("Imagine Neo Pixel light up: %i \n",position);
    Serial.printf("Green Pin encoder pos: %i \n",position);
    last_Pos = myEnc.read();
    //    delay(T);
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
//         end    N E W    E N C O D E R     S T U F F  end

  fairyNP.begin();
  // Current time
  now = millis();
  // Turn off the LED after the number of seconds defined in the timeSeconds variable
  if(startTimer && (now - lastTrigger > (timeSeconds*1000))) {
    Serial.println("Motion stopped...");
    // digitalWrite(led, LOW);
    digitalWrite(REDPIN, LOW);
    digitalWrite(GRNPIN, HIGH);

    startTimer = false;
    // turn off Fairy Lights
    fairyNP.clear();
    fairyNP.show();
    fairyLightsOn = false;
  }
  if (fairyLightsOn)  {
    digitalWrite(GRNPIN, LOW);
    fairyNP.setBrightness(random(1,4));
    fairyNP.setPixelColor(random(0,100), random(0,255), random(0,255), random(0,255));
    fairyNP.show();
    delay(random(50,100));        // *** needs to be embedded in timer function
  }
}

void runPRTchk() {               // Start print to monitor
  Serial.begin(9600);
  while(!Serial);
  Serial.printf("Printer initialized\n");
  delay(1000);
}

void detectsMovement() {
 Serial.println("MOTION DETECTED!!!");
 digitalWrite(REDPIN, HIGH);
 // Turn on Fairy Lights
 fairyLightsOn = true;
 startTimer = true;
 lastTrigger = millis();
}