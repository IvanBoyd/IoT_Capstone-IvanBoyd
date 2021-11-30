/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "c:/Users/boyd/Documents/IoT/IoT_Capstone-IvanBoyd/U2_PIR-MotionSensor/src/U2_PIR-MotionSensor.ino"
/*
 * Project      U2_PIR-MotionSensor
 * Description: Hook up Encoder & PIR Motion Sensor, use Encoder LED's to test Motion Sensor, then
 *              attach other lights, eg, 100 LED Fairy Lights
 * Author:      Ivan Boyd
 * Date:        11/30/21
 */

//======================================
#include "Encoder.h"
void setup();
void loop();
void runPRTchk();
void detectsMovement();
#line 11 "c:/Users/boyd/Documents/IoT/IoT_Capstone-IvanBoyd/U2_PIR-MotionSensor/src/U2_PIR-MotionSensor.ino"
int timeSeconds = 10;

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
  Serial.begin(115200);
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
  digitalWrite(GRNPIN, LOW);
  digitalWrite(REDPIN, LOW);
}

void loop() {
  // Current time
  now = millis();
  // Turn off the LED after the number of seconds defined in the timeSeconds variable
  if(startTimer && (now - lastTrigger > (timeSeconds*1000))) {
    Serial.println("Motion stopped...");
    // digitalWrite(led, LOW);
    digitalWrite(REDPIN, LOW);
    startTimer = false;
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
 startTimer = true;
 lastTrigger = millis();
}