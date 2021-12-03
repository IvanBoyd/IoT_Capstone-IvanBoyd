/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "c:/Users/boyd/Documents/IoT/IoT_Capstone-IvanBoyd/U2Main/src/U2Main.ino"
/*
 * Project:     U2Main.ini - Initial integration program foro YouTwo!
 * Description: Integrates three programs, 
 * 1. U2_Ble..., 
 * 2. U2_PIR, U2 PIR Motion Sensor with interrupt triggers the 100 NeoPixel Fairy Lights
 *            an attached Encoder is user setable to change the fairy light intensity
 * 3. U2_LIDAR
 * Author:      Ivan Boyd
 * Date:        12/2/21
 */

#include "Wire.h"
#include "LIDARLite_v4LED.h"
#include "neopixel.h"
#include "Encoder.h"
void setup();
void loop();
void lidarSub();
void npSetUp();
uint8_t distanceSingle(uint16_t *distance);
void runPRTchk();
void flSetUp();
void flSub();
void encLedSetUp();
void detectsMovement();
#line 16 "c:/Users/boyd/Documents/IoT/IoT_Capstone-IvanBoyd/U2Main/src/U2Main.ino"
#define PIXEL_TYPE SK6812RGBW       // fairy lights
#define PIXEL_TYPE WS2812B          // NeoPixels
const int motionSensor = D7;        // PIR Room sensor
const int led = 16;

bool  dBug    = true;
int   dBugDel = 300;

//                              ***  FAIRY LIGHT PIXEL HEADER  ***
const int FLPIXPIN     = D8;                 // Fairy Light Pix Pin 
const int FLPIXEL_NUM  = 100;                // Fairy Light Pix Number 
int   FLlow = 1, FLmed = 50, FLhigh = 145;   // NeoPix brightness 0-255, 145 is too bright for me 
int   FLi, FLj, timeSeconds = 10;
Adafruit_NeoPixel fairyNP(FLPIXEL_NUM, FLPIXPIN, PIXEL_TYPE);
bool fairyLightsOn = false;
unsigned long FLnow         = millis();
unsigned long FLlastTrigger = 0;
bool          FLstartTimer  = false;

//                              ***  NEOPIXEL HEADER  ***
const int NEOPIXPIN     = A3;
const int NEOPIXEL_NUM  = 12;
int   i, j;         // NeoPix brightness 0-155, 145 bout high enuf
int   minNP = 0,          maxNP = 150,                         // lower/upper NP intensity
      low = 3,            med = 50,         high = 127;        // for NP light intensity - var to hold mapped value from dist
int someColors[] = {0xFF0000, 0x0000FF, 0x8000080, 0xFFFF00, 0xFF0FF, 0x808080, 0xFFA500, 0xA52A2A, 0x008000, 0x808000};
                  //  RED,    BLUE,     PURPLE,     YELLOW,   MAGENTA,   GRAY,    ORANGE,  BROWN,    GREEN,   OLIVE
Adafruit_NeoPixel NEO_Pix(NEOPIXEL_NUM, NEOPIXPIN, PIXEL_TYPE);

//                              ***  LIDAR HEADER  ***
LIDARLite_v4LED myLidarLite;
uint8_t   newDistance,      NPintensity   = 0,  newNPintensity;
uint16_t  distance,         NPdistance;
int       maxDist = 600,    minDist = 1;                         // max must be less than any object that will constanly trigger
bool      inRange;
#define FAST_I2C
#define MonitorPin    SDA        //   SDA, pin 0  Yellow wire   LIDAR-Lite I2C SDA 
#define TriggerPin    SCL        //   SCL, pin 1  Green wire    LIDAR-Lite I2C SCL 
                                 //   Optional - D5, pin 5   White wire    LIDAR-Lite GPIOA (connected but not used at this point)
                                 //   Optional - D6, pin 6   Blue wire     LIDAR-Lite GPIOB (connected but not used at this point) 
// Encoder Header 
int  pinA     =  A5,          pinB      = A4,
     position = 0,            last_Pos  = 0; 
int enc_P     = 0,            enc_Low   = 0,        enc_High  = 0,       //set enc to Pix Ring vars for Map function
    pix_Low   = 0,            pix_High  = 0,        new_Pix_P = 0;       // enc = encoder, pix = pixel ring
 
const int SWITCHPIN = A0,     GRNPIN    = A2,       REDPIN    = A1,      T    = 1000;
              //pinmodes  22, input, 21 & 20 output to pixels,  21  output to pixels   20 output to pixels
              // PIXPIN    = A3,     // Neopixels neeeds a lib but no pin mode
                
Encoder myEnc(pinB,pinA);

SYSTEM_MODE(SEMI_AUTOMATIC);     //Using BLE and not Wifi

//  *****************   B E G I N    S E T U P    **********
void setup()
{
  runPRTchk();                    // Start print to monitor
  Wire.begin();                   // Initialize  I2C (for communication to LidarLite)
  npSetUp();                      // runs NEOPIXEL SETUP function
  flSetUp();                      // Set Up for Fairy Lights
  encLedSetUp();                  // Set Up for Encoder LED's
  FLlastTrigger = millis();
  pinMode(motionSensor, INPUT);   // PIR Motion Sensor mode INPUT_PULLUP
  // Set motionSensor pin as interrupt, assign interrupt function and set RISING mode
  attachInterrupt(motionSensor, detectsMovement, RISING);
  minDist = 2; maxDist = 650; minNP = 2; maxNP = 155;     // maxDist in cm, 1 meter = 100 cm, 39'ish inches & 1.1 yards 

}
//  *****************   E N D    S E T U P    **********


//  *****************  B E G I N     M A I N    V O I D    L O O P    **********
void loop()   {
  lidarSub();
  flSub();        // turns on FL's when PIR sensor is activated
}
//  *****************  E N D     M A I N    V O I D    L O O P    **********


//  *****************    B E G I N     F U N C T I O N S    **********

 /* Description: Utilizes the Garmin Lidar Distance Sensor to change NeoPixel light intensity. 
 *              - Lidar (returns distance values in cm) 1 m = 100 cm, 1 m = 1000 mm
 *              - 1 cm = 0.39370079 inch, 1 meter = 39.3700787 inches & 1.0936133 yards, 1 mile = 1609.344 meters
 *              - 5 meters = 197, 4 meters = 157 inches                     */
void lidarSub()    {
  newDistance = distanceSingle(&distance);
  NPdistance  = distance;
  inRange = ((NPdistance > 0) && (NPdistance < maxDist));
  if (!((NPdistance > 0) && (NPdistance < maxDist)))   {                  // object is not in range
      inRange = false;
  }
  if (!inRange)  {
    if(dBug) {Serial.printf("*NOT* inRange: %i Distance = %icm    NPDistance = %icm\n", inRange, distance, NPdistance); delay(dBugDel);} 
    NEO_Pix.clear();   // turn off NeoPixel
    NEO_Pix.show();   
  }
  else if (inRange) {
    if(dBug) {Serial.printf("** inRange **: %i, Distance = %icm    NPDistance = %icm\n", inRange, distance, NPdistance); delay(dBugDel);} 
    if (NPdistance > maxDist) {
        NPdistance = maxDist;
    }
    // NPintensity = map(distance, minDist, maxDist, minNP, maxNP);      // map(value, fromLow, fromHigh, toLow, toHigh)
    NPintensity = map(NPdistance, maxDist, minDist, minNP, maxNP/2);     // map(value, fromLow, fromHigh, toLow, toHigh)
    // NPintensity = map(newDistance, 2, 1000, 1, 150);
    // NPintensity = map(distance, maxDist, 0, 0, 127);                  // values used in sensor2Light program (maxDist=98) for reversing the LED when getting closer
    Serial.printf("Mapping Dist to NPs, Dist: %i NPdist: %i NPintensity: %i minDist: %i maxDist %i minNP %i maxNP %i  \n", distance, NPdistance, NPintensity, minDist, maxDist, minNP, maxNP);
    NEO_Pix.setBrightness(NPintensity); 
    for (i = 0; i < NEOPIXEL_NUM; i++)  {
      NEO_Pix.setPixelColor(i, 0xFF0000);                                // set to red
    }
    NEO_Pix.show(); 
    if (newDistance) {
      Serial.println(distance);
    }
  if(dBug) {delay(500);}
  }
}
void npSetUp()  {
  NEO_Pix.begin();
  NEO_Pix.setBrightness(5); 
  for (i = 0; i < NEOPIXEL_NUM; i++)  {
    NEO_Pix.setPixelColor(i, someColors[random(0,9)]);
    NEO_Pix.show(); 
    delay(75);
  }
  delay(2000); 
  NEO_Pix.clear();                // turn off NeoPixel
  NEO_Pix.show(); 
}

// bool notInRange(bool _inRange)  {
//   return(_inRange = false);
// }


// function to read distance
uint8_t distanceSingle(uint16_t *distance)  {
    myLidarLite.takeRange();                      // 1. Trigger range measurement.
    myLidarLite.waitForBusy();                    // 2. Wait for busyFlag to indicate device is idle.
    *distance = myLidarLite.readDistance();       // 3. Read new distance data from device registers
    return 1;
}

// Start print to monitor
void runPRTchk() {               
  Serial.begin(9600);
  while(!Serial);
  Serial.printf("Printer initialized\n");
  delay(1000);
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

  // Set up the Encoder LEDs
  void encLedSetUp() {
    pinMode(led, OUTPUT);
    pinMode(GRNPIN, OUTPUT);
    pinMode(REDPIN, OUTPUT);
    digitalWrite(led, LOW);
    digitalWrite(GRNPIN, HIGH);
    digitalWrite(REDPIN, LOW);
  }
  
void detectsMovement() {
//  Serial.println("MOTION DETECTED!!!");
 digitalWrite(REDPIN, HIGH);
 // Turn on Fairy Lights
 fairyLightsOn    = true;
 FLstartTimer     = true;
 FLlastTrigger    = millis();
}
