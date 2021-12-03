/*
 * Project U2_Lidar2NeoPixel.ino      <-- YouTwo-V4LED_I2C_Lidar.ino
 * Description: Utilizes the Garmin Lidar Distance Sensor to change NeoPixel light intensity. 
 *              - Lidar (returns distance values in cm) 1 m = 100 cm, 1 m = 1000 mm
 *              - 1 cm = 0.39370079 inch, 1 meter = 39.3700787 inches & 1.0936133 yards, 1 mile = 1609.344 meters
 *              - 5 meters = 197, 4 meters = 157 inches
 * Author:      Ivan Boyd
 * Date:        11/29/21
 */

#include "Wire.h"
#include "LIDARLite_v4LED.h"
#include "neopixel.h"
#define PIXEL_TYPE WS2812B
bool  dBug    = true;
int   dBugDel = 300;
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

SYSTEM_MODE(SEMI_AUTOMATIC);     //Using BLE and not Wifi

//  *****************   B E G I N    S E T U P    **********
void setup()
{
  Serial.begin(9600);
  waitFor(Serial.isConnected, 15000); 
  Wire.begin();                   // Initialize  I2C (for communication to LidarLite)
  npSetUp();//                              ***  runs NEOPIXEL SETUP function ***
  minDist = 2; maxDist = 650; minNP = 2; maxNP = 155;     // maxDist in cm, 1 meter = 100 cm, 39'ish inches & 1.1 yards 

}
//  *****************   E N D    S E T U P    **********


//  *****************  B E G I N     M A I N    V O I D    L O O P    **********
void loop()   {
  lidarSub();
}
//  *****************  E N D     M A I N    V O I D    L O O P    **********


//  *****************    B E G I N     F U N C T I O N S    **********

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
