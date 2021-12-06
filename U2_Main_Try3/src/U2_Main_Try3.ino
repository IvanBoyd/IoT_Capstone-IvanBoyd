/*
 * Project:     U2Main.ini - Initial integration program for YouTwo!
 * Description: * TRY THREE * Integrates three programs, 
 * 1. U2_Ble..., 
 * 2. U2_PIR, U2 PIR Motion Sensor with interrupt triggers the 100 NeoPixel Fairy Lights
 *            an attached Encoder is user setable to change the fairy light intensity
 * 3. U2_LIDAR
 * Author:      Ivan Boyd
 * Date:        12/2/21
 
 * Inclues: BLEImageLarge.ino by Brian Rashap, 11/29/21 modified for dotstar pixels, IB
 *         
 * Date: DEC 5, 2021
 *  5:43 - compiles & flashes, xfers images w Brian's BT interface
 *    * adding U2_PIR-MotionSensor w fairy Lights (uses NeoPixel for FL's w SK6812RGBW)
 *      - 6:17 totally hosed program w tons of cryptic error messages & exit codes
 *        - appears to be a conflict with neopixel library, deleting it doesn't fix. Have
 *          to start over.
 */

#include "Wire.h"
#include "LIDARLite_v4LED.h"
#define PIXEL_TYPE WS2812B

#include "Particle.h"
#include "Encoder.h"
int timeSeconds = 10;
#include "neopixel.h"
#include "colors.h"
#define NUMPIXELS 256 // Number of LEDs in strip
#define FAIRY_PIXEL_TYPE SK6812RGBW
#define PIXEL_TYPE WS2812B
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_DotStarMatrix.h"
// #include "TomThumb.h"
#include "dotstar.h"
#define DATAPIN   12                  // use const int PIXEL_PIN=D2 for NeoPix
#define CLOCKPIN  13                  
#define SHIFTDELAY 100
#define BRIGHTNESS 20

bool  dBug    = true;
int   dBugDel = 300;
//                              ***  FAIRY LIGHT PIXEL HEADER  ***
const int FLPIXPIN     = D8;                 // Fairy Light Pix Pin 
const int FLPIXEL_NUM  = 100;                // Fairy Light Pix Number 
int   FLlow = 1, FLmed = 50, FLhigh = 145;   // FL (Fairy Light), NeoPix brightness 0-255, 145 is too bright for me 
int   FLi, FLj;
// int   timeSeconds = 10;            // FL (Fairy Light) 
Adafruit_NeoPixel fairyNP(FLPIXEL_NUM, FLPIXPIN, FAIRY_PIXEL_TYPE);
bool fairyLightsOn = false;
unsigned long FLnow         = millis();
unsigned long FLlastTrigger = 0;
bool          FLstartTimer  = false;

// //                              ***  NEOPIXEL HEADER  ***
const int NEOPIXPIN     = A3;
const int NEOPIXEL_NUM  = 12;
int   main_i, main_j;                                          // NeoPix brightness 0-155, 145 bout high enuf
int   minNP = 0,          maxNP = 150;                         // lower/upper NP intensity
int   low = 3,            med = 50,         high = 127;        // for NP light intensity - var to hold mapped value from dist
int someColors[] = {0xFF0000, 0x0000FF, 0x8000080, 0xFFFF00, 0xFF0FF, 0x808080, 0xFFA500, 0xA52A2A, 0x008000, 0x808000};
//                   //  RED,    BLUE,     PURPLE,     YELLOW,   MAGENTA,   GRAY,    ORANGE,  BROWN,    GREEN,   OLIVE
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
// const int led = 16; 
const int SWITCHPIN = A0,     GRNPIN    = A2,       REDPIN    = A1,      T    = 1000;
              //pinmodes  22, input, 21 & 20 output to pixels,  21  output to pixels   20 output to pixels
              // PIXPIN    = A3,     // Neopixels neeeds a lib but no pin mode
const int motionSensor = D7;               
Encoder myEnc(pinB,pinA);

const int FASTFILL  = 2,  SLOWFILL  = 10,   MEDFILL = 5;
// This is with DotStar, use #define PIXEL_TYPE WS2812B for NeoPix

//  *** begin dotstar header ***
// param 1 = matrix width, param 2 = matrix height, param 3 = datapin, param 4 = clockpin
//  flags: 
//   DS_MATRIX_TOP, DS_MATRIX_BOTTOM, DS_MATRIX_LEFT, DS_MATRIX_RIGHT:
//     Position of the FIRST LED in the matrix; pick two, e.g.
//     DS_MATRIX_TOP + DS_MATRIX_LEFT for the top-left corner.
//   DS_MATRIX_ROWS, DS_MATRIX_COLUMNS: LEDs are arranged in horizontal
//     rows or in vertical columns, respectively; pick one or the other.
//   DS_MATRIX_PROGRESSIVE, DS_MATRIX_ZIGZAG: all rows/columns proceed
//     in the same order, or alternate lines reverse direction; pick one.
//   See example below for these values in action.

Adafruit_DotStarMatrix matrix = Adafruit_DotStarMatrix(
                                  16, 16, DATAPIN, CLOCKPIN,                // demo: 12, 6, DATAPIN, CLOCKPIN,
                                  DS_MATRIX_TOP     + DS_MATRIX_LEFT +
                                  DS_MATRIX_ROWS + DS_MATRIX_ZIGZAG,
                                  DOTSTAR_BGR);

const uint16_t primaryColors[] = {
  matrix.Color(255, 0, 0), matrix.Color(0, 255, 0), matrix.Color(0, 0, 255)
};

const uint16_t adaColors[] = {
  matrix.Color(255, 0, 0),   //A red
  matrix.Color(255, 125, 0), //D orange
  matrix.Color(200, 255, 0), //A yellowish
  matrix.Color(0, 255, 0),   //F green
  matrix.Color(0, 255, 225), //R blue
  matrix.Color(150, 0, 255), //U purple
  matrix.Color(255, 0, 220), //I pink
  matrix.Color(255, 65, 0),  //T reddish
  matrix.Color(255, 220, 0)  //! orange/yellow
};
//  end dotstar header

// Setup BLE UART
const size_t UART_TX_BUF_SIZE = 12;
uint8_t txBuf[UART_TX_BUF_SIZE];        // array of 12 bytes (0-255, char's)
uint8_t imgBuf[3];
uint16_t i,j;

// UUIDs by Nordic Semiconductor, defacto standard for UART-like services over BLE w UUIDs like Adafruit Bluefruit app.
const BleUuid serviceUuid("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
const BleUuid rxUuid("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
const BleUuid txUuid("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");

BleCharacteristic txCharacteristic("tx", BleCharacteristicProperty::NOTIFY, txUuid, serviceUuid);
BleCharacteristic rxCharacteristic("rx", BleCharacteristicProperty::WRITE_WO_RSP, rxUuid, serviceUuid, onDataReceived, NULL);
BleAdvertisingData data;

// Setup NeoPixels
const bool serpentine = true;   // set to true if neopixel matrix is wired in serpentine pattern
const int PIXEL_COUNT=256;      // 16 x 16 neopixel matrix


// Adafruit_NeoPixel matrix(PIXEL_COUNT, PIXEL_PIN, PIXEL_TYPE);
//  see dotstar maxtrix creation above with same name, "matrix"

int payloadNum;
int myImage[PIXEL_COUNT];
int myMatrix[PIXEL_COUNT];
int imgData[3*PIXEL_COUNT+7];     // 3 bytes per pix for RGB color

SYSTEM_MODE(SEMI_AUTOMATIC); //Using BLE and not Wifi

void setup() { 
  Serial.begin();
  waitFor(Serial.isConnected, 5000);  
  delay(2000);
  flSetUp();                      // Set Up for Fairy Lights
  encLedSetUp();                  // Set Up for Encoder LED's
  FLlastTrigger = millis();
  pinMode(motionSensor, INPUT);   // PIR Motion Sensor mode INPUT_PULLUP
  // Set motionSensor pin as interrupt, assign interrupt function and set RISING mode
  attachInterrupt(motionSensor, detectsMovement, RISING);
    
  // Garmin Lidar Set-Up
  Wire.begin();                   // Initialize  I2C (for communication to LidarLite)
  npSetUp();//                              ***  runs NEOPIXEL SETUP function ***
  minDist = 2; maxDist = 650; minNP = 2; maxNP = 155;     // maxDist in cm, 1 meter = 100 cm, 39'ish inches & 1.1 yards 

  // Initialize dotstar matrix (!neopixel) 
  matrix.begin(); 
  matrix.setBrightness(15);

  Serial.printf("Starting up BLE Connection \n");
  BLE.on();
  BLE.addCharacteristic(txCharacteristic);
  BLE.addCharacteristic(rxCharacteristic);
  data.appendServiceUUID(serviceUuid);
  BLE.advertise(&data);

  Serial.printf("Argon BLE Address: %s\n",BLE.address().toString().c_str());

  fillMyMatrix(myMatrix);

  // Send rainbow test pattern to matrix to validate serpentine set correctly
  for(j=0;j<PIXEL_COUNT;j++) {
    if(serpentine) {
      matrix.setPixelColor(myMatrix[j],rainbow[j%7]);
      }
      else {
        matrix.setPixelColor(j,rainbow[j%7]);
      }
    matrix.show();
    delay(FASTFILL);
  }
  matrix.show();
  // delay(2000);                    // Pause a half second then clear
  // clearMatrix();
  clearMatrixByPix();
  matrix.show();
  // delay(10);
}

//  *****************  B E G I N     M A I N    V O I D    L O O P    **********
// no void loop(), all action happens when data received from BLE
void loop() {
  flSub();        // turns on FL's when PIR sensor is activated
  lidarSub();
}
//  *****************  E N D     M A I N    V O I D    L O O P    **********


//  *****************    B E G I N     F U N C T I O N S    **********

// turns on NP's when objects are in range of the LIDAR
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

// function to read distance w Garmin LIDAR sensor
uint8_t distanceSingle(uint16_t *distance)  {
    myLidarLite.takeRange();                      // 1. Trigger range measurement.
    myLidarLite.waitForBusy();                    // 2. Wait for busyFlag to indicate device is idle.
    *distance = myLidarLite.readDistance();       // 3. Read new distance data from device registers
    return 1;
}

// set up and display NeoPixels (currently in an NP Ring)
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
    // pinMode(led, OUTPUT);
    pinMode(GRNPIN, OUTPUT);
    pinMode(REDPIN, OUTPUT);
    // digitalWrite(led, LOW);
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

// receive data from BLE and display image on neopixel matrix
void onDataReceived(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context) {
    //int pixelcount,pixelcolor;
    int i,j;
    int color;
    int payloadLen;

    Serial.printf("Received data from: %02X:%02X:%02X:%02X:%02X:%02X \n", peer.address()[0], peer.address()[1],peer.address()[2], peer.address()[3], peer.address()[4], peer.address()[5]);
    for (i = 0; i < len; i++) {
        Serial.printf("%02X",data[i]);
    }
    Serial.printf("\n\n Length = %i\n",len);


    // BLE messages sent in 244 character chunks, below code stitches multiple 244 message payloads together
    if(len == 244) {
      for(i=0;i<244;i++){
        imgData[payloadNum*244+i]=data[i];
        Serial.printf("%i:0x%02X --- %i:0x%02X\n",i,data[i],payloadNum*244+i,imgData[payloadNum*244+i]);
      }
      payloadNum++;
    }
    else {
      for(i=0;i<len-1;i++){
        imgData[payloadNum*244+i]=data[i];
        Serial.printf("%i:0x%02X --- %i:0x%02X\n",i,data[i],payloadNum*244+i,imgData[payloadNum*244+i]);
      }
      payloadLen = payloadNum*244+(len-1);

      // once total message received determine color of each pixel and display to matrix
      matrix.clear();
      for (i=0;i<((payloadLen-7)/3);i++) {
        j = 7+(i*3);
        imgBuf[0] = imgData[j];
        imgBuf[1] = imgData[j+1];
        imgBuf[2] = imgData[j+2];
        Serial.printf("%i: 0x%02X%02X%02X: \n",i,imgBuf[0],imgBuf[1],imgBuf[2]);
        
        color = imgBuf[0]<<16 | imgBuf[1]<<8 | imgBuf[2];      // R-G-B  Multiplying Blue by .8 to reduce light intensity --> stronger blue, I hope
        // color = imgBuf[0]<<16 | imgBuf[1]<<8 | imgBuf[2];
        //Serial.printf("byte %i, color %06X\n",i,color);
        if(serpentine) {
          matrix.setPixelColor(myMatrix[i],color);
        }
        else {
          matrix.setPixelColor(i,color);
        }
        matrix.show();
        delay(10);

      }
      Serial.printf("\n\n Payload Length = %i\n",payloadLen);
      // matrix.show();
      payloadNum = 0;
    }
}

// 
void fillMyMatrix(int *myMat) {
  byte i;
  byte j;

  for(i=0;i<16;i++) {
    for(j=0;j<16;j++) {
      if((i%2) == 1) {
        myMat[i*16+j] = i*16+j;
      }
      else {
        myMat[i*16+j] = i*16+(15-j);
      }
    }
  }
  return;
}

//   clears 16x16 Dot Pixel Matrix (serpentine order) 
void clearMatrix()  {
  for(j=0;j<PIXEL_COUNT;j++) {
    matrix.setPixelColor(myMatrix[j],0x000000);
  }
}

//   clears 16x16 Dot Pixel Matrix (serpentine order) 
void clearMatrixByPix()  {
  for(j=0;j<PIXEL_COUNT;j++) {
    matrix.setPixelColor(myMatrix[j],0x000000);
    matrix.show();
    delay(FASTFILL);
  }
}