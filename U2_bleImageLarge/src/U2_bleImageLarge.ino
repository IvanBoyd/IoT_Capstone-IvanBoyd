/*
 * Project BLEImageLarge
 * Description: Display images larger than 8x8 from Bluefruit Connect app on Neopixel matrix
 * Author: Brian Rashap
 *          11/29/21 modified for dotstar pixels, IB
 * Date: 14-NOV-2021
 */

#include "Particle.h"
// #include "neopixel.h"
#include "colors.h"
#define NUMPIXELS 256 // Number of LEDs in strip
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_DotStarMatrix.h"
// #include "TomThumb.h"
#include "dotstar.h"
#define DATAPIN   12                  // use const int PIXEL_PIN=D2 for NeoPix
#define CLOCKPIN  13                  
#define SHIFTDELAY 100
#define BRIGHTNESS 20

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

// no void loop(), all action happens when data received from BLE
void loop() {}

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