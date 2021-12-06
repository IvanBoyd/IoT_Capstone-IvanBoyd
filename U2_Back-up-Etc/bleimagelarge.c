/*
 * Project BLEImageLarge
 * Description: Display images larger than 8x8 from Bluefruit Connect app on Neopixel matrix
 * Author: Brian Rashap
 * Date: 14-NOV-2021
 */

#include "Particle.h"
#include "neopixel.h"
#include "colors.h"

// Setup BLE UART

const size_t UART_TX_BUF_SIZE = 12;
uint8_t txBuf[UART_TX_BUF_SIZE];
uint8_t imgBuf[3];
uint16_t i,j;

// These UUIDs were defined by Nordic Semiconductor and are now the defacto standard for
// UART-like services over BLE. Many apps support the UUIDs now, like the Adafruit Bluefruit app.
const BleUuid serviceUuid("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
const BleUuid rxUuid("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
const BleUuid txUuid("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");

BleCharacteristic txCharacteristic("tx", BleCharacteristicProperty::NOTIFY, txUuid, serviceUuid);
BleCharacteristic rxCharacteristic("rx", BleCharacteristicProperty::WRITE_WO_RSP, rxUuid, serviceUuid, onDataReceived, NULL);
BleAdvertisingData data;

// Setup NeoPixels
const bool serpentine = true;   // set to true if neopixel matrix is wired in serpentine pattern
const int PIXEL_COUNT=256;      // 16 x 16 neopixel matrix
const int PIXEL_PIN=D2;
#define PIXEL_TYPE WS2812B

Adafruit_NeoPixel matrix(PIXEL_COUNT, PIXEL_PIN, PIXEL_TYPE);

int payloadNum;
int myImage[PIXEL_COUNT];
int myMatrix[PIXEL_COUNT];
int imgData[3*PIXEL_COUNT+7];

SYSTEM_MODE(SEMI_AUTOMATIC); //Using BLE and not Wifi

void setup() { 
    Serial.begin();
    waitFor(Serial.isConnected, 5000);  
    delay(2000);
 
    // Initialize neopixel matrix
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
    }
    matrix.show();
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
        color = imgBuf[0]<<16 | imgBuf[1]<<8 | imgBuf[2];
        //Serial.printf("byte %i, color %06X\n",i,color);
        if(serpentine) {
          matrix.setPixelColor(myMatrix[i],color);
        }
        else {
          matrix.setPixelColor(i,color);
        }
        
      }
      Serial.printf("\n\n Payload Length = %i\n",payloadLen);
      matrix.show();
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