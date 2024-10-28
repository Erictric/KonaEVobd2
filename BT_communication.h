#ifndef BT_COMMUNICATION
#define BT_COMMUNICATION

#include "BluetoothSerial.h"
#include "ELMduino.h"
#include "TFT_eSPI.h"

BluetoothSerial SerialBT; //Object for Bluetooth

ELM327 myELM327;    //Object for OBD2 device

#define ELM_PORT SerialBT

void ConnectToOBD2(TFT_eSPI& tft){
  char strRetries[2];
  ELM_PORT.setPin("1234");  
  ELM_PORT.begin("ESP32", true);  
  
  tft.fillScreen(TFT_BLACK);
  tft.drawString("Connecting", tft.width() / 2, tft.height() / 2 - 16);
  tft.drawString("To", tft.width() / 2, tft.height() / 2);
  tft.drawString("OBDII", tft.width() / 2, tft.height() / 2 + 16);
  tft.drawString("Device", tft.width() / 2, tft.height() / 2 + 32);
  Serial.println("...Connecting to OBDII...");
  
  int retries = 0;
  while (!ELM_PORT.connect("Android-Vlink") && (retries++ < 1)) // Device name of iCar Vgate pro BT4.0 OBD adapter
  {
    dtostrf(retries,1,0,strRetries);
    Serial.println("Couldn't connect to OBD scanner - Phase 1");
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(2);
    tft.drawString("Couldn't", tft.width() / 2, tft.height() / 2 - 16);
    tft.drawString("connect to", tft.width() / 2, tft.height() / 2);
    tft.drawString("OBDII", tft.width() / 2, tft.height() / 2 + 16);
    tft.drawString("scanner", tft.width() / 2, tft.height() / 2 + 32);
    tft.drawString(" Phase 1", tft.width() / 2, tft.height() / 2 + 48); 
    delay(500);    
    tft.fillScreen(TFT_BLACK);
    tft.drawString("Connecting", tft.width() / 2, tft.height() / 2 - 16);
    tft.drawString("Retry:", tft.width() / 2, tft.height() / 2);
    tft.drawString(strRetries, tft.width() / 2, tft.height() / 2 + 16);        
  }

  if (!myELM327.begin(ELM_PORT)) // Stream, protocol (optional), payloadlen (optional)
  {
    Serial.println("Couldn't connect to OBD scanner - Phase 2");    
    tft.fillScreen(TFT_BLACK);    
    tft.drawString("Couldn't", tft.width() / 2, tft.height() / 2 - 16);
    tft.drawString("connect to", tft.width() / 2, tft.height() / 2);
    tft.drawString("OBDII", tft.width() / 2, tft.height() / 2 + 16);
    tft.drawString("scanner", tft.width() / 2, tft.height() / 2 + 32);
    tft.drawString(" Phase 2", tft.width() / 2, tft.height() / 2 + 48);
    delay(500);       
    
    esp_deep_sleep_start();
  }

  else{
  Serial.println("Connected to OBDII");
      
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.drawString("Connected",  tft.width() / 2, tft.height() / 2 - 16);
  tft.drawString("to OBDII", tft.width() / 2, tft.height() / 2);

  delay(500);
  tft.fillScreen(TFT_BLACK);
  }
}
#endif
