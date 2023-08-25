
/*  KonaEvObd for Hyundai Kona EV + OBD Vgate iCar Pro BT4.0 
 *  Version: v4.01
  *   
  * SafeString by Matthew Ford: https://www.forward.com.au/pfod/ArduinoProgramming/SafeString/index.html
  * Elmduino by PowerBroker2: https://github.com/PowerBroker2/ELMduino
  * https://randomnerdtutorials.com/esp32-esp8266-publish-sensor-readings-to-google-sheets/
  * 
  notes:
  ELMduino library used is version 2.41, problem not resolved with newer version
  ELMduino.h needs to be modified as follow: bool begin(Stream& stream, char protocol='6', uint16_t payloadLen = 240);
  For TFT_eSPI library, comment: //#include <User_Setup.h> and uncomment: #include <User_Setups/Setup25_TTGO_T_Display.h> 
*/
#include "Arduino.h"
#include "SafeString.h"
#include "ELMduino.h"
#include "EEPROM.h"
#include "Button.h"
#include "TFT_eSPI.h"
#include "BT_communication.h"
#include "Wifi_connection.h"
#include "FreeRTOS.h"
#include "WiFi.h"
#include "Free_Fonts.h"

#define DEBUG_PORT Serial

TaskHandle_t Task1;

TFT_eSPI tft = TFT_eSPI(135, 240); // Invoke custom display library

#ifndef TFT_DISPOFF
#define TFT_DISPOFF 0x28
#endif

#ifndef TFT_SLPIN
#define TFT_SLPIN   0x10
#endif

#define TFT_BL  4  // Display backlight control pin
#define ADC_EN  14  //ADC_EN is the ADC detection enable port
#define ADC_PIN 34

//TFT y positions for texts and numbers
#define textLvl1 5            // y coordinates for text
#define textLvl2 68
#define textLvl3 129
#define textLvl4 189
#define drawLvl1 35            // and numbers
#define drawLvl2 96
#define drawLvl3 156
#define drawLvl4 217            // TTGO 135x240 TFT display

#define BUTTON_PIN  0
#define BUTTON_2_PIN  35

Button bouton(BUTTON_PIN);
Button bouton2(BUTTON_2_PIN);

#define pagenumbers 7               // number of pages to display
#define N_km 10        //variable for the calculating kWh/100km over a N_km

int ledBacklight = 100; // Initial TFT backlight intensity on a scale of 0 to 255. Initial value is 80.

/*////// Setting PWM properties, do not change this! /////////*/
const int pwmFreq = 5000;
const int pwmResolution = 8;
const int pwmLedChannelTFT = 0;

const int VESSoff = 12;
boolean SelectOn = true;
unsigned long StartMillis;
unsigned long ToggleDelay = 1000;
boolean ResetOn = true;
int screenNbr = 0;

uint8_t record_code = 0;
float mem_energy = 0;
float mem_PrevSoC = 0;
float mem_SoC = 0;
float mem_Power = 0;
float mem_LastSoC = 0;
uint16_t nbr_powerOn = 0;
bool data_ready = false;
bool code_sent = false;
bool sd_condition1 = false;
bool sd_condition2 = false;
bool SoC_saved = false;
bool code_received = false;
bool shutdown_esp = false;
bool wifiReconn = false;

float BattMinT;
float BattMaxT;
float AuxBattV;
float AuxBattC;
float AuxBattSoC = 80;      
float Batt12V;
float BATTv;
float BATTc;
float MAXcellv;
float MINcellv;
int MAXcellvNb;
int MINcellvNb;
float CellVdiff;
float CEC;
float CED;
float CDC;
float CCC;
float BmsSoC;
float Max_Pwr;
float Max_Reg;
float SoC;
float SoCratio;
float Calc_kWh_corr;
float SOH;
float Deter_Min;
int MinDetNb;
int MaxDetNb;  
float Heater;
float COOLtemp;
float OUTDOORtemp;
float INDOORtemp;
char SpdSelect;
uint32_t Odometer;
float Speed;
byte TransSelByte;
byte Park;
byte Reverse;
byte Neutral;
byte Drive;
char selector[1];
byte StatusWord;
byte BMS_ign;
float OPtimemins;
float OPtimehours;
float TireFL_P;
float TireFR_P;
float TireRL_P;
float TireRR_P;
float TireFL_T;
float TireFR_T;
float TireRL_T;
float TireRR_T;
float Power;
float CurrInitOdo = 0;
float CurrInitCEC = 0;
float CurrInitCED = 0;
float CurrInitSoC= 0;
float CurrTripOdo;
float CurrNet_kWh;
float CurrUsedSoC;
float CurrTripDisc;
float CurrTripReg;
float CurrInitAccEnergy;
float CurrAccEnergy;
float Prev_kWh = 0;
float Net_kWh = 0;
float UsedSoC = 0;
float Net_Ah = 0;
float DischAh = 0;
float RegenAh = 0;
float TripOdo = 0;
int InitOdo = 0;
float PrevOPtimemins = 0;
float TripOPtime = 0;
float CurrTimeInit = 0;
float CurrOPtime = 0;
float InitSoC = 0;
float InitCEC = 0;
float InitCED = 0;
float InitCCC = 0;
float InitCDC = 0;
float PrevSoC = 0;
float PrevBmsSoC = 0;
float Regen = 0;
float Discharg = 0;
float LastSoC = 0;
float integrate_timer = 0.0;
float start_kwh;
float acc_energy = 0.0;
float acc_regen;
float acc_Ah = 0.0;
float last_energy = 0.0;
float last_time = 0.0;
float last_odo = 0.0;
int energy_array_index = 0;
int array_size = N_km + 1;
float energy_array[11];
float span_energy = 0.0;
float speed_interval = 0.0;
float init_speed_timer = 0.0;
float int_speed = 0.0;
float distance = 0.0;
float prev_dist = 0;
float interval_dist = 0;
float Trip_dist = 0;
float prev_odo = 0;
float prev_power = 0.0;
int pwr_changed = 0;
int loop_count = 0;
float full_kwh;
float EstFull_kWh;
float EstFull_Ah;
float kWh_corr;
float left_kwh;
float used_kwh;
float degrad_ratio;
float old_PIDkWh_100km = 14;
float old_kWh_100km = 14;
float old_lost = 1;
float EstLeft_kWh;
float MeanSpeed;
float Time_100km;
float TripkWh_100km;
float kWh_100km;
float span_kWh_100km;
float PIDkWh_100;
float Est_range;
float Est_range2;
float Est_range3;
float acc_kWh_25;
float acc_kWh_10;
float acc_kWh_0;
float acc_kWh_m10;
float acc_kWh_m20;
float acc_kWh_m20p;  
float acc_time_25;
float acc_time_10;
float acc_time_0;
float acc_time_m10;
float acc_time_m20;
float acc_time_m20p;
float acc_dist_25;
float acc_dist_10;
float acc_dist_0;
float acc_dist_m10;
float acc_dist_m20;
float acc_dist_m20p;
bool DriveOn = false;
bool StayOn = false;
bool SetupOn = false;
bool StartWifi = true;
bool initscan = false;
bool InitRst = false;
bool TrigRst = false;
bool kWh_update = false;
bool corr_update = false;
bool ESP_on = false;
bool DrawBackground = true;
char title1[12];
char title2[12];
char title3[12];
char title4[12];       
char value1[5];
char value2[5];
char value3[5];        
char value4[5];
char prev_value1[5];
char prev_value2[5];
char prev_value3[5];        
char prev_value4[5];        
bool negative_flag1;
bool negative_flag2;
bool negative_flag3;
bool negative_flag4;
float value1_float;
float value2_float;
float value3_float;
float value4_float;
int nbr_decimal1;
int nbr_decimal2;
int nbr_decimal3;
int nbr_decimal4;
bool Charge_page = false;
bool Power_page = false;

unsigned long ESPinitTimer = 0;
unsigned long ESPTimer = 0;
unsigned long ESPTimerInterval = 1200; // time in seconds to turn off ESP when it power-up during 12V battery charge cycle. 
unsigned long shutdown_timer = 0;

/*////// Variables for Google Sheet data transfer ////////////*/
bool send_enabled = false;
bool send_data = false;
bool data_sent = false;
int nbParam = 76;    //number of parameters to send to Google Sheet
unsigned long sendInterval = 5000;
unsigned long currentTimer = 0;
unsigned long previousTimer = 0;
bool sendIntervalOn = false;

const char* resource = "/trigger/SendData/with/key/bWQadqBNOh1P3PINCC1_Vr"; // Copy key from IFTTT applet

// Maker Webhooks IFTTT
const char* server = "maker.ifttt.com";

static bool flag = false;

/*////// Variables for OBD data timing ////////////*/
const long obd_update = 100; // interval to update OBD data (milliseconds)
unsigned long currentMillis;  // timing variable to sample OBD data
unsigned long previousMillis = 0; // timing variable to sample OBD data
uint8_t pid_counter = 0;

/*////// Variables for Buttons ////////////*/
const long Longinterval = 3000; // interval for long button press (milliseconds)
unsigned long InitMillis; // will store time when button is pressed
unsigned long InitMillis2; // will store time when button is pressed

/*///////Define a structure to store the PID query data frames ///////////*/
struct dataFrames_struct {
  char frames[9][20]; // 9 frames each of 20chars
};

typedef struct dataFrames_struct dataFrames; // create a simple name for this type of data
dataFrames results; // this struct will hold the results


/*////////////////////////////////////////////////////////////////////////*/
/*                         START OF SETUP                                 */
/*////////////////////////////////////////////////////////////////////////*/

void setup() {

  /*////////////////////////////////////////////////////////////////*/   
  /*              Open serial monitor communications                */
  /*////////////////////////////////////////////////////////////////*/

  Serial.begin(9600);  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
    
  Serial.println("Serial Monitor - STARTED");

  
  //pinMode(VESSoff, OUTPUT); // enable output pin that activate a relay to temporary disable the VESS

  /*//////////////Initialise buttons ////////////////*/

  bouton.begin(); // left button
  bouton2.begin(); // right button

  /*//////////////Initialise OLED display ////////////////*/

  pinMode(ADC_EN, OUTPUT);
  digitalWrite(ADC_EN, HIGH);

  Serial.print("Configuring PWM for TFT backlight... ");
  ledcSetup(pwmLedChannelTFT, pwmFreq, pwmResolution);
  ledcAttachPin(TFT_BL, pwmLedChannelTFT);
  Serial.println("DONE");

  Serial.print("Setting PWM for TFT backlight to default intensity... ");
  ledcWrite(pwmLedChannelTFT, ledBacklight);
  Serial.println("DONE");
  
  tft.init(); // display initialisation
  tft.begin();
  tft.setRotation(0);  // 0 & 2 Portrait. 1 & 3 landscape
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN);
  tft.setCursor(0, 0);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(2); 
  
  /*////// initialize EEPROM with predefined size ////////*/
  EEPROM.begin(148);

  /* uncomment if you need to display Safestring results on Serial Monitor */
  //SafeString::setOutput(Serial);

  esp_sleep_enable_ext0_wakeup(GPIO_NUM_35,0);  // initialize ESP wakeup on button 1 activation
   
  /*////// Get the stored values from last re-initialisation /////*/
  
  Net_kWh = EEPROM.readFloat(0);
  InitCED = EEPROM.readFloat(4);    
  InitCEC = EEPROM.readFloat(8);
  InitSoC = EEPROM.readFloat(12);
  UsedSoC = EEPROM.readFloat(16);
  InitOdo = EEPROM.readFloat(20);
  InitCDC = EEPROM.readFloat(24);
  InitCCC = EEPROM.readFloat(28);
  old_lost = EEPROM.readFloat(32);
  old_PIDkWh_100km = EEPROM.readFloat(36);
  nbr_powerOn = EEPROM.readFloat(40);
  PrevOPtimemins = EEPROM.readFloat(44);
  kWh_corr = EEPROM.readFloat(48);
  acc_energy = EEPROM.readFloat(52);
  LastSoC = EEPROM.readFloat(56);
  old_kWh_100km = EEPROM.readFloat(60);
  acc_Ah = EEPROM.readFloat(64);
  acc_kWh_25 = EEPROM.readFloat(68);
  acc_kWh_10 = EEPROM.readFloat(72);
  acc_kWh_0 = EEPROM.readFloat(76);
  acc_kWh_m10 = EEPROM.readFloat(80);
  acc_kWh_m20 = EEPROM.readFloat(84);
  acc_kWh_m20p = EEPROM.readFloat(88);   
  acc_time_25 = EEPROM.readFloat(92);
  acc_time_10 = EEPROM.readFloat(96);
  acc_time_0 = EEPROM.readFloat(100);
  acc_time_m10 = EEPROM.readFloat(104);
  acc_time_m20 = EEPROM.readFloat(108);
  acc_time_m20p = EEPROM.readFloat(112);
  acc_dist_25 = EEPROM.readFloat(116);
  acc_dist_10 = EEPROM.readFloat(120);
  acc_dist_0 = EEPROM.readFloat(124);
  acc_dist_m10 = EEPROM.readFloat(128);
  acc_dist_m20 = EEPROM.readFloat(132);
  acc_dist_m20p = EEPROM.readFloat(136);
  acc_regen = EEPROM.readFloat(140);

  //initial_eeprom(); //if a new eeprom memory is used it needs to be initialize to something first  
        
/*/////////////////////////////////////////////////////////////////*/
/*                    CONNECTION TO OBDII                          */
/*/////////////////////////////////////////////////////////////////*/
    
  ConnectToOBD2(tft);
             

/*/////////////////////////////////////////////////////////////////*/
/*                     CONNECTION TO WIFI                         */
/*/////////////////////////////////////////////////////////////////*/

  if (StartWifi){
    ConnectWifi(tft);
    if (WiFi.status() == WL_CONNECTED) {
      send_enabled = true;      
    }
  }

  initscan = true; // To write header name on Google Sheet on power up

  /*//////////////Initialise Task on core0 to send data on Google Sheet ////////////////*/
  
  xTaskCreatePinnedToCore(
    makeIFTTTRequest, /* Function to implement the task */
    "makeIFTTTRequest", /* Name of the task */
    10000,  /* Stack size in words */
    NULL,  /* Task input parameter */
    5,  /* Priority of the task */
    &Task1,  /* Task handle. */
    0); /* Core where the task should run */    
    delay(500);

  tft.fillScreen(TFT_BLACK);

  integrate_timer = millis()/1000;
  
  nbr_powerOn += 1;      
  EEPROM.writeFloat(40, nbr_powerOn);
  EEPROM.commit();
    
}   

/*////////////////////////////////////////////////////////////////////////*/
/*                         END OF SETUP                                   */
/*////////////////////////////////////////////////////////////////////////*/

//----------------------------------------------------------------------------------------
//              OBDII Payloads Processing Functions                                           
//----------------------------------------------------------------------------------------

void clearResultFrames(dataFrames& results) {
  for (int i = 0; i < 9; i++) {
    results.frames[i][0] = '\0';
  }
}

// format is <headerBytes> then <frameNumberByte>:<frameDataBytes> repeated
void processPayload(char *OBDdata, size_t datalen, dataFrames& results) {
  cSFPS(data, OBDdata, datalen); // wrap in a SafeString
  clearResultFrames(results);
  size_t idx = data.indexOf(':'); // skip over header and find first delimiter
  while (idx < data.length()) {
    int frameIdx = data[idx - 1] - '0'; // the char before :
    if ((frameIdx < 0) || (frameIdx > 8)) { // error in frame number skip this frame, print a message here
      
      //SafeString::Output.print("frameIdx:"); SafeString::Output.print(frameIdx); SafeString::Output.print(" outside range data: "); data.debug();
      idx = data.indexOf(':', idx + 1); // step over : and find next :
      continue;
    }
    cSFA(frame, results.frames[frameIdx]); // wrap a result frame in a SafeString to store this frame's data
    idx++; // step over :
    size_t nextIdx = data.indexOf(':', idx); // find next :
    if (nextIdx == data.length()) {
      data.substring(frame, idx);  // next : not found so take all the remaining chars as this field
    } else {
      data.substring(frame, idx, nextIdx - 1); // substring upto one byte before next :
    }
    //SafeString::Output.print("frameIdx:"); SafeString::Output.print(frameIdx); SafeString::Output.print(" "); frame.debug();
    idx = nextIdx; // step onto next frame
  }
}

//------------------------------------------------------------------------------------------
//                  End of Payloads Processing                                  
//------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------
//             Bytes extraction from dataFrame                                  
//------------------------------------------------------------------------------------------

int convertToInt(char* dataFrame, size_t startByte, size_t numberBytes) {
  int offset = (startByte -1) * 2;
  // define a local SafeString on the stack for this method
  cSFP(frame, dataFrame);
  cSF(hexSubString, frame.capacity()); // allow for taking entire frame as a substring
  frame.substring(hexSubString, offset, offset + (numberBytes * 2)); // endIdx in exclusive in SafeString V2
  hexSubString.debug(F(" hex number "));
  long num = 0;
  if (!hexSubString.hexToLong(num)) {
    hexSubString.debug(F(" invalid hex number "));
  }
  return num;
}

//------------------------------------------------------------------------------------------
//               End of Bytes extraction from dataFrame                                  
//------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------
//         Data retreived from OBD2 and extract values of it                               
//------------------------------------------------------------------------------------------

void read_data(){

  pid_counter++; 
    
  // read in rawData via ODBII
  //switch (pid_counter){
        
    //case 1:
        
        myELM327.sendCommand("AT SH 7E4");       // Set Header for BMS
        
        if (myELM327.queryPID("220101")) {      // Service and Message PID = hex 22 0101 => dec 34, 257
          char* payload = myELM327.payload;
          size_t payloadLen = myELM327.recBytes;
          
          processPayload(payload, payloadLen, results);
            
          int BattMinTraw = convertToInt(results.frames[2], 6, 1); //specify frame#, starting Byte(o in TorquePro) and # of bytes required
          if (BattMinTraw > 127){ //conversition for negative value
             BattMinT = -1 * (256 - BattMinTraw);
          }
          else{
            BattMinT = BattMinTraw;
          }
          int BattMaxTraw = convertToInt(results.frames[2], 5, 1); //specify frame#, starting Byte(o in TorquePro) and # of bytes required
          if (BattMaxTraw > 127){ //conversition for negative value
             BattMaxT = -1 * (256 - BattMaxTraw);
          }
          else{
            BattMaxT = BattMaxTraw;
          }
          AuxBattV = convertToInt(results.frames[4], 6, 1) * 0.1;
          BATTv = convertToInt(results.frames[2], 3, 2) * 0.1;
          int CurrentByte1= convertToInt(results.frames[2], 1, 1);
          int CurrentByte2= convertToInt(results.frames[2], 2, 1);
          if (CurrentByte1 > 127){  // the most significant bit is the sign bit so need to calculate commplement value if true
            BATTc = -1 * (((255 - CurrentByte1) * 256) + (256 - CurrentByte2)) * 0.1;
          }
          else{
            BATTc = ((CurrentByte1 * 256) + CurrentByte2) * 0.1;
          }
          CEC = convertToInt(results.frames[6], 1, 4) * 0.1;
          CED = ((convertToInt(results.frames[6], 5, 3) << 8) + convertToInt(results.frames[7], 1, 1)) * 0.1;
          CCC = ((convertToInt(results.frames[4], 7, 1) << 24) + convertToInt(results.frames[5], 1, 3)) * 0.1;
          CDC = convertToInt(results.frames[5], 4, 4) * 0.1;
          BmsSoC = convertToInt(results.frames[1], 2, 1) * 0.5;
          StatusWord = convertToInt(results.frames[7], 6, 1); // Extract byte that contain BMS status bits
          BMS_ign = bitRead(StatusWord,2);
          MAXcellv = convertToInt(results.frames[3], 7, 1) * 0.02;
          MAXcellvNb = convertToInt(results.frames[4], 1, 1);
          MINcellv = convertToInt(results.frames[4], 2, 1) * 0.02;
          MINcellvNb = convertToInt(results.frames[4], 3, 1);          
          OPtimemins = convertToInt(results.frames[7], 2, 4) * 0.01666666667;
          OPtimehours = OPtimemins * 0.01666666667;
          }
          if (BMS_ign){
            ESP_on = true;            
          }          
          UpdateNetEnergy();
          pwr_changed += 1;
          //break;
          
  switch (pid_counter){
    case 1:
  
        myELM327.sendCommand("AT SH 7E4");       // Set Header for BMS
        
        if (myELM327.queryPID("220105")) {      // Service and Message PID = hex 22 0105 => dec 34, 261
          char* payload = myELM327.payload;
          size_t payloadLen = myELM327.recBytes;
                  
          processPayload(payload, payloadLen, results);
          Max_Pwr = convertToInt(results.frames[3], 2, 2) * 0.01;    
          Max_Reg = (((convertToInt(results.frames[2], 7, 1)) << 8) + convertToInt(results.frames[3], 1, 1)) * 0.01;
          SoC = convertToInt(results.frames[5], 1, 1) * 0.5;
          SOH = convertToInt(results.frames[4], 2, 2) * 0.1;
          MaxDetNb = convertToInt(results.frames[4], 4, 1);
          MinDetNb = convertToInt(results.frames[4], 7, 1);
          Deter_Min = convertToInt(results.frames[4], 5, 2) * 0.1;  
          int HeaterRaw = convertToInt(results.frames[3], 7, 1);
          if (HeaterRaw > 127){ //conversition for negative value
             Heater = -1 * (256 - HeaterRaw);
          }
          else{
            Heater = HeaterRaw;          
          }
          
        }
        break;

    case 2:
  
        myELM327.sendCommand("AT SH 7E4");       // Set Header for BMS
        
        if (myELM327.queryPID("220106")) {      // Service and Message PID = hex 22 0106 => dec 34, 262
          char* payload = myELM327.payload;
          size_t payloadLen = myELM327.recBytes;          

          processPayload(payload, payloadLen, results);
          int COOLtempRaw = convertToInt(results.frames[1], 2, 1) * 0.01; // Cooling water temperature          
          if (COOLtempRaw > 127){ //conversition for negative value
             COOLtemp = -1 * (256 - COOLtempRaw);
          }
          else{
            COOLtemp = COOLtempRaw;          
          }
        }
        break;
        
     case 3: 
        myELM327.sendCommand("AT SH 7E2");     // Set Header for Vehicle Control Unit
        
        if (myELM327.queryPID("2101")) {      // Service and Message PID
          char* payload = myELM327.payload;
          size_t payloadLen = myELM327.recBytes;         

          processPayload(payload, payloadLen, results);
          TransSelByte = convertToInt(results.frames[1], 2, 1); // Extract byte that contain transmission selection bits
          //Serial.print("SelByte: "); Serial.println(TransSelByte, 1);
          Park = bitRead(TransSelByte,0);
          Reverse = bitRead(TransSelByte,1);
          Neutral = bitRead(TransSelByte,2);
          Drive = bitRead(TransSelByte,3);
          
          if (Park) selector[0] = 'P';
          if (Reverse) selector[0] = 'R';
          if (Neutral) selector[0] = 'N';
          if (Drive) selector[0] = 'D'; 
          SpdSelect =  selector[0];
          }
        break;

     case 4: 
        myELM327.sendCommand("AT SH 7E2");     // Set Header for Vehicle Control Unit
        if (myELM327.queryPID("2102")) {      // Service and Message PID
          char* payload = myELM327.payload;
          size_t payloadLen = myELM327.recBytes;          

          processPayload(payload, payloadLen, results);
          //AuxBattV = convertToInt(results.frames[3], 2, 2)* 0.001; //doesn't work...
          int AuxCurrByte1= convertToInt(results.frames[3], 4, 1);
          int AuxCurrByte2= convertToInt(results.frames[3], 5, 1);
          if (AuxCurrByte1 > 127){  // the most significant bit is the sign bit so need to calculate commplement value if true
            AuxBattC = -1 * (((255 - AuxCurrByte1) * 256) + (256 - AuxCurrByte2)) * 0.01;
          }
          else{
            AuxBattC = ((AuxCurrByte1 * 256) + AuxCurrByte2) * 0.01;
          }          
          AuxBattSoC = convertToInt(results.frames[3], 6, 1);
          }
        break;
        
     case 5: 
        myELM327.sendCommand("AT SH 7C6");       // Set Header for CLU Cluster Module
        if (myELM327.queryPID("22B002")) {      // Service and Message PID
          char* payload = myELM327.payload;
          size_t payloadLen = myELM327.recBytes;          
          
          processPayload(payload, payloadLen, results);
          Odometer = convertToInt(results.frames[1], 4, 3);
          }
        break;

     case 6:  
       myELM327.sendCommand("AT SH 7B3");       //Set Header Aircon 
        if (myELM327.queryPID("220100")) {      // Service and Message PID
          char* payload = myELM327.payload;
          size_t payloadLen = myELM327.recBytes;

          processPayload(payload, payloadLen, results);          
          INDOORtemp = (((convertToInt(results.frames[1], 3, 1)) * 0.5) - 40);
          OUTDOORtemp = (((convertToInt(results.frames[1], 4, 1)) * 0.5) - 40);
          }        
        break;

     case 7:  
        myELM327.sendCommand("AT SH 7D4");       //Set Speed Header 
        if (myELM327.queryPID("220101")) {      // Service and Message PID
          char* payload = myELM327.payload;
          size_t payloadLen = myELM327.recBytes;

          processPayload(payload, payloadLen, results);
          Speed = (((convertToInt(results.frames[1], 7, 1)) << 8) + convertToInt(results.frames[2], 1, 1)) * 0.1;
          }
        Integrat_speed();  
        break;

      case 8:  
        myELM327.sendCommand("AT SH 7A0");       //Set BCM Header 
        if (myELM327.queryPID("22C00B")) {      // Service and Message PID
          char* payload = myELM327.payload;
          size_t payloadLen = myELM327.recBytes;

          processPayload(payload, payloadLen, results);
          TireFL_P = convertToInt(results.frames[1], 2, 1) * 0.2;
          TireFL_T = convertToInt(results.frames[1], 3, 1) - 50;
          TireFR_P = convertToInt(results.frames[1], 6, 1) * 0.2;
          TireFR_T = convertToInt(results.frames[1], 7, 1) - 50;
          TireRL_P = convertToInt(results.frames[2], 7, 1) * 0.2;
          TireRL_T = convertToInt(results.frames[3], 1, 1) - 50;
          TireRR_P = convertToInt(results.frames[2], 3, 1) * 0.2;
          TireRR_T = convertToInt(results.frames[2], 4, 1) - 50;
          }        
        pid_counter = 0;
        data_ready = true;
        break;
  }
  

  /////// Miscellaneous calculations /////////   
    
  Power = (BATTv * BATTc) * 0.001;
  Integrat_power();
  Integrat_current();
  integrate_timer = millis();

  if(!ResetOn){     // On power On, wait for current trip values to be re-initialized before executing the next lines of code
    TripOdo = Odometer - InitOdo;
    
    CurrTripOdo = Odometer - CurrInitOdo;    
    
    CurrOPtime = OPtimemins - CurrTimeInit;

    TripOPtime = CurrOPtime + PrevOPtimemins;
  
    UsedSoC = InitSoC - SoC;
  
    CurrUsedSoC = CurrInitSoC - SoC;
  
    EstFull_Ah = 100 * Net_Ah / UsedSoC;
  
    CellVdiff = MAXcellv - MINcellv;       
    
    if(PrevBmsSoC > BmsSoC){  // perform a BmsSoC vs SoC ratio calculation when BmsSoC changes
      PrevBmsSoC = BmsSoC;
      SocRatioCalc();
    }

    if(PrevSoC != SoC){  // perform "used_kWh" and "left_kWh" when SoC changes
      if(InitRst){  // On Button Trip reset, initial kWh calculation
        Serial.print("1st Reset");
        initscan = true;
        record_code = 2;        
        reset_trip();        
        //TrigRst = true;
        kWh_corr = 0;
        PrevSoC = SoC;
        Prev_kWh = Net_kWh;
        used_kwh = calc_kwh(SoC, InitSoC);
        left_kwh = calc_kwh(0, SoC);        
        initscan = true;
        InitRst = false;
        
      }
      if(!InitRst){ // kWh calculation when the Initial reset is not active
        // After a Trip Reset, perform a new reset if SoC changed without a Net_kWh increase (in case SoC was just about to change when the reset was performed)
        if(((acc_energy < 0.3) && (PrevSoC > SoC)) || ((SoC > 98.5) && ((PrevSoC - SoC) > 0.5))){
        //if(((Net_kWh < 0.3) && (PrevSoC > SoC)) || ((SoC > 98.5) && ((PrevSoC - SoC) > 0.5)) || (TrigRst && (PrevSoC > SoC))){ 
        //if((acc_energy < 0.3) && (PrevSoC > SoC)){ 
           initscan = true;
          mem_energy = acc_energy;
          mem_PrevSoC = PrevSoC;
          mem_SoC = SoC;
          if((acc_energy < 0.3) && (PrevSoC > SoC))
          {
            record_code = 3;            
          }
          else
          {
            record_code = 4;
          }
          Serial.print("2nd Reset");
          //TrigRst = false;
          reset_trip();
          kWh_corr = 0;
          used_kwh = calc_kwh(SoC, InitSoC);
          left_kwh = calc_kwh(0, SoC);
          PrevSoC = SoC;
          Prev_kWh = Net_kWh;
          kWh_update = true;          
        }
        else if (((PrevSoC > SoC) && ((PrevSoC - SoC) < 1)) || ((PrevSoC < SoC) && (SpdSelect == 'P'))){ // Normal kWh calculation when SoC decreases and exception if a 0 gitch in SoC data
          kWh_corr = 0;          
          used_kwh = calc_kwh(SoC, InitSoC);
          left_kwh = calc_kwh(0, SoC);
          PrevSoC = SoC;
          Prev_kWh = Net_kWh;
          kWh_update = true;
          
          if((used_kwh >= 2) && (SpdSelect == 'D')){ // Wait till 2 kWh has been used to start calculating ratio to have a better accuracy
            degrad_ratio = Net_kWh / used_kwh;
            old_lost = degrad_ratio;
          }
          else{
            degrad_ratio = old_lost;
            if((degrad_ratio > 1.2) || (degrad_ratio < 0.8)){  // if a bad value got saved previously, initialize ratio to 1
              degrad_ratio = 1;
            }
          }
        }       
      }
        
    }
    else if((Prev_kWh < Net_kWh) && !kWh_update){  // since the SoC has only 0.5 kWh resolution, when the Net_kWh increases, a 0.1 kWh is added to the kWh calculation to interpolate until next SoC change.
      kWh_corr += 0.1;
      used_kwh = calc_kwh(PrevSoC, InitSoC) + kWh_corr;
      left_kwh = calc_kwh(0, PrevSoC) - kWh_corr;
      Prev_kWh = Net_kWh;
      corr_update = true;
    }
    else if((Prev_kWh > Net_kWh)&& !kWh_update){    // since the SoC has only 0.5 kWh resolution, when the Net_kWh decreases, a 0.1 kWh is substracted to the kWh calculation to interpolate until next SoC change.
      kWh_corr -= 0.1;
      used_kwh = calc_kwh(PrevSoC, InitSoC) + kWh_corr;
      left_kwh = calc_kwh(0, PrevSoC) - kWh_corr;
      Prev_kWh = Net_kWh;
      corr_update = true;
    }
    
    if(sendIntervalOn){ // add condition so "kWh_corr" is not trigger before a cycle after a "kWh_update" when wifi is not connected
      if(kWh_update){
        Prev_kWh = Net_kWh;        
        kWh_update = false; // reset kWh_update so correction logic starts again         
      }            
      if(corr_update){
        corr_update = false;  // reset corr_update since it's not being recorded 
      }      
      sendIntervalOn = false;
    }    
    
    if ((LastSoC + 1) < SoC && (Power > 0) && (LastSoC != 0)){ // reset trip after a battery recharge
      //if (LastSoC = 0){ // To fix a glich where LastSoC = 0 after a power up, not to trigger a reset_trip.
      //  LastSoC = SoC;
      //}
      //else{ 
        mem_Power = Power;
        mem_LastSoC = LastSoC;
        mem_SoC = SoC;
        initscan = true;
        record_code = 1; 
        reset_trip();
      //}   
    }
    
    EstFull_kWh = full_kwh * degrad_ratio;    
    EstLeft_kWh = left_kwh * degrad_ratio;
        
    RangeCalc();
    if(BMS_ign){
      EnergyTOC();
    }

    if(Max_Pwr < 100 && (Max_Pwr < (Power + 20)) && !Power_page){ //select the Max Power page if Power+20kW exceed Max_Pwr when Max_Pwr is lower then 100kW.
      DrawBackground = true;
      screenNbr = 4;
      Power_page = true;
    }
    if(Power < 0 && (SpdSelect == 'P') && !Charge_page){
      DrawBackground = true;
      screenNbr = 3;
      Charge_page = true; 
    }
  } 
  
  save_lost(SpdSelect);
}

//--------------------------------------------------------------------------------------------
//                   Net Energy Calculation Function
//--------------------------------------------------------------------------------------------
 
/*//////Function to calculate Discharge Energy Since last reset //////////*/

float UpdateNetEnergy(){        
        if (InitCED == 0){ //if discharge value have been reinitiate to 0 then      
            InitCED = CED;  //initiate to current CED for initial CED value and            
            InitSoC = SoC;  //initiate to current CED for initial SoC value and
            CurrInitCED = CED;
            }
        if (InitCDC == 0){
            InitCDC = CDC;
            }
        if (InitCEC == 0){ //if charge value have been reinitiate to 0 then
            InitCEC = CEC;  //initiate to current CEC for initial CEC value            
            CurrInitCEC = CEC;
            }
        if (InitCCC == 0){
            InitCCC = CCC;            
            }
                     
        Discharg = CED - InitCED;
        Regen = CEC - InitCEC;
        Net_kWh = Discharg - Regen;     
        
        DischAh = CDC - InitCDC;        
        RegenAh = CCC - InitCCC;
        Net_Ah = DischAh - RegenAh;
        
        CurrAccEnergy = acc_energy - CurrInitAccEnergy;        
        CurrTripDisc = CED - CurrInitCED;       
        CurrTripReg = CEC - CurrInitCEC;
        CurrNet_kWh = CurrTripDisc - CurrTripReg;        
}

//--------------------------------------------------------------------------------------------
//                   Net Energy based on Power integration Function
//--------------------------------------------------------------------------------------------
 
/*//////Function to calculate Energy by power integration last reset //////////*/

float Integrat_power(){
  float pwr_interval;
  float int_pwr;        
  pwr_interval = (millis() - integrate_timer) / 1000;
  int_pwr = Power * pwr_interval / 3600;
  acc_energy += int_pwr;
  if(int_pwr < 0){
    acc_regen += -(int_pwr);   
  }
}

//--------------------------------------------------------------------------------------------
//                   Net Energy based on Power integration Function
//--------------------------------------------------------------------------------------------
 
/*//////Function to calculate Energy by power integration last reset //////////*/

float Integrat_current(){
  float curr_interval;
  float int_curr;           
  curr_interval = (millis() - integrate_timer) / 1000;
  int_curr = BATTc * curr_interval / 3600;
  acc_Ah += int_curr;   
}

//--------------------------------------------------------------------------------------------
//                   Net Energy for last N km Function
//--------------------------------------------------------------------------------------------
 
/*//////Function to calculate Energy over last N km //////////*/

float N_km_energy(float latest_energy){          
  energy_array[energy_array_index] = latest_energy;
  energy_array_index ++;  
  if (energy_array_index > N_km){
    energy_array_index = 0 ;
  }
  span_energy = latest_energy - energy_array[energy_array_index];
  // return span_energy
}

//--------------------------------------------------------------------------------------------
//                   Distance based on Speed integration Function
//--------------------------------------------------------------------------------------------
 
/*//////Function to calculate distance by speed integration over time //////////*/

float Integrat_speed(){  
  speed_interval = (millis() - init_speed_timer) / 1000;
  int_speed = Speed * speed_interval / 3600;
  distance += (int_speed * 1.022); // need to apply a 1.022 to get correct distance
  init_speed_timer = millis();  
}

//--------------------------------------------------------------------------------------------
//                   Kilometer Range Calculation Function
//--------------------------------------------------------------------------------------------

float RangeCalc(){ 

  if ((prev_odo != CurrTripOdo) && (distance < 0.9)){
    if (TripOdo < 2){
       InitOdo = Odometer - distance;   // correct initial odometer value using speed integration if odometer changes within 0.9km
       TripOdo = Odometer - InitOdo;
    }
    CurrInitOdo = Odometer - distance;
    CurrTripOdo = Odometer - CurrInitOdo;    
    prev_dist = distance;    
    prev_odo = CurrTripOdo;    
    N_km_energy(acc_energy);        
  }
  else if(prev_odo != CurrTripOdo){
    prev_dist = distance;    
    prev_odo = CurrTripOdo;
    N_km_energy(acc_energy);
  }
  interval_dist = distance - prev_dist;
  Trip_dist = CurrTripOdo + interval_dist;
    
  if (Trip_dist >= 0.25 && !ResetOn){
    kWh_100km = CurrAccEnergy * 100 / Trip_dist;
    PIDkWh_100 = CurrNet_kWh * 100 / Trip_dist;
  }
  //else if (distance >= 2 && !ResetOn){
  //  kWh_100km = (0.5 * (CurrAccEnergy * 100 / Trip_dist)) + (0.5 * old_kWh_100km);
  //  PIDkWh_100 = (0.5 * (CurrNet_kWh * 100 / Trip_dist)) + (0.5 * old_PIDkWh_100km);    
  //}
  else{
    kWh_100km = old_kWh_100km;
    PIDkWh_100 = old_PIDkWh_100km;
  }

  if (Trip_dist >= (N_km + 1)){ // wait 11km before calculating consommation for last 10km 
    span_kWh_100km = span_energy * 100 / N_km;
  }
  else{
    span_kWh_100km = kWh_100km;
  }
  
  if (kWh_100km > 1){
    Est_range =  (EstLeft_kWh / kWh_100km) * 100;
    Est_range2 =  (EstLeft_kWh / span_kWh_100km) * 100;
    Est_range3 =  (EstLeft_kWh / PIDkWh_100) * 100;
  }
  else{
    Est_range = 999;
    Est_range2 = 999;
  }
}

//--------------------------------------------------------------------------------------------
//                   Ratio of Real Battery Capacity Used Function
//--------------------------------------------------------------------------------------------
 
/*//////Function to calculate the real  //////////*/

void SocRatioCalc(){  
  
  SoCratio = (BmsSoC / SoC) * 100;
  
  //if((SoCratio > 96) && (SoCratio < 97.5)){
  //  SoCratio = 97;
  //}
  //else if((SoCratio > 95.49) && (SoCratio <= 96)){
  //  SoCratio = 96;
  //}
  //else if((SoCratio > 94.49) && (SoCratio < 95.5)){
  //  SoCratio = 95;
  //}
  //else if((SoCratio > 93.49) && (SoCratio < 94.5)){
  //  SoCratio = 94;
  //}        
  Calc_kWh_corr = 1 - (0.97 - (SoCratio / 100));
}

//--------------------------------------------------------------------------------------------
//                   Energy TOC Function
//--------------------------------------------------------------------------------------------
 
/*//////Function to record time on condition  //////////*/

void EnergyTOC(){  
  if(OUTDOORtemp >= 25){
    acc_kWh_25 = acc_kWh_25 + (acc_energy - last_energy);
    acc_time_25 = acc_time_25 + (CurrOPtime - last_time);
    acc_dist_25 = acc_dist_25 + (distance - last_odo);    
  }
  else if((OUTDOORtemp < 25) && (OUTDOORtemp >= 10)){
    acc_kWh_10 = acc_kWh_10 + (acc_energy - last_energy);
    acc_time_10 = acc_time_10 + (CurrOPtime - last_time);
    acc_dist_10 = acc_dist_10 + (distance - last_odo);    
  }
  else if((OUTDOORtemp < 10) && (OUTDOORtemp >= 0)){
    acc_kWh_0 = acc_kWh_0 + (acc_energy - last_energy);
    acc_time_0 = acc_time_0 + (CurrOPtime - last_time);
    acc_dist_0 = acc_dist_0 + (distance - last_odo);    
  }
  else if((OUTDOORtemp < 0) && (OUTDOORtemp >= -10)){
    acc_kWh_m10 = acc_kWh_m10 + (acc_energy - last_energy);
    acc_time_m10 = acc_time_m10 + (CurrOPtime - last_time);
    acc_dist_m10 = acc_dist_m10 + (distance - last_odo);    
  }
  else if((OUTDOORtemp < -10) && (OUTDOORtemp >= -20)){
    acc_kWh_m20 = acc_kWh_m20 + (acc_energy - last_energy);
    acc_time_m20 = acc_time_m20 + (CurrOPtime - last_time);
    acc_dist_m20 = acc_dist_m20 + (distance - last_odo);    
  }
  else if(OUTDOORtemp < -20){
    acc_kWh_m20p = acc_kWh_m20p + (acc_energy - last_energy);
    acc_time_m20p = acc_time_m20p + (CurrOPtime - last_time);
    acc_dist_m20p = acc_dist_m20p + (distance - last_odo);    
  }
  last_energy = acc_energy;
  last_time = CurrOPtime;
  last_odo = distance;
}

//--------------------------------------------------------------------------------------------
//                   Function to calculate energy between two SoC values
//--------------------------------------------------------------------------------------------

double Interpolate(double xValues[], double yValues[], int numValues, double pointX, bool trim = true)
{
  if (trim)
  {
    if (pointX <= xValues[0]) return yValues[0];
    if (pointX >= xValues[numValues - 1]) return yValues[numValues - 1];
  }

  auto i = 0;
  if (pointX <= xValues[0]) i = 0;
  else if (pointX >= xValues[numValues - 1]) i = numValues - 1;
  else while (pointX >= xValues[i + 1]) i++;
  if (pointX == xValues[i + 1]) return yValues[i + 1];

  auto t = (pointX - xValues[i]) / (xValues[i + 1] - xValues[i]);
  t = t * t * (3 - 2 * t);
  return yValues[i] * (1 - t) + yValues[i + 1] * t;
}

float calc_kwh(float min_SoC, float max_SoC){
  
  /* variable for kWh/%SoC calculation: xValues = %SoC and yValues = kWh */ 
  const int numValues = 21;
  double xValues[] = {  0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100 };
  //double yValues[] = { 0.5487, 0.5921, 0.5979, 0.6053, 0.6139, 0.6199, 0.6238, 0.6268, 0.6295, 0.6324, 0.6362, 0.6418, 0.6524, 0.6601, 0.6684, 0.6771, 0.6859, 0.6951, 0.7046, 0.7147, 0.7249};
  double yValues[] = { 0.5432, 0.5867, 0.5931, 0.6011, 0.6102, 0.6168, 0.6213, 0.6249, 0.6282, 0.6317, 0.6362, 0.6424, 0.6537, 0.6621, 0.6711, 0.6805, 0.6900, 0.7000, 0.7102, 0.7211, 0.7321};
  float integral;
  float interval;
  float return_kwh;
  static int N = 100;
  interval = (max_SoC - min_SoC) / N;
  integral = 0,0;
  float x = 0;
  for (int i = 0; i < N; ++i){
    x = min_SoC + interval * i;    
    integral += Interpolate(xValues, yValues, numValues, x);  //64kWh battery energy equation       
  }
  //return_kwh = integral * interval;  
  return_kwh = (integral * interval) * Calc_kWh_corr;
  return return_kwh;
}

//----------------------------------------------------------------------------------------
//        Task on core 0 to Send data to Google Sheet via IFTTT web service Function                                            
//----------------------------------------------------------------------------------------

void makeIFTTTRequest(void * pvParameters){
  for(;;){
    if (send_enabled && send_data) {
      code_sent = false;
      Serial.print("Connecting to "); 
      Serial.print(server);
      
      WiFiClient client;
      int retries = 5;
      while(!!!client.connect(server, 80) && (retries-- > 0)) {
        Serial.print(".");
      }
      Serial.println();
      if(!!!client.connected()) {
        Serial.println("Failed to connect...");
        code_sent = true;
      }
      
      Serial.print("Request resource: "); 
      Serial.println(resource);
      
      float sensor_Values[nbParam];
      
      char column_name[ ][15]={"SoC","Power","BattMinT","Heater","Net_Ah","Net_kWh","AuxBattSoC","AuxBattV","Max_Pwr","Max_Reg","BmsSoC","MAXcellv","MINcellv","MAXcellvNb","MINcellvNb","BATTv","BATTc","Speed","Odometer","CEC","CED","CDC","CCC","SOH","BMS_ign","OPtimemins","OUTDOORtemp","INDOORtemp","SpdSelect","LastSoC","Calc_Used","Calc_Left","TripOPtime","CurrOPtime","PIDkWh_100","kWh_100km","degrad_ratio","EstLeft_kWh","span_kWh_100km","SoCratio","nbr_powerOn","TireFL_P","TireFR_P","TireRL_P","TireRR_P","TireFL_T","TireFR_T","TireRL_T","TireRR_T","acc_energy","Trip_dist","distance","BattMaxT","acc_Ah","acc_kWh_25","acc_kWh_10","acc_kWh_0","acc_kWh_m10","acc_kWh_m20","acc_kWh_m20p","acc_time_25","acc_time_10","acc_time_0","acc_time_m10","acc_time_m20","acc_time_m20p","acc_dist_25","acc_dist_10","acc_dist_0","acc_dist_m10","acc_dist_m20","acc_dist_m20p","acc_regen","MaxDetNb","MinDetNb","Deter_Min"};;
      
      sensor_Values[0] = SoC;
      sensor_Values[1] = Power;
      sensor_Values[2] = BattMinT;
      sensor_Values[3] = Heater;
      sensor_Values[4] = Net_Ah;
      sensor_Values[5] = Net_kWh;
      sensor_Values[6] = AuxBattSoC;
      sensor_Values[7] = AuxBattV;
      sensor_Values[8] = Max_Pwr;
      sensor_Values[9] = Max_Reg;  
      sensor_Values[10] = BmsSoC;
      sensor_Values[11] = MAXcellv;
      sensor_Values[12] = MINcellv;
      sensor_Values[13] = MAXcellvNb;
      sensor_Values[14] = MINcellvNb;
      sensor_Values[15] = BATTv;
      sensor_Values[16] = BATTc;
      sensor_Values[17] = Speed;
      sensor_Values[18] = Odometer;
      sensor_Values[19] = CEC;
      sensor_Values[20] = CED;
      sensor_Values[21] = CDC;
      sensor_Values[22] = CCC;
      sensor_Values[23] = SOH;
      sensor_Values[24] = BMS_ign;        
      sensor_Values[25] = OPtimemins;
      sensor_Values[26] = OUTDOORtemp;
      sensor_Values[27] = INDOORtemp;
      sensor_Values[28] = SpdSelect;
      sensor_Values[29] = LastSoC;
      sensor_Values[30] = used_kwh;
      sensor_Values[31] = left_kwh;
      sensor_Values[32] = TripOPtime;
      sensor_Values[33] = CurrOPtime;      
      sensor_Values[34] = PIDkWh_100;
      sensor_Values[35] = kWh_100km;
      sensor_Values[36] = degrad_ratio;
      sensor_Values[37] = EstLeft_kWh;
      sensor_Values[38] = span_kWh_100km;
      sensor_Values[39] = SoCratio;
      sensor_Values[40] = nbr_powerOn;
      sensor_Values[41] = TireFL_P;
      sensor_Values[42] = TireFR_P;
      sensor_Values[43] = TireRL_P;
      sensor_Values[44] = TireRR_P;
      sensor_Values[45] = TireFL_T;
      sensor_Values[46] = TireFR_T;
      sensor_Values[47] = TireRL_T;
      sensor_Values[48] = TireRR_T;
      sensor_Values[49] = acc_energy;
      sensor_Values[50] = Trip_dist;
      sensor_Values[51] = distance; 
      sensor_Values[52] = BattMaxT;
      sensor_Values[53] = acc_Ah;
      sensor_Values[54] = acc_kWh_25;
      sensor_Values[55] = acc_kWh_10;
      sensor_Values[56] = acc_kWh_0;
      sensor_Values[57] = acc_kWh_m10;
      sensor_Values[58] = acc_kWh_m20;
      sensor_Values[59] = acc_kWh_m20p;
      sensor_Values[60] = acc_time_25;
      sensor_Values[61] = acc_time_10;
      sensor_Values[62] = acc_time_0;
      sensor_Values[63] = acc_time_m10;
      sensor_Values[64] = acc_time_m20;
      sensor_Values[65] = acc_time_m20p;
      sensor_Values[66] = acc_dist_25;
      sensor_Values[67] = acc_dist_10;
      sensor_Values[68] = acc_dist_0;
      sensor_Values[69] = acc_dist_m10;
      sensor_Values[70] = acc_dist_m20;
      sensor_Values[71] = acc_dist_m20p;
      sensor_Values[72] = acc_regen;
      sensor_Values[73] = MaxDetNb;
      sensor_Values[74] = MinDetNb;
      sensor_Values[75] = Deter_Min;
      
      String headerNames = "";
      String payload ="";
      
      int i=0;
      
      if(initscan || record_code != 0 || shutdown_esp){
        switch (record_code)
        {
            case 0:   // No reset only header required, ESP32 power reboot
              while(i!=nbParam)
              {
                if(i==0){
                  headerNames = String("{\"value1\":\"") + column_name[i];
                  i++;
                }
                if(i==nbParam)
                  break;
                headerNames = headerNames + "|||" + column_name[i];
                i++;    
              }
              initscan = false;
              break;

            case 1:   // Write status for Reset after a battery was recharged
              headerNames = String("{\"value1\":\"") + "|||" + "Battery_Recharged" + "|||" + "LastSoc:" + "|||" + mem_LastSoC + "|||" + "Soc:" + "|||" + mem_SoC + "|||" + "Power:" + "|||"  + mem_Power;
              record_code = 0;
              initscan = true;
              break;

            case 2:   // Write status for Reset performed with reset button (right button)
              headerNames = String("{\"value1\":\"") + "|||" + "Button_Reset";
              record_code = 0;
              initscan = true;
              break;

            case 3:   // Write status for Reset when Acc_energy is less then 0.3kWh when SoC changes
              headerNames = String("{\"value1\":\"") + "|||" + "ACC_energy <0.3" + "|||" + "acc_energy:" + "|||" + mem_energy + "|||" + "PreSoC:" + "|||" + mem_PrevSoC + "|||" + "SoC:" + "|||" + mem_SoC;
              record_code = 0;
              initscan = true;
              break;

            case 4:   // Write status for Reset if SoC changes from 100 to 99% not going through 99.5%
              headerNames = String("{\"value1\":\"") + "|||" + "100_to_99SoC_reset" + "|||" + "PreSoc:" + "|||" + mem_PrevSoC + "|||" + "SoC:" + "|||" + mem_SoC;
              record_code = 0;
              initscan = true;
              break;

            case 5:   // Write that esp is going normal shutdown
            headerNames = String("{\"value1\":\"") + "|||" + "Normal Shutdown" + "|||" + "Power:" + "|||" + Power + "|||" + "SoC:" + "|||" + mem_SoC;
            //record_code = 0;
            //code_sent = true;
            code_received = true;
            //initscan = true;
            Serial.println("Code Received");
            break;

            case 6:   // Write that esp is going timed shutdown
            headerNames = String("{\"value1\":\"") + "|||" + "Timer Shutdown" + "|||" + "Power:" + "|||" + Power + "|||" + "Timer:" + "|||" + shutdown_timer;
            //record_code = 0;
            //code_sent = true;
            code_received = true;
            //initscan = true;
            break;

            case 7:   // Write that esp is going low 12V shutdown
            headerNames = String("{\"value1\":\"") + "|||" + "Low 12V Shutdown" + "|||" + "12V Batt.:" + "|||" + AuxBattSoC + "|||" + "Timer:" + "|||" + shutdown_timer;
            //record_code = 0;
            //code_sent = true;
            code_received = true;
            //initscan = true;
            break;

        }      
          payload = headerNames;
      }
        
      else{
        while(i!=nbParam) 
        {
          if(i==0)
          {
            payload = String("{\"value1\":\"") + sensor_Values[i];
            i++;
          }
          if(i==nbParam)
          {
             break;
          }
          payload = payload + "|||" + sensor_Values[i];
          i++;    
        }
      }      
      
      String jsonObject = payload + "\"}";                          
                       
      client.println(String("POST ") + resource + " HTTP/1.1");
      client.println(String("Host: ") + server); 
      client.println("Connection: close\r\nContent-Type: application/json");
      client.print("Content-Length: ");
      client.println(jsonObject.length());
      client.println();
      client.println(jsonObject);
                  
      int timeout = 5; // 50 * 100mS = 5 seconds            
      while(!!!client.available() && (timeout-- > 0)){
        delay(100);
      }
      if(!!!client.available()) {
        Serial.println("No response...");
        code_sent = true;
      }
      while(client.available()){
        Serial.write(client.read());
      }
      
      Serial.println();
      Serial.println("closing connection");
      client.stop();
          
      send_data = false;
      pwr_changed = 0;
      loop_count = 0;
      
      if(kWh_update){ //add condition so "kWh_corr" is not trigger before a cycle after a "kWh_update"
        Prev_kWh = Net_kWh;        
        kWh_update = false;  // reset kWh_update after it has been recorded and so the correction logic start again       
      }            
      if(corr_update){  
        corr_update = false;  // reset corr_update after it has been recorded
      }
      if (code_received){
        Serial.println("Sending code sent");
        code_sent = true;
      }      
    }    
    vTaskDelay(10); // some delay is required to reset watchdog timer
  }
}

//--------------------------------------------------------------------------------------------
//                        Button functions
//--------------------------------------------------------------------------------------------

void ButtonLoop() {      
        
    if (bouton.pressed()){
      InitMillis = millis();
      Serial.println("Button pressed");
    }    
    if (bouton.released()){
        if (millis() - InitMillis >= Longinterval){
          Serial.println("Button Long press");        
          if (screenNbr == 0){          
            InitRst = true;            
            PrevSoC = 0;                      
            return;
          }
        }
        
        else{
          Serial.println("Button short press");
          Serial.print("screenNbr");Serial.print(screenNbr);          
          DrawBackground = true;
          if(screenNbr == (pagenumbers - 1)){
                screenNbr = 0;
          }
          else if(screenNbr < pagenumbers -1){
                screenNbr++;
          } 
        }
    }
    if (bouton2.pressed()){
      InitMillis2 = millis();
      //Serial.println("Button2 pressed");
    }    
    
    if (bouton2.released()){
        if (millis() - InitMillis2 >= Longinterval){
            Serial.println("Button2 Long press");
            if(!SetupOn) {
              SetupOn = true;
            }
            else{
              EEPROM.writeBool(40, StayOn);
              EEPROM.commit();
              SetupOn = false;
              DrawBackground = true;
            }
          }            
        else{
            //Serial.println("Button2 short press");       
            if(SetupOn){
                if (StayOn){                  
                  StayOn = false;
                  Serial.println("StayOn: ");Serial.println(StayOn);                  
                }
                else{
                  StayOn = true;
                  Serial.println("StayOn: ");Serial.println(StayOn);                   
                }
            }
            else{
              screenNbr = 0;
              DrawBackground = true;
              ledBacklight = 80;
              ledcWrite(pwmLedChannelTFT, ledBacklight);
             
            }
        }
    }     
}

/*////////////// Full Trip Reset ///////////////// */

void reset_trip() { //Overall trip reset. Automatic if the car has been recharged to the same level as previous charge or if left button is pressed for more then 3 secondes
  
    Serial.println("saving");
    InitOdo = Odometer;                 
    InitCED = CED;  //initiate to current CED for initial CED value and
    InitSoC = SoC;  //initiate to current CED for initial SoC value and          
    InitCEC = CEC;  //initiate to current CEC for initial CEC value and
    InitCDC = CDC;
    InitCCC = CCC;
    Net_kWh = 0;
    acc_energy = 0;
    acc_Ah = 0;
    UsedSoC = 0;
    kWh_corr = 0;
    Discharg = 0;
    Regen = 0;
    Net_Ah = 0;
    DischAh = 0;
    RegenAh = 0;
    PrevOPtimemins = 0;
    LastSoC = SoC;
    PrevBmsSoC = BmsSoC;
    EEPROM.writeFloat(0, Net_kWh);    //save initial CED to Flash memory
    EEPROM.writeFloat(52, acc_energy);
    EEPROM.writeFloat(4, InitCED);    //save initial CED to Flash memory
    EEPROM.writeFloat(8, InitCEC);    //save initial CEC to Flash memory  
    EEPROM.writeFloat(12, InitSoC);    //save initial SoC to Flash memory
    EEPROM.writeFloat(16, UsedSoC);    //save initial SoC to Flash memory
    EEPROM.writeFloat(20, InitOdo);    //save initial Odometer to Flash memory
    EEPROM.writeFloat(24, InitCDC);    //save initial Calculated CED to Flash memory
    EEPROM.writeFloat(28, InitCCC);    //save initial Calculated CED to Flash memory    
    EEPROM.commit();
    Serial.println("Values saved to EEPROM");
    CurrInitCED = CED;
    CurrInitCEC = CEC;
    CurrInitOdo = Odometer;
    CurrInitSoC = SoC;
    CurrTripReg = 0;
    CurrTripDisc = 0;    
    CurrTimeInit = OPtimemins;
    integrate_timer = millis();    
    distance = 0;
    CurrInitAccEnergy = 0;
    SocRatioCalc();
    last_energy = acc_energy;
    start_kwh = calc_kwh(0, InitSoC);    
}

/*////////////// Current Trip Reset ///////////////// */

void ResetCurrTrip(){ // when the car is turned On, current trip values are resetted.
  
    if (
      ResetOn && (SoC > 1) && (Odometer > 1) && (CED > 1) && (CEC > 1) && data_ready){ // ResetOn condition might be enough, might need to update code...         
        CurrInitAccEnergy = acc_energy;        
        CurrInitCED = CED;
        CurrInitCEC = CEC;
        CurrInitOdo = Odometer;
        CurrInitSoC = SoC;
        CurrTripReg = 0;
        CurrTripDisc = 0;
        CurrTimeInit = OPtimemins;
        Serial.println("Trip Reset");
        Prev_kWh = Net_kWh;
        last_energy = acc_energy;        
        SocRatioCalc();
        used_kwh = calc_kwh(SoC, InitSoC) + kWh_corr;
        left_kwh = calc_kwh(0, SoC) - kWh_corr;
        
        PrevSoC = SoC;
        PrevBmsSoC = BmsSoC;
        full_kwh = calc_kwh(0, 100);
        start_kwh = calc_kwh(0, InitSoC);
        ResetOn = false;
        DrawBackground = true;
        for (uint8_t i = 0 ; i < N_km ; i++ ){
          energy_array[i] = acc_energy;
        }
        degrad_ratio = old_lost;
            if((degrad_ratio > 1.2) || (degrad_ratio < 0.8)){  // if a bad value got saved previously, initial ratio to 1
              degrad_ratio = 1;
            }
  }
}

/*//////Function to disable the VESS //////////*/

void setVessOff(char selector){  
        if (selector == 'D' && SelectOn){      
          //digitalWrite(LedPin, HIGH);
          digitalWrite(VESSoff, HIGH);   
          StartMillis = millis();
          SelectOn = false;
        }
        if ((millis() - StartMillis) >= ToggleDelay){     
          //digitalWrite(LedPin, LOW);
          digitalWrite(VESSoff, LOW);    
        }
        if (selector == 'P'){
          SelectOn = true;
        }
      }

void initial_eeprom(){
  EEPROM.writeFloat(92, 660);
          EEPROM.writeFloat(96, 3278);
          EEPROM.writeFloat(100, 4950);
  //for (int i = 68; i < 148; i+=4) {
    //if (isnan(EEPROM.readFloat(i))){    
    //  EEPROM.writeFloat(i, 0);
    //}  
  //}
  EEPROM.commit();  
}

/*//////Function to save some variable before turn off the car //////////*/

void save_lost(char selector){         
        if (selector == 'D' && !DriveOn){ 
          DriveOn = true;
        }        
        if (selector == 'P' && DriveOn && SoC > 0){  // when the selector is set to Park, some values are saved to be used the next time the car is started
          DriveOn = false;
          EEPROM.writeFloat(32, degrad_ratio);
          Serial.println("new_lost saved to EEPROM");
          EEPROM.writeFloat(36, PIDkWh_100);    //save actual kWh/100 in Flash memory          
          EEPROM.writeFloat(44, TripOPtime);  //save initial trip time to Flash memory
          EEPROM.writeFloat(48, kWh_corr);    //save cummulative kWh correction (between 2 SoC values) to Flash memory
          EEPROM.writeFloat(52, acc_energy);
          EEPROM.writeFloat(56, SoC);
          EEPROM.writeFloat(60, kWh_100km);
          EEPROM.writeFloat(64, acc_Ah);
          EEPROM.writeFloat(68, acc_kWh_25);
          EEPROM.writeFloat(72, acc_kWh_10);
          EEPROM.writeFloat(76, acc_kWh_0);
          EEPROM.writeFloat(80, acc_kWh_m10);
          EEPROM.writeFloat(84, acc_kWh_m20);
          EEPROM.writeFloat(88, acc_kWh_m20p);
          EEPROM.writeFloat(92, acc_time_25);
          EEPROM.writeFloat(96, acc_time_10);
          EEPROM.writeFloat(100, acc_time_0);
          EEPROM.writeFloat(104, acc_time_m10);
          EEPROM.writeFloat(108, acc_time_m20);
          EEPROM.writeFloat(112, acc_time_m20p);
          EEPROM.writeFloat(116, acc_dist_25);
          EEPROM.writeFloat(120, acc_dist_10);
          EEPROM.writeFloat(124, acc_dist_0);
          EEPROM.writeFloat(128, acc_dist_m10);
          EEPROM.writeFloat(132, acc_dist_m20);
          EEPROM.writeFloat(136, acc_dist_m20p);
          EEPROM.writeFloat(140, acc_regen);
          EEPROM.commit();
        }
      }

void stop_esp(){       
        ESP_on = false; 
        if (DriveOn && (SoC > 0)){      //           
          EEPROM.writeFloat(32, degrad_ratio);
          Serial.println("new_lost saved to EEPROM");
          EEPROM.writeFloat(36, PIDkWh_100);    //save actual kWh/100 in Flash memory          
          EEPROM.writeFloat(44, TripOPtime);  //save initial trip time to Flash memory
          EEPROM.writeFloat(48, kWh_corr);    //save cummulative kWh correction (between 2 SoC values) to Flash memory
          EEPROM.writeFloat(52, acc_energy);          
          EEPROM.writeFloat(56, mem_SoC);          
          EEPROM.writeFloat(60, kWh_100km);
          EEPROM.writeFloat(64, acc_Ah);
          EEPROM.writeFloat(68, acc_kWh_25);
          EEPROM.writeFloat(72, acc_kWh_10);
          EEPROM.writeFloat(76, acc_kWh_0);
          EEPROM.writeFloat(80, acc_kWh_m10);
          EEPROM.writeFloat(84, acc_kWh_m20);
          EEPROM.writeFloat(88, acc_kWh_m20p);
          EEPROM.writeFloat(92, acc_time_25);
          EEPROM.writeFloat(96, acc_time_10);
          EEPROM.writeFloat(100, acc_time_0);
          EEPROM.writeFloat(104, acc_time_m10);
          EEPROM.writeFloat(108, acc_time_m20);
          EEPROM.writeFloat(112, acc_time_m20p);
          EEPROM.writeFloat(116, acc_dist_25);
          EEPROM.writeFloat(120, acc_dist_10);
          EEPROM.writeFloat(124, acc_dist_0);
          EEPROM.writeFloat(128, acc_dist_m10);
          EEPROM.writeFloat(132, acc_dist_m20);
          EEPROM.writeFloat(136, acc_dist_m20p);
          EEPROM.writeFloat(140, acc_regen);
          EEPROM.commit();
        }
        tft.setTextFont(1);
        tft.setTextSize(2);
        tft.fillScreen(TFT_BLACK);
        tft.drawString("Wifi", tft.width() / 2, tft.height() / 2 - 16);
        tft.drawString("Stopped", tft.width() / 2, tft.height() / 2);          
        WiFi.disconnect();
        delay(1500);
        tft.fillScreen(TFT_BLACK);
        tft.drawString("OBD2", tft.width() / 2, tft.height() / 2 - 16);
        tft.drawString("Stopped", tft.width() / 2, tft.height() / 2);
        ELM_PORT.end();
        delay(1500);
        esp_deep_sleep_start();
        //tft.fillScreen(TFT_BLACK);            
        
}


void SetupMode(){
    tft.fillScreen(TFT_BLACK);    
    tft.setTextColor(TFT_WHITE,TFT_BLUE);
    tft.setTextPadding(135);
    //tft.drawRect(0, 0, 135, 59, TFT_BLUE); // Blue rectangle
    tft.drawString("Stay On", tft.width() / 2, textLvl1, 1);
    if (StayOn){
        tft.setTextColor(TFT_BLACK,TFT_GREEN); 
        tft.fillRect(1, 20, 134, 118, TFT_GREEN);    
        tft.setTextPadding(130);
        tft.drawString("ON", tft.width() / 2, 80, 2);
    }
    else {
        tft.setTextColor(TFT_GREEN,TFT_BLACK); 
        tft.fillRect(1, 20, 134, 118, TFT_BLACK);    
        tft.setTextPadding(130);
        tft.drawString("OFF", tft.width() / 2, 80, 2);
    }
}


//--------------------------------------------------------------------------------------------
//                   Draw Boxe objects Definition
//--------------------------------------------------------------------------------------------


void draw_warningbox_lvl1(){
                         tft.setTextColor(TFT_YELLOW,TFT_RED); 
                         tft.fillRect(1, 18, 134, 58, TFT_RED);
}

void draw_warningbox_lvl2(){
                         tft.setTextColor(TFT_YELLOW,TFT_RED); 
                         tft.fillRect(1, 78, 134, 118, TFT_RED);
}

void draw_warningbox_lvl3(){
                         tft.setTextColor(TFT_YELLOW,TFT_RED); 
                         tft.fillRect(1, 138, 134, 178, TFT_RED);
}

void draw_warningbox_lvl4(){
                         tft.setTextColor(TFT_YELLOW,TFT_RED); 
                         tft.fillRect(1, 198, 134, 234, TFT_RED);
}

void draw_normalbox_lvl1(){
                         //tft.setTextColor(TFT_GREEN,TFT_BLACK); 
                         tft.fillRect(1, 19, 133, 59, TFT_BLACK);
}

void draw_normalbox_lvl2(){
                         //tft.setTextColor(TFT_GREEN,TFT_BLACK); 
                         tft.fillRect(1, 79, 133, 119, TFT_BLACK);
}

void draw_normalbox_lvl3(){
                         //tft.setTextColor(TFT_GREEN,TFT_BLACK); 
                         tft.fillRect(1, 139, 133, 179, TFT_BLACK);
}

void draw_normalbox_lvl4(){
                         //tft.setTextColor(TFT_GREEN,TFT_BLACK); 
                         tft.fillRect(1, 199, 133, 239, TFT_BLACK);
}

void draw_greenbox_lvl1(){
                         tft.setTextColor(TFT_BLACK,TFT_GREEN); 
                         tft.fillRect(1, 18, 134, 58, TFT_GREEN);
}

void draw_greenbox_lvl2(){
                         tft.setTextColor(TFT_BLACK,TFT_GREEN); 
                         tft.fillRect(1, 78, 134, 118, TFT_GREEN);
}

void draw_greenbox_lvl3(){
                         tft.setTextColor(TFT_BLACK,TFT_GREEN); 
                         tft.fillRect(1, 138, 134, 178, TFT_GREEN);
}

void draw_greenbox_lvl4(){
                         tft.setTextColor(TFT_BLACK,TFT_GREEN); 
                         tft.fillRect(1, 198, 134, 234, TFT_GREEN);
}

//--------------------------------------------------------------------------------------------
//                   Format Displays in Pages
//--------------------------------------------------------------------------------------------

void DisplayPage(){ 
        Serial.print("DrawBackground: ");
        Serial.println(DrawBackground);      
        if(DrawBackground){
          tft.fillScreen(TFT_BLACK);
          tft.setTextDatum(MC_DATUM);
                    
          tft.setFreeFont(&FreeSans9pt7b);          
          tft.setTextSize(1);
          tft.setTextColor(TFT_WHITE,TFT_BLUE);
          tft.setTextPadding(135);            
          tft.drawString(title1, tft.width() / 2, textLvl1, 1);
          tft.drawString(title2, tft.width() / 2, textLvl2, 1);
          tft.drawString(title3, tft.width() / 2, textLvl3, 1);
          tft.drawString(title4, tft.width() / 2, textLvl4, 1);          
                 
          strcpy(prev_value1,"");
          strcpy(prev_value2,"");
          strcpy(prev_value3,"");
          strcpy(prev_value4,"");
          DrawBackground = false;
          tft.setFreeFont(&FreeSans24pt7b);                    
        }         

        if(value1_float < 0){ 
          value1_float = abs(value1_float);
          negative_flag1 = true;
        }
        else{
          negative_flag1 = false;
        }
        if(value2_float < 0){ 
          value2_float = abs(value2_float);
          negative_flag2 = true;
        }
        else{
          negative_flag2 = false;
        }
        if(value3_float < 0){ 
          value3_float = abs(value3_float);
          negative_flag3 = true;
        }
        else{
          negative_flag3 = false;
        }
        if(value4_float < 0){ 
          value4_float = abs(value4_float);
          negative_flag4 = true;
        }
        else{
          negative_flag4 = false;
        }
        
        dtostrf(value1_float,3,nbr_decimal1,value1);
        dtostrf(value2_float,3,nbr_decimal2,value2);
        dtostrf(value3_float,3,nbr_decimal3,value3);
        dtostrf(value4_float,3,nbr_decimal4,value4);
        
        if(value1 != prev_value1){            
          tft.setTextColor(TFT_BLACK,TFT_BLACK);        
          tft.drawString(prev_value1, tft.width()/2, drawLvl1, 1);
          if(negative_flag1){        
            tft.setTextColor(TFT_ORANGE,TFT_BLACK);        
            tft.drawString(value1, tft.width()/2, drawLvl1, 1);
          }
          else{            
            tft.setTextColor(TFT_GREEN,TFT_BLACK);        
            tft.drawString(value1, tft.width()/2, drawLvl1, 1);
          }
          strcpy(prev_value1,value1);
        }
        if(value2 != prev_value2){         
          tft.setTextColor(TFT_BLACK,TFT_BLACK);
          tft.drawString(prev_value2, tft.width()/2, drawLvl2, 1);     
          if(negative_flag2){     
            tft.setTextColor(TFT_ORANGE,TFT_BLACK);        
            tft.drawString(value2, tft.width()/2, drawLvl2, 1);
          }
          else{
            tft.setTextColor(TFT_GREEN,TFT_BLACK);
            tft.drawString(value2, tft.width()/2, drawLvl2, 1);            
          }
          strcpy(prev_value2,value2);
        }
        if(value3 != prev_value3){          
          tft.setTextColor(TFT_BLACK,TFT_BLACK);
          tft.drawString(prev_value3, tft.width()/2, drawLvl3, 1);
          if(negative_flag3){          
            tft.setTextColor(TFT_ORANGE,TFT_BLACK);        
            tft.drawString(value3, tft.width()/2, drawLvl3, 1);
          }
          else{
            tft.setTextColor(TFT_GREEN,TFT_BLACK);
            tft.drawString(value3, tft.width()/2, drawLvl3, 1);            
          }
          strcpy(prev_value3,value3);
        }
        if(value4 != prev_value4){          
          tft.setTextColor(TFT_BLACK,TFT_BLACK);
          tft.drawString(prev_value4, tft.width()/2, drawLvl4, 1);
          if(negative_flag4){          
            tft.setTextColor(TFT_ORANGE,TFT_BLACK);        
            tft.drawString(value4, tft.width()/2, drawLvl4, 1);
          }         
          else{
            tft.setTextColor(TFT_GREEN,TFT_BLACK);
            tft.drawString(value4, tft.width()/2, drawLvl4, 1);            
          }
          strcpy(prev_value4,value4);
        }                    
        
}
//-------------------------------------------------------------------------------------
//             Start of Pages content definition         
//-------------------------------------------------------------------------------------

/*///////////////// Display Page 1 //////////////////////*/
void page1(){ 
        
        strcpy(title1,"TripOdo");
        strcpy(title2,"PIDkWh_100");
        strcpy(title3,"Est_range");
        strcpy(title4,"EstLeft_kWh");
        value1_float = TripOdo;
        value2_float = PIDkWh_100;
        value3_float = Est_range3;
        value4_float = EstLeft_kWh;
        if(value1_float >= 100){
          nbr_decimal1 = 0;
        }
        else{
          nbr_decimal1 = 1;
        }
        if(value2_float >= 100){
          nbr_decimal2 = 0;
        }
        else{
          nbr_decimal2 = 1;
        }
        if(value3_float >= 100){
          nbr_decimal3 = 0;
        }
        else{
          nbr_decimal3 = 1;
        }
        if(value4_float >= 100){
          nbr_decimal4 = 0;
        }
        else{
          nbr_decimal4 = 1;
        }        

        DisplayPage();               
}
/*///////////////// End of Display Page 1 //////////////////////*/

/*///////////////// Display Page 2 //////////////////////*/
void page2(){
  
        strcpy(title1,"kWh/100km");
        strcpy(title2,"Est_range");
        strcpy(title3,"10Km_kWh");
        strcpy(title4,"Est_range");        
        value1_float = kWh_100km;
        value2_float = Est_range;
        value3_float = span_kWh_100km;
        value4_float = Est_range2;
        if(value1_float >= 100){
          nbr_decimal1 = 0;
        }
        else{
          nbr_decimal1 = 1;
        }
        if(value2_float >= 100){
          nbr_decimal2 = 0;
        }
        else{
          nbr_decimal2 = 1;
        }
        if(value3_float >= 100){
          nbr_decimal3 = 0;
        }
        else{
          nbr_decimal3 = 1;
        }
        if(value4_float >= 100){
          nbr_decimal4 = 0;
        }
        else{
          nbr_decimal4 = 1;
        }

        DisplayPage();       
}
/*///////////////// End of Display Page 2 //////////////////////*/

/*///////////////// Display Page 3 //////////////////////*/
void page3(){

        strcpy(title1,"Calc_Used");        
        strcpy(title2,"Calc_Left");
        strcpy(title3,"PID_kWh");
        strcpy(title4,"Int_Energ");  
        value1_float = used_kwh;
        value2_float = left_kwh;
        value3_float = Net_kWh;
        value4_float = acc_energy;
        if(value1_float >= 100){
          nbr_decimal1 = 0;
        }
        else{
          nbr_decimal1 = 1;
        }
        if(value2_float >= 100){
          nbr_decimal2 = 0;
        }
        else{
          nbr_decimal2 = 1;
        }
        if(value3_float >= 100){
          nbr_decimal3 = 0;
        }
        else{
          nbr_decimal3 = 1;
        }
        if(value4_float >= 100){
          nbr_decimal4 = 0;
        }
        else{
          nbr_decimal4 = 1;
        }

        DisplayPage();                            
}
/*///////////////// End of Display Page 3 //////////////////////*/

/*///////////////// Display Page 4 //////////////////////*/
void page4(){
  
        strcpy(title1,"MIN_Temp");
        strcpy(title2,"Heater");
        strcpy(title3,"Power");
        strcpy(title4,"MAX_Temp");        
        value1_float = BattMinT;
        value2_float = Heater;
        value3_float = Power;
        value4_float = BattMaxT;
        nbr_decimal1 = 1;
        nbr_decimal2 = 1;
        if(value1_float >= 100){
          nbr_decimal1 = 0;
        }
        else{
          nbr_decimal1 = 1;
        }
        if(value2_float >= 100){
          nbr_decimal2 = 0;
        }
        else{
          nbr_decimal2 = 1;
        }
        if(value3_float >= 100){
          nbr_decimal3 = 0;
        }
        else{
          nbr_decimal3 = 1;
        }
        if(value4_float >= 100){
          nbr_decimal4 = 0;
        }
        else{
          nbr_decimal4 = 1;
        }

        DisplayPage();        
}
/*///////////////// End of Display Page 4 //////////////////////*/

/*///////////////// Display Page 5 //////////////////////*/
void page5(){
          
        strcpy(title1,"Max_Pwr");
        strcpy(title2,"Power");
        strcpy(title3,"MAX_Temp");
        strcpy(title4,"SoC");        
        value1_float = Max_Pwr;
        value2_float = Power;
        value3_float = BattMinT;
        value4_float = SoC;
        if(value1_float >= 100){
          nbr_decimal1 = 0;
        }
        else{
          nbr_decimal1 = 1;
        }
        if(value2_float >= 100){
          nbr_decimal2 = 0;
        }
        else{
          nbr_decimal2 = 1;
        }
        if(value3_float >= 100){
          nbr_decimal3 = 0;
        }
        else{
          nbr_decimal3 = 1;
        }
        if(value4_float >= 100){
          nbr_decimal4 = 0;
        }
        else{
          nbr_decimal4 = 1;
        }      

        DisplayPage();        
}
/*///////////////// End of Display Page 5 //////////////////////*/

/*///////////////// Display Page 6 //////////////////////*/
void page6(){
                 
        strcpy(title1,"BmsSoC");
        strcpy(title2,"CellVdiff");
        strcpy(title3,"MAXcellv");
        strcpy(title4,"12V_SoC");               
        value1_float = BmsSoC;
        value2_float = CellVdiff;
        value3_float = MAXcellv;
        value4_float = AuxBattSoC;
        if(value1_float >= 100){
          nbr_decimal1 = 0;
        }
        else{
          nbr_decimal1 = 1;
        }
        if(value2_float >= 100){
          nbr_decimal2 = 0;
        }
        else{
          nbr_decimal2 = 2;
        }
        if(value3_float >= 100){
          nbr_decimal3 = 0;
        }
        else{
          nbr_decimal3 = 2;
        }
        if(value4_float >= 100){
          nbr_decimal4 = 0;
        }
        else{
          nbr_decimal4 = 1;
        }    

        DisplayPage();            
}
/*///////////////// End of Display Page 6 //////////////////////*/

/*///////////////// Display Page 7 //////////////////////*/
void page7(){
        
        strcpy(title1,"Full_Ah");
        strcpy(title2,"Deter_Min");
        strcpy(title3,"MaxDetNb");
        strcpy(title4,"SOH");        
        value1_float = EstFull_Ah;
        value2_float = Deter_Min;
        value3_float = MaxDetNb;
        value4_float = SOH;
        if(value1_float >= 100){
          nbr_decimal1 = 0;
        }
        else{
          nbr_decimal1 = 0;
        }
        if(value2_float >= 100){
          nbr_decimal2 = 0;
        }
        else{
          nbr_decimal2 = 2;
        }
        if(value3_float >= 100){
          nbr_decimal3 = 0;
        }
        else{
          nbr_decimal3 = 0;
        }
        if(value4_float >= 100){
          nbr_decimal4 = 0;
        }
        else{
          nbr_decimal4 = 1;
        }

        DisplayPage();        
}
/*///////////////// End of Display Page 7 //////////////////////*/                 
        


/*///////////////////////////////////////////////////////////////////////*/
/*                     START OF LOOP                                     */ 
/*///////////////////////////////////////////////////////////////////////*/

void loop() {
  
  loop_count += 1;
  ButtonLoop();

  currentTimer = millis();  
  if (currentTimer - previousTimer >= sendInterval) {    
      send_data = true; // This will trigger logic to send data to Google sheet
      previousTimer = currentTimer;     
    if (!send_enabled){
      sendIntervalOn = true;      
    }
  } 

               
  /*/////// Read each OBDII PIDs /////////////////*/

  read_data();
    
  /*/////// Display Page Number /////////////////*/
  
  if(!SetupOn && (ESP_on || (Power < 0))){      
      
      switch (screenNbr){  // select page to display                
               case 0: page1(); break;
               case 1: page2(); break;
               case 2: page3(); break;
               case 3: page4(); break;
               case 4: page5(); break;
               case 5: page6(); break;
               case 6: page7(); break;                                                                                        
               }
      }
  /*/////// Display Setup Page/////////////////*/
  else{ 
      if(ESP_on){
        SetupMode();
        }
      }
  
  /*/////// Stop ESP /////////////////*/               
  if(!BMS_ign && ESP_on && !StayOn && (SpdSelect == 'P')){  // When car is power off, call stop_esp which saves some data before powering ESP32 down
    record_code = 5;
    shutdown_esp = true;
    if (!SoC_saved)
      {
        mem_SoC = SoC;
        SoC_saved = true;                  
      }    
    if (code_sent)
    {      
      Serial.println("Code sent and Normal shutdown");
      stop_esp();
    }
    else if (!send_enabled)
    {
      Serial.println("No Code sent and Normal shutdown");
      stop_esp();
    }
    
  }

  else if (!BMS_ign && (Power >= 0) && !StayOn && data_ready){   // When the car is off but the BMS does some maintnance check, wait 30 mins before esp32 power down
    if (!SoC_saved)
    {
      ESPinitTimer = millis();
      mem_SoC = SoC;
      SoC_saved = true;
      
    }
    ESPTimer = millis();
    shutdown_timer = (ESPTimer - ESPinitTimer) / 1000;
    sd_condition1 = shutdown_timer >= ESPTimerInterval;
    sd_condition2 = ((AuxBattSoC > 0) && (AuxBattSoC < 75));
    if (sd_condition1 || sd_condition2) {
              
      if (sd_condition1)
      {
        record_code = 6;
        shutdown_esp = true;                 
      } 
      else if (sd_condition2)
      {
        record_code = 7;
        shutdown_esp = true;           
      }
      if (code_sent){
       Serial.println("Code sent and Timer shutdown");
       stop_esp();
      }
      else if (!send_enabled){
        Serial.println("No Code sent and Timer shutdown");
        stop_esp();
      }  
    }    
  }  
  
  if ((WiFi.status() != WL_CONNECTED) && BMS_ign && !wifiReconn) {  // If esp32 is On when start the car, reconnect wifi if not connected
    ConnectWifi(tft);
    wifiReconn = true;
    if (WiFi.status() == WL_CONNECTED) {
      send_enabled = true;
      send_data = true;      
    }
    DrawBackground = true;  
  }
  
  ResetCurrTrip();
  
  //setVessOff(SpdSelect);  //This will momentarely set an output ON to turned off the VESS //  
}
