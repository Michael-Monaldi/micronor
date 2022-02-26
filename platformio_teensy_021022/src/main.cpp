#include <Arduino.h>
// Libraries for the Adafruit RGB/LCD Shield

#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>
// The shield uses the I2C SCL and SDA pins. On classic Arduinos
// this is Analog 4 and 5 so you can't use those for analogRead() anymore
// However, you can connect other I2C sensors to the I2C bus and share I2C bus
// These #defines make it easy to set the backlight color
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
//#define WHITE 0x7
#define BUTTON_SHIFT BUTTON_SELECT
// define the degree symbol
const char symDegree = 223;
const char symU = 94;
const char symD = 118;
const char symL = 127;
const char symR = 126;
byte symDown[8] = {
        B00000,
        B00000,
        B00000,
        B00000,
        B00000,
        B10001,
        B01010,
        B00100};
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
#define MENU_DELAY 1500
#define bounciness 150
// So we can save and retrieve settings
#include <EEPROM.h>
#include <TimeLib.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_I2CDevice.h>

#include <stdio.h>
#include <stdlib.h>

/*
    pullup / pulldown resistors
xxx RTC
    "begin" button
    subject / session
    wait for 5 from keyboard
    >> send 5 to eprime
    >> begin txt / csv
    >> >> read values into file
    >> close file
    repeat x2
xxx configure MR430 to match MR310
xxx test homing
xxx add LCD or 2 LCDs
xxx DIN rail // cutting board
    >> HIGH to homingPin_R & homingPin_L >> delay >> LOW
 */


// ************************************************
// PID Variables and constants
// ************************************************
//Define Variables we'll be connecting to
double Setpoint;
double Input;
double Output;
volatile long onTime = 0;
// EEPROM addresses for persisted data
const int SessionAddress = 0;
const int SubjectIDAddress = 8;
const int TIMEAddress = 16;
const int DATEAddress = 24;
const int RUNAddress = 32;
// 10 second Time Proportional Output window
int WindowSize = 10000;
signed long windowStartTime;
boolean tuning = false;
boolean errorState = false;


unsigned long lastInput = 0; // last button press
const int logInterval = 10000; // log every 10 seconds
uint32_t logStartTime;
uint32_t logTime;



// ************************************************
// States for state machine
// ************************************************
enum operatingState { OFF = 0, SET_SUB, SET_SES, SET_RUN, SET_DATE, SET_TIME, SET_VERIFY, WAIT_TRIGGER, LOG_DATA, ERROR_INFO};
operatingState opState = OFF;

double SubjectID = 1;
double Session = 1;
int SID = 1;
int SES = 1;
int RUN_N = 1;
double DATE = 3;
double TIME = 5;

int increment = 1;
char sBuffer[100];
const char *opName[] = {"OFF", "set1 ", "run ", "ppp ", "iiii ", "dddd", "7sec"};
String opString = F("null");
String outString = F("empty");
int opVar = 0;


// ************************************************
// Pin definitions
// ************************************************
// Output Relays
#define MR310_RST 2
#define MR430_RST 3
// read ADCs
#define MR310_IN 16
#define MR430_IN 23
uint16_t adcdeg310 = 90;
uint16_t adcdeg430 = 90;
// Scanner pulse from BNC cable
#define TRIGGER_IN 39
// unused?
#define MeasurementP 7

// ************************************************
// Sensor Variables and constants
// ************************************************
int deg310 = 31;
int deg430 = 43;
int paramsSaved = 0;

// ************************************************
// Display Variables and constants
// ************************************************
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
//unsigned long ttt = 1551500792;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);

// ************************************************
//Filenaming and lcd display vars
// ************************************************
char* LOG_FILENAME = (char*) malloc(100);
char MM_BASE[100];
char MM[100];
char tLong[40];
char tShort[40];
String errorCondition = " All  is  well !!!  ";
u_int32_t errorValue = 0;

// ************************************************
// SD Card Logging Setup
// ************************************************
#include "SD.h"// new
//#include "SdFat.h"//old
#include "RingBuf.h"
// Use Teensy SDIO
//#define SD_CONFIG SdioConfig(FIFO_SDIO)//old
#define SD_CONFIG SdioConfig(BUILTIN_SDCARD)//new
// Interval between points for 25 ksps.
#define LOG_INTERVAL_USEC 500000
// Size to log 10 byte lines at 2.5 kHz for more than ten minutes.
#define LOG_FILE_SIZE 10*2500*600  // 150,000,000 bytes.
// Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.
#define RING_BUF_CAPACITY 100*512
SdFs sd;
FsFile file;
// RingBuf for File type FsFile.
RingBuf<FsFile, RING_BUF_CAPACITY> rb;






// ************************************************
// Check buttons and time-stamp the last press
// ************************************************
uint8_t ReadButtons()
{
  uint8_t buttons = lcd.readButtons();
  if (buttons != 0){
    lastInput = millis();}
  return buttons;
}




// ************************************************
// LOG_DATA State
// DATA LOGGING
// ************************************************
void LogData()
{
   // strcpy(LOG_FILENAME, "sub-000_ses-0_run-0.csv");
   lcd.setCursor(0, 0);  
   lcd.print(F("Logging"));
   lcd.print(" ");
   snprintf(tShort,sizeof(tShort),"%02d:%02d:%02d", hour(), minute(), second());
   lcd.setCursor(11, 0);
   lcd.print(tShort);
   lcd.print(" ");
   lcd.setCursor(7, 1);  lcd.print((char) symDegree);
   lcd.setCursor(18, 1); lcd.print((char) symDegree);
   lcd.setCursor(0, 1);
   lcd.print("L: "); lcd.print(adcdeg310); lcd.print("  ");
   lcd.setCursor(11, 1);
   lcd.print("R: "); lcd.print(adcdeg430); lcd.print("  ");
   lcd.setCursor(0, 3);
   lcd.print(MM_BASE);
   lcd.setCursor(0, 2);
   lcd.print(logStartTime);
   lcd.print(" ");
   
   // initialize the RingBuf.
   rb.begin(&file);
   // Max RingBuf used bytes. Useful to understand RingBuf overrun.
   size_t maxUsed = 0;
   // Min spare micros in loop.
   int32_t minSpareMicros = INT32_MAX;
   // Start time.
   logTime = micros();
     
   while (!Serial.available())
   {
      // Amount of data in ringBuf.
      size_t n = rb.bytesUsed();

      if ((n + file.curPosition()) > (LOG_FILE_SIZE - 20)) {
         errorCondition = "File full - EXITING ";
         errorValue = 242;
         lcd.clear();
         lcd.setCursor(0,1);  lcd.print(errorCondition);  lcd.setCursor(0,2);  lcd.print(errorValue);
         opState = ERROR_INFO;
         delay(MENU_DELAY);
         break;
       }
      if (n > maxUsed) {
         maxUsed = n;
      }
      if (n >= 512 && !file.isBusy()) {
         // Not busy only allows one sector before possible busy wait.
         // Write one sector from RingBuf to file.
         if (512 != rb.writeOut(512)) {
            errorCondition = "RB writeOut Failed";
            errorValue = 257;
            lcd.clear();
            lcd.setCursor(0,1);  lcd.print(errorCondition);  lcd.setCursor(0,2);  lcd.print(errorValue);
            opState = ERROR_INFO;
            delay(MENU_DELAY);
            break;
         }
      }
      // Time for next point.
      logTime += LOG_INTERVAL_USEC;
      int32_t spareMicros = logTime - micros();
      if (spareMicros < minSpareMicros) {
           minSpareMicros = spareMicros;
      }
      if (spareMicros <= 0) {
           errorCondition = "spareMicros Negative";
           errorValue = 273;
           lcd.clear();
           lcd.setCursor(0,1);  lcd.print(errorCondition);  lcd.setCursor(0,2);  lcd.print(errorValue);
           opState = ERROR_INFO;
           delay(MENU_DELAY);
           break;
      }
      // Wait until time to log data.
      //TODO SLEEP
      while (micros() < logTime)
      {
         
         // Log data until Serial input or file full.
         uint8_t buttons = 0;
          do 
          {
             buttons = ReadButtons();
             if ((buttons & BUTTON_SELECT)
                && (buttons & BUTTON_LEFT))
                {
                   // Force QUIT
                   errorCondition = "FORCE QUIT";
                   errorValue = 306;
                   lcd.clear();
                   lcd.setCursor(0,1);  lcd.print(errorCondition);  lcd.setCursor(0,2);  lcd.print(errorValue);
                   opState = ERROR_INFO;
                   delay(MENU_DELAY);
                   rb.print((logTime - logStartTime)/LOG_INTERVAL_USEC); // convert usec to seconds
                   rb.write(',');
                   rb.print(adcdeg310);
                   rb.write(',');
                   rb.println(adcdeg430);
                   delay(1);
                   // Write any RingBuf data to file.
                   rb.sync();
                   file.truncate();
                   file.rewind();
                   file.close();
                   return;
                }
          }while(buttons == 0);
         
      }
         
   
      // Read ADC0 - about 17 usec on Teensy 4, Teensy 3.6 is faster.
      uint16_t adcdeg310 = analogRead(MR310_IN);//*360/1024;
      uint16_t adcdeg430 = analogRead(MR430_IN);//*360/1024;
      // Print spareMicros into the RingBuf as test data.
      // Print adc into RingBuf.
      rb.print((logTime - logStartTime)/LOG_INTERVAL_USEC); // convert usec to seconds
      rb.write(',');
      rb.print(adcdeg310);
      rb.write(',');
      rb.println(adcdeg430);

      snprintf(tShort,sizeof(tShort),"%02d:%02d:%02d", hour(), minute(), second());
      lcd.setCursor(11, 0);
      lcd.print(tShort);
      lcd.print(" ");
      lcd.setCursor(7, 1);  lcd.print((char) symDegree);
      lcd.setCursor(18, 1); lcd.print((char) symDegree);
      lcd.setCursor(0, 1);
      lcd.print("L: "); lcd.print(adcdeg310); lcd.print(" ");
      lcd.setCursor(11, 1);
      lcd.print("R: "); lcd.print(adcdeg430); lcd.print(" ");
      lcd.setCursor(0, 2);
      lcd.print(logStartTime);
      lcd.print(" ");
      lcd.setCursor(14, 2);
      lcd.print((logTime - logStartTime)/LOG_INTERVAL_USEC); // convert usec to seconds
      lcd.print(" ");
      lcd.setCursor(19, 3);

      if (rb.getWriteError()) {
         // Error caused by too few free bytes in RingBuf.
         errorCondition = "Buffer WriteError";
         errorValue = 316;
         lcd.clear();
         lcd.setCursor(0,1);  lcd.print(errorCondition);  lcd.setCursor(0,2);  lcd.print(errorValue);
         opState = ERROR_INFO;
         delay(MENU_DELAY);
         break;
      }
  }

  // Write any RingBuf data to file.
  rb.sync();
  file.truncate();
  file.rewind();
  file.close();
  // TODO
  // attach interrupt signal stop
 
  // Print first twenty lines of file.
  Serial.println(errorCondition);
  for (uint8_t n = 0; n < 20 && file.available();) {
    int c = file.read();
    if (c < 0) {
      break;
    }
    Serial.write(c);
    if (c == '\n') n++;
  }

  lcd.setCursor(0, 0);
  lcd.print("fileSize:maxByteUsed");
  lcd.setCursor(0, 1);
  lcd.print((uint32_t)file.fileSize()); lcd.print(" "); lcd.print(maxUsed); lcd.print(" ");
  lcd.setCursor(0, 2);
  lcd.print("minSpareMicros");
  lcd.setCursor(0, 3);
  lcd.print(minSpareMicros); lcd.print(" ");

  file.close();
  delay(MENU_DELAY);

}




// ************************************************
// ERROR_INFO State
// UP/DOWN - N/A
// RIGHT for xxx
// LEFT for SET_VERIFY
// ************************************************
void ErrorInfo()
{
   lcd.setCursor(14, 2);
   lcd.print((logTime - logStartTime)/LOG_INTERVAL_USEC); // convert usec to seconds
   lcd.print(" ");
   lcd.setCursor(0, 3);
   lcd.print("Start Over ?       <");
   lcd.setCursor(19, 3);
   delay(2*MENU_DELAY);
   lcd.setCursor(0, 3);
   lcd.print(MM_BASE);
   lcd.setCursor(19, 3);
   delay(MENU_DELAY/2);

   uint8_t buttons = 0;
   while(true)
   {
      buttons = ReadButtons();
      if ((buttons & BUTTON_SELECT)
         && (buttons & BUTTON_RIGHT)) {
             errorCondition = " All  is  well !!!  ";
             errorValue = 0;
             opState = WAIT_TRIGGER;
             return;}
      if (buttons & BUTTON_LEFT) {
         errorCondition = " All  is  well !!!  ";
         errorValue = 0;
         opState = SET_VERIFY;
         paramsSaved = 0;
         return;}
      //if (buttons & BUTTON_RIGHT) {
      //   //opState = WAIT_TRIGGER;
      //   return;}
      if (buttons & BUTTON_UP) {
         Setpoint += increment;
         delay(bounciness);
         }
      if (buttons & BUTTON_DOWN) {
         Setpoint -= increment;
         delay(bounciness);
         }
      // return to RUN after 3 seconds idle
      //if ((millis() - lastInput) > (2*MENU_DELAY)) {
      //   opState = WAIT_TRIGGER;
      //   return;}
      //LogData();
      //lcd.print("614");
   }
}





// ************************************************
// Read floating point values from EEPROM
// ************************************************
double EEPROM_readDouble(int address)
{
   double value = 0.0;
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++){
      *p++ = EEPROM.read(address++);}
   return value;
}
// ************************************************
// Write floating point values to EEPROM
// ************************************************
void EEPROM_writeDouble(int address, double value)
{
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++){
      EEPROM.write(address++, *p++);}
}
// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
void SaveParameters()
{
   if (SubjectID != EEPROM.read(SubjectIDAddress)){
      EEPROM.write(SubjectIDAddress, SubjectID);}
   if (Session != EEPROM.read(SessionAddress)){
      EEPROM.write(SessionAddress, Session);}
   if (TIME != EEPROM_readDouble(TIMEAddress)){
      EEPROM_writeDouble(TIMEAddress, TIME);}
   if (DATE != EEPROM.read(DATEAddress)){
      EEPROM_writeDouble(DATEAddress, DATE);}
   if (RUN_N != EEPROM.read(RUNAddress)){
      EEPROM_writeDouble(RUNAddress, RUN_N);}
}






// ************************************************
// OFF - Initial State - press RIGHT to enter subject
// ************************************************
void Off()
{
   // make sure outputs are off
   digitalWrite(MR310_RST, LOW);
   digitalWrite(MR430_RST, LOW);
   digitalWrite(MeasurementP, LOW);
   lcd.setCursor(0, 0);
   lcd.print(F("BNC cable, SD card &"));
   lcd.setCursor(0, 1);
   lcd.print(" BOTH Fiber Optics ");
   lcd.setCursor(0, 2);
   lcd.print(" All plugged in ???");
   delay(MENU_DELAY/4);
   lcd.setCursor(0, 3);
   lcd.print(F("Enter Setup Press "));
   lcd.setCursor(19, 3); lcd.print(">"); lcd.setCursor(19, 3);

   uint8_t buttons = 0;
   while( !(buttons & (BUTTON_RIGHT)) )
   {
      buttons = ReadButtons();
   }
   windowStartTime = millis();
   opState = SET_SUB;
}



// ************************************************
// SET_SUB State
// LEFT for off
// UP/DOWN to change SubjectID
// RIGHT for TIME
// ************************************************
void SetSub()
{
   lcd.setCursor(0, 1);
   lcd.print((char) symU);
   lcd.write(1);
   lcd.print(F(" to Change Values"));
   lcd.setCursor(0, 2);
   lcd.print((char) symL);
   lcd.print((char) symR);
   lcd.print(F(" to Navigate Menu"));
   lcd.setCursor(0, 3);
   lcd.print(F("sub: "));

   uint8_t buttons = 0;
   while(true)
   {
      buttons = ReadButtons();
      int increment = 1;
      if (buttons & BUTTON_SELECT) {
         increment *= 10;
         }
      if (buttons & BUTTON_LEFT) {
         opState = OFF;
         return;
         }
      if (buttons & BUTTON_RIGHT) {
         opState = SET_SES;
         return;
         }
      if (buttons & BUTTON_UP) {
         SubjectID += increment;
         delay(bounciness);
         }
      if (buttons & BUTTON_DOWN) {
         SubjectID -= increment;
         delay(bounciness);
         }
      if ( SubjectID <= 0 ) {
      SubjectID = 0;
      }
      SID = SubjectID;
      lcd.setCursor(5, 3);
      lcd.print(SID);
      lcd.print(" ");
      lcd.setCursor(18, 3); lcd.print("<>"); lcd.setCursor(19, 3);
      opVar = SubjectID;
      outString = opString + F(": ") + String(opVar) + '\n';
      Serial.print(outString);
   }
   lcd.setCursor(19, 3);
}


// ************************************************
// SET_SES opState
// UP/DOWN to change Session
// RIGHT for DATE
// LEFT for subject
// ************************************************
void SetSes()
{
   lcd.setCursor(0, 0);
   lcd.print("Set Session");
   lcd.setCursor(0, 1);
   lcd.print("1=PRE  2=POST");

   uint8_t buttons = 0;
   while(true)
   {
      buttons = ReadButtons();
      int increment = 1;
      //if (buttons & BUTTON_SELECT) {
         //increment *= 10;}
      if (buttons & BUTTON_LEFT) {
         opState = SET_SUB;
         return;}
      if (buttons & BUTTON_RIGHT) {
         opState = SET_RUN;
         return;}
      if (buttons & BUTTON_UP) {
         Session += increment;
         delay(bounciness);
         }
      if (buttons & BUTTON_DOWN) {
         Session -= increment;
         delay(bounciness);
         }
      // return to SET_VERIFY after 3 seconds idle
      //if ((millis() - lastInput) > MENU_DELAY) {
      //   opState = SET_VERIFY;
      //   return;}
      lcd.setCursor(0, 3);
      lcd.print("ses: ");
      if ( Session <= 0 ) {
      Session = 0;
      }
      SES = Session;
     
      lcd.setCursor(5, 3);
      lcd.print(SES);
      lcd.print(" ");
      lcd.setCursor(18, 3); lcd.print("<>"); lcd.setCursor(19, 3);
      opVar = SES;
      outString = opString + F(": ") + String(opVar) + '\n';
      Serial.print(outString);

   }
   lcd.setCursor(19, 3);
   
 
}


// ************************************************
// SET_RUN opState
// UP/DOWN to change Session
// RIGHT for DATE
// LEFT for subject
// ************************************************
void SetRun()
{
   lcd.setCursor(0, 0);
   lcd.print("Set RUN");
   lcd.setCursor(0, 1);
   lcd.print("1=Run1  2=Run2");
   lcd.setCursor(0, 2);
   lcd.print(">=3: scanner problem");
   uint8_t buttons = 0;
   while(true)
   {
      buttons = ReadButtons();
      int increment = 1;
      //if (buttons & BUTTON_SELECT) {
         //increment *= 10;}
      if (buttons & BUTTON_LEFT) {
         opState = SET_SES;
         return;}
      if (buttons & BUTTON_RIGHT) {
         opState = SET_DATE;
         return;}
      if (buttons & BUTTON_UP) {
         RUN_N += increment;
         delay(bounciness);
         }
      if (buttons & BUTTON_DOWN) {
         RUN_N -= increment;
         delay(bounciness);
         }
      // return to SET_VERIFY after 3 seconds idle
      //if ((millis() - lastInput) > MENU_DELAY) {
      //   opState = SET_VERIFY;
      //   return;}
      lcd.setCursor(0, 3);
      lcd.print("run: ");
      if ( RUN_N <= 0 || RUN_N >=10) {
      RUN_N = 0;
      }
      //SES = Session;
      lcd.setCursor(5, 3);
      lcd.print(RUN_N);
      lcd.print(" ");
      lcd.setCursor(18, 3); lcd.print("<>"); lcd.setCursor(19, 3);
      opVar = RUN_N;
      outString = opString + F(": ") + String(opVar) + '\n';
      Serial.print(outString);
   }
   lcd.setCursor(19, 3);
}



// ************************************************
// UP/DOWN to change DATE
// RIGHT for TIME
// LEFT for SESSION
// SHIFT for 10x tuning
// ************************************************
void SetDate(){
   lcd.setCursor(0, 0);
   lcd.print("Set DATE");

   uint8_t buttons = 0;
   while(true){
      buttons = ReadButtons();
      int increment = 1;
      if (buttons & BUTTON_SELECT) {
        increment *= 10;}
      if (buttons & BUTTON_LEFT) {
         opState = SET_RUN;
         return;}
      if (buttons & BUTTON_RIGHT) {
         opState = SET_TIME;
         return;}
      if (buttons & BUTTON_UP) {
         DATE += increment;
         delay(bounciness);
         }
      if (buttons & BUTTON_DOWN) {
         DATE -= increment;
         delay(bounciness);
         }
      // return to RUN after 3 seconds idle
      //if ((millis() - lastInput) > MENU_DELAY) {
      //   opState = SET_VERIFY;
      //   return;}
      lcd.setCursor(0, 1);
      char dddd[40];
      snprintf(dddd,sizeof(dddd),"%02d/%02d/%04d",  month(), day(), year());
      lcd.print(dddd);
      lcd.print(" ");

      lcd.setCursor(0, 3);
      lcd.print("Date: ");
      lcd.print(DATE);
      lcd.print(" ");
      lcd.setCursor(18, 3); lcd.print("<>"); lcd.setCursor(19, 3);
      opVar = DATE;
      outString = opString + F(": ") + String(opVar) + '\n';
      Serial.print(outString);
   }
}


// ************************************************
// Time setup State
// UP/DOWN to change TIME
// LEFT for DATE
// RIGHT for VERIFY
// SHIFT for 10x tuning
// ************************************************
void SetTime()
{
   lcd.setCursor(0, 0);
   lcd.print(F("Set TIME "));

   uint8_t buttons = 0;
   while(true)
   {
      buttons = ReadButtons();
      int increment = 1;
      if (buttons & BUTTON_SELECT) {
        increment *= 10;}
      if (buttons & BUTTON_LEFT) {
         opState = SET_DATE;
         return;}
      if (buttons & BUTTON_RIGHT) {
         opState = SET_VERIFY;
         return;}
      if (buttons & BUTTON_UP) {
         TIME += increment;
         delay(bounciness);
         }
      if (buttons & BUTTON_DOWN) {
         TIME -= increment;
         delay(bounciness);
         }
      // return to RUN after 3 seconds idle
      //if ((millis() - lastInput) > MENU_DELAY) {
      //   opState = SET_VERIFY;
      //   return;}
      lcd.setCursor(0, 1);
      snprintf(tShort,sizeof(tShort),"%02d:%02d:%02d", hour(), minute(), second());
      lcd.print(tShort);
      lcd.print(" ");

      lcd.setCursor(0, 3);
      lcd.print("Time: ");
      lcd.print(TIME);
      lcd.print(" ");
      lcd.setCursor(18, 3); lcd.print("<>"); lcd.setCursor(19, 3);
      opVar = TIME;
      outString = opString + F(": ") + String(opVar) + '\n';
      Serial.print(outString);
   }
}





void printAngles(void)
{
   //void testdrawTIME(void) {
   display.clearDisplay();
   char tLong[40];
   snprintf( tLong,sizeof(tLong),"%02d/%02d/%02d %02d:%02d:%02d", month(), day(), year(), hour(), minute(), second() );
   //display.clearDisplay();
   display.setTextSize(1);            // Normal 1:1 pixel scale
   //display.setTextColor(SSD1306_WHITE);        // Draw white text
   display.setCursor(0,0);             // Start at top-left corner
   display.print(tLong);
   //display.display();
   display.setTextSize(1);             // Normal 1:1 pixel scale
   display.setTextColor(SSD1306_WHITE);        // Draw white text
   display.setCursor(0,12);             // Start at top-left corner
   display.print(F("MR310"));
   display.setCursor(64,12);             // Start at top-left corner
   display.print(F("MR430"));
   display.setTextSize(3);
   display.setCursor(0,24);
   display.print(deg310);
   display.setCursor(64,24);
   display.print(deg430);
   display.display();
   delayMicroseconds(1);
}



void SetVerify()
{
  if (paramsSaved == 0) {
   SaveParameters();
   paramsSaved = 1;
  }
   //lcd.print("Verify ");
   lcd.setCursor(0, 0);
   snprintf(tLong,sizeof(tLong),"%02d/%02d/%02d %02d:%02d:%02d", month(), day(), year(), hour(), minute(), second());
   lcd.print(tLong);
   lcd.setCursor(0, 1);
   snprintf(MM_BASE,sizeof(MM_BASE),"sub-%01d_ses-%01d_run-%01d", SID, SES, RUN_N);
   lcd.print(MM_BASE);
   //lcd.print("sub-"); lcd.print(SubjectID); lcd.print("_ses-"); lcd.print(Session);
   lcd.setCursor(0, 2);
   lcd.print("run: "); lcd.print(RUN_N);
   //printAngles();
   delay(MENU_DELAY);
   lcd.setCursor(0, 3);
   lcd.print("< back ");
   lcd.setCursor(14, 3);
   lcd.print(" OK +>");
   lcd.setCursor(19, 3);

   uint8_t buttons = 0;
   while(true)
   {
      buttons = ReadButtons();
      int increment = 1;
      if ((buttons & BUTTON_SELECT)
         && (buttons & BUTTON_RIGHT) ) {
             opState = WAIT_TRIGGER;
             paramsSaved = 0;
             return;}
      if (buttons & BUTTON_LEFT) {
         opState = SET_TIME;
         paramsSaved = 0;
         return;}
      // if (buttons & BUTTON_RIGHT) {
      //   opState = WAIT_TRIGGER;
      //   return;}
      if (buttons & BUTTON_UP) {
         Setpoint += increment;
         delay(bounciness);
         }
      if (buttons & BUTTON_DOWN) {
         Setpoint -= increment;
         delay(bounciness);
         }
      lcd.setCursor(0, 0);
      snprintf(tLong,sizeof(tLong),"%02d/%02d/%02d %02d:%02d:%02d", month(), day(), year(), hour(), minute(), second());
      lcd.print(tLong);
      lcd.setCursor(19, 3);
   }
}







// ************************************************
// WAIT_TRIGGER State
// LEFT to go back to SET_VERIFY
// SHIFT and RIGHT to FORCE LOGGING
// ************************************************
void WaitTrigger()
{
   //display.display();
   // char MM[100];
   //int RID = RUN_N;
   //int NNN[] = {SID, Session, RUN_N};
   //int SID = SubjectID;
   //int SES = Session;
   snprintf(MM_BASE,sizeof(MM_BASE),"sub-%01d_ses-%01d_run-%01d", SID, SES, RUN_N);
   snprintf(MM,sizeof(MM),"sub-%01d_ses-%01d_run-%01d.csv", SID, SES, RUN_N);
   LOG_FILENAME = MM;
   //lcd.print("sub-"); lcd.print(SubjectID); lcd.print("_ses-"); lcd.print(Session);
   snprintf(tLong,sizeof(tLong),"%02d/%02d/%02d %02d:%02d:%02d", month(), day(), year(), hour(), minute(), second());
   lcd.setCursor(0, 0);
   lcd.print(tLong);
   lcd.setCursor(0, 1);
   lcd.print(MM_BASE);
   lcd.setCursor(0, 2);
   lcd.print("run: "); lcd.print(RUN_N);
   lcd.print(" ");
   lcd.setCursor(0, 2);
   lcd.print(F("Initializing SD card"));
   lcd.setCursor(19, 3);

  // Initialize the SD.
  if (!sd.begin(SD_CONFIG)) {
     errorCondition = "SD InitErrorHalt";
     errorValue = 879;
     lcd.clear();
     lcd.setCursor(0,1);  lcd.print(errorCondition);  lcd.setCursor(0,2);  lcd.print(errorValue);
     opState = ERROR_INFO;
     // sd.initErrorHalt();
     delay(MENU_DELAY);
     return;
   }
  // Open or create file - truncate existing file.
  // todo prevent overwrite
  if (!file.open(LOG_FILENAME, O_RDWR | O_CREAT | O_TRUNC)) {
      errorCondition = "SD FailedToOpen file";
      errorValue = 891;
      lcd.clear();
      lcd.setCursor(0,1);  lcd.print(errorCondition);  lcd.setCursor(0,2);  lcd.print(errorValue);
      opState = ERROR_INFO;
      delay(MENU_DELAY);
      return;
   }
  // File must be pre-allocated to avoid huge delays searching for free clusters.
  if (!file.preAllocate(LOG_FILE_SIZE)) {
     errorCondition = "SD preAllocate Fail";
     errorValue = 901;
     lcd.clear();
     lcd.setCursor(0,1);  lcd.print(errorCondition);  lcd.setCursor(0,2);  lcd.print(errorValue);
     file.close();
     opState = ERROR_INFO;
     delay(MENU_DELAY);
     return;
   }

   //String("sub-") +String(SubjectID,0) + String("_ses-")+String(Session,0) + String("_run-") + String(RUN_N,1);//%03d_ses-%02d_run-%01d", SID, Session, RUN_N);
   //        char nnn[40];
   //       Serial.print(SubjectID); Serial.print(",");
   //       Serial.print(SID); Serial.print(",");
   //       Serial.print(Session); Serial.print(",");
   //       Serial.println(RUN_N);
   //LOG_FILENAME = snprintf(LOG_FILENAME,sizeof(LOG_FILENAME),"sub-%3d_ses-%1d_run-%1d.csv", SID, Session, RUN_N);
   //Serial.print(LCD_FILENAME);

   digitalWrite(MR310_RST, HIGH);
   digitalWrite(MR430_RST, HIGH);
   delay(2*MENU_DELAY);
   digitalWrite(MR430_RST, LOW);
   digitalWrite(MR310_RST, LOW);
   lcd.setCursor(0, 2);
   lcd.print(F("                    "));
   lcd.setCursor(0, 3);
   lcd.print(F("Waiting for Trigger "));
   lcd.setCursor(19, 3);

   // todo make sure interrupt on trigger_pin
   uint8_t buttons = 0;
   while(true) {
      buttons = ReadButtons();
      if ((buttons & BUTTON_SELECT)
         && (buttons & BUTTON_RIGHT) ) {
            // Force Logging
            opState = LOG_DATA;
            return;
            }
            //LogData();
      //else if (buttons & BUTTON_RIGHT) {
         //opState = SET_VERIFY;
         //  return;
         // }
      if (buttons & BUTTON_LEFT) {
        opState = SET_VERIFY;
        return;
        }
      // return to RUN after 3 seconds idle
      //if ((millis() - lastInput) > (2*MENU_DELAY)) {
      //   opState = WAIT_TRIGGER;
      //   return;}
      lcd.setCursor(0, 0);
      snprintf(tLong,sizeof(tLong),"%02d/%02d/%02d %02d:%02d:%02d", month(), day(), year(), hour(), minute(), second());
      lcd.print(tLong);
    }


    // while !trigger
    // do all the above
     // else
      //LogData();
    //      deg310 = analogRead(MR310_IN)*360/1024;
    //      deg430 = analogRead(MR430_IN)*360/1024;
         //printAngles();
         // periodically log to serial port in csv format
    //      if (millis() - lastLogTime > logInterval) {
    //        lastLogTime = millis();
    //        Serial.print("A ");
    //        Serial.print(",");
    //        Serial.println("B ");}
    //end while loop
      //delayMicroseconds(1);
}





// ************************************************
// Called by ISR every 15ms to drive the output
// ************************************************
void DriveOutput()
{  
  long now = millis();
  // Set the output
  // "on time" is proportional to the PID output
  if(now - windowStartTime>WindowSize)
  { //time to shift the Relay Window
     windowStartTime += WindowSize;}
  if( (onTime > 100)
      && (onTime > (now - windowStartTime)) ){
     digitalWrite(MeasurementP,HIGH);}
  else{
     digitalWrite(MeasurementP,LOW);}
}



time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}



/*
// ************************************************
// Set Backlight based on the state of control
// ************************************************
void setBacklight()
{
   if (errorState){
      lcd.setBacklight(VIOLET);} // Tuning Mode
   else if (abs(Input - Setpoint) > 1.0)  {
      lcd.setBacklight(VIOLET);}  // High Alarm - off by more than 1 degree
   else if (abs(Input - Setpoint) > 0.2)  {
      lcd.setBacklight(VIOLET);}  // Low Alarm - off by more than 0.2 degrees
   else{
      lcd.setBacklight(VIOLET);}  // We're on target!
}
// ************************************************
// Start the Auto-Tuning cycle
// ************************************************
void StartAutoTune()
{   // REmember the mode we were in
//   ATuneModeRemember = myPID.GetMode();
   // set up the auto-tune parameters
//   aTune.SetNoiseBand(aTuneNoise);
//   aTune.SetOutputStep(aTuneStep);
//   aTune.SetLookbackSec((int)aTuneLookBack);
   tuning = true;
}
// ************************************************
// Return to normal control
// ************************************************
void FinishAutoTune()
{   tuning = false;
   // Extract the auto-tune calculated parameters
//   SubjectID = aTune.GetSubjectID();
 //  TIME = aTune.GetTIME();
 //  DATE = aTune.GetDATE();
   // Re-tune the PID and revert to normal control mode
   //myPID.SetTunings(SubjectID,TIME,DATE);
   //myPID.SetMode(ATuneModeRemember);
      // Persist any changed parameters to EEPROM
   SaveParameters();
}
*/








// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters()
{
  // Load from EEPROM
   SubjectID = EEPROM.read(SubjectIDAddress);
   Session = EEPROM.read(SessionAddress);
   RUN_N = EEPROM_readDouble(RUNAddress);
   DATE = EEPROM_readDouble(DATEAddress);
   TIME = EEPROM_readDouble(TIMEAddress);
   // Use defaults if EEPROM values are invalid
   if (isnan(SubjectID)){
     SubjectID = 1;}
   if (isnan(Session)){
     Session = 1;}
   //if (isnan(RUN_N)){
   //  RUN_N = 1;}
   if (isnan(TIME)){
     TIME = 0;}
   if (isnan(DATE)){
     DATE = 0;}

   SubjectID = int(SubjectID);
   SID = SubjectID;
   Session = int(Session);
   SES = Session;
   DATE = int(DATE);
   TIME = int(TIME);
   RUN_N = int(RUN_N);
}












// ************************************************
// Setup and display initial screen
// ************************************************
void setup()
{
   //
   // put your setup code here, to run once:
   Serial.begin(115200);
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
    {
       Serial.println(F("SSD1306 allocation failed"));
       for(;;); // Don't proceed, loop forever
    }
   setSyncProvider(getTeensy3Time);
   display.display();
   // Initialize LCD DiSplay
   lcd.begin(20, 4);
   // create down symbol from the binary
   lcd.createChar(1, symDown);
   delayMicroseconds(bounciness);
   lcd.blink();
   lcd.setBacklight(BLUE);
   lcd.setCursor(0, 0);  
   lcd.print(F("    RoseLab fMRI    "));
   lcd.setCursor(19, 3);
   // Initialize Relay Control:
   pinMode(MeasurementP, OUTPUT);    // Output mode to drive relay
   pinMode(MR310_RST, OUTPUT);
   pinMode(MR430_RST, OUTPUT);
   // make sure it is off to start
   digitalWrite(MeasurementP, LOW);
   digitalWrite(MR310_RST, LOW);
   digitalWrite(MR430_RST, LOW);
   // Set up Ground & Power for the sensor from GPIO pins
   pinMode(MR310_IN, INPUT_PULLDOWN);
   pinMode(MR430_IN, INPUT);
   pinMode(TRIGGER_IN, INPUT);
   // Splash screen
   delay(MENU_DELAY/4);
   display.clearDisplay();
   // Initialize the variables
   LoadParameters();
   paramsSaved = 0;
 

     // Run timer2 interrupt every 15 ms
   //  TCCR2A = 0;
   //  TCCR2B = 1<<CS22 | 1<<CS21 | 1<<CS20;
     //Timer2 Overflow Interrupt Enable
   //  TIMSK2 |= 1<<TOIE2;
}


// ************************************************
// Timer Interrupt Handler
// ************************************************
/*
 SIGNAL(TIMER2_OVF_vect)
{
  if (opState == OFF)
  {
    digitalWrite(MeasurementP, LOW);  // make sure relay is off
  }
  else
  {
    DriveOutput();
  }
}
*/


// put your main code here, to run repeatedly:
void loop()
{
   while(ReadButtons() != 0)
   {
      // waits for button release before changing state
   }
   int SID = SubjectID;
   int SES = Session;
   int increment = 1;
   lcd.clear();
   switch (opState)
   {
      case OFF:
         Serial.println("OFF");
         opVar = 0;
         opString = F("OFF");
         lcd.setBacklight(TEAL);
         Off();
         break;
      case SET_SUB:
         opString = F("SET_SUB");
         lcd.setBacklight(BLUE);
         lcd.setCursor(0, 0);
         lcd.print("Set Subject ID");
         SetSub();
         break;
      case SET_SES:
         lcd.setBacklight(TEAL);
         opVar = SES;
         opString = F("SET_SES");
         outString = opString + F(": ") + String(opVar) + '\n';
         Serial.print(outString);
         SetSes();
         break;
      case SET_RUN:
         opString = F("SET_RUN");
         lcd.setBacklight(RED);
         SetRun();
         break;
      case SET_DATE:
         opString = F("SET_DATE");
         lcd.setBacklight(BLUE);
         SetDate();
         break;
      case SET_TIME:
         opString = F("SET_TIME");
         lcd.setBacklight(GREEN);
         SetTime();
         break;
      case SET_VERIFY:
         opString = F("SET_VERIFY");
         lcd.setBacklight(YELLOW);
         SetVerify();
         break;
      case WAIT_TRIGGER:
         lcd.setBacklight(RED);
         WaitTrigger();
         break;
      case LOG_DATA:
         lcd.setBacklight(GREEN);
         logStartTime = micros();
         LogData();
         break;
      case ERROR_INFO:
         lcd.setBacklight(RED);
         lcd.setCursor(0, 0);
         lcd.print("!!!     ERROR    !!!");
         lcd.setCursor(0, 1);
         lcd.print(String(errorCondition));
         lcd.setCursor(0, 2);
         lcd.print(errorValue);  
         lcd.setCursor(0, 3);
         lcd.print(F(MM_BASE));
         lcd.setCursor(19, 3);
         ErrorInfo();
         break;
   }
   //display.clearDisplay();
   //testdrawTIME();
}



