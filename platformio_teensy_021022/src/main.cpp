// Micronor fMRI Data Logger - Michael Monaldi
// #include <Arduino.h>
// Libraries for the Adafruit RGB/LCD Shield
#include <Adafruit_RGBLCDShield.h>
#include <Wire.h>
// So we can save and retrieve settings
#include <EEPROM.h>
#include <TimeLib.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_I2CDevice.h>

#include <stdio.h>
#include <stdlib.h>





// ************************************************
// Display Variables and constants
// ************************************************
// These #defines make it easy to set the backlight color
#define RED 0x1
#define GREEN 0x2
#define YELLOW 0x3
#define BLUE 0x4
#define VIOLET 0x5
#define TEAL 0x6
#define WHITE 0x7
#define BUTTON_SHIFT BUTTON_SELECT
// //!< Select button #define BUTTON_SELECT  0x01
// //!< Right button  #define BUTTON_RIGHT   0x02
// //!< Down button   #define BUTTON_DOWN    0x04
// //!< Up button     #define BUTTON_UP      0x08
// //!< Left button   #define BUTTON_LEFT    0x10


// define the degree symbol
#define symDegree 223
//#define symUp 94
//#define symDown 118
#define symLeft 127
#define symRight 126
byte byteUp[8] = {
        B00100,
        B01110,
        B10101,
        B00100,
        B00100,
        B00100,
        B00100,
        B00100};
byte byteDown[8] = {
        B00100,
        B00100,
        B00100,
        B00100,
        B00100,
        B10101,
        B01110,
        B00100};
#define symUp 1
#define symDown 2
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
// ************************************************
// States for state machine
// ************************************************
enum operatingState  { TEST = 0,  OFF,    SET_DATE,   SET_SPEED,  SET_SUB,  SET_SES,  SET_RUN,  SET_VERIFY,  WAIT_TRIGGER,   LOG_DATA,     ERROR_INFO  };
operatingState opState = OFF;
unsigned int opColor[]={RED,    TEAL,    TEAL,      TEAL,         TEAL,      TEAL,      TEAL,      GREEN,       VIOLET,          GREEN,            RED    };
String opString[] = {"TESTING!", "OFF", "SET_DATE", "SET_SPEED",  "SET_SUBJECT","SET_SESSION","SET_RUN","SET_VERIFY","WAIT_TRIGGER", "Recording",    "ERROR_INFO"};
String opVarStr[] = {" FIXME  ", "OFF=",  "DATE="   , "sampHz=",  "SID="    ,"SES="    ,"RUN="   ,"  FIXME "   ,"  FIXME  " , "  LOG_DATA "  ,    "ErrVar="};

double SubjectID = 1;
double Session = 1;
double Run = 1;
int SID = 1;
int SES = 1;
int RUN = 1;
double TIME = 5;
double DATE = 3;
// EEPROM addresses for persisted data
const int SessionAddress = 0;
const int SubjectIDAddress = 8;
const int RunAddress = 16;
const int TIMEAddress = 24;
const int DATEAddress = 32;


int increment = 1;
char sBuffer[100];

String outString = F("_^_^_^_^_^_^_^_^_^_^_^_");
int opVar = 0;
int pulses = 0;
// ************************************************
// Pin definitions
// ************************************************
// Output Relays
#define MR310_RST 2
#define MR430_RST 3
// read ADCs
#define MR310_IN 16
#define MR430_IN 23
uint16_t adcdeg310 = 1;
uint16_t adcdeg430 = 1;
short deg310 = 1;
short deg430 = 1;
// Scanner pulse from BNC cable
#define TRIGGER_INPUT_PIN 39

// ************************************************
// Sensor Variables and constants
// ************************************************

int paramsSaved = 0;

#define MENU_DELAY 1500
#define PRINT_DELAY 800
#define bounciness 150

// ************************************************
//Filenaming and lcd display vars
// ************************************************
char* LOG_FILENAME = (char*) malloc(100);
char MM_BASE[100];
char MM[100];
char tLong[40];
char tShort[40];
String errorCondition = " All  is  well !!!  ";
uint32_t errorValue = 0;

volatile long onTime = 0;
boolean errorState = false;
boolean refreshNeeded = true;
// ************************************************
// SD Card Logging Setup
// ************************************************
//#include "SD.h"// new
#include "SdFat.h"
#include "RingBuf.h"
#define SD_CONFIG SdioConfig(FIFO_SDIO)// Use Teensy SDIO
//#define SD_CONFIG SdioConfig(BUILTIN_SDCARD)//new
#define LOG_FILE_SIZE 10*2500*600  // 1,500,000 bytes.   //Size to log 10 byte lines at 2.5 kHz for more than ten minutes.
#define RING_BUF_CAPACITY 400*512   // Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.
#define log_duration_msec 10000 //330000 total duration to auto-stop a run

uint32_t logTimeStamp = 1;
uint32_t i_samp=0;
uint32_t log_tzero_msec;
uint32_t logTime;
u_int32_t LOG_INTERVAL_USEC = 100000;  // Interval between points uSec
double sampHz; // sampHz = 1/(LOG_INTERVAL_USEC/fsConvert)   f(Hz) = 1 / T
double fsConvert = 1000000;   // usec to sec
SdFs sd;
FsFile file;
RingBuf<FsFile, RING_BUF_CAPACITY> rb; // RingBuf for File type FsFile.
// variables to define time spent waiting until a reresh of print LCD/serial
elapsedMillis EL_msec;
uint32_t delta = 0;
uint32_t start = 0;
elapsedMicros t_usec;
int pcount = 0;


#define ISNAN(XX) (!((XX)==(XX))) // todo what was this for...print floats on lcd?


/**********************************************************************/
/*!
// Check buttons and time-stamp the last press
*/
/**********************************************************************/
uint8_t ReadButtons()
{
  uint8_t buttons = lcd.readButtons();
  if (buttons != 0){
    refreshNeeded = true;}
  return buttons;
}

/**********************************************************************/
/*!
// Read floating point values from EEPROM
  */
  /**********************************************************************/
double EEPROM_readDouble(int address)
{
   double value = 0.0;
   byte* p = (byte*)(void*)&value;
   for (unsigned int i = 0; i < sizeof(value); i++)   {
      *p++ = EEPROM.read(address++);}
   return value;
}

/**********************************************************************/
/*!
// Write floating point values to EEPROM
*/
/**********************************************************************/
void EEPROM_writeDouble(int address, double value)
{
   byte* p = (byte*)(void*)&value;
   for (unsigned int i = 0; i < (sizeof(value)); i++) {
      EEPROM.write(address++, *p++);}
}

/**********************************************************************/
/*!
// Save any parameter changes to EEPROM
*/
/**********************************************************************/
void SaveParameters()
{
   if (SubjectID != EEPROM.read(SubjectIDAddress)){
      EEPROM.write(SubjectIDAddress, SubjectID);}
   if (Session != EEPROM.read(SessionAddress)){
      EEPROM.write(SessionAddress, Session);}
   if (Run != EEPROM.read(RunAddress)){
      EEPROM.update(RunAddress, Run);}
   if (TIME != EEPROM_readDouble(TIMEAddress)){
      EEPROM_writeDouble(TIMEAddress, TIME);}
   if (DATE != EEPROM.read(DATEAddress)){
      EEPROM_writeDouble(DATEAddress, DATE);}
}

/**********************************************************************/
/*!
// Load parameters from EEPROM
*/
/**********************************************************************/
void LoadParameters()
{
  // Load from EEPROM
   SubjectID = EEPROM.read(SubjectIDAddress);
   Session = EEPROM.read(SessionAddress);
   Run = EEPROM.read(RunAddress);
   DATE = EEPROM_readDouble(DATEAddress);
   TIME = EEPROM_readDouble(TIMEAddress);
   // Use defaults if EEPROM values are invalid
   if (isnan(SubjectID)){
     SubjectID = 1;}
   if (isnan(Session)){
     Session = 1;}
   if (isnan(Run)){
     Run = 1;}
   if (isnan(DATE)){
     DATE = 0;}
   if (isnan(TIME)){
     TIME = 0;}

   SubjectID = int(SubjectID);
   SID = SubjectID;
   Session = int(Session);
   SES = Session;
   Run = int(Run);
   RUN = Run;

   DATE = int(DATE);
   TIME = int(TIME);
}

/**********************************************************************/
/*!
// Set Time RTC
*/
/**********************************************************************/
time_t getTeensy3Time() {  return Teensy3Clock.get();}

/**********************************************************************/
/*!
// Set Time more RTC
*/
/**********************************************************************/
void dateTime(uint16_t* date, uint16_t* time) {
  // after your rtc is set up and working you code just needs:
  //DateTime now = now();
  *date = FS_DATE(year(), month(), day());// return date using FAT_DATE macro to format fields
  *time = FS_TIME(hour(), minute(), second());// return time using FAT_TIME macro to format fields
  // https://forum.arduino.cc/t/file-creation-date-and-time-in-sd-card/336037/12
}

/**********************************************************************/
/*!
// Print date/time on LCD
*/
/**********************************************************************/
void printTIMELCD() {
   //enum operatingState { TEST = 0,   OFF,   SET_DATE,   SET_SPEED,   SET_SUB,   SET_SES,   SET_RUN,   SET_VERIFY,   WAIT_TRIGGER,   LOG_DATA,  ERROR_INFO };
   snprintf(tLong,sizeof(tLong),"%02d/%02d/%02d  %02d:%02d:%02d", month(), day(), year(), hour(), minute(), second());
   snprintf(tShort,sizeof(tShort),"%02d:%02d:%02d", hour(), minute(), second());
   switch (opState)
   {
      case OFF:
         lcd.setCursor(0, 0);   lcd.print(tLong);
         break;
      case WAIT_TRIGGER:
         lcd.setCursor(0, 0);   lcd.print(tLong);
         break;
      case LOG_DATA:
         lcd.setCursor(12,0);   lcd.print(tShort);
         break;
      case SET_DATE:
         lcd.setCursor(0, 1);   lcd.print(tLong);
         break;
      case SET_VERIFY:
         lcd.setCursor(0, 0);   lcd.print(tLong);
         break;
      default:
         lcd.setCursor(12,0);   lcd.print(tShort);   lcd.setCursor(19,3);
         break;
   }
}

/**********************************************************************/
//Print angle of left and right ankle
/**********************************************************************/
void printAngles(uint16_t LEFT, uint16_t RIGHT) {
   uint8_t  cx  = 0;
   uint8_t  ry  = 0;
   uint16_t VAL = 0;
   uint8_t  jj  = 0;
   lcd.setCursor(0, 2);  lcd.print("L:");
   lcd.setCursor(0, 3);  lcd.print("R:");
   for (jj = 1; jj <=2 ; jj++)
   {
      if ( jj==1 )     {cx = 2;ry = 2;VAL = LEFT; }
      else if (jj==2)  {cx = 2;ry = 3;VAL = RIGHT;}
      lcd.setCursor(cx, ry);
      if      (VAL<10)              {lcd.print(F("   ")); }
      else if (VAL>10  && VAL<100)  {lcd.print(F("  ")); }
      else if (VAL>100 && VAL<1000) {lcd.print(F(" ")); } 
      lcd.print(VAL);
   }
}




void printDigitLCD(uint8_t cx, uint8_t ry, uint32_t VAL, uint8_t padding) {
   const String pad_vector[] = {
   " ",
   "  ",
   "   ",
   "    ",
   "     ",
   "      "};
   if (cx >19 || ry >3 ) { lcd.clear();  lcd.setCursor(0, 0); lcd.print(F("LIMIT c<=20 & r<=3 ")); }
   else if (padding!=0)
   {
      lcd.setCursor(cx, ry);
      lcd.print(pad_vector[padding]);
      lcd.print(VAL);
   }
   else
   {
      lcd.setCursor(cx, ry);
      if      (VAL<10)                      {lcd.print(F("   "));}
      else if (VAL>=10      && VAL<100)     {lcd.print(F("  "));}
      else if (VAL>=100     && VAL<1000)    {lcd.print(F(" "));}
      //else if (VAL>=1000    && VAL<10000)   {lcd.print(F("   "));}
      //else if (VAL>=10000   && VAL<100000)  {lcd.print(F("  "));}
      //else if (VAL>=100000  && VAL<1000000) {lcd.print(F(" "));}
      //else if (VAL>=1000000)                {           }     
      lcd.print(VAL);
      lcd.print(" ");
   }
   //lcd.noAutoscroll();
}


/**********************************************************************/
/*!
  @brief       TEST - Initial State
  @returns     RIGHT to OFF
*/
/**********************************************************************/
void Test()
{
   EL_msec = 0;
   start = t_usec;
   uint8_t buttons = 0;
   while(true)// && (num>x))//*!(buttons & (BUTTON_RIGHT)) )
   {
      buttons = ReadButtons();
      if (buttons & BUTTON_SELECT)
      {
         pcount = 0;
         unsigned long pulseDuration = 0;
         //EL_msec = 0;
         lcd.setBacklight(RED);
         lcd.setCursor(0, 1);  lcd.print(" pulses   duration U");
         while(pcount < 9)
         {
            while( (digitalReadFast(TRIGGER_INPUT_PIN)==LOW) )
            {
               if (EL_msec > PRINT_DELAY)
               {
                  outString = F("LOW____  pcount:") + String(pcount) + F("  start:") + String(start) + F("  delta:") + String(delta) + F("  pulseDur:") + String(delta-start) + F("  EL_msec") + String(EL_msec) + '\n';
                  lcd.setCursor(5, 2);  lcd.print(pcount); lcd.print("    ");  lcd.print(delta-start);  lcd.print(" ");
                  printTIMELCD();
                  EL_msec = 0;
                  if (pcount > 0)
                  {  SerialUSB1.print(outString);   }
               }
            }
            pulseDuration = pulseIn(TRIGGER_INPUT_PIN, HIGH);
            start = t_usec;
            //pulseIn()
            while ( (digitalReadFast(TRIGGER_INPUT_PIN)==HIGH) )
            {
               //  lcd.setBacklight(GREEN);
               //  lcd.setCursor(0,3);  lcd.print(F("HIGH                "));
            }
            delta = t_usec;
            //outString = F("____HIGH  pcount:") + String(pcount) + F("  start:") + String(start) + F("  delta:") + String(delta) + F("  pulseDur:") + String(delta-start) + F("  EL_msec") + String(EL_msec) + '\n';
            outString = F("pulseIn: ") + String(pulseDuration) + "  DONE" + '\n';
            SerialUSB1.print(outString);
            pcount += 1;
         }
         lcd.setBacklight(BLUE);
         lcd.setCursor(5, 2);  lcd.print(pcount); lcd.print("    ");  lcd.print(delta-start);  lcd.print(" ");
         lcd.setCursor(0, 3);  lcd.print(outString);
         SerialUSB1.println("line 357 - - I am outside the pcount loop");
      }
      if (buttons & BUTTON_RIGHT){
         opState = OFF;
         return;
      }
      if (buttons & BUTTON_DOWN){
         lcd.clear();
      }
      if (buttons & BUTTON_LEFT){
         lcd.clear();
         lcd.setBacklight(RED);
         lcd.setCursor(0, 1);  lcd.print("AVG:"); 
         lcd.setCursor(0, 2);  lcd.print("L:");
         lcd.setCursor(0, 3);  lcd.print("R:");
         lcd.setCursor(6, 2);  lcd.print((char)symDegree);
         lcd.setCursor(6, 3);  lcd.print((char)symDegree);
         while (ReadButtons()==0)
         {
            adcdeg310 = analogRead(MR310_IN);  deg310 = adcdeg310*360/1023;
            adcdeg430 = analogRead(MR430_IN);  deg430 = adcdeg430*360/1023;      
            opVar = ( (deg310 + deg430) / 2);
            printAngles(deg310, deg430);
            printDigitLCD(5, 1, opVar, 0); 
            outString = opString[opState] + F("---") + opVarStr[opState] + String(opVar) + F("   L:") + String(deg310) + F("  R:") + String(deg430) + F("  t_usec:") + String(t_usec) + '\n';
            SerialUSB1.print(outString);
            delay(2*bounciness);
         }
      }

   }
}





/**********************************************************************/
/*!
// OFF - Initial State
// RIGHT to SET_DATE
*/
/**********************************************************************/
void Off()
{
   // make sure outputs are off
   digitalWrite(MR310_RST, LOW);
   digitalWrite(MR430_RST, LOW);
   lcd.setCursor(0, 0);  lcd.print(F("BNC cable, SD card &"));
   lcd.setCursor(0, 1);  lcd.print(F(" BOTH Fiber Optics  "));
   lcd.setCursor(0, 2);  lcd.print(F(" All plugged in ???"));
   lcd.setCursor(19,3);
   delay(MENU_DELAY/4);
   lcd.setCursor(0, 3);  lcd.print(F("Enter Setup Press >"));
   lcd.setCursor(19,3);

   uint8_t buttons = 0;
   while(true)
   {
      buttons = ReadButtons();
      
      if (buttons & BUTTON_LEFT) {
         opState = TEST;
         return;
         }
      if (buttons & BUTTON_RIGHT) {
         opState = SET_DATE;
         return;
         }
   }
}


/**********************************************************************/
/*!
// SET_DATE
// RIGHT - SPEED
// LEFT - OFF
// SHIFT for 10x tuning
*/
/**********************************************************************/
void SetDate()
{
   lcd.setCursor(0, 3);  lcd.print("Date: ");  lcd.print(DATE);
   lcd.setCursor(18,3);  lcd.print("<>");
   
   uint8_t buttons = 0;
   while(true){
      buttons = ReadButtons();
      increment = 1;
      if (buttons & BUTTON_SELECT) {
        increment *= 10;}
      if (buttons & BUTTON_LEFT) {
         opState = OFF;
         return;}
      if (buttons & BUTTON_RIGHT) {
         opState = SET_SPEED;
         return;}
      if (buttons & BUTTON_UP) {
         DATE += increment;
         delay(bounciness);
         }
      if (buttons & BUTTON_DOWN) {
         DATE -= increment;
         delay(bounciness);
         }
      
      opVar = DATE; 
      printTIMELCD();
      if (buttons)
      {
         lcd.setCursor(0, 3);  lcd.print("Date: ");  lcd.print(DATE);
         lcd.setCursor(18,3);  lcd.print("<>");
         outString = opString[opState] + F("---") + opVarStr[opState] + String(opVar) + '\n';
         SerialUSB1.print(outString);
      }
      
   }
}


/**********************************************************************/
/*!
// SET_SPEED
// LEFT - DATE
// RIGHT - SET_SUB
// SHIFT for 10x tuning
*/
/**********************************************************************/
void SetSpeed()
{

   lcd.setCursor(0, 2);  lcd.print("   uSec    (Hz)  ");
   lcd.setCursor(0, 3);  lcd.print("Fs: ");
   lcd.setCursor(18,3);  lcd.print("<>");

   EL_msec = 0;
   uint8_t buttons = 0;
   
   while(true)
   {
      buttons = ReadButtons();
      increment = 10000;

      if (buttons & BUTTON_SELECT) {
        increment /= 10;}
      if (buttons & BUTTON_LEFT) {
         opState = SET_DATE;
         return;}
      if (buttons & BUTTON_RIGHT) {
         opState = SET_SUB;
         return;}
      if (buttons & BUTTON_UP) {
         LOG_INTERVAL_USEC += increment;
         delay(bounciness);
         }
      if (buttons & BUTTON_DOWN) {
         LOG_INTERVAL_USEC -= increment;
         delay(bounciness);
         }

      double sampHz = (1/(LOG_INTERVAL_USEC/fsConvert));
      opVar = sampHz;
      if ((buttons) || (EL_msec > PRINT_DELAY))
      {
         printDigitLCD(10, 3, opVar, 0);
         printDigitLCD(4, 3, LOG_INTERVAL_USEC, 0);
         printTIMELCD();
         outString = opString[opState] + F("---") + opVarStr[opState] + String(opVar) + '\n';
         SerialUSB1.print(outString);
         EL_msec = 0;
      }
   }
}


/**********************************************************************/
/*!
// SET_SUB State
// UP/DOWN to change SubjectID
// RIGHT - SESSION
// LEFT - off
*/
/**********************************************************************/
void SetSub()
{
   lcd.setCursor(0, 1);  lcd.write(symUp);  lcd.write(symDown);  lcd.print(F(" to Change Values"));
   lcd.setCursor(0, 2);  lcd.print(F("<> to Navigate Menu"));
   lcd.setCursor(0, 3);  lcd.print(F("sub: "));
   
   EL_msec = 0;
   uint8_t buttons = 0;
   while(true)
   {
      buttons = ReadButtons();
      increment = 1;
      if (buttons & BUTTON_SELECT) {
         increment *= 10;
         }
      if (buttons & BUTTON_LEFT) {
         opState = SET_SPEED;
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

      if ( SubjectID < 1 )
         { SubjectID = 1; }
      SID = SubjectID;
      opVar = SID;
      if ( (buttons) || (EL_msec > PRINT_DELAY) )
      {
         printTIMELCD();
         lcd.setCursor(5, 3);  lcd.print(SID);  lcd.print(" ");
         lcd.setCursor(18,3);  lcd.print("<>");
         lcd.setCursor(19, 3);
         outString = opString[opState] + F("---") + opVarStr[opState] + String(opVar) + '\n';
         SerialUSB1.print(outString);
         EL_msec = 0;
      }
   }
}


/**********************************************************************/
/*!
// SET_SES opState
// UP/DOWN to change Session
// RIGHT - DATE
// LEFT - subject
*/
/**********************************************************************/
void SetSes()
{
   lcd.setCursor(0, 1);  lcd.print("1=PRE  2=POST");
   lcd.setCursor(0, 3);  lcd.print("ses: ");  lcd.print(SES);
   lcd.setCursor(18, 3); lcd.print("<>");

   EL_msec = 0;
   uint8_t buttons = 0;
   while(true)
   {
      buttons = ReadButtons();
      increment = 1;
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

      if ( (Session <1) || (Session >9) )
         {  Session = 1;}
      SES = Session;
      opVar = SES;

      if ( (buttons) || (EL_msec > PRINT_DELAY) )
      {
         lcd.setCursor(5, 3);  lcd.print(SES);  lcd.print(" ");
         //lcd.setCursor(18, 3); lcd.print("<>");
         printTIMELCD();
         outString = opString[opState] + "---" + opVarStr[opState] + String(opVar) + '\n';
         SerialUSB1.print(outString);
         EL_msec = 0;
      }
      
   }
}


/**********************************************************************/
/*!
// SET_RUN opState
// UP/DOWN to change RUN #
// RIGHT - DATE
// LEFT - subject
*/
/**********************************************************************/
void SetRun()
{
   lcd.setCursor(0, 1);  lcd.print("1=Run1  2=Run2");
   lcd.setCursor(0, 2);  lcd.print(">=3: scanner problem");
   lcd.setCursor(0, 3);  lcd.print("run: ");  lcd.print(RUN);
   lcd.setCursor(18,3);  lcd.print("<>");

   uint8_t buttons = 0;
   while(true)
   {
      
      buttons = ReadButtons();
      increment = 1;
      if (buttons & BUTTON_SELECT) {
         increment *= 10;}
      if (buttons & BUTTON_LEFT) {
         opState = SET_SES;
         return;}
      if (buttons & BUTTON_RIGHT) {
         opState = SET_VERIFY;
         return;}
      if (buttons & BUTTON_UP) {
         Run += increment;
         delay(bounciness);
         }
      if (buttons & BUTTON_DOWN) {
         Run -= increment;
         delay(bounciness);
         }

      if ( Run <1 || Run >9)
         { Run = 1;}
      RUN = Run;
      opVar = RUN;

      if ( (buttons) || (EL_msec > PRINT_DELAY) )
      {
         lcd.setCursor(5, 3);  lcd.print(RUN);  lcd.print(" ");
         printTIMELCD();
         outString = opString[opState] + F("---") + opVarStr[opState] + String(opVar) + '\n';
         SerialUSB1.print(outString);
         EL_msec = 0;
      }
   }
}


/**********************************************************************/
/*!
// SET_VERIFY
// LEFT - SET_RUN
// SHIFT+RIGHT - WAIT_TRIGGER
*/
/**********************************************************************/
void SetVerify()
{
   //double sampHz = (1/(LOG_INTERVAL_USEC/fsConvert));
   snprintf(MM_BASE,sizeof(MM_BASE),"sub-%03d_ses-%01d_run-%01d", SID, SES, RUN);
   lcd.setCursor(0, 1);  lcd.print(MM_BASE);
   lcd.setCursor(0, 2);  lcd.print(F("EVERYTHING CORRECT ?"));
   lcd.setCursor(19,3);
   //lcd.setCursor(0, 2);  lcd.print("Fs:");  lcd.print(LOG_INTERVAL_USEC); lcd.print(" "); lcd.print("("); lcd.print(sampHz); lcd.print("Hz");lcd.print(")");
   delay(MENU_DELAY);
   lcd.setCursor(0, 3);  lcd.print("< back         OK +>");

   uint8_t buttons = 0;
   while(true)
   {
      buttons = ReadButtons();
      increment = 0;
      if ( (buttons & BUTTON_SELECT)){
         //&& (buttons & BUTTON_RIGHT) ) {
             opState = WAIT_TRIGGER;
             paramsSaved = 0;
             return;}
      if (buttons & BUTTON_LEFT) {
         opState = SET_RUN;
         paramsSaved = 0;
         return;}
      // if (buttons & BUTTON_RIGHT) {
      //   opState = WAIT_TRIGGER;
      //   return;}
      if (buttons & BUTTON_UP) {
         //opVar += increment;
         delay(bounciness);}
      if (buttons & BUTTON_DOWN) {
         //opVar -= increment;
         delay(bounciness);}

      if (EL_msec > PRINT_DELAY)
      {
         //lcd.setCursor(0, 3);  lcd.print("Fs_opVar");  lcd.print(opVar);
         printTIMELCD();
         outString = opString[opState] + F("---") + opVarStr[opState] + String(opVar) + '\n';
         SerialUSB1.print(outString);
         EL_msec = 0;
      }
   }
}


/**********************************************************************/
/*!
// WAIT_TRIGGER State
// LEFT to go back to SET_VERIFY
// SHIFT and RIGHT to FORCE LOGGING
*/
/**********************************************************************/
void WaitTrigger()
{
   snprintf(MM_BASE,sizeof(MM_BASE),"sub-%03d_ses-%01d_run-%01d", SID, SES, RUN);
   snprintf(MM,sizeof(MM),"sub-%03d_ses-%01d_run-%01d.csv", SID, SES, RUN);
   LOG_FILENAME = MM;
   lcd.setCursor(0, 1);  lcd.print(MM_BASE);
   //lcd.setCursor(0, 2);  lcd.print("RUN: ");  lcd.print(RUN);  lcd.print(" ");//todo
   lcd.setCursor(0, 2);  lcd.print(F("Initializing SD card"));
   lcd.setCursor(19,2);
 
  // Initialize the SD.
  if (!sd.begin(SD_CONFIG)) {
     errorCondition = "SD InitErrorHalt";
     errorValue = 948;
     lcd.clear();
     lcd.setCursor(0,1);  lcd.print(errorCondition);  lcd.setCursor(0,2);  lcd.print(errorValue);
     opState = ERROR_INFO;    errorState = true;
     // sd.initErrorHalt();
     delay(MENU_DELAY);
     return;
   }
  // create file   OR   open&truncate existing file.
  // todo prevent overwrite / enable browse/delete
  if (!file.open(LOG_FILENAME, O_RDWR | O_CREAT | O_TRUNC)) {
      errorCondition = "SD FailedToOpen file";
      errorValue = 960;
      lcd.clear();
      lcd.setCursor(0,1);  lcd.print(errorCondition);  lcd.setCursor(0,2);  lcd.print(errorValue);
      opState = ERROR_INFO;    errorState = true;
      delay(MENU_DELAY);
      return;
   }
  // File must be pre-allocated to avoid huge delays searching for free clusters.
  if (!file.preAllocate(LOG_FILE_SIZE)) {
     errorCondition = "SD preAllocate Fail";
     errorValue = 970;
     lcd.clear();
     lcd.setCursor(0,1);  lcd.print(errorCondition);  lcd.setCursor(0,2);  lcd.print(errorValue);
     file.close();
     opState = ERROR_INFO;    errorState = true;
     delay(MENU_DELAY);
     return;
   }

   digitalWrite(MR310_RST, HIGH);
   digitalWrite(MR430_RST, HIGH);
   delay(2*MENU_DELAY);
   digitalWrite(MR430_RST, LOW);
   digitalWrite(MR310_RST, LOW);
   delay(bounciness);
   adcdeg310 = analogRead(MR310_IN);  deg310 = adcdeg310*360/1023;
   adcdeg430 = analogRead(MR430_IN);  deg430 = adcdeg430*360/1023;

   file.println(tLong);
   file.println(MM_BASE);
   file.print(F("MR310 (Left-Orange) reset point: "));  file.println(deg310);
   file.print(F("MR430 (Right-Black) reset point: "));  file.println(deg430);
   file.println("");

   String stringHeader[] = {"Sample","logTime-ms","Left-Orange","Right-Black"};
   for(unsigned int jj = 0; jj<=3; jj++) {
      //data[jj] = analogRead( A0 + i);
      file.print(stringHeader[jj]);
      if (jj == 3){
         file.println("");}
      else {
         file.print(",");}
   }
   //const char *stringHeader[] = {"Sample","logTime-ms","Left-Orange","Right-Black"};
   //file.print(stringHeader[0:3]);  file.write(',');  file.print("logTime-ms");  file.write(',');  file.print("Left-Orange");  file.write(',');  file.println("Right-Black");



   if (paramsSaved == 0) 
   {   
      lcd.setCursor(0, 2);  lcd.print(F("Saving your Settings"));
      lcd.setCursor(19, 3);
      SaveParameters();
      paramsSaved = 1;
   }
   delay(MENU_DELAY);

 
   lcd.setCursor(0, 2);  lcd.print(F("Waiting for Trigger "));
   lcd.setCursor(0, 3);  lcd.print(F("TO FORCE:  hold 'OK'"));  lcd.setCursor(19,2);
   
   EL_msec = 0;
   uint8_t buttons = 0;
   while( (digitalReadFast(TRIGGER_INPUT_PIN)==LOW) )
   {
      if (EL_msec > PRINT_DELAY)
      {  
         buttons = ReadButtons();
         if (buttons & BUTTON_SELECT) {// Force Logging
            //&& (buttons & BUTTON_RIGHT) ) {
               opState = LOG_DATA;
               return;}
         if (buttons & BUTTON_LEFT) {
           opState = SET_VERIFY;
           return;}
         printTIMELCD();
         EL_msec = 0;
      }
   }
   opState = LOG_DATA;
}


/**********************************************************************/
/*!
// LOG_DATA State
// DATA LOGGING
*/
/**********************************************************************/
void LogData()
{
   //lcd.setCursor(0, 0);  lcd.print(F("Logging"));  lcd.print(" ");
   lcd.setCursor(0, 1);  lcd.print(MM_BASE);
   lcd.setCursor(7, 2);  lcd.print((char)symDegree);
   lcd.setCursor(7, 3);  lcd.print((char)symDegree);
   lcd.setCursor(12,3);  lcd.print("t :");
   printAngles(deg310, deg430);
   //lcd.setCursor(0, 0);  lcd.print("t0:"); lcd.print(log_tzero_msec/1000); lcd.print(" sec");

   
   // initialize the RingBuf.
   rb.begin(&file);
   // Max RingBuf used bytes. Useful to understand RingBuf overrun.
   size_t maxUsed = 0;
   // Min spare micros in loop.
   int32_t minSpareMicros = INT32_MAX;
   // Start time.
   log_tzero_msec = millis();
   logTime = micros();
   i_samp = 0;
   
   //              while (!SerialUSB1.available() || (logTimeStamp < (log_tzero_msec + log_duration_msec)) )
   // todo try elapsedMicros
   while ( logTimeStamp < log_duration_msec )// Log data until duration over
   {
      // Amount of data in ringBuf.
      size_t n = rb.bytesUsed();
      if ((n + file.curPosition()) > (LOG_FILE_SIZE - 20)) { //check if file full.
         errorCondition = "File full - EXITING ";
         errorValue = 257;
         opState = ERROR_INFO;    errorState = true;
         RUN += 1;
         break;//todo break or return?
       }
      if (n > maxUsed) {
         maxUsed = n;
      }
      if (n >= 512 && !file.isBusy()) {
         // Not busy only allows one sector before possible busy wait.
         // Write one sector from RingBuf to file.
         if (512 != rb.writeOut(512)) {
            errorCondition = "RB writeOut Failed";
            errorValue = 273;
            opState = ERROR_INFO;    errorState = true;
            RUN += 1;
            return;//rb
         }
      }

      // Time for next point.
      logTime += LOG_INTERVAL_USEC;
      int32_t spareMicros = logTime - micros();
      logTimeStamp = ( (logTime/1000) - log_tzero_msec );
      i_samp++;
      if (spareMicros < minSpareMicros) {
           minSpareMicros = spareMicros;
      }
      if (spareMicros <= 0) {
           errorCondition = "spareMicros Negative";
           errorValue = spareMicros;
           outString = opString[opState] + F("---") + opVarStr[opState] + errorCondition + "-" + String(errorValue) + '\n';
           SerialUSB1.println(outString);
           opState = ERROR_INFO;    errorState = true;
           RUN += 1;
           return;// todo? write to rb??
      }


      // SLEEP until time to log data.
      uint8_t buttons = 0;
      while ( micros() < (logTime-40000) )//todo elapseMicros instead of a call to micros
      {
         //EL_msec = 0;
         buttons = ReadButtons();
         if ((buttons & BUTTON_SELECT)) {
            //&& (buttons & BUTTON_LEFT))
              // Force QUIT & Write any RingBuf data to file.
               rb.print(i_samp); rb.write(','); rb.print(logTimeStamp); rb.write(','); rb.print(deg310); rb.write(','); rb.println(deg430); rb.sync();
               file.truncate();  file.rewind(); file.close();
               errorCondition = "     FORCE QUIT     ";
               errorValue = 965;
               RUN += 1;
               outString = opString[opState] + F("---") + opVarStr[opState] + errorCondition + String(errorValue) + '\n';
               SerialUSB1.println(outString);
               opState = ERROR_INFO;    errorState = true;
               return;
            }
         //if (EL_msec > 10)
         //{ //todo
            printAngles(deg310, deg430);
            // // lcd.setCursor(2, 1);  lcd.print(adcdeg310);   lcd.print("  ");
            // // lcd.setCursor(13,1);  lcd.print(adcdeg430);   lcd.print("  ");
            lcd.setCursor(15,3);  lcd.print(logTimeStamp/1000);
            printTIMELCD();
            //lcd.setCursor(0, 3);  lcd.print(spareMicros); lcd.print("  ");
            EL_msec = 0;
         //}
         //end sleep while loop
      }


      // Read ADC0 - about 17 usec on Teensy 4, Teensy 3.6 is faster.
      adcdeg310 = analogRead(MR310_IN);  deg310 = adcdeg310*360/1023;
      adcdeg430 = analogRead(MR430_IN);  deg430 = adcdeg430*360/1023;
      // Print adc into RingBuf.// convert usec to seconds
      rb.print(i_samp); rb.write(','); rb.print(logTimeStamp); rb.write(','); rb.print(deg310); rb.write(','); rb.println(deg430);
      if (rb.getWriteError()) {
         // Error caused by too few free bytes in RingBuf.
         errorCondition = " Buffer  WriteError ";
         errorValue = 973;
         RUN += 1;
         outString = opString[opState] + F("---") + opVarStr[opState] + errorCondition + String(errorValue) + '\n';
           SerialUSB1.println(outString);
         opState = ERROR_INFO;    errorState = true;
         return;
      }
   //end log duration loop
   } 




  // Write any RingBuf data to file.
  rb.sync();
  file.truncate();  file.rewind();  file.close();
  // Print first twenty lines of file.
  SerialUSB1.println(errorCondition);
  for (uint8_t n = 0; n < 20 && file.available();) {
    int c = file.read();
    if (c < 0) {
      break;
    }
    SerialUSB1.write(c);
    if (c == '\n') n++;
  }
  file.close();

  lcd.clear();
  lcd.setBacklight(BLUE);
  lcd.setCursor(0, 0);  lcd.print("fileSize:maxByteUsed");
  lcd.setCursor(0, 1);  lcd.print((uint32_t)file.fileSize());  lcd.print("       ");  lcd.print(maxUsed);  lcd.print(" ");
  lcd.setCursor(0, 2);  lcd.print("RUN ");  lcd.print(RUN);  lcd.print(":  ");  lcd.print(logTimeStamp/1000);  lcd.print(" seconds");
  lcd.setCursor(0, 3);  lcd.print("minSM:");  lcd.print(minSpareMicros);
  delay(4*MENU_DELAY);
  errorCondition = " (:   ! DONE !   :) ";
  SerialUSB1.println(String(MM_BASE));
  SerialUSB1.println(errorCondition);
  //todo errorVar
  errorValue = (logTimeStamp/1000);
  RUN += 1;
  logTime = 0;
  log_tzero_msec = millis();
  opState = ERROR_INFO;    errorState = false;
}


/**********************************************************************/
/*!
// ERROR_INFO State
// UP/DOWN - N/A
// RIGHT - xxx
// LEFT - SET_VERIFY
*/
/**********************************************************************/
void ErrorInfo()
{
   lcd.setBacklight(opColor[opState]);
   if (!errorState) {lcd.setBacklight(BLUE);}
   lcd.setCursor(0, 0);  lcd.print("LastRun:"); lcd.print(RUN-1); lcd.print(" t:"); lcd.print(logTimeStamp/1000); lcd.print(" secs");
   lcd.setCursor(0, 1);  lcd.print(errorCondition);
   lcd.setCursor(0, 2);  lcd.print(errorValue);
   lcd.setCursor(0, 3);  lcd.print("Start Over ?       <");
   lcd.setCursor(19,3);
   delay(MENU_DELAY);
   lcd.setCursor(0, 1);  lcd.print(MM_BASE);
   lcd.setCursor(19, 3);
   
   logTimeStamp = 0;


   uint8_t buttons = 0;
   while(true)
   {
      buttons = ReadButtons();
      if ((buttons & BUTTON_SELECT)){
         //&& (buttons & BUTTON_RIGHT)) {
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
         delay(bounciness);
         }
      if (buttons & BUTTON_DOWN) {
         delay(bounciness);
         }
   }
}










/**********************************************************************/
/*!
// Setup and display initial screen
*/
/**********************************************************************/
void setup()
{
   SerialUSB1.begin(9600);
   setSyncProvider(getTeensy3Time);
   //put this next line *Right Before* any file open line: //todo check timestamps
   SdFile::dateTimeCallback(dateTime);
   // Initialize LCD DiSplay
   lcd.begin(20, 4);
   lcd.createChar(1, byteUp); // create degree symbol from the binary
   lcd.createChar(2, byteDown);
   lcd.setBacklight(BLUE);
   lcd.blink();
   printTIMELCD();
   lcd.setCursor(0, 1);
   lcd.print(F("    RoseLab fMRI    "));
   lcd.setCursor(19, 3);
   // Initialize PINS:
   pinMode(MR310_IN, INPUT_PULLDOWN);
   pinMode(MR430_IN, INPUT_PULLDOWN);
   pinMode(TRIGGER_INPUT_PIN, INPUT);
   pinMode(MR310_RST, OUTPUT);
   pinMode(MR430_RST, OUTPUT);
   // make sure pins are off to start
   digitalWrite(MR310_RST, LOW);
   digitalWrite(MR430_RST, LOW);
   delay(2*MENU_DELAY);
   // Initialize the variables
   LoadParameters();
   paramsSaved = 0;
}







// put your main code here, to run repeatedly:
void loop()
{
   while(ReadButtons() != 0)
   {  }  // waits for button release before changing state
   outString = opString[opState] + F("---") + opVarStr[opState] + String(opVar) + '\n';
   SerialUSB1.print(outString);

   increment = 1;
   lcd.clear();
   lcd.setBacklight(opColor[opState]);
   lcd.setCursor(0, 0);
   lcd.print(opString[opState]);
   printTIMELCD();

   switch (opState)
   {
      case TEST:
         opVar = 0;
         Test();
         break;
      case OFF:
         opVar = 0;
         Off();
         break;
      case SET_DATE:
         opVar = DATE;
         SetDate();
         break;
      case SET_SPEED:
         opVar = LOG_INTERVAL_USEC;
         SetSpeed();
         break;
      case SET_SUB:
         opVar = SID;
         SetSub();
         break;
      case SET_SES:
         opVar = SES;
         SetSes();
         break;
      case SET_RUN:
         opVar = RUN;
         SetRun();
         break;
      case SET_VERIFY:
         opVar = LOG_INTERVAL_USEC;
         SetVerify();
         break;
      case WAIT_TRIGGER:
         opVar = 0;
         WaitTrigger();
         break;
      case LOG_DATA:
         log_tzero_msec = millis();
         LogData();
         log_tzero_msec = millis();
         break;
      case ERROR_INFO:
         lcd.setCursor(0, 0);  lcd.print(opString[opState]); 
         lcd.setCursor(0, 1);  lcd.print(F(MM_BASE));
         lcd.setCursor(0, 2);  lcd.print(errorValue);  
         lcd.setCursor(0, 3);  lcd.print(String(errorCondition));
         lcd.setCursor(19,3);
         delay(2*MENU_DELAY);
         ErrorInfo();
         break;
   }
}




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
