// Micronor fMRI Data Logger - Michael Monaldi

// #include <Arduino.h>
#ifndef Micronor_Definitions_h
#define Micronor_Definitions_h
#endif
#include "Adafruit_RGBLCDShield.h"      // Libraries for the Adafruit RGB/LCD Shield
#include <Wire.h>
#include <EEPROM.h>     // So we can save and retrieve settings
#include <TimeLib.h>
#include <SPI.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_I2CDevice.h>
#include <stdio.h>
#include <stdlib.h>
//#include <Micronor_Definitions.h>

// ************************************************
// Display Variables and constants
// ************************************************
// These #defines make it easy to set the backlight color
#define RED                     0x1
#define GREEN                   0x2
#define YELLOW                  0x3
#define BLUE                    0x4
#define VIOLET                  0x5
#define TEAL                    0x6
#define WHITE                   0x7
#define BUTTON_SHIFT BUTTON_SELECT
uint8_t buttons =               0;
int increment =                 1;
// //!< Select button #define BUTTON_SELECT  0x01
// //!< Right button  #define BUTTON_RIGHT   0x02
// //!< Down button   #define BUTTON_DOWN    0x04
// //!< Up button     #define BUTTON_UP      0x08
// //!< Left button   #define BUTTON_LEFT    0x10
// define the degree symbol
#define symDegree               223
//#define symUp                 94
//#define symDown               118
#define symLeft                 127
#define symRight                126
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
#define symUp                   1
#define symDown                 2
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

// ************************************************
// States for state machine
// ************************************************
enum operatingState  {TEST = 0,  OFF,    SET_DATE,   SET_SPEED,  SET_SUB,  SET_SES,  SET_RUN,  SET_VERIFY,  WAIT_TRIGGER,   LOG_DATA,     ERROR_INFO  };
unsigned int opColor[]={RED,    TEAL,    TEAL,      TEAL,         TEAL,      TEAL,      TEAL,      TEAL,       BLUE,          GREEN,            RED    };
String opString[] = {"TESTING!", "OFF", "SET_DATE", "SET_SPEED",  "SET_SUBJECT","SET_SESSION","SET_RUN","SET_VERIFY","WAIT_TRIGGER", "Recording",    "ERROR_INFO"};
String opVarStr[] = {" FIXME  ", "OFF=",  "DATE="   , "sampHz=",  "SID="        ,"SES="      ,"RUN="   ,"  LoguSec= "   ,"  WAITING? " , "  LOG_DATA "  ,    "ErrVar="};
operatingState opState = OFF;

double SubjectID =              1;
double Session =                1;
double Run =                    1;
double TIME =                   1;
double DATE =                   1;
int SID =                       1;
int SES =                       1;
int RUN =                       1;

// EEPROM addresses for persisted data
const int SubjectIDAddress =    0;
const int SessionAddress =      8;
const int RunAddress =          16;
const int TIMEAddress =         24;
const int DATEAddress =         32;

char sBuffer[100];
String outString = F("_^_^_^_^_^_^_^_^_^_^_^_");
int opVar =                     0;
int pulses =                    0;


// ************************************************
// Pin definitions
// ************************************************
#define MR310_RST               2 // pin - output relay for 24V signal to Micronor Controller homing input
#define MR430_RST               3 // pin - output relay for 24V signal to Micronor Controller homing input
#define MR310_IN                16 // pin - read ADC LEFT
#define MR430_IN                23 // pin - read ADC RIGHT
#define TRIGGER_INPUT_PIN       39 // Scanner pulse from BNC cable
uint16_t adcdeg310 =            1;
uint16_t adcdeg430 =            1;
short    deg310_offset =        999; //used to record the homed micronor output prior to experiment
short    deg430_offset =        999; //used to record the homed micronor output prior to experiment
short    deg310 =               1;
short    deg430 =               1;
boolean calibrated =            false;


// ************************************************
//Filenaming and lcd display vars
// ************************************************
#define MENU_DELAY              1500
#define PRINT_DELAY             800
#define bounciness              150
char* LOG_FILENAME =            (char*) malloc(100);
char MM_BASE[100];
char MM[100];
char tLong[40];
char tShort[40];
String errorCondition =         " All  is  well !!!  ";
uint32_t errorValue =           0;
boolean errorState =            false;
boolean paramsSaved =           false;
boolean refreshNeeded =         true;


// ************************************************
// SD Card Logging Setup
// ************************************************
//#include "SD.h"// new
#include "SdFat.h"
#include "RingBuf.h"
#define SD_CONFIG SdioConfig(FIFO_SDIO)// Use Teensy SDIO
//#define SD_CONFIG SdioConfig(BUILTIN_SDCARD)//new
#define LOG_FILE_SIZE           10*2500*600  // 1,500,000 bytes.   //Size to log 10 byte lines at 2.5 kHz for more than ten minutes.
#define RING_BUF_CAPACITY       400*512   // Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.
#define log_duration_msec       330000 //330000 total duration to auto-stop a run
#define fsADC                   359/1023
uint32_t logTimeStamp =         1;
uint32_t i_samp =               0;
uint32_t log_tzero_msec;
uint32_t logTime;
u_int32_t LOG_INTERVAL_USEC =   10000;  // Interval between points uSec
double sampHz; // sampHz = 1/(LOG_INTERVAL_USEC/fsConvert)   f(Hz) = 1 / T
int sampHz_int;
int32_t spareMicros;
double fsConvert =              1000000;   // usec to sec
SdFs sd;
FsFile file;
RingBuf<FsFile, RING_BUF_CAPACITY> rb; // RingBuf for File type FsFile.
// variables to define time spent waiting until a reresh of print LCD/serial
elapsedMillis EL_msec;
elapsedMicros EL_usec;
elapsedMicros loop_EL_usec;

// variables used only in TEST
int pcount =                    0;
uint32_t delta =                0;
uint32_t start =                0;
String newString;// = ((String) tShort +"L: " +deg310 + "R: " +deg430 +"  t:" +(logTimeStamp/1000));

#define ISNAN(XX) (!((XX)==(XX))) // special isnan function for comparing ints / floats / doubles I THINK - unused