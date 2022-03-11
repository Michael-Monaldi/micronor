

#ifndef Micronor_Definitions_h
#define Micronor_Definitions_h
#endif

// Micronor fMRI Data Logger - Michael Monaldi
// #include <Arduino.h>
// Libraries for the Adafruit RGB/LCD Shield
#include "Adafruit_RGBLCDShield.h"
#include <Wire.h>
// So we can save and retrieve settings
#include <EEPROM.h>
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
#define RED 0x1
#define GREEN 0x2
#define YELLOW 0x3
#define BLUE 0x4
#define VIOLET 0x5
#define TEAL 0x6
#define WHITE 0x7
#define BUTTON_SHIFT BUTTON_SELECT
uint8_t buttons = 0;
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
String opVarStr[] = {" FIXME  ", "OFF=",  "DATE="   , "sampHz=",  "SID="        ,"SES="      ,"RUN="   ,"  LOGINTu "   ,"  FIXME  " , "  LOG_DATA "  ,    "ErrVar="};

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
short deg310_offset = 9999; //used to record the homed micronor output prior to experiment
short deg430_offset = 9999; //used to record the homed micronor output prior to experiment
boolean calibrated = false;
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
#define log_duration_msec 330000 //330000 total duration to auto-stop a run

uint32_t logTimeStamp = 1;
uint32_t i_samp=0;
uint32_t log_tzero_msec;
uint32_t logTime;
u_int32_t LOG_INTERVAL_USEC = 50000;  // Interval between points uSec
double sampHz; // sampHz = 1/(LOG_INTERVAL_USEC/fsConvert)   f(Hz) = 1 / T
int sampHz_int;
int32_t spareMicros;
double fsConvert = 1000000;   // usec to sec
#define fsADC 359/1023
SdFs sd;
FsFile file;
RingBuf<FsFile, RING_BUF_CAPACITY> rb; // RingBuf for File type FsFile.
// variables to define time spent waiting until a reresh of print LCD/serial
elapsedMillis EL_msec;
uint32_t delta = 0;
uint32_t start = 0;
elapsedMicros EL_usec;
elapsedMicros loop_EL_usec;
int pcount = 0;


#define ISNAN(XX) (!((XX)==(XX))) // todo what was this for...print floats on lcd?