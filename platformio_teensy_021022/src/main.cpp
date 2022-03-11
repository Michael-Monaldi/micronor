#include <Micronor_Definitions.h>


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
void printTimeLCD() {
   //enum operatingState { TEST = 0,   OFF,   SET_DATE,   SET_SPEED,   SET_SUB,   SET_SES,   SET_RUN,   SET_VERIFY,   WAIT_TRIGGER,   LOG_DATA,  ERROR_INFO };
   snprintf(tLong,sizeof(tLong),"%02d/%02d/%02d  %02d:%02d:%02d", month(), day(), year(), hour(), minute(), second());
   snprintf(tShort,sizeof(tShort),"%02d:%02d:%02d", hour(), minute(), second());
   switch (opState)
   {
      case OFF:
         lcd.setCursor(0, 0);   lcd.print(tLong);
         break;
      case WAIT_TRIGGER:
         lcd.setCursor(0, 0);   lcd.print(tLong);   lcd.setCursor(19,2);
         break;
      case LOG_DATA:
         lcd.setCursor(12,0);   lcd.print(tShort);
         break;
      case SET_DATE:
         lcd.setCursor(0, 1);   lcd.print(tLong);
         break;
      case SET_VERIFY:
         lcd.setCursor(0, 0);   lcd.print(tLong);   lcd.setCursor(19,2);
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
   start = EL_usec;
   buttons = 0;
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
                  printTimeLCD();
                  EL_msec = 0;
                  if (pcount > 0)
                  {  SerialUSB1.print(outString);   }
               }
            }
            pulseDuration = pulseIn(TRIGGER_INPUT_PIN, HIGH);
            start = EL_usec;
            //pulseIn()
            while ( (digitalReadFast(TRIGGER_INPUT_PIN)==HIGH) )
            {
               //  lcd.setBacklight(GREEN);
               //  lcd.setCursor(0,3);  lcd.print(F("HIGH                "));
            }
            delta = EL_usec;
            //outString = F("____HIGH  pcount:") + String(pcount) + F("  start:") + String(start) + F("  delta:") + String(delta) + F("  pulseDur:") + String(delta-start) + F("  EL_msec") + String(EL_msec) + '\n';
            outString = F("pulseIn: ") + String(pulseDuration) + "  DONE" + '\n';
            SerialUSB1.print(outString);
            pcount += 1;
         }
         lcd.setBacklight(BLUE);
         lcd.setCursor(5, 2);  lcd.print(pcount); lcd.print("    ");  lcd.print(delta-start);  lcd.print(" ");
         lcd.setCursor(0, 3);  lcd.print(outString);
         SerialUSB1.println("line 259- - I am outside the pcount loop");
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
            adcdeg310 = analogRead(MR310_IN);  deg310 = adcdeg310*fsADC;
            adcdeg430 = analogRead(MR430_IN);  deg430 = adcdeg430*fsADC;      
            opVar = ( (deg310 + deg430) / 2);
            printAngles(deg310, deg430);
            printDigitLCD(5, 1, opVar, 0); 
            outString = opString[opState] + F("---") + opVarStr[opState] + String(opVar) + F("   L:") + String(deg310) + F("  R:") + String(deg430) + F("  EL_usec:") + String(EL_usec) + '\n';
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
   lcd.setCursor(0, 2);  lcd.print(F(" All plugged in ?? "));
   delay(MENU_DELAY/4);
   lcd.setCursor(0, 3);  lcd.print(F("To calibrate hit OK"));
   delay(MENU_DELAY/4);

   buttons = 0;
   while(true)
   {
      buttons = ReadButtons();
      if (buttons & BUTTON_SELECT)
      {
         buttons = 0;
         lcd.clear();
         printTimeLCD();
         lcd.setCursor(0, 1);  lcd.print(F("CALIBRATING"));
         digitalWrite(MR310_RST, HIGH);
         digitalWrite(MR430_RST, HIGH);
         lcd.print(" . ");
         delay(PRINT_DELAY);
         lcd.print(". ");
         delay(PRINT_DELAY);
         lcd.print(". ");
         digitalWrite(MR430_RST, LOW);
         digitalWrite(MR310_RST, LOW);
         delay(bounciness);
         adcdeg310 = analogRead(MR310_IN);  deg310 = adcdeg310*fsADC;  deg310_offset = deg310;  
         adcdeg430 = analogRead(MR430_IN);  deg430 = adcdeg430*fsADC;  deg430_offset = deg430;
         
         if ( (deg310_offset<10) && (deg430_offset<10) ) {
               calibrated = true;
               printAngles(deg310_offset, deg430_offset);
               lcd.setCursor(6, 2);  lcd.print((char)symDegree);
               lcd.setCursor(6, 3);  lcd.print((char)symDegree);
               lcd.setBacklight(GREEN);
               lcd.setCursor(0, 1);  lcd.print(F("      Finished      "));
               while(ReadButtons()==0)
               {if (EL_msec > PRINT_DELAY) {printTimeLCD(); EL_msec = 0;}}
               lcd.setBacklight(opColor[opState]);
               lcd.setCursor(0, 1);  lcd.write(symUp);  lcd.write(symDown);  lcd.print(F(" to Change Values"));
               lcd.setCursor(0, 2);  lcd.print(F("<> to Navigate Menus"));
               lcd.setCursor(0, 3);  lcd.print(F("To Proceed Press   >"));
               delay(PRINT_DELAY);
            }
         else {
               calibrated = false;
               printAngles(deg310_offset, deg430_offset);
               lcd.setBacklight(RED);
               lcd.setCursor(0, 1);  lcd.print(F("CALIBRATION FAILED"));
               lcd.setCursor(6, 2);  lcd.print((char)symDegree);
               lcd.setCursor(6, 3);  lcd.print((char)symDegree);
               lcd.setCursor(7, 3);  lcd.print(F("Retry? Hit OK"));
               while(ReadButtons()==0)
               {if (EL_msec > PRINT_DELAY) {printTimeLCD(); EL_msec = 0;}}
               lcd.setBacklight(opColor[opState]);
            }
      }
      
      if (buttons & BUTTON_LEFT) {
         opState = TEST;
         return;
         }
      if (buttons & BUTTON_RIGHT) {
         opState = SET_DATE;
         return;
         }
      if ( (calibrated) && (EL_msec > PRINT_DELAY) ) {printTimeLCD(); EL_msec = 0;}
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
   outString = ((String)"Date: " + DATE + "        <>");
   lcd.setCursor(0, 3);  lcd.print(outString);
   SerialUSB1.println(outString);

   buttons = 0;
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
      
      if ( (buttons) || (EL_msec > PRINT_DELAY) )
      {
         printTimeLCD();
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
   buttons = 0;
   
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
         delay(bounciness);         }
      if (buttons & BUTTON_DOWN) {
         LOG_INTERVAL_USEC -= increment;
         delay(bounciness);         }

      double sampHz = (1/(LOG_INTERVAL_USEC/fsConvert));
      opVar = sampHz;
      sampHz_int = sampHz;
      
      if (buttons)
         {
            printDigitLCD(10, 3, opVar, 0);
            printDigitLCD(4, 3, LOG_INTERVAL_USEC, 0);
            outString = opString[opState] + F("---") + opVarStr[opState] + String(opVar) + '\n';
            SerialUSB1.print(outString);
         }
      if (EL_msec > PRINT_DELAY) {printTimeLCD();EL_msec = 0;  printDigitLCD(10, 3, opVar, 0);  printDigitLCD(4, 3, LOG_INTERVAL_USEC, 0);}
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
   lcd.setCursor(0, 3);  lcd.print(F("sub: "));
   
   EL_msec = 0;
   buttons = 0;
   while(true)
   {
      buttons = ReadButtons();
      increment = 1;
      if (buttons & BUTTON_SELECT) {
         increment *= 10;}
      if (buttons & BUTTON_LEFT) {
         opState = SET_SPEED;
         return;}
      if (buttons & BUTTON_RIGHT) {
         opState = SET_SES;
         return;}
      if (buttons & BUTTON_UP) {
         SubjectID += increment;
         delay(bounciness);}
      if (buttons & BUTTON_DOWN) {
         SubjectID -= increment;
         delay(bounciness);}

      if ( SubjectID < 1 )
         { SubjectID = 1; }
      SID = SubjectID;
      opVar = SID;

      if ( (buttons) || (EL_msec > PRINT_DELAY) )
      {
         printTimeLCD();
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
   buttons = 0;
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
         delay(bounciness);}
      if (buttons & BUTTON_DOWN) {
         Session -= increment;
         delay(bounciness);}

      if ( (Session <1) || (Session >9) )
         {  Session = 1;}
      SES = Session;
      opVar = SES;

      if ( (buttons) || (EL_msec > PRINT_DELAY) )
      {
         lcd.setCursor(5, 3);  lcd.print(SES);  lcd.print(" ");
         //lcd.setCursor(18, 3); lcd.print("<>");
         printTimeLCD();
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

   EL_msec = 0;
   buttons = 0;
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
         printTimeLCD();
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

   buttons = 0;
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
      //if (buttons & BUTTON_RIGHT) {
         //opState = WAIT_TRIGGER;
         //return;}
      if (buttons & BUTTON_UP) {
         // // opVar += increment;
         delay(bounciness);}
      if (buttons & BUTTON_DOWN) {
         // // opVar -= increment;
         delay(bounciness);}

      if (EL_msec > PRINT_DELAY)
      {
         //lcd.setCursor(0, 3);  lcd.print("Fs_opVar");  lcd.print(opVar);
         printTimeLCD();
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

      
   
   printTimeLCD();
   file.println(tLong);
   file.println(MM_BASE);
   file.print(F("MR310 (Left-Orange) reset point: "));  file.println(deg310_offset);
   file.print(F("MR430 (Right-Black) reset point: "));  file.println(deg430_offset);
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
      lcd.setCursor(0, 2);  lcd.print(F("Saving your setting "));
      lcd.setCursor(19, 3);
      SaveParameters();
      paramsSaved = 1;
   }
   delay(MENU_DELAY);

 
   
   lcd.setCursor(0, 3);  lcd.print(F("TO FORCE:  hold 'OK'"));
   lcd.setCursor(0, 2);  lcd.print(F("Waiting for Trigger"));

   EL_msec = 0;
   buttons = 0;
   while( (digitalReadFast(TRIGGER_INPUT_PIN)==LOW) )
   {
      if (EL_msec > MENU_DELAY)
      {  
         buttons = ReadButtons();
         if (buttons & BUTTON_SELECT) {// Force Logging
            //&& (buttons & BUTTON_RIGHT) ) {
            opState = LOG_DATA;
            return;}
         if (buttons & BUTTON_LEFT) {
           opState = SET_VERIFY;
           return;}
         printTimeLCD();
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
   lcd.setCursor(6, 2);  lcd.print((char)symDegree);
   lcd.setCursor(6, 3);  lcd.print((char)symDegree);
   lcd.setCursor(12,3);  lcd.print("t :");
   printAngles(deg310, deg430);
   //lcd.setCursor(0, 0);  lcd.print("t0:"); lcd.print(log_tzero_msec/1000); lcd.print(" sec");

   
   // initialize the RingBuf.
   rb.begin(&file);
   // Max RingBuf used bytes. Useful to understand RingBuf overrun.
   size_t maxUsed = 0;
   // Min spare micros in loop.
   int32_t minSpareMicros = INT32_MAX;
   //int32_t spareMicros;
   // Start time.
   log_tzero_msec = millis();
   logTime = micros();
   i_samp = 0;
   EL_usec = 0;
   loop_EL_usec = 0;
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
      //SerialUSB1.println("### TIME_RB " +String(loop_EL_usec) + " ###"); loop_EL_usec = 0;
      // Time for next point.
      logTime += LOG_INTERVAL_USEC;
      spareMicros = logTime - micros();
      logTimeStamp = ( (logTime/1000) - log_tzero_msec );
      i_samp++;

      if (spareMicros < minSpareMicros) {
           minSpareMicros = spareMicros;
      }
      if (spareMicros <= 0) {
           errorCondition = "spareMicrosNegative";
           errorValue = spareMicros;
           SerialUSB1.println((String)"LOG_INT_USEC" + "---" + "sampHz"  + "---" + "neg:spareMicros" + "---" + "minSpare"    + "---" + "logTime");
           outString =    ((String)LOG_INTERVAL_USEC +"  ---  "    +sampHz_int +"  ---  "    +spareMicros        +"  ---  "    +minSpareMicros +"  ---  "    +logTime    +'\n');
           //outString = opString[opState] + F("---") + opVarStr[opState] + errorCondition + "=" + String(errorValue) + F("   minSpare=") + minSpareMicros + '\n';
           SerialUSB1.print(outString);
           opState = ERROR_INFO;    errorState = true;
           RUN += 1;
           return;// todo? write to rb??
      }
      //SerialUSB1.println("### 861 TIME_CHECKING_SPARE " +String(loop_EL_usec) + " ###"); 
      


      // SLEEP until time to log data.
      
      //EL_msec = 0;
      buttons = 0;
      while ( micros() < logTime-(LOG_INTERVAL_USEC/2) )     //while ( micros() < (logTime-40000) )//todo elapseMicros instead of a call to micros
      {
         if ( loop_EL_usec < (LOG_INTERVAL_USEC/5) )
         {  SerialUSB1.println(String(i_samp) +"      I AM ASLEEP         " +String(loop_EL_usec));
            buttons = ReadButtons();
            
            // Force QUIT & Write any RingBuf data to file.
            if ((buttons & BUTTON_SELECT)) {
                  rb.print(i_samp); rb.write(','); rb.print(logTimeStamp); rb.write(','); rb.print(deg310); rb.write(','); rb.println(deg430); rb.sync();
                  file.truncate();  file.rewind(); file.close();
                  errorCondition = "     FORCE QUIT     ";
                  errorValue = minSpareMicros;
                  RUN += 1;
                  outString = opString[opState] + F("---") + opVarStr[opState] + errorCondition + String(errorValue) + '\n';
                  SerialUSB1.print(outString);
                  opState = ERROR_INFO;    errorState = true;
                  return;
               }
            if (EL_msec > MENU_DELAY) {
               printAngles(deg310, deg430);
               //lcd.setCursor(15,3);  lcd.print(logTimeStamp/1000);
               printTimeLCD();
               EL_msec = 0;
               }
            
         }
         //end sleep while loop
      }
      SerialUSB1.println("### TIME_SLEEPING ###   " +String(loop_EL_usec));


      loop_EL_usec = 0;
      // Read ADC0 - about 17 usec on Teensy 4, Teensy 3.6 is faster.
      adcdeg310 = analogRead(MR310_IN);  deg310 = adcdeg310*fsADC;
      adcdeg430 = analogRead(MR430_IN);  deg430 = adcdeg430*fsADC;
      // Print adc into RingBuf.// convert usec to seconds
      rb.print(i_samp); rb.write(','); rb.print(logTimeStamp); rb.write(','); rb.print(deg310); rb.write(','); rb.println(deg430);
      if (rb.getWriteError()) {
         // Error caused by too few free bytes in RingBuf.
         errorCondition = " Buffer  WriteError ";
         errorValue = 895;
         RUN += 1;
         outString = opString[opState] + F("---") + opVarStr[opState] + errorCondition + String(errorValue) + '\n';
         SerialUSB1.print(outString);
         opState = ERROR_INFO;   errorState = true;
         return;
      }
      //SerialUSB1.println("### 908 TIME_WRITING " +String(loop_EL_usec) + " ###");
   //end log duration loop
   }




   // Write any RingBuf data to file.
   rb.sync();
   file.truncate();  file.rewind();  file.close();
   // Print first twenty lines of file.
   file.open(LOG_FILENAME, O_RDONLY);
   for (uint8_t n = 0; n < 20 && file.available();) {
     int c = file.read();
     if (c < 0) {
       break;
     }
     SerialUSB1.write(c);
     if (c == '\n') n++;
   }
   file.close();
   SerialUSB1.println(errorCondition);

   SerialUSB1.println((String)"LOG_INT_USEC" + "---" + "sampHz" + "---" + "neg:spareMicros" + "---" + "minSpare"    + "---" + "logTime");
   outString =    ((String)LOG_INTERVAL_USEC +"  ---  "    +sampHz_int +"  ---  "    +spareMicros        +"  ---  "    +minSpareMicros +"  ---  "    +logTime    +'\n');
   //outString = opString[opState] + F("---") + opVarStr[opState] + errorCondition + "=" + String(errorValue) + F("   minSpare=") + minSpareMicros + '\n';
   SerialUSB1.print(outString);

   lcd.clear();
   lcd.setBacklight(BLUE);
   lcd.setCursor(0, 0);  lcd.print("fileSize:maxByteUsed");
   lcd.setCursor(0, 1);  lcd.print((uint32_t)file.fileSize());  lcd.print("       ");  lcd.print(maxUsed);  lcd.print(" ");
   lcd.setCursor(0, 2);  lcd.print("RUN ");  lcd.print(RUN);  lcd.print(":  ");  lcd.print(logTimeStamp/1000);  lcd.print(" seconds");
   lcd.setCursor(0, 3);  lcd.print("minSM:");  lcd.print(minSpareMicros);
   while (ReadButtons()==0){
   }
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
   lcd.setCursor(18,3);  lcd.print("<+");  //lcd.print("Start Over ?       <");
   lcd.setCursor(19,3);
   delay(MENU_DELAY);
   lcd.setCursor(0, 1);  lcd.print(MM_BASE);
   lcd.setCursor(19, 3);
   
   logTimeStamp = 0;


   buttons = 0;
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
   printTimeLCD();
   lcd.setCursor(0, 1);
   lcd.print(F("x   RoseLab fMRI   x"));
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
   lcd.setCursor(0, 0);  lcd.print(opString[opState]);
   printTimeLCD();

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
         opVar = LOG_INTERVAL_USEC;
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
