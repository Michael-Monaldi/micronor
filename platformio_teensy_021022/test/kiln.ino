//V1.0
//This code is licenced using the MIT open source licence:
//Copyright (c)2012 Pat Clay, pat@wekadesign.com


#include <Wire.h>
#include <Adafruit_RGBLCDShield.h>
#include <utility/Adafruit_MCP23017.h>
// The shield uses the I2C SCL and SDA pins. On classic Arduinos
// this is Analog 4 and 5 so you can't use those for analogRead() anymore
// However, you can connect other I2C sensors to the I2C bus and share I2C bus
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
#define VIOLET 0x5

//#include <Time.h>
#include <TimeLib.h>
#include <Adafruit_MAX31855.h>
// // Initialize the MAX6675 Library for our chip
// // MAX6675 temp0(CS,SO,SCKpin,units,error);
#include <PID_v1.h>

//pin setup
byte LED1 = 2;               // Status LED Pin
byte kiln = 13;   //Pin for SSR
//byte kiln2 = 4 additional SSR for 240V
#define MAXDO   5
#define MAXCS   6
#define MAXCLK  7                // CS pin on MAX6675

// //define display pins
// #define upbutton 18
// #define downbutton 15
// #define rightbutton 17
// #define leftbutton 16
// #define enterbutton 14

//temp setup
float error = 0.0;        // Temperature compensation error
byte units = 1;            // Units to readout temp (0 = raw, 1 = ËC, 2 = ËF)
//Functions only support temps to 999 - may be an issue if F used
int temperature = 0.0;  // Temperature output variable
// Initialize the MAX31855 Library for our chip
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);


//pid setup
//Define PID Variables
double Setpoint, Input, Output;
//Specify the links and initial tuning parameters
//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPID(&Input, &Output, &Setpoint, 150, 1, 0,  DIRECT);
//SetSampleTime(1000); //Only sample every second
int WindowSize = 5000;  //Milliseconds for PWN for PID
unsigned long windowStartTime;

//button setup
byte x = 0;
byte y = 1;
//cursor position
int action = 0;
boolean enter = false;


boolean pausenow = false;
int pausetime = 200; //lower gives less lag on the buttons, but more bounce issues

int phase = -1;
boolean initialise = true;
boolean review = false;
boolean newscreen = true;
//counter to determine if you should initialise the step when entering
byte timeloc;

unsigned long refreshed;
byte pause = 1;
//timer to control refresh of the screen to every second

const char *phasename[] = {"Melt ", "Ramp ", "Burn ", "Drop ", "Hold "};
//char* phasename[]={"Melt ","Ramp ","Burn ","Drop ","Hold "};
//char* phasedesc[]={"Initial melt", "Ramp to hot over time", "Hold hot", "Drop to cool est", "Casting temp","Ready to cast"};
const char *phasedesc[] = {"Initial melt", "Ramp t over time", "Hold hot", "Drop to cool est", "Casting temp", "Ready to cast"};

unsigned int phasetemp[4] = {150, 730, 730, 600};
//4 phases of burnout phase, temp goals
unsigned int length[4] = {120, 300, 120, 15};
unsigned int casttemp[5] = {600, 0, 0, 0, 0};
unsigned int castlength[5] = {30, 0, 0, 0, 0};
unsigned int cumlength[4] = {0, 0, 0, 0};
unsigned int estduration = 0;
int castings = 3;
int castcounter = 0;
//length in mins of each phase
int maxtemp = 999;

unsigned long start = 0;
//what time we aim to/started the burncycle in seconds
byte kilnstate = 0;

unsigned long elapsed = 0;
//How many minutes we have been running the current cycle
unsigned int temptarget = 0;


void setup() {
  //pinMode(rightbutton, INPUT);
  //pinMode(leftbutton, INPUT);
  //pinMode(upbutton, INPUT);
  //pinMode(downbutton, INPUT);
  //pinMode(enterbutton, INPUT);
  setTime(19, 13, 30, 25, 2, 2022);
  pinMode(kiln, OUTPUT);
  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, LOW);
  digitalWrite(kiln, LOW); //lets turn the kiln off to start!

  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("MONALDI/RUSSELL");
  lcd.setCursor(0, 1);
  lcd.print("PID Kiln I2C");
  lcd.cursor();
  delay(1000);
  enter = true;
}


void loop() {
  if (pausenow) {
    delay (pausetime);
    pausenow = false;
  }
  translateButton();

  if (initialise) {
    digitalWrite(kiln, LOW); //lets turn the kiln off to start!
    if (enter == true) {
      if (phase == 6 and castcounter < castings - 1) {
        castcounter += 1;
      } else {
        phase += 1;
      }
      enter = false;
      newscreen = true;
      lcd.clear();
    }
    if (phase == 0) {
      if (newscreen) {
        newscreen = false;
        lcd.print("set time");
        lcd.setCursor(0, 1);
        printtime(now(), true);

        x = 0;
        y = 1;
      } else {
        lcd.setCursor(x, y);
      }

      if (action != 0) {
        changetime(true);
      } else {
        if (now() - refreshed > pause) {
          lcd.setCursor(0, 1);
          printtime(now(), true);
          lcd.setCursor(x, y);
          refreshed = now();
        }
      }
    } //end phase 0
    if (phase >= 1 && phase < 4) {
      if (newscreen) {
        newscreen = false;
        lcd.print(phasedesc[phase - 1]);
        x = 0;
        y = 1;
      } else {
        showphase(phase - 1);
        lcd.setCursor(x, y);
        if (action != 0) {
          changephase(phase - 1);
        }
      }
    }//end of profile setup
    if (phase == 4) {
      if (newscreen) {
        newscreen = false;
        lcd.print(phasedesc[3]);
        lcd.setCursor(0, 1);
        showhours(length[3]);
        x = 0;
        y = 1;
      } else {
        lcd.setCursor(0, 1);
        lcd.noCursor();
        showhours(length[3]);
        lcd.setCursor(x, y);
        lcd.cursor();
        if (action != 0) {
          changecool();
        }
      }
    }//end of cooldown est
    if (phase == 5) {
      if (newscreen) {
        newscreen = false;
        lcd.print("# of cast temps");
        lcd.setCursor(0, 1);
        lcd.print(castings);
        x = 0;
      } else {
        lcd.setCursor(0, 1);
        lcd.setCursor(x, y);
        if (action != 0) {
          changecastings();
          lcd.print(castings);
        }
      }
    }//end phase 5
    if (phase == 6) {
      if (newscreen) {
        newscreen = false;
        lcd.print(phasedesc[4]);
        lcd.print("#");
        lcd.print(castcounter + 1);
        lcd.setCursor(0, 1);
        if (castcounter > 0 and !review) {
          casttemp[castcounter] = casttemp[castcounter - 1];
          castlength[castcounter] = castlength[castcounter - 1];
        }
      } else {
        showphase(4);
        lcd.setCursor(x, y);
        if (action != 0) {
          changephase(4);
        }
      }
    }//end of casting temp initialisation
    if (phase == 7) {
      if (newscreen) {
        castcounter = 0;
        newscreen = false;
        lcd.print("Casting time");
        lcd.setCursor(0, 1);
        cumlength[0] = length[0];
        for (int i = 1; i < 4; i++) {
          cumlength[i] = length[i] + cumlength[i - 1];
        }
        estduration = cumlength[3] + castlength[0];
        start = max(now(), start);
      } else {
        lcd.setCursor(0, 1);
        changetarget();
        lcd.setCursor(x, y);
      }
    }//end set casting time
    if (phase == 9 || phase == 8) {
      if (newscreen) {
        lcd.setCursor(0, 0);
        newscreen = false;
        lcd.print("<start> <review>");
        if (phase == 8) {
          x = 1;
        }
      } else {
        lcd.setCursor(x, 0);
        if (phase == 9) {
          if (x >= 0 && x <= 7) {
            //we clicked on start
            phase += 1;
            newscreen = true;
          } else {
            //we clicked review
            review = true;
            phase = 1;
            newscreen = true;
            lcd.clear();
            enter = false;
          }
        }
      }
    }//end review phase
    if (phase == 10) {
      if (now() - refreshed > pause) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("time:");
        printtime(now(), false);
        lcd.print("start:");
        lcd.setCursor(0, 1);
        printtime(start, true);
        refreshed = now();
      } //waiting for time to start burnout sequence
      if (now() >= start) {
        lcd.noCursor();
        initialise = false;
        phase = 0;
        castcounter = 0;
        lcd.clear();
        //PID setup
        windowStartTime = millis();
        //tell the PID to range between 0 and the full window size
        myPID.SetOutputLimits(0, WindowSize);
        //turn the PID on
        myPID.SetMode(AUTOMATIC);
      } //start burnout
    }//end of countdown

  }//end of initialise if
  else { //burnout loop
    temperature = thermocouple.readCelsius();     // Read the temp 5 times and return the average value to the var
    elapsed = (now() - start) / 60;
    phase = calcphase(elapsed, phase);
    temptarget = calctarget(phase, elapsed);

    if (phase == 3) { //cooling phase

      if (temperature <= temptarget) {
        //we were in cooling phase and reached target
        phase = 4;
        start = now();
        elapsed = (now() - start) / 60;
        estduration = castlength[castcounter];
        newscreen = true;

      } else {
        //we are still cooling
        if (newscreen) {
          start = now();
          estduration = castlength[castcounter] + length[3];
          newscreen = false;
        }
        elapsed = (now() - start) / 60;
        if (estduration - elapsed < castlength[castcounter]) {
          //stops etc dropping to 0 if the kiln cools down slower than programmed
          estduration = castlength[castcounter] + elapsed;
        }
      }
    }

    if (phase == 5) {
      if (newscreen) {
        digitalWrite(LED1, HIGH);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Enter when done");
        newscreen = false;
      }
      if (!newscreen) {
        lcd.setCursor(0, 1);
        lcd.print("A");
        lcd.print(temperature );
        lcd.print(" Target:");
        lcd.print(temptarget );
        lcd.print((char)223);
      }

      if (enter == true) {
        digitalWrite(LED1, LOW);
        if (castcounter == castings - 1) {
          //we are done casting
          phase = 99;
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Casting Complete");
          initialise = true;
          digitalWrite(kiln, LOW);
        } else {
          castcounter += 1;
          phase = 3;
        }
        newscreen = true;
        enter = false;
      }
    }//end casting phase wait

    if (temperature <= 0 || temperature > maxtemp) {
      // If there is an error with the TC, temperature will be < 0
      //If the kiln is over temp, shut it down.
      lcd.clear();
      lcd.print("ERROR!");

      digitalWrite(LED1, HIGH);
      digitalWrite(kiln, LOW);
    } else {
      lcd.setCursor(0, 0);
      if (phase < 5) {
        lcd.print(phasename[phase]);
        lcd.print(temptarget );
        lcd.print((char)223);
        lcd.print(" ");
        if (phase < 3) {
          showhours(elapsed);
        } else {
          lcd.print("     ");
        }
        lcd.setCursor(0, 1);
        lcd.print("A");
        lcd.print(temperature);
        lcd.print((char)223);
        lcd.print("   ETC");
        showhours(estduration - elapsed);
      }

      Input = temperature;
      Setpoint = temptarget;
      myPID.Compute();

      /************************************************
          turn the output pin on/off based on pid output
        ************************************************/
      unsigned long now = millis();
      if (now - windowStartTime > WindowSize)
      { //time to shift the Relay Window
        windowStartTime += WindowSize;
      }
      if (Output > now - windowStartTime) {
        digitalWrite(kiln, HIGH);
      } else {
        digitalWrite(kiln, LOW);
      }
    }//end of ok temperature if

    delay(500);

  }//end burnout loop
}//end void loop


void changetime(boolean date) {
  int currday = day();
  int currmonth = month();
  int curryear = year();
  int currhour = hour();
  int currmin = minute();

  if (date) {
    switch (x) {
      case 0:
        currday += 10 * action;
        break;
      case 1:
        currday += action;;
        break;
      case 2:
        currmonth += action;
        break;
      case 3:
        currmonth += action;
        break;
      case 4:
        currmonth += action;
        break;
      case 7:
        curryear += 10 * action;
        break;
      case 8:
        curryear += action;
        break;
    }
  }
  switch (x) {
    case 10:
      currhour += 10 * action;
      break;
    case 11:
      currhour += action;
      break;
    case 13:
      currmin += 10 * action;
      break;
    case 14:
      currmin += action;
      break;
  }
  if (currmin <= 0) {
    currmin += 59;
    currhour -= 1;
  }
  if (currhour < 0) {
    currhour += 23;
    currday -= 1;
  }
  if (currday <= 0) {
    currday -= 30;
    currmonth -= 1;
  }
  if (currmonth == 0) {
    currmonth = 12;
    curryear -= 1;
  }

  setTime(currhour, currmin, second(), currday, currmonth, curryear);

  if (enter) {
    phase += 1;
    enter = false;
  }
  action = 0;
  lcd.setCursor(0, 1);
  printtime(now(), true);
}


void translateButton() {
  /*void translateButton() {
    bounceright.update ( );
    bounceleft.update ( );
    bounceup.update ( );
    bouncedown.update ( );
    bounceenter.update ( );
    // Get the update value
    int right = bounceright.read();
    int left = bounceleft.read();
    int up = bounceup.read();
    int down = bouncedown.read();
    int enterresult = bounceenter.read();
  */

  uint8_t buttons = lcd.readButtons();
  if (buttons) {

    if (buttons & BUTTON_UP) {
      action = 1;
      pausenow = true;
    } else {
      if (buttons & BUTTON_LEFT) {
        pausenow = true;
        if (x == 0) {
          x = 15;
        } else {
          x -= 1;
        }
      } else {
        if (buttons & BUTTON_RIGHT) {
          pausenow = true;
          if (x == 15) {
            x = 0;
          } else {
            x += 1;
          }
        } else {
          if (buttons & BUTTON_DOWN) {
            pausenow = true;
            action = -1;
          } else {
            if (buttons & BUTTON_SELECT) {
              pausenow = true;
              enter = true;
            } else {
              action = 0;
            }
          }
        }
      }
    }
  }
}


void printtime(unsigned long displaytime, boolean date) {
  if (date) {
    if (day(displaytime) < 10) {
      lcd.print("0");
    }
    lcd.print(day(displaytime));
    printmonth(month(displaytime));
    lcd.print(year(displaytime));
    lcd.print(" ");
  }
  if (hour(displaytime) < 10) {
    lcd.print("0");
  }
  lcd.print(hour(displaytime));
  lcd.print(":");

  if (minute(displaytime) < 10) {
    lcd.print("0");
  }
  lcd.print(minute(displaytime));
}


void printmonth(byte month) {
  switch (month) {
    case 1:
      lcd.print("Jan");
      break;
    case 2:
      lcd.print("Feb");
      break;
    case 3:
      lcd.print("Mar");
      break;
    case 4:
      lcd.print("Apr");
      break;
    case 5:
      lcd.print("May");
      break;
    case 6:
      lcd.print("Jun");
      break;
    case 7:
      lcd.print("Jul");
      break;
    case 8:
      lcd.print("Aug");
      break;
    case 9:
      lcd.print("Sep");
      break;
    case 10:
      lcd.print("Oct");
      break;
    case 11:
      lcd.print("Nov");
      break;
    case 12:
      lcd.print("Dec");
      break;
  }
}


void showphase(int phaseinfo) {
  unsigned int disptemp = 0;
  unsigned int displength = 0;
  if (phaseinfo == 4) {
    disptemp = casttemp[castcounter];
    displength = castlength[castcounter];
  } else {
    disptemp = phasetemp[phaseinfo];
    displength = length[phaseinfo];
  }

  lcd.setCursor(0, 1);
  lcd.print("t=");
  if (disptemp < 10) {
    lcd.print("00");
  } else {
    if (disptemp < 10) {
      lcd.print("0");
    }
  }
  lcd.print(disptemp);
  lcd.print("C Time");
  showhours(displength);
}


void showhours(int time) {
  if (time < 0) {
    time = 0;
  }
  if (time < 10 * 60) {
    lcd.print("0");
  } else {
    if (time < 60) {
      lcd.print("0");
    }
  }
  lcd.print(time / 60);
  lcd.print(":");
  if (time - (time / 60 * 60) < 10) {
    lcd.print("0");
  }
  lcd.print(time - time / 60 * 60);
}


void changephase(byte phaseinfo) {
  unsigned int disptemp = 0;
  unsigned int displength = 0;
  if (phaseinfo == 4) {
    disptemp = casttemp[castcounter];
    displength = castlength[castcounter];
  } else {
    disptemp = phasetemp[phaseinfo];
    displength = length[phaseinfo];
  }
  switch (x) {
    case 2:
      disptemp += 100 * action;
      break;
    case 3:
      disptemp += 10 * action;
      break;
    case 4:
      disptemp += 1 * action;
      break;
    case 11:
      displength += 600 * action;
      break;
    case 12:
      displength += 60 * action;
      break;
    case 14:
      displength += 10 * action;
      break;
    case 15:
      displength += 1 * action;
      break;
  }
  disptemp = checkinput(disptemp, maxtemp, 100);
  displength = checkinput(displength, maxtemp, 1);

  if (phaseinfo == 4) {
    if (disptemp > casttemp[castcounter - 1] && castcounter > 0) {
      disptemp = casttemp[castcounter];
    }
    casttemp[castcounter] = disptemp;
    castlength[castcounter] = displength;
  } else {
    phasetemp[phaseinfo] = disptemp;
    length[phaseinfo] = displength;
  }

}


void changecool() {
  switch (x) {
    case 0:
      length[3] += action * 10 * 60;
      break;
    case 1:
      length[3] += action * 60;
      break;
    case 3:
      length[3] += action * 10;
      break;
    case 4:
      length[3] += action;
      break;
  }
  length[3] = checkinput(length[3], 4 * 60, 1);
}


unsigned int checkinput (unsigned int input, unsigned int maximum, unsigned int minimum) {
  if (input < minimum || input > 65000) {
    input = minimum;
  }
  if (input > maximum) {
    input = maximum;
  }
  return input;

}


void changetarget() {
  switch (x) {
    case 1:
      start = start + action * 86400; //add 1 day to the time
      break;
    case 10:
      start = start + action * 36000; //add 10 hours to the time;
      break;
    case 11:
      start = start + action * 1 * 60 * 60; //add 1 hour to the time
      break;
    case 13:
      start = start + action * 10 * 60;
      break;
    case 14:
      start = start + action * 1 * 60;
      break;
  }
  start = max(now(), start);
  start = min(start, now() + 259200);
  //cannot program to start >3 days ahead
  printtime(estduration * 60 + start, true);

}


int calcphase(unsigned long elapsed, int phase) {
  /*
    0 melt
    1 ramp
    2 hold hot
    3 cool
    4 hold at casting temp
    5 ready to cast
  */

  if (phase < 3) {
    if (elapsed >= cumlength[2]) {
      phase = 3;
    } else {
      if (elapsed >= cumlength[1]) {
        phase = 2;
      } else {
        if (elapsed >= cumlength[0]) {
          phase = 1;
        } else {
          phase = 0;
        }
      }
    }
  } else {
    if (phase == 4 && elapsed >= castlength[castcounter]) {
      phase = 5;
    }
  }

  return phase;
}


unsigned int calctarget(byte phase, unsigned long elapsed) {
  int target = 0;
  switch (phase) {
    case 0:
      target = phasetemp[0];
      break;
    case 1:
      target = (phasetemp[1] - phasetemp[0]) / (double)length[1] * (elapsed - length[0]) + phasetemp[0];
      break;
    case 2:
      target = phasetemp[2];
      break;
    case 3:
      target = casttemp[castcounter];
      break;
    case 4:
      target = casttemp[castcounter];
      break;
    case 5:
      target = casttemp[castcounter];
      break;
    default:
      target = 0;
  }
  return target;
}


void changecastings() {
  castings += action;
  castings = checkinput(castings, 5, 1);
}
