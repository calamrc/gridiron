/*
  Gridiron.ino - An automated elevation measurement device.
  Copyright (c) 2021 Ramon Cristopher Calam.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "Wire.h" // For I2C
#include "LCD.h" // For LCD
#include "LiquidCrystal_I2C.h" // Added library*
#include "Math.h"
#include "Servo.h"
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X
#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>
#include <SPI.h>
#include <SD.h>

#define MINIMUM_DELAY 250

#define BUTTON_A 2
#define BUTTON_B 3
#define BUTTON_C 4
#define BUTTON_D 5

#define RESET_PIN 6
#define POWEROFF_PIN 7

#define SERVO_A 9
#define SERVO_B 10

#define START 0
#define UP 1
#define DOWN 2
#define END 3
#define LEFT 4
#define RIGHT 5

#define SERVO_A_0 0
#define SERVO_A_90 1
#define SERVO_B_0 2
#define SERVO_B_90 3

#define SERVO_A_MANUAL 0
#define SERVO_B_MANUAL 1

#define CALIBRATE_SCREEN_DISPLAY_NUMBER 4
#define SETTINGS_SCREEN_DISPLAY_NUMBER 4
#define MANUAL_CONTROL_SCREEN_DISPLAY_NUMBER 2
#define MAIN_SCREEN_DISPLAY_NUMBER 3

#define START_SCREEN 0
#define SETTINGS_SCREEN 1
#define CALIBRATE_SCREEN 2
#define MANUAL_CONTROL_SCREEN 3

#define GRID_POINTS 8
#define COUNTDOWN_TIMER 5

#define INFO_NUMBER 6
#define ELEVATION_INFO 0
#define SERVO_A_ANGLE_INFO 1
#define DISTANCE_INFO 2
#define X_INFO 3
#define Y_INFO 4
#define SERVO_B_ANGLE_INFO 5

#define V_MENU 0
#define H_MENU 1

#define INSTRUMENT_HEIGHT 0
#define BENCHMARK 1
#define GRID_AREA 2
#define SAMPLES 3

int lineIndex[2] = {0, 0};
double data[INFO_NUMBER][GRID_POINTS];
int variables[] = {
  1000,
  0,
  1000,
  3
};

double manualControlServoAngles[] = {
  90,
  90,
};

int servoDutyCycles[] = {
  570,
  1570,
  880,
  1900
};

Servo servoA;
Servo servoB;

SFEVL53L1X distanceSensor;
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7); // 0x27 is the default I2C bus address of the backpack-see article

void setup() {
  initLCD();
  initButtons();
  initSDCard();
  initServoMotors();
  initDistanceSensor();
  initVariables();

  /*printStatus("Init... done");*/
  lcd.clear();
  lcd.setCursor(0,3);
  lcd.print(F("Init... done"));
  delay(MINIMUM_DELAY);

  welcomeScreen();
}

void loop() {
  mainScreen();
}

void printStatus(char statusMessage[20]) {
  lcd.clear();
  lcd.setCursor(0,3);
  lcd.print(statusMessage);

  delay(MINIMUM_DELAY);
}

void initVariables() {
  File entry;

  lcd.clear();
  lcd.setCursor(0,3);
  lcd.print(F("Init variables..."));
  delay(MINIMUM_DELAY);

  if(SD.exists("sa0")) {
    char buf[10];
    entry = SD.open("sa0", FILE_READ);
    entry.read(buf, 10);
    entry.close();

    servoDutyCycles[SERVO_A_0] = atoi(buf);
  }

  if(SD.exists("sa90")) {
    char buf[10];
    entry = SD.open("sa90", FILE_READ);
    entry.read(buf, 10);
    entry.close();

    servoDutyCycles[SERVO_A_90] = atoi(buf);
  }

  if(SD.exists("sb0")) {
    char buf[10];
    entry = SD.open("sb0", FILE_READ);
    entry.read(buf, 10);
    entry.close();

    servoDutyCycles[SERVO_B_0] = atoi(buf);
  }

  if(SD.exists("sb90")) {
    char buf[10];
    entry = SD.open("sb90", FILE_READ);
    entry.read(buf, 10);
    entry.close();

    servoDutyCycles[SERVO_B_90] = atoi(buf);
  }

  if(SD.exists("ih")) {
    char buf[10];
    entry = SD.open("ih", FILE_READ);
    entry.read(buf, 10);
    entry.close();

    variables[INSTRUMENT_HEIGHT] = atoi(buf);
  }

  if(SD.exists("bm")) {
    char buf[10];
    entry = SD.open("bm", FILE_READ);
    entry.read(buf, 10);
    entry.close();

    variables[BENCHMARK] = atoi(buf);
  }

  if(SD.exists("ga")) {
    char buf[10];
    entry = SD.open("ga", FILE_READ);
    entry.read(buf, 10);
    entry.close();

    variables[GRID_AREA] = atoi(buf);
  }

  if(SD.exists("sp")) {
    char buf[10];
    entry = SD.open("sp", FILE_READ);
    entry.read(buf, 10);
    entry.close();

    variables[SAMPLES] = atoi(buf);
  }

  lcd.clear();
  lcd.setCursor(0,3);
  lcd.print(F("Init variables... ok"));
  delay(MINIMUM_DELAY);
}

void initLCD() {
  lcd.begin (20,4);
  lcd.setBacklightPin(3, POSITIVE);
  lcd.setBacklight(HIGH);

  delay(MINIMUM_DELAY);
}

void initButtons() {
  /*printStatus("Init buttons...");*/
  lcd.clear();
  lcd.setCursor(0,3);
  lcd.print(F("Init buttons..."));

  delay(MINIMUM_DELAY);

  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
  pinMode(BUTTON_D, INPUT_PULLUP);

  /*printStatus("Init buttons... ok");*/
  lcd.clear();
  lcd.setCursor(0,3);
  lcd.print(F("Init buttons... ok"));

  delay(MINIMUM_DELAY);
}

void initDistanceSensor() {
  /*printStatus("Init sensor...");*/
  lcd.clear();
  lcd.setCursor(0,3);
  lcd.print(F("Init sensor..."));

  delay(MINIMUM_DELAY);

  Wire.begin();

  if (distanceSensor.begin() != 0) { //Begin returns 0 on a good init
    /*printStatus("Init sensor... fail");*/
    lcd.clear();
    lcd.setCursor(0,3);
    lcd.print(F("Init sensor... fail"));

    delay(MINIMUM_DELAY);
    while (1);
  }

  distanceSensor.setDistanceModeLong();

  /*printStatus("Init sensor... ok");*/
  lcd.clear();
  lcd.setCursor(0,3);
  lcd.print(F("Init sensor... ok"));

  delay(MINIMUM_DELAY);
}

void initServoMotors() {
  int dutyCycle;

  /*printStatus("Init servo...");*/
  lcd.clear();
  lcd.setCursor(0,3);
  lcd.print(F("Init servo..."));

  delay(MINIMUM_DELAY);

  servoA.attach(SERVO_A);
  servoB.attach(SERVO_B);
  delay(1000);

  /*dutyCycle = map(90, 0, 90, servoDutyCycles[SERVO_B_0], servoDutyCycles[SERVO_B_90]);*/
  /*servoB.writeMicroseconds(dutyCycle);*/

  /*dutyCycle = map( 0, 0, 90, servoDutyCycles[SERVO_A_0], servoDutyCycles[SERVO_A_90]);*/
  /*servoA.writeMicroseconds(dutyCycle);*/
  /*delay(1000);*/

  dutyCycle = map( 0, 0, 90, servoDutyCycles[SERVO_B_0], servoDutyCycles[SERVO_B_90]);
  servoB.writeMicroseconds(dutyCycle);

  dutyCycle = map(90, 0, 90, servoDutyCycles[SERVO_A_0], servoDutyCycles[SERVO_A_90]);
  servoA.writeMicroseconds(dutyCycle);
  delay(1000);

  /*printStatus("Init servo... ok");*/
  lcd.clear();
  lcd.setCursor(0,3);
  lcd.print(F("Init servo... ok"));

  delay(MINIMUM_DELAY);
}

void initSDCard() {
  /*printStatus("Init SDCard...");*/
  lcd.clear();
  lcd.setCursor(0,3);
  lcd.print(F("Init SDCard..."));

  delay(MINIMUM_DELAY);

  if (!SD.begin(8)) {
    /*printStatus("Init SDCard... fail");*/
    lcd.clear();
    lcd.setCursor(0,3);
    lcd.print(F("Init SDCard... fail"));

    delay(MINIMUM_DELAY);
    while (1);
  }

  /*printStatus("Init SDCard... ok");*/
  lcd.clear();
  lcd.setCursor(0,3);
  lcd.print(F("Init SDCard... ok"));

  delay(MINIMUM_DELAY);
}

void updateMainScreen(int offset) {
  lcd.setCursor(6, 0);
  lcd.print(F("GRIDIRON"));

  if(offset == 0) {
    lcd.setCursor(1, 1);
    lcd.print(F("Start         "));
    lcd.setCursor(1, 2);
    lcd.print(F("Settings      "));
    lcd.setCursor(1, 3);
    lcd.print(F("Calibrate     "));
  }
  else {
    lcd.setCursor(1, 1);
    lcd.print(F("Settings      "));
    lcd.setCursor(1, 2);
    lcd.print(F("Calibrate     "));
    lcd.setCursor(1, 3);
    lcd.print(F("Manual Control"));
  }
}

void mainScreen() {
  int offset = 0;
  int index = 0;

  lcd.clear();
  offset = updateCursor(START, MAIN_SCREEN_DISPLAY_NUMBER, 1);
  updateMainScreen(offset);

  while(true) {
    int buttonsState = checkButtons();

    switch(buttonsState) {
      case 0b1110: // Up
        offset = updateCursor(UP, MAIN_SCREEN_DISPLAY_NUMBER, 1);
        updateMainScreen(offset);
        break;
      case 0b1101: // Down
        offset = updateCursor(DOWN, MAIN_SCREEN_DISPLAY_NUMBER, 1);
        updateMainScreen(offset);
        break;
      case 0b1011: // Ok
        if(index == 3) {
          lineIndex[V_MENU] = -1;
          index = 0;
        }
        if(lineIndex[V_MENU] == START_SCREEN) {
          startScreen();
        }
        else if(lineIndex[V_MENU] == SETTINGS_SCREEN) {
          settingsScreen();
        }
        else if(lineIndex[V_MENU] == CALIBRATE_SCREEN) {
          calibrateScreen();
        }
        else if(lineIndex[V_MENU] == MANUAL_CONTROL_SCREEN) {
          /*manualControlScreen();*/
        }
        else {
          creditsScreen();
        }

        lcd.clear();
        offset = updateCursor(START, MAIN_SCREEN_DISPLAY_NUMBER, 1);
        updateMainScreen(offset);

        break;
      case 0b0111: // Back
        if(index < 3) {
          index++;
          if(index == 3) {
            lcd.setCursor(19,0);
            lcd.print(F("!"));
          }
        }
        else index = 0;
        delay(MINIMUM_DELAY);

        lcd.setCursor(19,0);
        lcd.print(F(" "));
    }
  }
}

void creditsScreen() {
  /*const char upperCase[] = " ABCDEFGHIJKLMNOPQRSTUVWXYZ";*/
  /*const int creditsInformation[4][10] = {*/
    /*{18,  1, 13, 15, 14,  0,  0,  0,  0,  0},*/
    /*{ 3, 18,  9, 19, 20, 15, 16,  8,  5, 18},*/
    /*{13,  0,  0,  0,  0,  0,  0,  0,  0,  0},*/
    /*{ 3,  1, 12,  1, 13,  0,  0,  0,  0,  0}*/
  /*};*/

  /*lcd.clear();*/
  /*delay(500);*/

  /*for(int i=0; i<4; i++) {*/
    /*lcd.setCursor(0, i);*/
    /*for(int j=0; j<10; j++) {*/
      /*lcd.print(upperCase[creditsInformation[i][j]]);*/
      /*delay(MINIMUM_DELAY);*/
    /*}*/
  /*}*/
  /*delay(3000);*/

  /*lcd.clear();*/
  /*delay(500);*/
}

void welcomeScreen() {
    lcd.clear();
    delay(500);

    lcd.setCursor(6, 1);
    lcd.print(F("GRIDIRON"));

    delay(3000);

    lcd.clear();
    delay(500);

}

void initStartScreen() {
  for(int i=0; i<COUNTDOWN_TIMER; i++) {
    lcd.clear();
    lcd.setCursor(9, 1);
    lcd.print(COUNTDOWN_TIMER - i);
    delay(1000);
  }
}

void startScreen() {
  initStartScreen();
  calculateGridPoints();

  double dutyCycle;
  double angle;

  for(int i=0; i<GRID_POINTS; i++) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("Grid Point "));
    lcd.setCursor(11,0);
    lcd.print(i+1);
    lcd.setCursor(12,0);
    lcd.print(F(":"));
    lcd.setCursor(14,0);
    switch(i) {
      case 0:
        lcd.print(F("(0,1)"));
        break;
      case 1:
        lcd.print(F("(0,2)"));
        break;
      case 2:
        lcd.print(F("(1,2)"));
        break;
      case 3:
        lcd.print(F("(2,2)"));
        break;
      case 4:
        lcd.print(F("(1,1)"));
        break;
      case 5:
        lcd.print(F("(2,1)"));
        break;
      case 6:
        lcd.print(F("(2,0)"));
        break;
      case 7:
        lcd.print(F("(1,0)"));
        break;
    }

    dutyCycle = map( 0, 0, 90, servoDutyCycles[SERVO_B_0], servoDutyCycles[SERVO_B_90]);
    servoB.writeMicroseconds(dutyCycle);
    dutyCycle = map(90, 0, 90, servoDutyCycles[SERVO_A_0], servoDutyCycles[SERVO_A_90]);
    servoA.writeMicroseconds(dutyCycle);
    delay(3000);

    angle = data[SERVO_B_ANGLE_INFO][i] * RAD_TO_DEG;
    dutyCycle = map(angle, 0, 90, servoDutyCycles[SERVO_B_0], servoDutyCycles[SERVO_B_90]);
    servoB.writeMicroseconds(dutyCycle);

    angle = data[SERVO_A_ANGLE_INFO][i] * RAD_TO_DEG;
    dutyCycle = map(angle, 0, 90, servoDutyCycles[SERVO_A_0], servoDutyCycles[SERVO_A_90]);
    servoA.writeMicroseconds(dutyCycle);
    delay(5000);

    data[DISTANCE_INFO][i] = measureDistance();
    data[Y_INFO][i] = data[DISTANCE_INFO][i] * sin(data[SERVO_A_ANGLE_INFO][i]);
    data[ELEVATION_INFO][i] = variables[INSTRUMENT_HEIGHT] + variables[BENCHMARK] - data[Y_INFO][i];

    lcd.setCursor(0, 1);
    lcd.print(F("Elevation (mm):"));
    lcd.setCursor(0, 2);
    lcd.print(data[ELEVATION_INFO][i]);

    delay(2000);
  }

  dutyCycle = map( 0, 0, 90, servoDutyCycles[SERVO_B_0], servoDutyCycles[SERVO_B_90]);
  servoB.writeMicroseconds(dutyCycle);

  dutyCycle = map(90, 0, 90, servoDutyCycles[SERVO_A_0], servoDutyCycles[SERVO_A_90]);
  servoA.writeMicroseconds(dutyCycle);

  elevationSummaryScreen();
}

void updateElevationSummaryScreen(int offset, int infoIndex) {
  switch(infoIndex) {
    case ELEVATION_INFO:
      lcd.setCursor(0, 0);
      lcd.print(F(" ELEVATION SUMMARY  "));
      break;
    case DISTANCE_INFO:
      lcd.setCursor(0, 0);
      lcd.print(F("  DISTANCE SUMMARY  "));
      break;
    case SERVO_A_ANGLE_INFO:
      lcd.setCursor(0, 0);
      lcd.print(F(" HOR. ANGLE SUMMARY "));
      break;
    case X_INFO:
      lcd.setCursor(0, 0);
      lcd.print(F("      X SUMMARY     "));
      break;
    case Y_INFO:
      lcd.setCursor(0, 0);
      lcd.print(F("      Y SUMMARY     "));
      break;
    default:
      lcd.setCursor(0, 0);
      lcd.print(F(" ELEVATION SUMMARY  "));
  }

  for(int i=0; i<3; i++) {
    lcd.setCursor(1, i+1);
    lcd.print(F("                   "));
    lcd.setCursor(1, i+1);
    lcd.print(i+1+offset);
    lcd.setCursor(3, i+1);
    if(infoIndex == SERVO_A_ANGLE_INFO) {
      lcd.print(data[infoIndex][i+offset] * RAD_TO_DEG);
    }
    else {
      lcd.print(data[infoIndex][i+offset]);
    }
    lcd.setCursor(17, i+1);
    /*lcd.print(dataUnit[infoIndex]);*/
    switch(infoIndex) {
      case 1:
        lcd.print("deg");
        break;
      default:
        lcd.print("mm");
    }
  }
}

void elevationSummaryScreen() {
  int offset = 0;

  lcd.clear();

  offset = updateCursor(START, GRID_POINTS, 5);
  updateElevationSummaryScreen(offset, lineIndex[H_MENU]);

  while(true) {
    int buttonsState = checkButtons();

    switch(buttonsState) {
      case 0b1110: // Up
        offset = updateCursor(UP, GRID_POINTS, 5);
        updateElevationSummaryScreen(offset, lineIndex[H_MENU]);
        break;
      case 0b1101: // Down
        offset = updateCursor(DOWN, GRID_POINTS, 5);
        updateElevationSummaryScreen(offset, lineIndex[H_MENU]);
        break;
      case 0b1011: // Ok
        offset = updateCursor(RIGHT, GRID_POINTS, 5);
        updateElevationSummaryScreen(offset, lineIndex[H_MENU]);
        break;
      case 0b0111:
        /*if(SD.exists("data.csv")) {*/
          /*SD.remove("data.csv");*/
        /*}*/

        File elevationData = SD.open("data.csv", FILE_WRITE);

        if(elevationData) {
          lcd.clear();
          lcd.setCursor(0, 3);
          lcd.print(F("Saving data..."));

          delay(MINIMUM_DELAY);

          elevationData.println(F("***start of data***"));

          for(int i=0; i<INFO_NUMBER; i++) {
            switch(i) {
              case ELEVATION_INFO:
                elevationData.print(F("Elevation (mm), "));
                break;
              case SERVO_A_ANGLE_INFO:
                elevationData.print(F("ServoA Angle (rad), "));
                break;
              case DISTANCE_INFO:
                elevationData.print(F("Distance (mm), "));
                break;
              case X_INFO:
                elevationData.print(F("X (mm), "));
                break;
              case Y_INFO:
                elevationData.print(F("Y (mm), "));
                break;
              case SERVO_B_ANGLE_INFO:
                elevationData.print(F("ServoB Angle (rad)"));
                break;
            }
          }

          elevationData.println();

          for(int i=0; i<GRID_POINTS; i++) {
            for(int j=0; j<INFO_NUMBER; j++) {
              elevationData.print(data[j][i]);

              if(j < INFO_NUMBER - 1) {
                elevationData.print(F(", "));
              }
            }
            elevationData.println();
          }

          elevationData.println();
          elevationData.close();

          lcd.clear();
          lcd.setCursor(0,3);
          lcd.print(F("Saving data... done"));

          delay(MINIMUM_DELAY);
        }
        else {
          lcd.clear();
          lcd.setCursor(0,3);
          lcd.print(F("Saving data... fail"));

          delay(MINIMUM_DELAY);
        }

        return;
    }

  }
}

int updateCursor(const int dir, int numberOfOptionsLeft, int numberOfOptionsRight) {
  if(dir == UP) {
    if(lineIndex[V_MENU] < numberOfOptionsLeft - 1) lineIndex[V_MENU]++;
    else lineIndex[V_MENU] = 0;
  }
  else if(dir == DOWN) {
    if(lineIndex[V_MENU] > 0) lineIndex[V_MENU]--;
    else lineIndex[V_MENU] = numberOfOptionsLeft - 1;
  }
  else if(dir == RIGHT) {
    if(lineIndex[H_MENU] < numberOfOptionsRight - 1) lineIndex[H_MENU]++;
    else lineIndex[H_MENU] = 0;
  }
  else if(dir == LEFT) {
    if(lineIndex[H_MENU] > 0) lineIndex[H_MENU]--;
    else lineIndex[H_MENU] = numberOfOptionsRight - 1;
  }
  else {
    lineIndex[V_MENU] = 0;
    lineIndex[H_MENU] = 0;
  }

  int maxOptions = 3;
  int indexDiff = lineIndex[V_MENU] - 2;
  if(indexDiff < 0) indexDiff = 0;

  if(numberOfOptionsLeft < 3) maxOptions = numberOfOptionsLeft;

  for(int i=0; i<maxOptions; i++) {
    int tempCursor;

    if(indexDiff > 0) tempCursor = 2;
    else tempCursor = lineIndex[V_MENU];

    if(i == tempCursor) {
      lcd.setCursor(0, i+1);
      lcd.print(F(">"));
    }
    else {
      lcd.setCursor(0, i+1);
      lcd.print(F(" "));
    }
  }

  delay(MINIMUM_DELAY);

  return indexDiff;

}

void manualControlServo() {
  double angle;
  double dutyCycle;

  switch(lineIndex[V_MENU]) {
    case SERVO_A_MANUAL:
      angle = manualControlServoAngles[SERVO_A_MANUAL];
      dutyCycle = map(angle, 0, 90, servoDutyCycles[SERVO_A_0], servoDutyCycles[SERVO_A_90]);
      servoA.writeMicroseconds(dutyCycle);
      break;
    case SERVO_B_MANUAL:
      angle = manualControlServoAngles[SERVO_B_MANUAL];
      dutyCycle = map(angle, 0, 90, servoDutyCycles[SERVO_B_0], servoDutyCycles[SERVO_B_90]);
      servoB.writeMicroseconds(dutyCycle);
      break;
  }
  lcd.setCursor(0, 3);
  lcd.print(F("Distance (mm): "));
  lcd.setCursor(15, 3);
  lcd.print(measureDistance());
}

void updateManualControlScreen(int offset) {
  lcd.setCursor(0, 0);
  lcd.print(F("Manual Control"));

  lcd.setCursor(1, 1);
  lcd.print(F("ServoA"));

  lcd.setCursor(1, 2);
  lcd.print(F("ServoB"));
}

void manualControlScreen() {
  int offset = 0;
  int increment = 1;

  lcd.clear();

  offset = updateCursor(START, MANUAL_CONTROL_SCREEN_DISPLAY_NUMBER, 2);
  updateManualControlScreen(offset);

  while(true) {
    int buttonsState = checkButtons();

    switch(buttonsState) {
      case 0b1110: // Up
        if(lineIndex[H_MENU] == 0) {
          offset = updateCursor(UP, MANUAL_CONTROL_SCREEN_DISPLAY_NUMBER, 2);
          updateManualControlScreen(offset);
        }
        else {
          manualControlServoAngles[lineIndex[V_MENU]] -= increment;

          if(manualControlServoAngles[lineIndex[V_MENU]] > 90) {
            manualControlServoAngles[lineIndex[V_MENU]] = 90;
          }

          if(offset > 0) {
            lcd.setCursor(1, 3);
            lcd.print(F("                   "));
            lcd.setCursor(1, 3);
            lcd.print(manualControlServoAngles[lineIndex[V_MENU]]);
            lcd.setCursor(17, 3);
            lcd.print(F("deg"));
          }
          else {
            lcd.setCursor(1, lineIndex[V_MENU]+1);
            lcd.print(F("                   "));
            lcd.setCursor(1, lineIndex[V_MENU]+1);
            lcd.print(manualControlServoAngles[lineIndex[V_MENU]]);
            lcd.setCursor(17, lineIndex[V_MENU]+1);
            lcd.print(F("deg"));
          }

          manualControlServo();

        }
        delay(MINIMUM_DELAY);
        break;
      case 0b1101: // Down
        if(lineIndex[H_MENU] == 0) {
          offset = updateCursor(DOWN, MANUAL_CONTROL_SCREEN_DISPLAY_NUMBER, 2);
          updateManualControlScreen(offset);
        }
        else {
          manualControlServoAngles[lineIndex[V_MENU]] += increment;

          if(manualControlServoAngles[lineIndex[V_MENU]] < 0) {
            manualControlServoAngles[lineIndex[V_MENU]] = 0;
          }

          if(offset > 0) {
            lcd.setCursor(1, 3);
            lcd.print(F("                   "));
            lcd.setCursor(1, 3);
            lcd.print(manualControlServoAngles[lineIndex[V_MENU]]);
            lcd.setCursor(17, 3);
            lcd.print(F("deg"));
          }
          else {
            lcd.setCursor(1, lineIndex[V_MENU]+1);
            lcd.print(F("                   "));
            lcd.setCursor(1, lineIndex[V_MENU]+1);
            lcd.print(manualControlServoAngles[lineIndex[V_MENU]]);
            lcd.setCursor(17, lineIndex[V_MENU]+1);
            lcd.print(F("deg"));
          }

          manualControlServo();

        }
        delay(MINIMUM_DELAY);
        break;
      case 0b1011: // Ok
        offset = updateCursor(RIGHT, MANUAL_CONTROL_SCREEN_DISPLAY_NUMBER, 2);

        if(offset > 0) {
          lcd.setCursor(1, 3);
          lcd.print(F("                   "));
          lcd.setCursor(1, 3);

          if(lineIndex[H_MENU] == 0) {
            /*lcd.print(manualControlScreenDisplay[lineIndex[V_MENU]]);*/
            if(lineIndex[V_MENU] == 0) {
              lcd.print(F("ServoA"));
            }
            else {
              lcd.print(F("ServoB"));
            }
          }
          else {
            lcd.print(manualControlServoAngles[lineIndex[V_MENU]]);
            lcd.setCursor(17, 3);
            lcd.print(F("deg"));
            manualControlServo();
          }
        }
        else {
          lcd.setCursor(1, lineIndex[V_MENU]+1);
          lcd.print(F("                   "));
          lcd.setCursor(1, lineIndex[V_MENU]+1);

          if(lineIndex[H_MENU] == 0) {
            /*lcd.print(manualControlScreenDisplay[lineIndex[V_MENU]]);*/
            if(lineIndex[V_MENU] == 0) {
              lcd.print(F("ServoA"));
            }
            else {
              lcd.print(F("ServoB"));
            }
          }
          else {
            lcd.print(manualControlServoAngles[lineIndex[V_MENU]]);
            lcd.setCursor(17, lineIndex[V_MENU]+1);
            lcd.print(F("deg"));
            manualControlServo();
          }
        }

        delay(MINIMUM_DELAY);

        break;
      case 0b0111: // NA
        if(lineIndex[H_MENU] == 0) {
          return;
        }
        else {
          increment *= 10;
          if(increment == 100) increment = 1;
          delay(MINIMUM_DELAY);
        }
        break;
      default:
        break;
    }
  }
}

void calibrateServo() {
  switch(lineIndex[V_MENU]) {
    case SERVO_A_0:
      servoA.writeMicroseconds(servoDutyCycles[SERVO_A_0]);
      break;
    case SERVO_A_90:
      servoA.writeMicroseconds(servoDutyCycles[SERVO_A_90]);
      break;
    case SERVO_B_0:
      servoB.writeMicroseconds(servoDutyCycles[SERVO_B_0]);
      break;
    case SERVO_B_90:
      servoB.writeMicroseconds(servoDutyCycles[SERVO_B_90]);
      break;
  }
}

void updateCalibrateScreen(int offset) {
  lcd.setCursor(0, 0);
  lcd.print(F("Calibrate"));

  if(offset == 0) {
    lcd.setCursor(1, 1);
    lcd.print(F("ServoA -  0"));
    lcd.setCursor(1, 2);
    lcd.print(F("ServoA - 90"));
    lcd.setCursor(1, 3);
    lcd.print(F("ServoB -  0"));
  }
  else {
    lcd.setCursor(1, 1);
    lcd.print(F("ServoA - 90"));
    lcd.setCursor(1, 2);
    lcd.print(F("ServoB -  0"));
    lcd.setCursor(1, 3);
    lcd.print(F("ServoB - 90"));
  }
}

void calibrateScreen() {
  File entry;
  int offset = 0;
  int increment = 1;

  lcd.clear();

  offset = updateCursor(START, CALIBRATE_SCREEN_DISPLAY_NUMBER, 2);
  updateCalibrateScreen(offset);

  while(true) {
    int buttonsState = checkButtons();

    switch(buttonsState) {
      case 0b1110: // Up
        if(lineIndex[H_MENU] == 0) {
          offset = updateCursor(UP, CALIBRATE_SCREEN_DISPLAY_NUMBER, 2);
          updateCalibrateScreen(offset);
        }
        else {
          servoDutyCycles[lineIndex[V_MENU]] -= increment;

          if(offset > 0) {
            lcd.setCursor(1, 3);
            lcd.print(F("                   "));
            lcd.setCursor(1, 3);
            lcd.print(servoDutyCycles[lineIndex[V_MENU]]);
            lcd.setCursor(17, 3);
            lcd.print(F("ms"));
          }
          else {
            lcd.setCursor(1, lineIndex[V_MENU]+1);
            lcd.print(F("                   "));
            lcd.setCursor(1, lineIndex[V_MENU]+1);
            lcd.print(servoDutyCycles[lineIndex[V_MENU]]);
            lcd.setCursor(17, lineIndex[V_MENU]+1);
            lcd.print(F("ms"));
          }

          calibrateServo();

        }
        delay(MINIMUM_DELAY);
        break;
      case 0b1101: // Down
        if(lineIndex[H_MENU] == 0) {
          offset = updateCursor(DOWN, CALIBRATE_SCREEN_DISPLAY_NUMBER, 2);
          updateCalibrateScreen(offset);
        }
        else {
          servoDutyCycles[lineIndex[V_MENU]] += increment;

          if(offset > 0) {
            lcd.setCursor(1, 3);
            lcd.print(F("                   "));
            lcd.setCursor(1, 3);
            lcd.print(servoDutyCycles[lineIndex[V_MENU]]);
            lcd.setCursor(17, 3);
            lcd.print(F("ms"));
          }
          else {
            lcd.setCursor(1, lineIndex[V_MENU]+1);
            lcd.print(F("                   "));
            lcd.setCursor(1, lineIndex[V_MENU]+1);
            lcd.print(servoDutyCycles[lineIndex[V_MENU]]);
            lcd.setCursor(17, lineIndex[V_MENU]+1);
            lcd.print(F("ms"));
          }

          calibrateServo();

        }
        delay(MINIMUM_DELAY);
        break;
      case 0b1011: // Ok
        offset = updateCursor(RIGHT, CALIBRATE_SCREEN_DISPLAY_NUMBER, 2);

        if(offset > 0) {
          lcd.setCursor(1, 3);
          lcd.print(F("                   "));
          lcd.setCursor(1, 3);

          if(lineIndex[H_MENU] == 0) {
            /*lcd.print(calibrateScreenDisplay[lineIndex[V_MENU]]);*/
            switch(lineIndex[V_MENU]) {
              case 0:
                lcd.print(F("ServoA -  0"));
                break;
              case 1:
                lcd.print(F("ServoA - 90"));
                break;
              case 2:
                lcd.print(F("ServoB -  0"));
                break;
              case 3:
                lcd.print(F("ServoB - 90"));
                break;
            }
          }
          else {
            lcd.print(servoDutyCycles[lineIndex[V_MENU]]);
            lcd.setCursor(17, 3);
            lcd.print(F("ms"));
            calibrateServo();
          }
        }
        else {
          lcd.setCursor(1, lineIndex[V_MENU]+1);
          lcd.print(F("                   "));
          lcd.setCursor(1, lineIndex[V_MENU]+1);

          if(lineIndex[H_MENU] == 0) {
            /*lcd.print(calibrateScreenDisplay[lineIndex[V_MENU]]);*/
            switch(lineIndex[V_MENU]) {
              case 0:
                lcd.print(F("ServoA -  0"));
                break;
              case 1:
                lcd.print(F("ServoA - 90"));
                break;
              case 2:
                lcd.print(F("ServoB -  0"));
                break;
              case 3:
                lcd.print(F("ServoB - 90"));
                break;
            }
          }
          else {
            lcd.print(servoDutyCycles[lineIndex[V_MENU]]);
            lcd.setCursor(17, lineIndex[V_MENU]+1);
            lcd.print(F("ms"));
            calibrateServo();
          }
        }

        delay(MINIMUM_DELAY);

        break;
      case 0b0111: // NA
        if(lineIndex[H_MENU] == 0) {
          if(SD.exists("sa0")) SD.remove("sa0");
          entry = SD.open("sa0", FILE_WRITE);
          entry.print(servoDutyCycles[SERVO_A_0]);
          entry.close();

          if(SD.exists("sa90")) SD.remove("sa90");
          entry = SD.open("sa90", FILE_WRITE);
          entry.print(servoDutyCycles[SERVO_A_90]);
          entry.close();

          if(SD.exists("sb0")) SD.remove("sb0");
          entry = SD.open("sb0", FILE_WRITE);
          entry.print(servoDutyCycles[SERVO_B_0]);
          entry.close();

          if(SD.exists("sb90")) SD.remove("sb90");
          entry = SD.open("sb90", FILE_WRITE);
          entry.print(servoDutyCycles[SERVO_B_90]);
          entry.close();

          return;
        }
        else {
          increment *= 10;
          if(increment == 10000) increment = 1;
          delay(MINIMUM_DELAY);
        }
        break;
      default:
        break;
    }
  }
}

void updateSettingsScreen(int offset) {
  lcd.setCursor(0, 0);
  lcd.print(F("Settings"));

  if(offset == 0) {
    lcd.setCursor(1, 1);
    lcd.print(F("Inst. height  "));
    lcd.setCursor(1, 2);
    lcd.print(F("Benchmark     "));
    lcd.setCursor(1, 3);
    lcd.print(F("Grid dimension"));
  }
  else {
    lcd.setCursor(1, 1);
    lcd.print(F("Benchmark     "));
    lcd.setCursor(1, 2);
    lcd.print(F("Grid dimension"));
    lcd.setCursor(1, 3);
    lcd.print(F("Samples       "));
  }
}

void settingsScreen() {
  File entry;
  int offset = 0;
  int increment = 1;

  lcd.clear();

  offset = updateCursor(START, SETTINGS_SCREEN_DISPLAY_NUMBER, 2);
  updateSettingsScreen(offset);

  while(true) {
    int buttonsState = checkButtons();

    switch(buttonsState) {
      case 0b1110: // Up
        if(lineIndex[H_MENU] == 0) {
          offset = updateCursor(UP, SETTINGS_SCREEN_DISPLAY_NUMBER, 2);
          updateSettingsScreen(offset);
        }
        else {
          variables[lineIndex[V_MENU]] -= increment;

          if(offset > 0) {
            lcd.setCursor(1, 3);
            lcd.print(F("                   "));
            lcd.setCursor(1, 3);
            lcd.print(variables[lineIndex[V_MENU]]);
            lcd.setCursor(17, 3);
            /*lcd.print(variablesUnit[lineIndex[V_MENU]]);*/
            switch(lineIndex[V_MENU]) {
              case 3:
                lcd.print(F(""));
                break;
              default:
                lcd.print(F("mm"));
            }
          }
          else {
            lcd.setCursor(1, lineIndex[V_MENU]+1);
            lcd.print(F("                   "));
            lcd.setCursor(1, lineIndex[V_MENU]+1);
            lcd.print(variables[lineIndex[V_MENU]]);
            lcd.setCursor(17, lineIndex[V_MENU]+1);
            /*lcd.print(variablesUnit[lineIndex[V_MENU]]);*/
            switch(lineIndex[V_MENU]) {
              case 3:
                lcd.print(F(""));
                break;
              default:
                lcd.print(F("mm"));
            }
          }

        }
        delay(MINIMUM_DELAY);
        break;
      case 0b1101: // Down
        if(lineIndex[H_MENU] == 0) {
          offset = updateCursor(DOWN, SETTINGS_SCREEN_DISPLAY_NUMBER, 2);
          updateSettingsScreen(offset);
        }
        else {
          variables[lineIndex[V_MENU]] += increment;

          if(offset > 0) {
            lcd.setCursor(1, 3);
            lcd.print(F("                   "));
            lcd.setCursor(1, 3);
            lcd.print(variables[lineIndex[V_MENU]]);
            lcd.setCursor(17, 3);
            /*lcd.print(variablesUnit[lineIndex[V_MENU]]);*/
            switch(lineIndex[V_MENU]) {
              case 3:
                lcd.print(F(""));
                break;
              default:
                lcd.print(F("mm"));
            }
          }
          else {
            lcd.setCursor(1, lineIndex[V_MENU]+1);
            lcd.print(F("                   "));
            lcd.setCursor(1, lineIndex[V_MENU]+1);
            lcd.print(variables[lineIndex[V_MENU]]);
            lcd.setCursor(17, lineIndex[V_MENU]+1);
            /*lcd.print(variablesUnit[lineIndex[V_MENU]]);*/
            switch(lineIndex[V_MENU]) {
              case 3:
                lcd.print(F(""));
                break;
              default:
                lcd.print(F("mm"));
            }
          }

        }
        delay(MINIMUM_DELAY);
        break;
      case 0b1011: // Ok
        offset = updateCursor(RIGHT, SETTINGS_SCREEN_DISPLAY_NUMBER, 2);

        if(offset > 0) {
          lcd.setCursor(1, 3);
          lcd.print(F("                   "));
          lcd.setCursor(1, 3);

          if(lineIndex[H_MENU] == 0) {
            /*lcd.print(settingsScreenDisplay[lineIndex[V_MENU]]);*/
            switch(lineIndex[V_MENU]) {
              case 0:
                lcd.print(F("Inst. height  "));
                break;
              case 1:
                lcd.print(F("Benchmark     "));
                break;
              case 2:
                lcd.print(F("Grid dimension"));
                break;
              case 3:
                lcd.print(F("Samples       "));
                break;
            }
          }
          else {
            lcd.print(variables[lineIndex[V_MENU]]);
            lcd.setCursor(17, 3);
            /*lcd.print(variablesUnit[lineIndex[V_MENU]]);*/
            switch(lineIndex[V_MENU]) {
              case 3:
                lcd.print(F(""));
                break;
              default:
                lcd.print(F("mm"));
            }
          }

        }
        else {
          lcd.setCursor(1, lineIndex[V_MENU]+1);
          lcd.print(F("                   "));
          lcd.setCursor(1, lineIndex[V_MENU]+1);

          if(lineIndex[H_MENU] == 0) {
            /*lcd.print(settingsScreenDisplay[lineIndex[V_MENU]]);*/
            switch(lineIndex[V_MENU]) {
              case 0:
                lcd.print(F("Inst. height  "));
                break;
              case 1:
                lcd.print(F("Benchmark     "));
                break;
              case 2:
                lcd.print(F("Grid dimension"));
                break;
              case 3:
                lcd.print(F("Samples       "));
                break;
            }
          }
          else {
            lcd.print(variables[lineIndex[V_MENU]]);
            lcd.setCursor(17, lineIndex[V_MENU]+1);
            /*lcd.print(variablesUnit[lineIndex[V_MENU]]);*/
            switch(lineIndex[V_MENU]) {
              case 3:
                lcd.print(F(""));
                break;
              default:
                lcd.print(F("mm"));
            }
          }
        }

        break;
      case 0b0111: // NA
        if(lineIndex[H_MENU] == 0) {
          if(SD.exists("ih")) SD.remove("ih");
          entry = SD.open("ih", FILE_WRITE);
          entry.print(variables[INSTRUMENT_HEIGHT]);
          entry.close();

          if(SD.exists("bm")) SD.remove("bm");
          entry = SD.open("bm", FILE_WRITE);
          entry.print(variables[BENCHMARK]);
          entry.close();

          if(SD.exists("ga")) SD.remove("ga");
          entry = SD.open("ga", FILE_WRITE);
          entry.print(variables[GRID_AREA]);
          entry.close();

          if(SD.exists("sp")) SD.remove("sp");
          entry = SD.open("sp", FILE_WRITE);
          entry.print(variables[SAMPLES]);
          entry.close();

          return;
        }
        else {
          increment *= 10;
          if(increment == 10000) increment = 1;
          delay(MINIMUM_DELAY);
        }
        break;
      default:
        break;
    }
  }
}

int checkButtons() {
  int buttonStateA = digitalRead(BUTTON_A);
  int buttonStateB = digitalRead(BUTTON_B);
  int buttonStateC = digitalRead(BUTTON_C);
  int buttonStateD = digitalRead(BUTTON_D);

  return (buttonStateA << 0) | (buttonStateB << 1) | (buttonStateC << 2) | (buttonStateD << 3);
}

int measureDistance() {
  int distance = 0;

  for(int i=0; i<variables[SAMPLES]; i++) {
    distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
    for(int j=0; j<10; j++) {
      if(distanceSensor.checkForDataReady()) break;
      else {
        while(1) {
          Wire.begin();

          if (distanceSensor.begin() != 0) { //Begin returns 0 on a good init
            lcd.clear();
            lcd.setCursor(0,3);
            lcd.print(F("Init sensor... fail"));
            delay(MINIMUM_DELAY);
          }
          else break;
        }

        distanceSensor.setDistanceModeLong();
        distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
      }
      delay(1);
    }
    distance += distanceSensor.getDistance(); //Get the result of the measurement from the sensor
    distanceSensor.clearInterrupt();
    distanceSensor.stopRanging();
  }

  return distance/variables[SAMPLES];
}

void calculateGridPoints() {
  data[X_INFO][0] = 0.5 * variables[GRID_AREA];
  data[X_INFO][1] = variables[GRID_AREA];
  data[X_INFO][2] = sqrt(1.25) * variables[GRID_AREA]; // * variables[GRID_AREA] * variables[GRID_AREA]);
  data[X_INFO][3] = sqrt(0.5) * variables[GRID_AREA]; //  * variables[GRID_AREA] * variables[GRID_AREA]);
  data[X_INFO][4] = sqrt(2) * variables[GRID_AREA]; //    * variables[GRID_AREA] * variables[GRID_AREA]);
  data[X_INFO][5] = sqrt(1.25) * variables[GRID_AREA]; // * variables[GRID_AREA] * variables[GRID_AREA]);
  data[X_INFO][6] = 0.5 * variables[GRID_AREA];
  data[X_INFO][7] = variables[GRID_AREA];

  for(int i=0; i<GRID_POINTS; i++) {
    data[SERVO_A_ANGLE_INFO][i] = atan2(variables[INSTRUMENT_HEIGHT], data[X_INFO][i]);
  }

  double alpha = atan2(0.5 * variables[GRID_AREA], variables[GRID_AREA]);
  double beta = atan2(variables[GRID_AREA], variables[GRID_AREA]);
  double gamma = HALF_PI - alpha - beta;

  data[SERVO_B_ANGLE_INFO][0] = 0; // 0.5 * HALF_PI;
  data[SERVO_B_ANGLE_INFO][1] = 0; // 0.5 * HALF_PI;
  data[SERVO_B_ANGLE_INFO][2] = 0.5 * HALF_PI - gamma; // HALF_PI - gamma;
  data[SERVO_B_ANGLE_INFO][3] = 0.5 * HALF_PI; // HALF_PI;
  data[SERVO_B_ANGLE_INFO][4] = 0.5 * HALF_PI; // HALF_PI;
  data[SERVO_B_ANGLE_INFO][5] = 0.5 * HALF_PI + gamma; // HALF_PI + gamma;
  data[SERVO_B_ANGLE_INFO][6] = HALF_PI; // 1.5 * HALF_PI;
  data[SERVO_B_ANGLE_INFO][7] = HALF_PI; // 1.5 * HALF_PI;
}
