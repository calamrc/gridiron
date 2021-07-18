/*
  SD card read/write

 This example shows how to read and write data to and from an SD card file
 The circuit:
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)

 created   Nov 2010
 by David A. Mellis
 modified 9 Apr 2012
 by Tom Igoe

 This example code is in the public domain.

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
#define MAIN_SCREEN_DISPLAY_NUMBER 4

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

Servo servoA;
Servo servoB;
SFEVL53L1X distanceSensor;
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7); // 0x27 is the default I2C bus address of the backpack-see article

int lineIndex[2] = {0, 0};
int data[INFO_NUMBER][GRID_POINTS];
int variables[] = {
  1000,
  0,
  1000,
  3
};

int manualControlServoAngles[] = {
  90,
  90,
};

int servoDutyCycles[] = {
  570,
  1570,
  880,
  1900
};

const char *gridPointLocation[] = {
  "(0,1)",
  "(0,2)",
  "(1,2)",
  "(2,2)",
  "(1,1)",
  "(2,1)",
  "(2,0)",
  "(1,0)"
};

void setup() {
  initLCD();
  initButtons();
  initSDCard();
  initServoMotors();
  initDistanceSensor();

  lcd.clear();
  lcd.setCursor(0,3);
  lcd.print(F("Init... done"));

  delay(MINIMUM_DELAY);

  welcomeScreen();
}

void loop() {
  // nothing happens after setup
  mainScreen();
}

void printStatus(char statusMessage[20]) {
  lcd.clear();
  lcd.setCursor(0,3);
  lcd.print(statusMessage);

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

  lcd.clear();
  lcd.setCursor(0,3);
  lcd.print(F("Init buttons... ok"));

  delay(MINIMUM_DELAY);
  /*printStatus("Init buttons... ok");*/
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

  dutyCycle = map(90, 0, 90, servoDutyCycles[SERVO_B_0], servoDutyCycles[SERVO_B_90]);
  servoB.writeMicroseconds(dutyCycle);

  dutyCycle = map( 0, 0, 90, servoDutyCycles[SERVO_A_0], servoDutyCycles[SERVO_A_90]);
  servoA.writeMicroseconds(dutyCycle);
  delay(1000);

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

int checkButtons() {
  int buttonStateA = digitalRead(BUTTON_A);
  int buttonStateB = digitalRead(BUTTON_B);
  int buttonStateC = digitalRead(BUTTON_C);
  int buttonStateD = digitalRead(BUTTON_D);

  return (buttonStateA << 0) | (buttonStateB << 1) | (buttonStateC << 2) | (buttonStateD << 3);
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
      lcd.print(">");
    }
    else {
      lcd.setCursor(0, i+1);
      lcd.print(" ");
    }
  }

  delay(MINIMUM_DELAY);

  return indexDiff;

}

int measureDistance() {
  int distance = 0;

  for(int i=0; i<variables[SAMPLES]; i++) {
    distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
    while (!distanceSensor.checkForDataReady()) {
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
  data[X_INFO][2] = sqrt(1.25 * variables[GRID_AREA] * variables[GRID_AREA]);
  data[X_INFO][3] = sqrt(0.5  * variables[GRID_AREA] * variables[GRID_AREA]);
  data[X_INFO][4] = sqrt(2    * variables[GRID_AREA] * variables[GRID_AREA]);
  data[X_INFO][5] = sqrt(1.25 * variables[GRID_AREA] * variables[GRID_AREA]);
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

/*void printDirectory(File dir, int numTabs) {*/
void getDisplayText(File dir, int numDisplay, char *displayText[20]) {
  for(int i=0; i<numDisplay; i++) {
    File entry =  dir.openNextFile();
    if(!entry) break;

    if(!entry.isDirectory()) {
      String fileNameString = entry.name();
      char fileNameBuffer[20];

      fileNameString.toCharArray(fileNameBuffer, 20);

      displayText[i] = fileNameBuffer;
    }

    entry.close();
  }
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

void updateMainScreen(int offset, const char *mainScreenDisplay[]) {
  lcd.setCursor(6, 0);
  lcd.print(F("GRIDIRON"));

  for(int i=0; i<3; i++) {
    lcd.setCursor(1, i+1);
    lcd.print(mainScreenDisplay[i+offset]);
  }
}

void mainScreen() {
  const char *mainScreenDisplay[] = {
    "Start         ",
    "Settings      ",
    "Calibrate     ",
    "Manual Control"
  };
  int offset = 0;
  int index = 0;

  lcd.clear();
  offset = updateCursor(START, MAIN_SCREEN_DISPLAY_NUMBER, 1);
  updateMainScreen(offset, mainScreenDisplay);

  while(true) {
    int buttonsState = checkButtons();

    switch(buttonsState) {
      case 0b1110: // Up
        offset = updateCursor(UP, MAIN_SCREEN_DISPLAY_NUMBER, 1);
        updateMainScreen(offset, mainScreenDisplay);
        break;
      case 0b1101: // Down
        offset = updateCursor(DOWN, MAIN_SCREEN_DISPLAY_NUMBER, 1);
        updateMainScreen(offset, mainScreenDisplay);
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
          /*settingsScreen();*/
        }
        else if(lineIndex[V_MENU] == CALIBRATE_SCREEN) {
          /*calibrateScreen();*/
        }
        else if(lineIndex[V_MENU] == MANUAL_CONTROL_SCREEN) {
          /*manualControlScreen();*/
        }
        else {
          /*creditsScreen();*/
        }

        lcd.clear();
        offset = updateCursor(START, MAIN_SCREEN_DISPLAY_NUMBER, 1);
        updateMainScreen(offset, mainScreenDisplay);

        break;
      case 0b0111: // Back
        if(index < 3) {
          index++;
          if(index == 3) {
            lcd.setCursor(19,0);
            lcd.print("!");
          }
        }
        else index = 0;
        delay(MINIMUM_DELAY);

        lcd.setCursor(19,0);
        lcd.print(" ");
    }
  }
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
    lcd.print("Grid Point ");
    lcd.setCursor(11,0);
    lcd.print(i+1);
    lcd.setCursor(12,0);
    lcd.print(":");
    lcd.setCursor(14,0);
    lcd.print(gridPointLocation[i]);

    dutyCycle = map( 0, 0, 90, servoDutyCycles[SERVO_B_0], servoDutyCycles[SERVO_B_90]);
    servoB.writeMicroseconds(dutyCycle);
    dutyCycle = map(90, 0, 90, servoDutyCycles[SERVO_A_0], servoDutyCycles[SERVO_A_90]);
    servoA.writeMicroseconds(dutyCycle);
    delay(1000);

    angle = data[SERVO_B_ANGLE_INFO][i] * RAD_TO_DEG;
    dutyCycle = map(angle, 0, 90, servoDutyCycles[SERVO_B_0], servoDutyCycles[SERVO_B_90]);
    servoB.writeMicroseconds(dutyCycle);

    angle = data[SERVO_A_ANGLE_INFO][i] * RAD_TO_DEG;
    dutyCycle = map(angle, 0, 90, servoDutyCycles[SERVO_A_0], servoDutyCycles[SERVO_A_90]);
    servoA.writeMicroseconds(dutyCycle);
    delay(2000);

    data[DISTANCE_INFO][i] = measureDistance();
    data[Y_INFO][i] = data[DISTANCE_INFO][i] * sin(data[SERVO_A_ANGLE_INFO][i]);
    data[ELEVATION_INFO][i] = variables[INSTRUMENT_HEIGHT] + variables[BENCHMARK] - data[Y_INFO][i];

    lcd.setCursor(0, 1);
    lcd.print(F("Elevation (mm):"));
    lcd.setCursor(0, 2);
    lcd.print(data[ELEVATION_INFO][i]);

    delay(1000);
  }

  dutyCycle = map( 0, 0, 90, servoDutyCycles[SERVO_B_0], servoDutyCycles[SERVO_B_90]);
  servoB.writeMicroseconds(dutyCycle);

  dutyCycle = map(90, 0, 90, servoDutyCycles[SERVO_A_0], servoDutyCycles[SERVO_A_90]);
  servoA.writeMicroseconds(dutyCycle);

  elevationSummaryScreen();
}

void updateElevationSummaryScreen(int offset, int infoIndex) {
  const char *dataUnit[] = {
    "mm",
    "deg",
    "mm",
    "mm",
    "mm"
  };

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
    lcd.print(gridPointLocation[i+offset]);
    lcd.setCursor(7, i+1);
    if(infoIndex == SERVO_A_ANGLE_INFO) {
      lcd.print(data[infoIndex][i+offset] * RAD_TO_DEG);
    }
    else {
      lcd.print(data[infoIndex][i+offset]);
    }
    lcd.setCursor(17, i+1);
    lcd.print(dataUnit[infoIndex]);
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
        return;
        break;
    }

  }
}
