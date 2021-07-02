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

const int BUTTON_A = 2;
const int BUTTON_B = 3;
const int BUTTON_C = 4;
const int BUTTON_D = 5;

const int SERVO_A = 9;
const int SERVO_B = 10;

const int START = 0;
const int UP = 1;
const int DOWN = 2;
const int END = 3;
const int LEFT = 4;
const int RIGHT = 5;

const int MINIMUM_DELAY = 250;

const int settingsScreenDisplayNumber = 4;
const char *settingsScreenDisplay[] = {
  "Inst. height       ",
  "Benchmark          ",
  "Grid dimension     ",
  "Samples            "
};

const int SERVO_A_0 = 0;
const int SERVO_A_180 = 1;
const int SERVO_B_0 = 2;
const int SERVO_B_180 = 3;

const int calibrateScreenDisplayNumber = 4;
const char *calibrateScreenDisplay[] = {
  "Servo A - 0 deg    ",
  "Servo A - 180 deg  ",
  "Servo B - 0 deg    ",
  "Servo B - 180 deg  ",
};

int servoDutyCycles[] = {
  490,
  2530,
  410,
  2520
};

const int SERVO_A_MANUAL = 0;
const int SERVO_B_MANUAL = 1;
const int manualControlScreenDisplayNumber = 2;
const char *manualControlScreenDisplay[] = {
  "Servo A            ",
  "Servo B            ",
};

double manualControlServoAngles[] = {
  90,
  90,
};

const int mainScreenDisplayNumber = 4;
const char *mainScreenDisplay[] = {
  "Start              ",
  "Settings           ",
  "Calibrate          ",
  "Manual Control     "
};


const int START_SCREEN = 0;
const int SETTINGS_SCREEN = 1;
const int CALIBRATE_SCREEN = 2;
const int MANUAL_CONTROL_SCREEN = 3;

const int GRID_POINTS = 8;
const int COUNTDOWN_TIMER = 5;

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


Servo servoA;
Servo servoB;

SFEVL53L1X distanceSensor;
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7); // 0x27 is the default I2C bus address of the backpack-see article

const int INFO_NUMBER = 6;
const int ELEVATION_INFO = 0;
const int SERVO_A_ANGLE_INFO = 1;
const int DISTANCE_INFO = 2;
const int X_INFO = 3;
const int Y_INFO = 4;
const int SERVO_B_ANGLE_INFO = 5;

double data[INFO_NUMBER][GRID_POINTS];
const char *dataUnit[] = {
  "mm",
  "deg",
  "mm",
  "mm",
  "mm"
};

int lineIndex[2] = {0, 0};

const int INSTRUMENT_HEIGHT = 0;
const int BENCHMARK = 1;
const int GRID_AREA = 2;
const int SAMPLES = 3;
const char *variablesUnit[] = {
  "mm",
  "mm",
  "mm",
  ""
};
double variables[] = {
  1000,
  0,
  1000,
  3
};

void setup() {
  initLCD();
  initButtons();
  initServoMotors();
  initDistanceSensor();

  lcd.clear();
  lcd.setCursor(0,3);
  lcd.print("Init... done");
  delay(MINIMUM_DELAY);

  welcomeScreen();
}

void loop() {
  mainScreen();
}

void clearScreen() {
  for(int i=1; i<4; i++) {
    lcd.setCursor(1, i);
    lcd.print("                    ");
  }
}

void welcomeScreen() {
    lcd.clear();
    delay(500);

    lcd.setCursor(6, 1);
    lcd.print("GRIDIRON");

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
    lcd.print("Grid Point ");
    lcd.setCursor(11,0);
    lcd.print(i+1);
    lcd.setCursor(12,0);
    lcd.print(":");
    lcd.setCursor(14,0);
    lcd.print(gridPointLocation[i]);

    angle = data[SERVO_A_ANGLE_INFO][i] * RAD_TO_DEG;
    dutyCycle = map(angle, 0, 180, servoDutyCycles[SERVO_A_0], servoDutyCycles[SERVO_A_180]);
    servoA.writeMicroseconds(dutyCycle);

    angle = data[SERVO_B_ANGLE_INFO][i] * RAD_TO_DEG;
    dutyCycle = map(angle, 0, 180, servoDutyCycles[SERVO_B_0], servoDutyCycles[SERVO_B_180]);
    servoB.writeMicroseconds(dutyCycle);

    delay(3000);

    data[DISTANCE_INFO][i] = measureDistance();
    data[Y_INFO][i] = data[DISTANCE_INFO][i] * sin(data[SERVO_A_ANGLE_INFO][i]);
    data[ELEVATION_INFO][i] = variables[INSTRUMENT_HEIGHT] + variables[BENCHMARK] - data[Y_INFO][i];

    lcd.setCursor(0, 1);
    lcd.print("Elevation (mm):");
    lcd.setCursor(0, 2);
    lcd.print(data[ELEVATION_INFO][i]);

    delay(2000);
  }

  dutyCycle = map(90, 0, 180, servoDutyCycles[SERVO_A_0], servoDutyCycles[SERVO_A_180]);
  servoA.writeMicroseconds(dutyCycle);

  dutyCycle = map(90, 0, 180, servoDutyCycles[SERVO_B_0], servoDutyCycles[SERVO_B_180]);
  servoB.writeMicroseconds(dutyCycle);

  elevationSummaryScreen();
}

void updateElevationSummaryScreen(int offset, int infoIndex) {
  switch(infoIndex) {
    case ELEVATION_INFO:
      lcd.setCursor(0, 0);
      lcd.print(" ELEVATION SUMMARY  ");
      break;
    case DISTANCE_INFO:
      lcd.setCursor(0, 0);
      lcd.print("  DISTANCE SUMMARY  ");
      break;
    case SERVO_A_ANGLE_INFO:
      lcd.setCursor(0, 0);
      lcd.print(" HOR. ANGLE SUMMARY ");
      break;
    case X_INFO:
      lcd.setCursor(0, 0);
      lcd.print("      X SUMMARY     ");
      break;
    case Y_INFO:
      lcd.setCursor(0, 0);
      lcd.print("      Y SUMMARY     ");
      break;
    default:
      lcd.setCursor(0, 0);
      lcd.print(" ELEVATION SUMMARY  ");
  }

  for(int i=0; i<3; i++) {
    lcd.setCursor(1, i+1);
    lcd.print("                   ");
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
  updateElevationSummaryScreen(offset, lineIndex[RIGHT]);

  while(true) {
    int buttonsState = checkButtons();

    switch(buttonsState) {
      case 0b1110: // Up
        offset = updateCursor(UP, GRID_POINTS, 5);
        updateElevationSummaryScreen(offset, lineIndex[RIGHT]);
        break;
      case 0b1101: // Down
        offset = updateCursor(DOWN, GRID_POINTS, 5);
        updateElevationSummaryScreen(offset, lineIndex[RIGHT]);
        break;
      case 0b1011: // Ok
        offset = updateCursor(RIGHT, GRID_POINTS, 5);
        updateElevationSummaryScreen(offset, lineIndex[RIGHT]);
        break;
      case 0b0111:
        return;
        break;
    }

  }
}

int updateCursor(const int dir, int numberOfOptionsLeft, int numberOfOptionsRight) {
  if(dir == UP) {
    if(lineIndex[LEFT] < numberOfOptionsLeft - 1) lineIndex[LEFT]++;
    else lineIndex[LEFT] = 0;
  }
  else if(dir == DOWN) {
    if(lineIndex[LEFT] > 0) lineIndex[LEFT]--;
    else lineIndex[LEFT] = numberOfOptionsLeft - 1;
  }
  else if(dir == RIGHT) {
    if(lineIndex[RIGHT] < numberOfOptionsRight - 1) lineIndex[RIGHT]++;
    else lineIndex[RIGHT] = 0;
  }
  else if(dir == LEFT) {
    if(lineIndex[RIGHT] > 0) lineIndex[RIGHT]--;
    else lineIndex[RIGHT] = numberOfOptionsRight - 1;
  }
  else {
    lineIndex[LEFT] = 0;
    lineIndex[RIGHT] = 0;
  }

  int indexDiff = lineIndex[LEFT] - 2;
  if(indexDiff < 0) indexDiff = 0;

  for(int i=0; i<3; i++) {
    int tempCursor;

    if(indexDiff > 0) tempCursor = 2;
    else tempCursor = lineIndex[LEFT];

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

void manualControlServo() {
  double angle;
  double dutyCycle;

  switch(lineIndex[LEFT]) {
    case SERVO_A_MANUAL:
      angle = manualControlServoAngles[SERVO_A_MANUAL];
      dutyCycle = map(angle, 0, 180, servoDutyCycles[SERVO_A_0], servoDutyCycles[SERVO_A_180]);
      servoA.writeMicroseconds(dutyCycle);
      break;
    case SERVO_B_MANUAL:
      angle = manualControlServoAngles[SERVO_B_MANUAL];
      dutyCycle = map(angle, 0, 180, servoDutyCycles[SERVO_B_0], servoDutyCycles[SERVO_B_180]);
      servoB.writeMicroseconds(dutyCycle);
      break;
  }
  lcd.setCursor(0, 3);
  lcd.print("Distance (mm): ");
  lcd.setCursor(15, 3);
  lcd.print(measureDistance());
}

void updateManualControlScreen(int offset) {
  lcd.setCursor(0, 0);
  lcd.print("Manual Control");

  for(int i=0; i<3; i++) {
    lcd.setCursor(1, i+1);
    lcd.print(manualControlScreenDisplay[i+offset]);
  }
}

void manualControlScreen() {
  int offset = 0;
  double increment = 0.1;

  lcd.clear();

  offset = updateCursor(START, manualControlScreenDisplayNumber, 2);
  updateManualControlScreen(offset);

  while(true) {
    int buttonsState = checkButtons();

    switch(buttonsState) {
      case 0b1110: // Up
        if(lineIndex[RIGHT] == 0) {
          offset = updateCursor(UP, manualControlScreenDisplayNumber, 2);
          updateManualControlScreen(offset);
        }
        else {
          manualControlServoAngles[lineIndex[LEFT]] += increment;

          if(manualControlServoAngles[lineIndex[LEFT]] > 180) {
            manualControlServoAngles[lineIndex[LEFT]] = 180;
          }

          if(offset > 0) {
            lcd.setCursor(1, 3);
            lcd.print("                   ");
            lcd.setCursor(1, 3);
            lcd.print(manualControlServoAngles[lineIndex[LEFT]]);
            lcd.setCursor(17, 3);
            lcd.print("deg");
          }
          else {
            lcd.setCursor(1, lineIndex[LEFT]+1);
            lcd.print("                   ");
            lcd.setCursor(1, lineIndex[LEFT]+1);
            lcd.print(manualControlServoAngles[lineIndex[LEFT]]);
            lcd.setCursor(17, lineIndex[LEFT]+1);
            lcd.print("deg");
          }

          manualControlServo();

        }
        delay(MINIMUM_DELAY);
        break;
      case 0b1101: // Down
        if(lineIndex[RIGHT] == 0) {
          offset = updateCursor(DOWN, manualControlScreenDisplayNumber, 2);
          updateManualControlScreen(offset);
        }
        else {
          manualControlServoAngles[lineIndex[LEFT]] -= increment;

          if(manualControlServoAngles[lineIndex[LEFT]] < 0) {
            manualControlServoAngles[lineIndex[LEFT]] = 0;
          }

          if(offset > 0) {
            lcd.setCursor(1, 3);
            lcd.print("                   ");
            lcd.setCursor(1, 3);
            lcd.print(manualControlServoAngles[lineIndex[LEFT]]);
            lcd.setCursor(17, 3);
            lcd.print("deg");
          }
          else {
            lcd.setCursor(1, lineIndex[LEFT]+1);
            lcd.print("                   ");
            lcd.setCursor(1, lineIndex[LEFT]+1);
            lcd.print(manualControlServoAngles[lineIndex[LEFT]]);
            lcd.setCursor(17, lineIndex[LEFT]+1);
            lcd.print("deg");
          }

          manualControlServo();

        }
        delay(MINIMUM_DELAY);
        break;
      case 0b1011: // Ok
        offset = updateCursor(RIGHT, manualControlScreenDisplayNumber, 2);

        if(offset > 0) {
          lcd.setCursor(1, 3);
          lcd.print("                   ");
          lcd.setCursor(1, 3);

          if(lineIndex[RIGHT] == 0) {
            lcd.print(manualControlScreenDisplay[lineIndex[LEFT]]);
          }
          else {
            lcd.print(servoDutyCycles[lineIndex[LEFT]]);
            lcd.setCursor(17, 3);
            lcd.print("deg");
            manualControlServo();
          }
        }
        else {
          lcd.setCursor(1, lineIndex[LEFT]+1);
          lcd.print("                   ");
          lcd.setCursor(1, lineIndex[LEFT]+1);

          if(lineIndex[RIGHT] == 0) {
            lcd.print(manualControlScreenDisplay[lineIndex[LEFT]]);
          }
          else {
            lcd.print(manualControlServoAngles[lineIndex[LEFT]]);
            lcd.setCursor(17, lineIndex[LEFT]+1);
            lcd.print("deg");
            manualControlServo();
          }
        }

        delay(MINIMUM_DELAY);

        break;
      case 0b0111: // NA
        if(lineIndex[RIGHT] == 0) {
          return;
        }
        else {
          increment *= 10;
          if(increment == 100) increment = 0.1;
          delay(MINIMUM_DELAY);
        }
        break;
      default:
        break;
    }
  }
}

void calibrateServo() {
  switch(lineIndex[LEFT]) {
    case SERVO_A_0:
      servoA.writeMicroseconds(servoDutyCycles[SERVO_A_0]);
      break;
    case SERVO_A_180:
      servoA.writeMicroseconds(servoDutyCycles[SERVO_A_180]);
      break;
    case SERVO_B_0:
      servoB.writeMicroseconds(servoDutyCycles[SERVO_B_0]);
      break;
    case SERVO_B_180:
      servoB.writeMicroseconds(servoDutyCycles[SERVO_B_180]);
      break;
  }
}

void updateCalibrateScreen(int offset) {
  lcd.setCursor(0, 0);
  lcd.print("Calibrate");

  for(int i=0; i<3; i++) {
    lcd.setCursor(1, i+1);
    lcd.print(calibrateScreenDisplay[i+offset]);
  }
}

void calibrateScreen() {
  int offset = 0;
  int increment = 1;

  lcd.clear();

  offset = updateCursor(START, calibrateScreenDisplayNumber, 2);
  updateCalibrateScreen(offset);

  while(true) {
    int buttonsState = checkButtons();

    switch(buttonsState) {
      case 0b1110: // Up
        if(lineIndex[RIGHT] == 0) {
          offset = updateCursor(UP, calibrateScreenDisplayNumber, 2);
          updateCalibrateScreen(offset);
        }
        else {
          servoDutyCycles[lineIndex[LEFT]] += increment;

          if(offset > 0) {
            lcd.setCursor(1, 3);
            lcd.print("                   ");
            lcd.setCursor(1, 3);
            lcd.print(servoDutyCycles[lineIndex[LEFT]]);
            lcd.setCursor(17, 3);
            lcd.print("ms");
          }
          else {
            lcd.setCursor(1, lineIndex[LEFT]+1);
            lcd.print("                   ");
            lcd.setCursor(1, lineIndex[LEFT]+1);
            lcd.print(servoDutyCycles[lineIndex[LEFT]]);
            lcd.setCursor(17, lineIndex[LEFT]+1);
            lcd.print("ms");
          }

          calibrateServo();

        }
        delay(MINIMUM_DELAY);
        break;
      case 0b1101: // Down
        if(lineIndex[RIGHT] == 0) {
          offset = updateCursor(DOWN, calibrateScreenDisplayNumber, 2);
          updateCalibrateScreen(offset);
        }
        else {
          servoDutyCycles[lineIndex[LEFT]] -= increment;

          if(offset > 0) {
            lcd.setCursor(1, 3);
            lcd.print("                   ");
            lcd.setCursor(1, 3);
            lcd.print(servoDutyCycles[lineIndex[LEFT]]);
            lcd.setCursor(17, 3);
            lcd.print("ms");
          }
          else {
            lcd.setCursor(1, lineIndex[LEFT]+1);
            lcd.print("                   ");
            lcd.setCursor(1, lineIndex[LEFT]+1);
            lcd.print(servoDutyCycles[lineIndex[LEFT]]);
            lcd.setCursor(17, lineIndex[LEFT]+1);
            lcd.print("ms");
          }

          calibrateServo();

        }
        delay(MINIMUM_DELAY);
        break;
      case 0b1011: // Ok
        offset = updateCursor(RIGHT, calibrateScreenDisplayNumber, 2);

        if(offset > 0) {
          lcd.setCursor(1, 3);
          lcd.print("                   ");
          lcd.setCursor(1, 3);

          if(lineIndex[RIGHT] == 0) {
            lcd.print(calibrateScreenDisplay[lineIndex[LEFT]]);
          }
          else {
            lcd.print(servoDutyCycles[lineIndex[LEFT]]);
            lcd.setCursor(17, 3);
            lcd.print("ms");
            calibrateServo();
          }
        }
        else {
          lcd.setCursor(1, lineIndex[LEFT]+1);
          lcd.print("                   ");
          lcd.setCursor(1, lineIndex[LEFT]+1);

          if(lineIndex[RIGHT] == 0) {
            lcd.print(calibrateScreenDisplay[lineIndex[LEFT]]);
          }
          else {
            lcd.print(servoDutyCycles[lineIndex[LEFT]]);
            lcd.setCursor(17, lineIndex[LEFT]+1);
            lcd.print("ms");
            calibrateServo();
          }
        }

        delay(MINIMUM_DELAY);

        break;
      case 0b0111: // NA
        if(lineIndex[RIGHT] == 0) {
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
  lcd.print("Settings");

  for(int i=0; i<3; i++) {
    lcd.setCursor(1, i+1);
    lcd.print(settingsScreenDisplay[i+offset]);
  }
}

void settingsScreen() {
  int offset = 0;
  double increment = 0.1;

  lcd.clear();

  offset = updateCursor(START, settingsScreenDisplayNumber, 2);
  updateSettingsScreen(offset);

  while(true) {
    int buttonsState = checkButtons();

    switch(buttonsState) {
      case 0b1110: // Up
        if(lineIndex[RIGHT] == 0) {
          offset = updateCursor(UP, settingsScreenDisplayNumber, 2);
          updateSettingsScreen(offset);
        }
        else {
          variables[lineIndex[LEFT]] += increment;

          if(offset > 0) {
            lcd.setCursor(1, 3);
            lcd.print("                   ");
            lcd.setCursor(1, 3);
            lcd.print(variables[lineIndex[LEFT]]);
            lcd.setCursor(17, 3);
            lcd.print(variablesUnit[lineIndex[LEFT]]);
          }
          else {
            lcd.setCursor(1, lineIndex[LEFT]+1);
            lcd.print("                   ");
            lcd.setCursor(1, lineIndex[LEFT]+1);
            lcd.print(variables[lineIndex[LEFT]]);
            lcd.setCursor(17, lineIndex[LEFT]+1);
            lcd.print(variablesUnit[lineIndex[LEFT]]);
          }

        }
        delay(MINIMUM_DELAY);
        break;
      case 0b1101: // Down
        if(lineIndex[RIGHT] == 0) {
          offset = updateCursor(DOWN, settingsScreenDisplayNumber, 2);
          updateSettingsScreen(offset);
        }
        else {
          variables[lineIndex[LEFT]] -= increment;

          if(offset > 0) {
            lcd.setCursor(1, 3);
            lcd.print("                   ");
            lcd.setCursor(1, 3);
            lcd.print(variables[lineIndex[LEFT]]);
            lcd.setCursor(17, 3);
            lcd.print(variablesUnit[lineIndex[LEFT]]);
          }
          else {
            lcd.setCursor(1, lineIndex[LEFT]+1);
            lcd.print("                   ");
            lcd.setCursor(1, lineIndex[LEFT]+1);
            lcd.print(variables[lineIndex[LEFT]]);
            lcd.setCursor(17, lineIndex[LEFT]+1);
            lcd.print(variablesUnit[lineIndex[LEFT]]);
          }

        }
        delay(MINIMUM_DELAY);
        break;
      case 0b1011: // Ok
        offset = updateCursor(RIGHT, settingsScreenDisplayNumber, 2);

        if(offset > 0) {
          lcd.setCursor(1, 3);
          lcd.print("                   ");
          lcd.setCursor(1, 3);

          if(lineIndex[RIGHT] == 0) {
            lcd.print(settingsScreenDisplay[lineIndex[LEFT]]);
          }
          else {
            lcd.print(variables[lineIndex[LEFT]]);
            lcd.setCursor(17, 3);
            lcd.print(variablesUnit[lineIndex[LEFT]]);
          }

        }
        else {
          lcd.setCursor(1, lineIndex[LEFT]+1);
          lcd.print("                   ");
          lcd.setCursor(1, lineIndex[LEFT]+1);

          if(lineIndex[RIGHT] == 0) {
            lcd.print(settingsScreenDisplay[lineIndex[LEFT]]);
          }
          else {
            lcd.print(variables[lineIndex[LEFT]]);
            lcd.setCursor(17, lineIndex[LEFT]+1);
            lcd.print(variablesUnit[lineIndex[LEFT]]);
          }
        }

        break;
      case 0b0111: // NA
        if(lineIndex[RIGHT] == 0) {
          return;
        }
        else {
          increment *= 10;
          if(increment == 10000) increment = 0.01;
          delay(MINIMUM_DELAY);
        }
        break;
      default:
        break;
    }
  }
}

void updateMainScreen(int offset) {
  lcd.setCursor(6, 0);
  lcd.print("GRIDIRON");

  for(int i=0; i<3; i++) {
    lcd.setCursor(1, i+1);
    lcd.print(mainScreenDisplay[i+offset]);
  }
}

void mainScreen() {
  int offset = 0;

  lcd.clear();
  offset = updateCursor(START, mainScreenDisplayNumber, 1);
  updateMainScreen(offset);

  while(true) {
    int buttonsState = checkButtons();

    switch(buttonsState) {
      case 0b1110: // Up
        offset = updateCursor(UP, mainScreenDisplayNumber, 1);
        updateMainScreen(offset);
        break;
      case 0b1101: // Down
        offset = updateCursor(DOWN, mainScreenDisplayNumber, 1);
        updateMainScreen(offset);
        break;
      case 0b1011: // Ok
        if(lineIndex[LEFT] == START_SCREEN) {
          startScreen();
        }
        else if(lineIndex[LEFT] == SETTINGS_SCREEN) {
          settingsScreen();
        }
        else if(lineIndex[LEFT] == CALIBRATE_SCREEN) {
          calibrateScreen();
        }
        else if(lineIndex[LEFT] == MANUAL_CONTROL_SCREEN) {
          manualControlScreen();
        }

        lcd.clear();
        offset = updateCursor(START, mainScreenDisplayNumber, 1);
        updateMainScreen(offset);

        break;
    }

  }
}

void initButtons() {
  lcd.clear();
  lcd.setCursor(0,3);
  lcd.print("Init buttons...");

  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
  pinMode(BUTTON_D, INPUT_PULLUP);

  delay(MINIMUM_DELAY);

  lcd.clear();
  lcd.setCursor(0,3);
  lcd.print("Init buttons... ok");

  delay(MINIMUM_DELAY);
}

void initLCD() {
  lcd.begin (20,4);
  lcd.setBacklightPin(3, POSITIVE);
  lcd.setBacklight(HIGH);
  delay(MINIMUM_DELAY);
}

int checkButtons() {
  int buttonStateA = digitalRead(BUTTON_A);
  int buttonStateB = digitalRead(BUTTON_B);
  int buttonStateC = digitalRead(BUTTON_C);
  int buttonStateD = digitalRead(BUTTON_D);

  return (buttonStateA << 0) | (buttonStateB << 1) | (buttonStateC << 2) | (buttonStateD << 3);
}

void initDistanceSensor() {
  lcd.clear();
  lcd.setCursor(0,3);
  lcd.print("Init sensor...");

  Wire.begin();

  if (distanceSensor.begin() != 0) { //Begin returns 0 on a good init
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Init sensor... fail");
    while (1);
  }

  distanceSensor.setDistanceModeLong();

  delay(MINIMUM_DELAY);

  lcd.clear();
  lcd.setCursor(0,3);
  lcd.print("Init sensor... ok");

  delay(MINIMUM_DELAY);
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

void initServoMotors() {
  lcd.clear();
  lcd.setCursor(0,3);
  lcd.print("Init servo...");

  servoA.attach(SERVO_A);
  servoB.attach(SERVO_B);
  delay(1000);

  lcd.clear();
  lcd.setCursor(0,3);
  lcd.print("Init servo... ok");

  delay(MINIMUM_DELAY);
}

void calculateGridPoints() {
  data[X_INFO][0] = 0.5 * variables[GRID_AREA];
  data[X_INFO][1] = variables[GRID_AREA];
  data[X_INFO][2] = sqrt(1.25 * variables[GRID_AREA] * variables[GRID_AREA]);
  data[X_INFO][3] = sqrt(2    * variables[GRID_AREA] * variables[GRID_AREA]);
  data[X_INFO][4] = sqrt(0.5  * variables[GRID_AREA] * variables[GRID_AREA]);
  data[X_INFO][5] = sqrt(1.25 * variables[GRID_AREA] * variables[GRID_AREA]);
  data[X_INFO][6] = variables[GRID_AREA];
  data[X_INFO][7] = 0.5 * variables[GRID_AREA];

  for(int i=0; i<GRID_POINTS; i++) {
    data[SERVO_A_ANGLE_INFO][i] = atan2(variables[INSTRUMENT_HEIGHT], data[X_INFO][i]);
  }

  double alpha = atan2(0.5 * variables[GRID_AREA], variables[GRID_AREA]);
  double beta = atan2(variables[GRID_AREA], variables[GRID_AREA]);
  double gamma = HALF_PI - alpha - beta;

  data[SERVO_B_ANGLE_INFO][0] = 0.5 * HALF_PI;
  data[SERVO_B_ANGLE_INFO][1] = 0.5 * HALF_PI;
  data[SERVO_B_ANGLE_INFO][2] = HALF_PI - gamma;
  data[SERVO_B_ANGLE_INFO][3] = HALF_PI;
  data[SERVO_B_ANGLE_INFO][4] = HALF_PI;
  data[SERVO_B_ANGLE_INFO][5] = HALF_PI + gamma;
  data[SERVO_B_ANGLE_INFO][6] = 1.5 * HALF_PI;
  data[SERVO_B_ANGLE_INFO][7] = 1.5 * HALF_PI;
}
