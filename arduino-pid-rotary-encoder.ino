//  Copyright 2018 Alec B. Plumb
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include <max6675.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <PID_v1.h>
#include <EEPROM.h>

#define LCD_ADDR 0x27
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

MAX6675 thermocouple(11, 10, 9); // CLK, CS, D0
MAX6675 thermocouple2(7, 6, 5); // CLK, CS, D0

#define RELAY_PIN 12

#define DEGREE_CHAR 0
byte degreeChar[8]  = {
  0b01100,
  0b10010,
  0b10010,
  0b01100,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};

#define UP_CHAR 1
byte upChar[8] = {
  0b00100,
  0b01110,
  0b10101,
  0b00100,
  0b00100,
  0b00100,
  0b00100,
  0b00100
};

#define DEFAULT_SET_POINT_C 371.111  // 700°F
#define MIN_SET_POINT_C 0.0  // °C
#define MAX_SET_POINT_C 537.222  // 999°F
#define BANG_ON_C 27.77777 // 50°F Turn relay fully on if PV is more than this below the setpoint
#define BANG_OFF_C 13.88888 // 25°F Turn relay fully off if PV is more than this above the setpoint

//////////////////////////////
// PID
//////////////////////////////
double inputTempC; // °C
double input2TempC; // °C
double setPointTempC = -1.0; // °C
double pidOutput;

// These tunings are almost entirely arbitrary, and should be tuned.
#define KP 850
#define KI .5
#define KD .1

//Specify the links and initial tuning parameters
PID myPID(&inputTempC, &pidOutput, &setPointTempC, KP, KI, KD, P_ON_E, DIRECT);

char degreesBuff[5];
int tempReadInterval = 500;
int lastTempReadTime = 0;

//////////////////////////////
// Relay Window
//////////////////////////////
int WindowSize = 5000;
unsigned long windowStartTime;
bool oldRelayEnabled = true; // we will initialize to false in setup, triggering a change

//////////////////////////////
// Settings
//////////////////////////////
struct Settings {
  byte checkVal;
  double setPointC;
  boolean displayCelsius;
};

#define SETTINGS_WRITE_INTERVAL 10000
#define CHECK_VAL 0x69
Settings settings;
boolean displayCelsius = false;
long lastSettingsWrite;

//////////////////////////////
// Rotary Encoder
//////////////////////////////
#define ENCODER_A_PIN  2
#define ENCODER_B_PIN  3
#define ENCODER_READ_INTERVAL 250

volatile int encoderPos = 0;
int oldEncoderPos = 0;
long lastEncoderRead;
boolean encoderWasFast = false;

//////////////////////////////
// Toggle Button
//////////////////////////////
#define TOGGLE_BUTTON_PIN 4
#define TOGGLE_DEBOUNCE 200
// Note that the toggle is wired backwards, so the toggle logic is inverted
bool toggleState = false;
int togglePrevious = HIGH;
long lastToggleTime = 0;

void setup() {
  // Setup Serial output
  Serial.begin(9600);

  setupSettings();
  setupEncoder();
  setupToggle();

  // Setup Relay
  pinMode(RELAY_PIN, OUTPUT);
  setRelay(false);

  // Setup PID
  windowStartTime = millis();
  //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, WindowSize);
  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  // Setup LCD
  lcd.init();
  lcd.backlight();
  lcd.createChar(DEGREE_CHAR, degreeChar);
  lcd.createChar(UP_CHAR, upChar);

  lcd.setCursor(0, 0);
  lcd.print("S:");
  lcd.setCursor(8, 0);
  lcd.print("P:");
  lcd.setCursor(7, 1);
  lcd.print("P2:");
}

void loop() {

  readEncoder();
  updateToggleState();
  updateSettings();

  // Show setpoint temp
  lcd.setCursor(2, 0);
  printDegrees(setPointTempC);

  updateTemp();

  lcd.setCursor(10, 0); // probe 1 temp
  printDegrees(inputTempC);

  lcd.setCursor(10, 1); // probe 2 temp
  printDegrees(input2TempC);

  if ( isnan(inputTempC) ) {
    // No probe is connected, there is nothing to do.
    myPID.SetMode(MANUAL);
    pidOutput = 0;
  } else {
    // Show current temp
    lcd.setCursor(10, 0);
    printDegrees(inputTempC);

    // if we are more than BANG_ON degrees below the setpoint, then always turn on the relay.
    if ( inputTempC + BANG_ON_C < setPointTempC ) {
      myPID.SetMode(MANUAL);
      pidOutput = WindowSize;
    }
    // if we are more than BANG_OFF degrees above the setpoint, then always turn off
    else if ( inputTempC - BANG_OFF_C > setPointTempC ) {
      myPID.SetMode(MANUAL);
      pidOutput = 0;
    }
    else {
      myPID.SetMode(AUTOMATIC);
    }
  }

  myPID.Compute();

  // Show Output
  lcd.setCursor(0, 1);
  sprintf(degreesBuff, "%4d", (int)pidOutput);
  lcd.print(degreesBuff);

  // if we are inside the BANG range, turn on the relay for a portion of the Relay Window
  // based on the pidOutput
  unsigned long now = millis();
  if (now - windowStartTime > WindowSize) { //time to shift the Relay Window
    windowStartTime += WindowSize;
  }
  setRelay(pidOutput > now - windowStartTime);
}

////////////////////////////////
// Rotary Encoder
////////////////////////////////
void setupEncoder() {
  pinMode(ENCODER_A_PIN, INPUT);
  pinMode(ENCODER_B_PIN, INPUT);

  //digitalWrite(ENCODER_A_PIN, HIGH); // enable pull-up
  //digitalWrite(ENCODER_B_PIN, HIGH);

  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), doEncoderA, FALLING);

  // encoder pin on interrupt 1 (pin 3)
  //attachInterrupt(1, doEncoderB, CHANGE);
}

void readEncoder() {
  long now = millis();
  if (now - lastEncoderRead < ENCODER_READ_INTERVAL) return;
  lastEncoderRead = now;

  int currentPos = encoderPos;
  double delta = currentPos - oldEncoderPos;
  if (delta == 0) {
    encoderWasFast = false;
    return;
  }

  Serial.print("Encoder delta: ");
  Serial.println(delta);

  oldEncoderPos = currentPos;

  double deltaAbs = fabs(delta);
  double deltaSign = delta / deltaAbs;

  double displaySetPoint = setPointTempC;
  if (!displayCelsius) displaySetPoint = cToF(setPointTempC);

  // if it was a single click, change by one
  if (deltaAbs < 2) {
    if (!encoderWasFast) setDisplaySetPointTemp(displaySetPoint + delta);
    encoderWasFast = false;
    return;
  }

  // otherwise, change by 5 for each additional click, and round to nearest 5
  encoderWasFast = true;
  delta = (deltaAbs - 1) * deltaSign * 5;
  setDisplaySetPointTemp(round((displaySetPoint + delta) / 5) * 5);
}

// Interrupt on A changing state
void doEncoderA() {
  // Test transition
  bool encoderASet = digitalRead(ENCODER_A_PIN) == HIGH;
  bool encoderBSet = digitalRead(ENCODER_B_PIN) == HIGH;
  // and adjust counter + if A leads B
  encoderPos += (encoderASet != encoderBSet) ? +1 : -1;
}

////////////////////////////////
// Settings
////////////////////////////////
void setupSettings() {
  // Setup Settings
  EEPROM.get(0, settings);

  Serial.print("setupSettings ");
  printSettings();

  if (settings.checkVal == CHECK_VAL) {
    setPointTempC = settings.setPointC;
    displayCelsius = settings.displayCelsius;
  } else {
    setPointTempC = DEFAULT_SET_POINT_C;
    displayCelsius = false;
  }
  toggleState = displayCelsius;
}

void setDisplaySetPointTemp(double newSetPoint) {
  if (!displayCelsius) newSetPoint = fToC(newSetPoint);
  if (isnan(newSetPoint)) newSetPoint = DEFAULT_SET_POINT_C;

  newSetPoint = min(newSetPoint, MAX_SET_POINT_C);
  newSetPoint = max(newSetPoint, MIN_SET_POINT_C);
  if (newSetPoint == setPointTempC) return;

  setPointTempC = newSetPoint;

  Serial.print("New set point: ");
  Serial.print(setPointTempC);
  Serial.println("°C");
}

// Write the settings if theyhave changed. Will only write as often as SETTINGS_WRITE_INTERVAL.
// Should be called in loop.
void updateSettings() {
  long now = millis();
  if (now - lastSettingsWrite < SETTINGS_WRITE_INTERVAL) return;

  if (settings.checkVal != CHECK_VAL
      || settings.setPointC != setPointTempC
      || settings.displayCelsius != displayCelsius) {
    settings.checkVal = CHECK_VAL;
    settings.setPointC = setPointTempC;
    settings.displayCelsius = displayCelsius;

    Serial.print("updateSettings ");
    printSettings();
    EEPROM.put(0, settings);
    lastSettingsWrite = now;
  }
}

void printSettings() {
  Serial.print("checkVal: ");
  Serial.print(settings.checkVal);
  Serial.print(", setPointC: ");
  Serial.print(settings.setPointC);
  Serial.print(", displayCelsius: ");
  Serial.println(settings.displayCelsius);
}

////////////////////////////////
// Relay
////////////////////////////////
void setRelay(bool enabled) {
  if (enabled == oldRelayEnabled) return;

  oldRelayEnabled = enabled;

  // set the relay pin
  digitalWrite(RELAY_PIN, enabled ? HIGH : LOW);

  // set the lcd indicator
  lcd.setCursor(15, 0);
  if (enabled) lcd.write((byte) UP_CHAR);
  else lcd.print(" ");
}

////////////////////////////////
// Temperature
////////////////////////////////
void updateTemp() {
  int now = millis();
  if (now - lastTempReadTime < tempReadInterval) return false;
  lastTempReadTime = now;
  inputTempC = thermocouple.readCelsius();
  input2TempC = thermocouple2.readCelsius();
  //  Serial.print("Read temp 1:");
  //  Serial.print(inputTempC);
  //  Serial.print("°C");
  //  Serial.print(", temp2: ");
  //  Serial.print(input2TempC);
  //  Serial.println("°C");
}

void printDegrees(double tempC) {
  if ( isnan(tempC) ) {
    lcd.print("-----");
    return;
  }
  double temp = tempC;
  if (!displayCelsius) temp = cToF(tempC);
  temp = round(temp);
  sprintf(degreesBuff, "%3d", (int)temp);
  lcd.print(degreesBuff);
  lcd.write((byte)DEGREE_CHAR);
  if (displayCelsius) lcd.print("C");
  else(lcd.print("F"));
}

float cToF(float c) {
  return (c * 9 / 5) + 32;
}

float fToC(float f) {
  return (f - 32) * 5 / 9;
}

void setDisplayCelsius(boolean newDisplayCelsius) {
  if (displayCelsius == newDisplayCelsius) return;
  Serial.print("set displayCelsius: ");
  Serial.println(newDisplayCelsius);
  displayCelsius = newDisplayCelsius;
}

//////////////////////////////
// Toggle Button
//////////////////////////////
void setupToggle() {
  pinMode(TOGGLE_BUTTON_PIN, INPUT);
  lastToggleTime = millis();
  toggleState = displayCelsius;
}

void updateToggleState() {
  int toggleRead = digitalRead(TOGGLE_BUTTON_PIN);

  // if the input just went from HIGH to LOW and we've waited long enough
  // to ignore any noise on the circuit, toggle the output pin and remember
  // the time
  if (toggleRead == LOW && togglePrevious == HIGH && millis() - lastToggleTime > TOGGLE_DEBOUNCE) {
    toggleState = !toggleState;
    lastToggleTime = millis();
  }

  setDisplayCelsius(toggleState);

  togglePrevious = toggleRead;
}

