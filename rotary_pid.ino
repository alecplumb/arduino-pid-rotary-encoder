// this example is public domain. enjoy!
// www.ladyada.net/learn/sensors/thermocouple

#include <max6675.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include <PID_v1.h>
#include <EEPROM.h>

LiquidCrystal lcd(4,5,6,7,8,9); // RS, E, D4, D5, D6, D7

MAX6675 thermocouple(10, 11, 12); // CLK, CS, D0

#define RELAY_PIN 13

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
#define HAPPY_CHAR 2
byte happyChar[8] = {
  0b00000,
  0b00000,
  0b01010,
  0b00000,
  0b10001,
  0b01110,
  0b00000,
  0b00000
};

#define DEFAULT_SET_POINT_C 371.111  // 700°F
#define MIN_SET_POINT_C 0.0  // °C
#define MAX_SET_POINT_C 1024.0  // °C
#define BANG_ON_C 50.0 // Turn relay fully on if PV is more than this below the setpoint
#define BANG_OFF_C 25.0 // Turn relay fully off if PV is more than this above the setpoint

//////////////////////////////
// PID 
//////////////////////////////
double inputTempC; // °C
double setPointTempC = -1.0; // °C
double pidOutput;

// These tunings are almost entirely arbitrary, and should be tuned.
#define KP 2
#define KI 5
#define KD 1

//Specify the links and initial tuning parameters
PID myPID(&inputTempC, &pidOutput, &setPointTempC, KP, KI, KD, P_ON_E, DIRECT);

char degreesBuff[5];
int tempReadInterval=500;
int lastTempReadTime = 0;

float oldDisplaySetPointC = -1.0;
float oldOutput = -1.0;
boolean displayCelsius = false;

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
#define TOGGLE_BUTTON_PIN A0
#define TOGGLE_DEBOUNCE 200
boolean toggleState = false;
long lastTogglePush = 0;

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
  lcd.begin(16, 2);
  lcd.createChar(DEGREE_CHAR, degreeChar);
  lcd.createChar(UP_CHAR, upChar);
  lcd.createChar(HAPPY_CHAR, happyChar);
  
  lcd.setCursor(7, 0);
  lcd.print("S:");
  lcd.setCursor(7, 1);
  lcd.print("P:");
}

void loop() {

  readEncoder();
  updateToggleState();
  updateSettings();
  
  // Show setpoint temp
  lcd.setCursor(9, 0);
  printDegrees(setPointTempC);

  updateTemp()  ;
    
  // Show current temp
  lcd.setCursor(9, 1);
  printDegrees(inputTempC);

  myPID.Compute();
  
  // Show Output
  lcd.setCursor(0, 1);
  sprintf(degreesBuff, "%4d", (int)pidOutput);
  lcd.print(degreesBuff);
 
  // if we are more than BANG_ON degrees below the setpoint, then always turn on the relay.
  if( inputTempC + BANG_ON_C < setPointTempC ) setRelay(true);
  // if we are more than BANG_OFF degrees above the setpoint, then always turn
  else if( inputTempC - BANG_OFF_C > setPointTempC ) setRelay(false);
  else {
    // if we are inside the BANG range, turn on the relay for a portion of the Relay Window
    // based on the pidOutput
    unsigned long now = millis();
    if (now - windowStartTime > WindowSize) { //time to shift the Relay Window
      windowStartTime += WindowSize;
    }    
    setRelay(pidOutput > now - windowStartTime);
  }
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
  if(now - lastEncoderRead < ENCODER_READ_INTERVAL) return;
  lastEncoderRead = now;
  
  int currentPos = encoderPos;
  double delta = currentPos - oldEncoderPos;
  if(delta == 0) {
    encoderWasFast = false;
    return;
  }

  oldEncoderPos = currentPos;

  double deltaAbs = fabs(delta);
  double deltaSign = delta / deltaAbs;

  double displaySetPoint = setPointTempC;
  if(!displayCelsius) displaySetPoint = cToF(setPointTempC);
  
  // if it was a single click, change by one
  if(deltaAbs < 2) {
    if(!encoderWasFast) setDisplaySetPointTemp(displaySetPoint + delta);
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
  
  if(settings.checkVal == CHECK_VAL) {
    setPointTempC = settings.setPointC;
    displayCelsius = settings.displayCelsius;
  } else {
    setPointTempC = DEFAULT_SET_POINT_C;
    displayCelsius = false;
  }
  toggleState = displayCelsius;
}

void setDisplaySetPointTemp(double newSetPoint) {
  if(!displayCelsius) newSetPoint = fToC(newSetPoint);
  if(isnan(newSetPoint)) newSetPoint = DEFAULT_SET_POINT_C;
  
  newSetPoint = min(newSetPoint, MAX_SET_POINT_C);
  newSetPoint = max(newSetPoint, MIN_SET_POINT_C);
  if(newSetPoint == setPointTempC) return;

  setPointTempC = newSetPoint;
}

// Write the settings if theyhave changed. Will only write as often as SETTINGS_WRITE_INTERVAL.
// Should be called in loop.
void updateSettings() {
  long now = millis();
  if(now - lastSettingsWrite < SETTINGS_WRITE_INTERVAL) return;
  
  if(settings.checkVal != CHECK_VAL 
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

void setRelay(bool enabled) {
  if(enabled == oldRelayEnabled) return;

  oldRelayEnabled = enabled;
  
  //Serial.print("Relay Enabled: ");
  //Serial.println(enabled);
  
  // set the relay pin
  digitalWrite(RELAY_PIN, enabled ? HIGH : LOW);

  // set the lcd indicator
  lcd.setCursor(15, 1);
  if(enabled) lcd.write((byte) UP_CHAR);
  else lcd.print(" ");
}

//return true if the temp has changed.
bool updateTemp() {
  int now = millis();
  if(now - lastTempReadTime < tempReadInterval) return false;
  lastTempReadTime = now;
  float newTemp = thermocouple.readCelsius();
  //Serial.print("Read temp:");
  //Serial.print(newTemp);
  //Serial.println(" degrees C");
  if(newTemp == inputTempC) {
    return false;
  }
  inputTempC = newTemp;
  return true;
}

void printDegrees(double tempC) {
  double temp = tempC;
  if(!displayCelsius) temp = cToF(tempC);
  temp = round(temp);
  sprintf(degreesBuff, "%4d", (int)temp);
  lcd.print(degreesBuff);
  lcd.write((byte)DEGREE_CHAR);
  if(displayCelsius) lcd.print("C");
  else(lcd.print("F"));
}

float cToF(float c) {
  return (c * 9 / 5) + 32;
}

float fToC(float f) {
  return (f - 32) * 5 / 9;
}

void setDisplayCelsius(boolean newDisplayCelsius) {
  if(newDisplayCelsius == displayCelsius) return;
  displayCelsius = newDisplayCelsius;
  Serial.print("displayCelsius: ");
  Serial.println(displayCelsius);
  oldDisplaySetPointC = -1.0;
  oldOutput = -1.0;
}

//////////////////////////////
// Toggle Button
//////////////////////////////
void setupToggle() {
  pinMode(TOGGLE_BUTTON_PIN, INPUT);
}

void updateToggleState() {
  boolean isPushed = digitalRead(TOGGLE_BUTTON_PIN);
  long now = millis();
  
  if(isPushed && now - lastTogglePush > TOGGLE_DEBOUNCE) {
    Serial.println("toggle pushed");
    toggleState = !toggleState;
    lastTogglePush = now;
    setDisplayCelsius(toggleState);
  }
}

