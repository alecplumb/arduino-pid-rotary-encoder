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

#define DEFAULT_SET_POINT 90.0
#define MIN_SET_POINT 32.0
#define MAX_SET_POINT 1200.0
#define BANG_ON 50.0
#define BANG_OFF 25.0

//////////////////////////////
// PID 
//////////////////////////////
double inputTemp; // °F
double setPointTemp = -1.0; // °F
double pidOutput;

#define KP 2
#define KI 5
#define KD 1

//Specify the links and initial tuning parameters
PID myPID(&inputTemp, &pidOutput, &setPointTemp, KP, KI, KD, P_ON_E, DIRECT);

char degreesBuff[5];
int tempReadInterval=500;
int lastTempReadTime = 0;

float oldSetPoint = -1.0;
float oldOutput = -1.0;

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
  double setPoint;
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

void setup() {
  // Setup Serial output
  Serial.begin(9600);

  setupSettings();
  setupEncoder();
  
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
  updateSettings();
    
  if(oldSetPoint != setPointTemp) {
    oldSetPoint = setPointTemp;
    lcd.setCursor(9, 0);
    printDegrees(setPointTemp);
  }
  
  if(oldOutput != pidOutput) {
    oldOutput = pidOutput;
    lcd.setCursor(0, 1);
    sprintf(degreesBuff, "%4d", (int)pidOutput);
    lcd.print(degreesBuff);
  }
  if(updateTemp()) {
    // go to line #1
    lcd.setCursor(9, 1);
    printDegrees(inputTemp);
  }

  myPID.Compute();
  
  // if we are more than BANG_ON degrees below the setpoint, then always turn on the relay.
  if( inputTemp + BANG_ON < setPointTemp ) setRelay(true);
  // if we are more than BANG_OFF degrees above the setpoint, then always turn
  else if( inputTemp - BANG_OFF > setPointTemp ) setRelay(false);
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
  
  // if it was a single click, change by one
  if(deltaAbs < 2) {
    if(!encoderWasFast) setSetPointTemp(setPointTemp + delta);
    encoderWasFast = false;
  return;
  }

  // otherwise, change by 5 for each additional click, and round to nearest 5
  encoderWasFast = true;
  delta = (deltaAbs - 1) * deltaSign * 5;
  setSetPointTemp(round((setPointTemp + delta) / 5) * 5);
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
    setSetPointTemp(settings.setPoint);
  } else {
    setSetPointTemp(DEFAULT_SET_POINT);
  }
}

void setSetPointTemp(double newSetPoint) {
  if(isnan(newSetPoint)) newSetPoint = DEFAULT_SET_POINT;
  newSetPoint = min(newSetPoint, MAX_SET_POINT);
  newSetPoint = max(newSetPoint, MIN_SET_POINT);
  if(newSetPoint == setPointTemp) return;

  setPointTemp = newSetPoint;
}

// Write the settings if theyhave changed. Will only write as often as SETTINGS_WRITE_INTERVAL.
// Should be called in loop.
void updateSettings() {
  long now = millis();
  if(now - lastSettingsWrite < SETTINGS_WRITE_INTERVAL) return;
  
  if(settings.checkVal != CHECK_VAL || settings.setPoint != setPointTemp) {
    settings.checkVal = CHECK_VAL;
    settings.setPoint = setPointTemp;

    Serial.print("updateSettings ");
    printSettings();
    EEPROM.put(0, settings);
    lastSettingsWrite = now;
  }
}

void printSettings() {
  Serial.print("checkVal: ");
  Serial.print(settings.checkVal);
  Serial.print(", setPoint: ");
  Serial.println(settings.setPoint);
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
  float newTemp = thermocouple.readFarenheit();
  //Serial.print("Read temp:");
  //Serial.print(newTemp);
  //Serial.println(" degrees F");
  if(newTemp == inputTemp) {
    return false;
  }
  inputTemp = newTemp;
  return true;
}

void printDegrees(double temp) {
  sprintf(degreesBuff, "%4d", (int)temp);
  lcd.print(degreesBuff);
  lcd.write((byte)DEGREE_CHAR);
  lcd.print("F");
}

float cToF(float c) {
  return (c * 9 / 5) + 32;
}

