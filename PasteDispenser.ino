#include <math.h>
#include <EEPROM.h>
#include <AccelStepper.h>
#include <TM1637Display.h>
#include <Encoder.h>

#define encoderPinDT 2  // Must be 2! Do not change!
#define encoderPinCLK 3 // Must be 3! Do not change!
#define enaPin 4
#define dirPin 5
#define stpPin 6
#define dosePin 7
#define scrClkPin 8
#define scrDioPin 9
#define encoderPinSW 10
#define doseFineTunePin 11
#define manualDownPin 12
#define manualUpPin A3 // by some reason 13 pin wount work
#define maxSpeedPin A0
#define accelerationPin A1

Encoder encoder(encoderPinDT, encoderPinCLK);
AccelStepper stepper(AccelStepper::DRIVER, stpPin, dirPin);
TM1637Display screen(scrClkPin, scrDioPin);
int loopCounter = 0;
long mililiters = 0;
const long mililitersMin = 10;
const long mililitersMax = 3000;
const long mililitersStep = 50;
const long mililitersStepLow = 5;
const int encoderStep = 4;
long prevEncoderPosition = 0;
int maxSpeed = 50; // steps per second
int acceleration = 50;
long stepsFor100ml = 1333;
const long stepsFor100mlDefault = 1333;

// max speed limits
const int maxSpeedFrom = 50;
const int maxSpeedTo = 3000;

// acceleration limits
const int accelerationFrom = 50;
const int accelerationTo = 10000;

const int stepsForRev = 400; // steps for 1 revolution
bool isMotorEnabled = false;
bool isDoseFineTune = false;
bool isManualDown = false;
bool isManualUp = false;

// buttons processing additional variables
const long buttonRepressDelay = 300; // mills between presses
const long buttonLongPressDelay = 2000;
unsigned long dosePressedTime = 0;
boolean dosePressDetected = false;
boolean doseReleaseDetected = true;
unsigned long doseFineTunePressedTime = 0;
boolean doseFineTunePressDetected = false;
boolean doseFineTuneReleaseDetected = true;

void setup() {
  // Serial.begin(115200);

  pinMode(encoderPinSW, INPUT_PULLUP);
  digitalWrite(encoderPinSW, HIGH);

  pinMode(enaPin, OUTPUT);
  digitalWrite(enaPin, HIGH);

  pinMode(dosePin, INPUT_PULLUP);
  digitalWrite(dosePin, HIGH);

  pinMode(doseFineTunePin, INPUT_PULLUP);
  digitalWrite(doseFineTunePin, HIGH);

  pinMode(manualDownPin, INPUT_PULLUP);
  digitalWrite(manualDownPin, HIGH);

  pinMode(manualUpPin, INPUT_PULLUP);
  digitalWrite(manualUpPin, HIGH);

  screen.setBrightness(5);
  stepper.setMinPulseWidth(1);
  
  loadSettings();
}

void processDoseButton() {
  if (isManualDown || isManualUp) {
    return;
  }
  
  int value = digitalRead(dosePin);

  if (doseReleaseDetected) {
    if (value == LOW) { // if pressed
      // Serial.println("dose button pressed");
      if (millis() - dosePressedTime > buttonRepressDelay) {
        dosePressedTime = millis();
        dosePressDetected = true;
        doseReleaseDetected = false;

        if (isDoseFineTune) {
          onDoseFineTuneClick();
        } else {
          callMotor();
        }
      }
    }
  } else if (dosePressDetected) {
    if (value == HIGH) { // if released
      dosePressDetected = false;
      doseReleaseDetected = true;
    }
  }
}

void processDoseFineTuneButton() {
  if (stepper.distanceToGo() != 0 || isManualDown || isManualUp) {
    return;
  }

  int value = digitalRead(doseFineTunePin);

  if (doseFineTuneReleaseDetected) {
    if (value == LOW) { // if pressed
      // Serial.println("fine tune button pressed");
      if (millis() - doseFineTunePressedTime > buttonRepressDelay) {
        doseFineTunePressedTime = millis();
        doseFineTunePressDetected = true;
        doseFineTuneReleaseDetected = false;
        onDoseFineTuneClick();
      }
    }
  } else if (doseFineTunePressDetected) {
    if (value == HIGH) { // if released
      doseFineTunePressDetected = false;
      doseFineTuneReleaseDetected = true;
    } else if (isDoseFineTune && millis() - doseFineTunePressedTime > buttonLongPressDelay) {
      doseFineTunePressedTime = millis();
      // on long press reset stepsFor100ml to default
      stepsFor100ml = stepsFor100mlDefault;
    }
  }
}

void processManualControl() {
  if (isDoseFineTune) {
    return;
  }
  
  boolean wasMoving = isManualDown || isManualUp;
  isManualDown = digitalRead(manualDownPin) == LOW;
  isManualUp = digitalRead(manualUpPin) == LOW;

  if (isManualDown) {
    // Serial.println("manual down button pressed");
    if (stepper.distanceToGo() == 0) {
      readMaxSpeed();
      readAcceleration();
      stepper.setMaxSpeed(maxSpeed);
      stepper.setAcceleration(acceleration);
      stepper.move((long)100000 * stepsForRev);
    }
  } else if (isManualUp) {
    // Serial.println("manual up button pressed");
    if (stepper.distanceToGo() == 0) {
      readMaxSpeed();
      readAcceleration();
      stepper.setMaxSpeed(maxSpeed);
      stepper.setAcceleration(acceleration);
      stepper.move((long)100000 * -stepsForRev);
    }
  } else if (wasMoving) {
    // Serial.println("manual control stopped");
    stepper.setAcceleration(accelerationTo); // set high acceleration to stop quickly
    stepper.stop();
  }
}

void onDoseFineTuneClick()
{
  isDoseFineTune = !isDoseFineTune;

  if (!isDoseFineTune) {
    EEPROM.put(sizeof(mililiters), stepsFor100ml); // save dose multiplier next to mililiters memory
  }
}

void callMotor() {
  if (stepper.distanceToGo() == 0) {
    if (mililiters > 0) {
      readMaxSpeed();
      readAcceleration();
      stepper.setMaxSpeed(maxSpeed);
      stepper.setAcceleration(acceleration);
      stepper.move(mililiters * stepsFor100ml / 100);
    }
  } else {
    stepper.setAcceleration(accelerationTo); // set high acceleration to stop quickly
    stepper.stop();
  }
}

void enableMotor() {
  isMotorEnabled = true;
  digitalWrite(enaPin, LOW);
}

void disableMotor() {
  isMotorEnabled = false;
  digitalWrite(enaPin, HIGH);
}

void onMotorStopped() {
  prevEncoderPosition = 0;
  encoder.write(0);
}

void readDoseFineTune() {
  long newPosition = encoder.read() / encoderStep;
  
  if (newPosition < prevEncoderPosition) {
    prevEncoderPosition = newPosition;
    stepsFor100ml -= 1;
    if (stepsFor100ml < 100) {
      stepsFor100ml = 100;
    }
  } else if (newPosition > prevEncoderPosition) {
    prevEncoderPosition = newPosition;
    stepsFor100ml += 1;
  }
}

void readMililiters() {
  long newPosition = encoder.read() / encoderStep;
  
  if (newPosition < prevEncoderPosition) {
    prevEncoderPosition = newPosition;
    decreaseMililiters();
    EEPROM.put(0, mililiters);
  } else if (newPosition > prevEncoderPosition) {
    prevEncoderPosition = newPosition;
    increaseMililiters();
    EEPROM.put(0, mililiters);
  }
}

void increaseMililiters() {
  int step = getMililitersStepSize();
  mililiters += step;
  mililiters = mililiters / step * step;

  if (mililiters > mililitersMax) {
    mililiters = mililitersMax;
  }
}

void decreaseMililiters() {
  int step = getMililitersStepSize();
  mililiters -= step;
  mililiters = mililiters / step * step;

  if (mililiters < mililitersMin) {
    mililiters = mililitersMin;
  }
}

void displayNumber(long value) {
  screen.showNumberDec(value, false);
}

void loadSettings() {
  // mililiters
  EEPROM.get(0, mililiters); // get from flash memory

  if (mililiters < mililitersMin) {
    mililiters = mililitersMin;
  } else if (mililiters > mililitersMax) {
    mililiters = mililitersMax;
  }

  // multiplier
  EEPROM.get(sizeof(mililiters), stepsFor100ml);
}

int getMililitersStepSize() {
  if (digitalRead(encoderPinSW)) {
    return mililitersStep;
  }
  return mililitersStepLow;
}

int getPotentiometerValue(uint8_t pinNumber) {
  int readings[30];
  int readingsCount = 30;
  long total = 0;
  int average = -1;
  int averageCount = 0;
  float deviationLimit = 0.25;
  
  while (average < 0) {
    for(int i = 0; i < readingsCount; i++) {
      delay(1);
      readings[i] = analogRead(pinNumber) + 2; // add 2 to avoid zeros
      total += readings[i];
    }
    
    average = total / readingsCount;
    averageCount = 0;
    total = 0;
    
    for(int i = 0; i < readingsCount; i++) {
      if (abs(((float)(readings[i] - average)) / average) < deviationLimit) {
        total += readings[i];
        averageCount++;
      }
    }

    if (averageCount > 0) {
      average = total / averageCount;
    } else {
      average = -1;
    }
  }

  return average - 2; // subtract added 2 above
}

void readMaxSpeed() {
  int value = getPotentiometerValue(maxSpeedPin);
  // Serial.println("readMaxSpeed " + String(value));
  maxSpeed = map(value, 0, 1023, maxSpeedFrom, maxSpeedTo);
}

void readAcceleration() {
  int value = getPotentiometerValue(accelerationPin);
  // Serial.println("readAcceleration " + String(value));
  acceleration = map(value, 0, 1023, accelerationFrom, accelerationTo);
}

void loop() {
  // update loop counter
  loopCounter++;
  if (loopCounter > 200) {
    loopCounter = 0;
  }

  if (loopCounter == 200) {
    if (stepper.distanceToGo() == 0 && !isMotorEnabled) {
      if (isDoseFineTune) {
        readDoseFineTune();
        displayNumber(stepsFor100ml);
      } else {
        readMililiters();
        displayNumber(mililiters);
      }
    }

    processDoseButton();
    processDoseFineTuneButton();
    processManualControl();

    if (stepper.distanceToGo() != 0) {
      enableMotor();
    } else {
      if (isMotorEnabled) {
        onMotorStopped();
      }
      disableMotor();
    }
  }
  
  stepper.run();
}
