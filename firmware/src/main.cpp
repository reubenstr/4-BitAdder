 /*
	4-Bit Adder

  Reuben Strangelove

	Designed: Summer 2015
	Created: Fall 2020

	4-Bit adder sums two 4-bit ports configured by the user via toggle switches.
	The resulting combinational logic is displayed via LEDs combined with engraved 
	graphical data paths.

	Upon no user activity, automatic mode randomly generates input values for effect.

	Microcontroller: 
    Arduino Nano (AtMega328p)	
	
	LED Drivers: 
    2x PCA9685 via I2C
*/

#include <Arduino.h>
#include "PCA9685.h" // https://github.com/NachtRaveVL/PCA9685-Arduino

PCA9685 pwmController1;
PCA9685 pwmController2;

// LED brigtness: 0 - 4095
#define MAX_PWM 2048

#define PIN_TOGGLE_A0 2
#define PIN_TOGGLE_A1 3
#define PIN_TOGGLE_A2 4
#define PIN_TOGGLE_A3 5
#define PIN_TOGGLE_B0 6
#define PIN_TOGGLE_B1 7
#define PIN_TOGGLE_B2 8
#define PIN_TOGGLE_B3 9

const unsigned int automaticModeValueSwitchDelayMs = 2000;
const unsigned int manualModeTimeoutDelayMs = 10000;

byte portA;
byte portB;
byte portS;

enum State
{
  manual,
  automatic
} state;

// If no activity over an interval then change state.
void CheckForTimeout(bool resetFlag)
{
  static unsigned long oldMillis;
  unsigned long currentMillis = millis();

  if (resetFlag)
  {
    oldMillis = currentMillis;
  }

  if (oldMillis + manualModeTimeoutDelayMs < currentMillis)
  {
    state = automatic;
  }
}

// Return true if toggle states changed.
bool CheckToggles()
{
  static byte oldPortATogglesState;
  static byte oldPortBTogglesState;

  byte portAToggles = 0;
  byte portBToggles = 0;

  bitWrite(portAToggles, 0, !digitalRead(PIN_TOGGLE_A0));
  bitWrite(portAToggles, 1, !digitalRead(PIN_TOGGLE_A1));
  bitWrite(portAToggles, 2, !digitalRead(PIN_TOGGLE_A2));
  bitWrite(portAToggles, 3, !digitalRead(PIN_TOGGLE_A3));

  bitWrite(portBToggles, 0, !digitalRead(PIN_TOGGLE_B0));
  bitWrite(portBToggles, 1, !digitalRead(PIN_TOGGLE_B1));
  bitWrite(portBToggles, 2, !digitalRead(PIN_TOGGLE_B2));
  bitWrite(portBToggles, 3, !digitalRead(PIN_TOGGLE_B3));

  bool changeFlag = portAToggles != oldPortATogglesState || portBToggles != oldPortBTogglesState;

  oldPortATogglesState = portAToggles;
  oldPortBTogglesState = portBToggles;

  if (changeFlag)
  {
    portA = portAToggles;
    portB = portBToggles;
    return true;
  }

  oldPortATogglesState = portAToggles;
  oldPortBTogglesState = portBToggles;

  return false;
}

// Sum input ports together;
void SumPorts()
{
  portS = portA + portB;
}

// Randomize value on ports on an interval.
void RandomPorts()
{
  static unsigned long oldMillis;
  unsigned long currentMillis = millis();

  if (oldMillis + automaticModeValueSwitchDelayMs < currentMillis)
  {
    oldMillis = currentMillis;

    portA = random(0, 16);
    portB = random(0, 16);
  }
}

// Update PWM values to reflect port and addition states.
void UpdatePWMs()
{
  uint16_t pwms1[16];
  uint16_t pwms2[16];

  for (int i = 0; i < 16; i++)
  {
    pwms1[i] = 0;
    pwms2[i] = 0;
  }

  // Generate carryout bits.
  byte carryOut = 0;
  bool carryIn = false;
  for (int i = 0; i < 4; i++)
  {
    carryIn = (i == 0) ? 0 : bitRead(carryOut, i - 1);
    bool carryResult = ((bitRead(portA, i) ^ bitRead(portB, i)) & carryIn) | (bitRead(portA, i) & bitRead(portB, i));
    bitWrite(carryOut, i, carryResult);
  }

  pwms1[0] = bitRead(portS, 3) ? MAX_PWM : 0;
  pwms1[1] = bitRead(portS, 2) ? MAX_PWM : 0;
  pwms1[2] = bitRead(portS, 1) ? MAX_PWM : 0;
  pwms1[3] = bitRead(portS, 0) ? MAX_PWM : 0;

  pwms1[4] = bitRead(carryOut, 3) ? MAX_PWM : 0;
  pwms1[5] = bitRead(carryOut, 2) ? MAX_PWM : 0;
  pwms1[6] = bitRead(carryOut, 1) ? MAX_PWM : 0;
  pwms1[7] = bitRead(carryOut, 0) ? MAX_PWM : 0;

  pwms1[8] = bitRead(portB, 3) ? MAX_PWM : 0;
  pwms1[9] = bitRead(portB, 2) ? MAX_PWM : 0;
  pwms1[10] = bitRead(portB, 1) ? MAX_PWM : 0;
  pwms1[11] = bitRead(portB, 0) ? MAX_PWM : 0;

  pwms1[12] = bitRead(portA, 3) ? MAX_PWM : 0;
  pwms1[13] = bitRead(portA, 2) ? MAX_PWM : 0;
  pwms1[14] = bitRead(portA, 1) ? MAX_PWM : 0;
  pwms1[15] = bitRead(portA, 0) ? MAX_PWM : 0;

  pwms2[0] = bitRead(portA, 3) ^ bitRead(portB, 3) ? MAX_PWM : 0;
  pwms2[1] = (bitRead(portA, 3) ^ bitRead(portB, 3)) & bitRead(carryOut, 2) ? MAX_PWM : 0;
  pwms2[2] = bitRead(portA, 3) & bitRead(portB, 3) ? MAX_PWM : 0;

  pwms2[3] = bitRead(portA, 2) ^ bitRead(portB, 2) ? MAX_PWM : 0;
  pwms2[4] = (bitRead(portA, 2) ^ bitRead(portB, 2)) & bitRead(carryOut, 1) ? MAX_PWM : 0;
  pwms2[5] = bitRead(portA, 2) & bitRead(portB, 2) ? MAX_PWM : 0;

  pwms2[6] = bitRead(portA, 1) ^ bitRead(portB, 1) ? MAX_PWM : 0;
  pwms2[7] = (bitRead(portA, 1) ^ bitRead(portB, 1)) & bitRead(carryOut, 0) ? MAX_PWM : 0;
  pwms2[8] = bitRead(portA, 1) & bitRead(portB, 1) ? MAX_PWM : 0;

  pwms2[9] = bitRead(portA, 0) ^ bitRead(portB, 0) ? MAX_PWM : 0;
  pwms2[10] = (bitRead(portA, 0) ^ bitRead(portB, 0)) & 0 ? MAX_PWM : 0;
  pwms2[11] = bitRead(portA, 0) & bitRead(portB, 0) ? MAX_PWM : 0;

  pwms2[14] = state == automatic ? MAX_PWM / 2 : 0;
  pwms2[15] = state == manual ? MAX_PWM : 0;

  pwmController1.setChannelsPWM(0, 16, pwms1);
  pwmController2.setChannelsPWM(0, 16, pwms2);
}

void setup()
{
  pinMode(PIN_TOGGLE_A0, INPUT_PULLUP);
  pinMode(PIN_TOGGLE_A1, INPUT_PULLUP);
  pinMode(PIN_TOGGLE_A2, INPUT_PULLUP);
  pinMode(PIN_TOGGLE_A3, INPUT_PULLUP);
  pinMode(PIN_TOGGLE_B0, INPUT_PULLUP);
  pinMode(PIN_TOGGLE_B1, INPUT_PULLUP);
  pinMode(PIN_TOGGLE_B2, INPUT_PULLUP);
  pinMode(PIN_TOGGLE_B3, INPUT_PULLUP);

  Wire.begin();

  pwmController1.resetDevices();
  pwmController2.resetDevices();
  pwmController1.init(0x40);
  pwmController1.setPWMFrequency(1500);
  pwmController2.init(0x41);
  pwmController2.setPWMFrequency(1500);

  state = automatic;
}

void loop()
{
  bool activityFlag = CheckToggles();

  if (activityFlag)
  {
    state = manual;
  }

  CheckForTimeout(activityFlag);

  if (state == automatic)
  {
    RandomPorts();
  }

  SumPorts();

  UpdatePWMs();
}