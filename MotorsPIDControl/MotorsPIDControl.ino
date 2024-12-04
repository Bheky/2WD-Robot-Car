/*
  MotorsPIDControl.ino
  Test Code v1 -- PID Control
  Author: Bheki Ndlovu
*/

#include "MotorsControl.h"

#include <ARB.h>
#include <Wire.h>

MotorsControl motorscontrol;

void setup() {
  ARBSetup(true);
  motorscontrol.initialize();
  Serial.begin(9600);  // Start serial for debugging
}

// This loop runs each robot component sequencially
void loop() {
  motorscontrol.runMotors();

  // Call the serialUpdate function at least once per loop
  serialUpdate();

  delay(0.1);
}