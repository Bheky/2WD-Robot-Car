/* First draft for controlling everything
*/

#include <ARB.h>
#include <Wire.h>

#include "InfraredSensors.h"
#include "UltrasonicSensors.h"
#include "MotorControl.h"
#include "Arduino.h"

// enum CurrentState {
//   START,
//   BACKWARDS,
//   Left,
//   Right,
//   Forward,
// }

// CurrentState currentState;

// Instantiate the sensors
// InfraredSensors rightIR(IR_1_BUS_NUMBER);
// InfraredSensors leftIR(IR_2_BUS_NUMBER);
// InfraredSensors backLeftIR(IR_3_BUS_NUMBER);
// InfraredSensors backRightIR(IR_4_BUS_NUMBER);

//UltrasonicSensors ultrasonicsensors;
// UltrasonicSensors ultrasonicSensors;

// Create an instance of the motor control class
MotorControl motorControl;

void setup() {
  // currentState = START;

  // Setup ARB and Serial Communication
  ARBSetup(true); 
  Serial.begin(9600);

  // Initialize sensors

  motorControl.initialize(); // Initialize motor control

  // rightIR.initialize();
  // leftIR.initialize();
  // backLeftIR.initialize();
  // backRightIR.initialize();

}

// Loop runs the robot component sequencially
void loop() {

  motorControl.setRobotDirection(FORWARD);
  motorControl.setRobotSpeed(200);
  delay(500);

  motorControl.setRobotSpeed(0);
  delay(500);

  motorControl.setRobotDirection(LEFT);
  motorControl.setRobotSpeed(150);
  delay(1000);

  motorControl.setRobotSpeed(0);
  delay(500);

  motorControl.setRobotDirection(BACKWARD);
  motorControl.setRobotSpeed(255);
  delay(500);

  motorControl.setRobotSpeed(0);
  delay(500);

  motorControl.setRobotDirection(RIGHT);
  motorControl.setRobotSpeed(150);
  delay(1000);

  motorControl.setRobotSpeed(0);
  delay(500);

  // Read IR sensors
  // int rightIrDistance = rightIR.readIR();
  // int leftIrDistance = leftIR.readIR();
  // int backLeftIrDistance = backLeftIR.readIR();
  // int backRightIrDistance = backRightIR.readIR();
  
  // // Read ultrasonic sensors
  // //ultrasonicsensors.readUltrasonicSensors();
  // int frontUsDistance = ultrasonicSensors.readDistance(FRONT);
  // int backUsDistance = ultrasonicSensors.readDistance(BACK);
  // int frontLeftUsDistance = ultrasonicSensors.readDistance(FRONTLEFT);
  // int frontRightUsDistance = ultrasonicSensors.readDistance(FRONTRIGHT);

  // switch(currentState){
  //   case CurrentState.Start:
  //     currentState = CurrentState.Forward;
  //     break;

  //   case CurrentState.Forward:
  //     motors.moveForward();

  //     distance1 = ultrasonicSensors.readDistance(0);

  //     if(distance1 == threshold){
  //       currentState = CurrentState.Forward;
  //     }


  //     break;

  //   case CurrentState.Left:
  //     break;

  //   case CurrentState.Right:
  //     break;

  //   case CurrentState.Backward:
  //     break;
  // }


  // Serial.println("");

  // Call the serialUpdate function at least once per loop
  serialUpdate();

  // delay(50); // Delay before the loop starts again

}