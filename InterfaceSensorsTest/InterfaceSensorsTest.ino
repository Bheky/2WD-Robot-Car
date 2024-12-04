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
InfraredSensors rightIR(IR_1_BUS_NUMBER);
InfraredSensors leftIR(IR_2_BUS_NUMBER);
InfraredSensors frontLeftIR(IR_3_BUS_NUMBER);
InfraredSensors frontRightIR(IR_4_BUS_NUMBER);

//UltrasonicSensors ultrasonicsensors;
UltrasonicSensors ultrasonicSensors;

// Create an instance of the motor control class
// MotorControl motorControl;

void setup() {
  // currentState = START;

  // Setup ARB and Serial Communication
  ARBSetup(true); 
  Serial.begin(9600);

  // Initialize sensors

  // motorControl.initialize(); // Initialize motor control

  rightIR.initialize();
  leftIR.initialize();
  frontLeftIR.initialize();
  frontRightIR.initialize();

}

// Loop runs the robot component sequencially
void loop() {

  // Read IR sensors
  int rightIrDistance = rightIR.readIR();
  int leftIrDistance = leftIR.readIR();
  int frontLeftIrDistance = frontLeftIR.readIR();
  int frontRightIrDistance = frontRightIR.readIR();
  
  // Read ultrasonic sensors
  //ultrasonicsensors.readUltrasonicSensors();
  int frontUsDistance = ultrasonicSensors.readDistance(FRONT);
  int backUsDistance = ultrasonicSensors.readDistance(BACK);
  int backLeftUsDistance = ultrasonicSensors.readDistance(BACKLEFT);
  int backRightUsDistance = ultrasonicSensors.readDistance(BACKRIGHT);

  Serial.print("US Distance Front: ");
  Serial.print(frontUsDistance);
  Serial.println(" cm.");

  Serial.print("US Distance Back: ");
  Serial.print(backUsDistance);
  Serial.println(" cm.");

  Serial.print("US Distance Back-Left: ");
  Serial.print(backLeftUsDistance);
  Serial.println(" cm.");

  Serial.print("US Distance Back-Right: ");
  Serial.print(backRightUsDistance);
  Serial.println(" cm.");

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

  //Print the sensor reading with appropriate labels
  Serial.print("IR Distance Right:");
  Serial.print(rightIrDistance); // Adjust for 1-based index in output
  Serial.println(" cm");

  // Print the sensor reading with appropriate labels
  Serial.print("IR Distance Left:");
  Serial.print(leftIrDistance); // Adjust for 1-based index in output
  Serial.println(" cm");

  // Print the sensor reading with appropriate labels
  Serial.print("IR Distance Front-Left:");
  Serial.print(frontLeftIrDistance); // Adjust for 1-based index in output
  Serial.println(" cm");

  // Print the sensor reading with appropriate labels
  Serial.print("IR Distance Front-Right:");
  Serial.print(frontRightIrDistance); // Adjust for 1-based index in output
  Serial.println(" cm");

  Serial.println("");

  // Call the serialUpdate function at least once per loop
  serialUpdate();

  delay(50); // Delay before the loop starts again

}