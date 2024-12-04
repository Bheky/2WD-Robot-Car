/* First draft for main file
*/

#include <ARB.h>
#include <Wire.h>

#include "InfraredSensors.h"
#include "UltrasonicSensors.h"
#include "MotorControl.h"
#include "Arduino.h"

enum STATE {
  START,
  MOVEFORWARD,
  ADJUSTBACKWARDS,
  ADJUSTRIGHT,
  ADJUSTLEFT
};

int wallFrontThreshold = 20;
int rightWallThreshold = 12;
int lowerLeftWallThreshold = 10;
int upperLeftWallThreshold = 20;
int middleLeftWallThreshold = 15;


STATE currentState;

// Instantiate the sensors
InfraredSensors rightIR(IR_1_BUS_NUMBER);
InfraredSensors leftIR(IR_2_BUS_NUMBER);
InfraredSensors frontLeftIR(IR_3_BUS_NUMBER);
InfraredSensors frontRightIR(IR_4_BUS_NUMBER);

UltrasonicSensors ultrasonicSensors;

// Create an instance of the motor control class
MotorControl motorControl;

//bool obstacleInfront();

void setup() {
  currentState = START;

  // Setup ARB and Serial Communication
  ARBSetup(true); 
  Serial.begin(9600);

  // Initialize sensors

  motorControl.initialize(); // Initialize motor control

  rightIR.initialize();
  leftIR.initialize();
  frontLeftIR.initialize();
  frontRightIR.initialize();

}

// Loop runs the robot component sequencially
void loop() {

  // Read ultrasonic sensors
  int frontUsDistance = ultrasonicSensors.readDistance(FRONT);
  int backUsDistance = ultrasonicSensors.readDistance(BACK);
  int backLeftUsDistance = ultrasonicSensors.readDistance(BACKLEFT);
  int backRightUsDistance = ultrasonicSensors.readDistance(BACKRIGHT);


  switch(currentState){
    case START:
      start();
      currentState = MOVEFORWARD;
      break;

    case MOVEFORWARD:
      moveForward();

      if(obstacleInFront()){
        currentState = ADJUSTBACKWARDS;
      } else if(wallOnTheRight()){
        currentState = ADJUSTLEFT;
      } else if(wallNotOnTheLeft()){
        currentState = ADJUSTLEFT;
      } else if(leftWallClose()){
        currentState = ADJUSTRIGHT;
      } else if(leftWallFar()){
        currentState = ADJUSTLEFT;
      } else {
        currentState = MOVEFORWARD;
      }

      break;

    case ADJUSTBACKWARDS:
      adjustBackwards();

      if(obstacleInFront()){
        currentState = ADJUSTBACKWARDS;
      } else if(wallOnTheRight()){
        currentState = ADJUSTLEFT;
      } else if(wallNotOnTheLeft()){
        currentState = MOVEFORWARD;
      } else if(leftWallClose()){
        currentState = ADJUSTRIGHT;
      } else if(leftWallFar()){
        currentState = ADJUSTLEFT;
      } else {
        currentState = MOVEFORWARD;
      }

      break;

    case ADJUSTRIGHT:
      adjustRight();

      if(obstacleInFront()){
        currentState = ADJUSTBACKWARDS;
      } else if(wallOnTheRight()){
        currentState = ADJUSTLEFT;
      } else if(wallNotOnTheLeft()){
        currentState = MOVEFORWARD;
      } else if(leftWallClose()){
        currentState = ADJUSTRIGHT;
      } else if(leftWallFar()){
        currentState = ADJUSTLEFT;
      } else {
        currentState = MOVEFORWARD;
      }

      break;

    case ADJUSTLEFT:
      adjustLeft();

      if(obstacleInFront()){
        currentState = ADJUSTBACKWARDS;
      } else if(wallOnTheRight()){ 
        currentState = ADJUSTLEFT;
      } else if(wallNotOnTheLeft()){
        currentState = MOVEFORWARD;
      } else if(leftWallClose()){
        currentState = ADJUSTRIGHT;
      } else if(leftWallFar()){
        currentState = ADJUSTLEFT;
      } else {
        currentState = MOVEFORWARD;
      }

      break;

    default:
      currentState = MOVEFORWARD;
      break;
  }

  // Call the serialUpdate function at least once per loop
  serialUpdate();

}

void start(){
  Serial.println("Starting Robot..");
}

// ACTUATE MOTORS IN THESE FUNCTIONS

void moveForward(){
  Serial.println("Moving Forward");
  motorControl.setRobotSpeed(150);
  motorControl.setRobotDirection(FORWARD);

  delay(700);
}

void adjustRight(){
  Serial.println("Adjusting Right");
  motorControl.setRobotDirection(RIGHT);
  motorControl.setRobotSpeed(100);

  delay(500);

  motorControl.setRobotDirection(FORWARD);
  motorControl.setRobotSpeed(150);

  delay(700);

}

void adjustLeft(){
  Serial.println("Adjusting Left");
  motorControl.setRobotDirection(LEFT);
  motorControl.setRobotSpeed(100);

  delay(500);

  motorControl.setRobotDirection(FORWARD);
  motorControl.setRobotSpeed(150); 

  delay(200);

}

void adjustBackwards(){
  Serial.println("Ajusting Backwards");
  motorControl.setRobotDirection(RIGHT);
  motorControl.setRobotSpeed(100);

  delay(500);

  motorControl.setRobotDirection(BACKWARD);
  motorControl.setRobotSpeed(100);

  delay(200);

  motorControl.setRobotDirection(FORWARD);
  motorControl.setRobotSpeed(150);
}


// Sensor check functions with error handling and print statements
bool obstacleInFront(){
  int frontUsDistance = ultrasonicSensors.readDistance(FRONT);

  if(frontUsDistance < 0 || frontUsDistance > 400) { // Check for valid range
    Serial.println("Error: Invalid front ultrasonic sensor reading");
    return false; // If invalid, assume no obstacle to avoid unnecessary stops
  }

  if(frontUsDistance <= wallFrontThreshold) {
    Serial.println("Obstacle detected in front!");
    return true;
  }
  else{
    return false;
  }
}

bool wallOnTheRight(){
  int rightIrDistance = rightIR.readIR();
  //int frontRightUsDistance = ultrasonicSensors.readDistance(FRONTRIGHT);

  if(rightIrDistance < 0 || rightIrDistance > 100) { // Check for valid range
  Serial.println("Error: Invalid right IR sensor reading");
  return false;
  }

  if(rightIrDistance <= rightWallThreshold) {
    return true;
  }
  else{
    return false;
  }

}

bool wallNotOnTheLeft(){
  int leftIrDistance = leftIR.readIR();
  int rightIrDistance = rightIR.readIR();

  if((leftIrDistance < 0 || leftIrDistance > 100) ||
     (rightIrDistance < 0 || rightIrDistance > 100)) {// Check for valid ranges
    Serial.println("Error: Invalid Ir sensor reading on left or right");
    return false; // If either reading is invalid, skip adjustment
  }

  if(leftIrDistance >= upperLeftWallThreshold && rightIrDistance <= rightWallThreshold) { 
    Serial.println("Right wall detected, no wall on the left");
    return true;
  }
  else{
    return false;
  }

}

bool leftWallClose(){
  int leftIrDistance = leftIR.readIR();

  if(leftIrDistance < 0 || leftIrDistance > 100) { // Check for valid range
    Serial.println("Error: Invalid left IR sensor reading");
    return false;
  }

  if(leftIrDistance <= lowerLeftWallThreshold) {
    Serial.println("Left wall close!");
    return true;
  }
  else{
    return false;
  }

}

bool leftWallFar(){
  int leftIrDistance = leftIR.readIR();

  if (leftIrDistance < 0 || leftIrDistance > 100) { // Check for valid range
    Serial.println("Error: Invalid left IR sensor reading");
    return false;
  }

  if(leftIrDistance <= upperLeftWallThreshold && leftIrDistance >= middleLeftWallThreshold) {
    Serial.println("Left wall far");
    return true;
  }
  else{
    return false;
  }
  
}