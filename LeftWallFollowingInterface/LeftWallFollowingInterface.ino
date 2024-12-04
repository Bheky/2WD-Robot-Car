/* First draft for main file
*/

#include <ARB.h>
#include <Wire.h>

#include "InfraredSensors.h"
#include "UltrasonicSensors.h"
#include "MotorControl.h"
#include "PDController.h"
#include "Arduino.h"

int wallFrontThreshold = 15;
int rightWallThreshold = 7;
int lowerLeftWallThreshold = 5;
int upperLeftWallThreshold = 10;
int middleLeftWallThreshold = 7;

// Thresholds for the side range sensors
int frontRightIrThreshold = 8;
int frontLeftIrThreshold = 8;
int backRightUsThreshold = 7;
int backLeftUsThreshold = 7; 
int backUsThreshold = 7;

enum STATE {
  START,
  MOVEFORWARD,
  ADJUSTBACKWARDS,
  ADJUSTFORWARD,
  ADJUSTRIGHT,
  ADJUSTLEFT
};

STATE currentState;

// Instantiate the sensors
InfraredSensors rightIR(IR_1_BUS_NUMBER);
InfraredSensors leftIR(IR_2_BUS_NUMBER);
InfraredSensors frontLeftIR(IR_3_BUS_NUMBER);
InfraredSensors frontRightIR(IR_4_BUS_NUMBER);

UltrasonicSensors ultrasonicSensors;

// Create an instance of the motor control class
MotorControl motorControl;

PDController pdController(0.47, 0.2); // Set Kp = 0.47, Kd = 0.2

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
  
  delay(2000);
}

// Loop runs the robot component sequencially
void loop() {
  motorControl.update(); // Update encoder values

  // Set desired speed
  float desiredSpeed = 150;
  motorControl.setSpeedWithControl(desiredSpeed);

  // Read ultrasonic sensors
  int frontUsDistance = ultrasonicSensors.readDistance(FRONT);
  int backUsDistance = ultrasonicSensors.readDistance(BACK);
  int backLeftUsDistance = ultrasonicSensors.readDistance(BACKLEFT);
  int backRightUsDistance = ultrasonicSensors.readDistance(BACKRIGHT);


  // State machine for robot behavior
  switch(currentState){
    case START:
      start();
      currentState = MOVEFORWARD;
      break;

    case MOVEFORWARD:
      moveForward();
      checkObstacles();
      break;

    case ADJUSTBACKWARDS:
      adjustBackwards();
      checkObstacles();
      break;

    case ADJUSTFORWARD:
      adjustForward();
      checkObstacles();
      break;

    case ADJUSTRIGHT:
      adjustRight();
      checkObstacles();
      break;


    case ADJUSTLEFT:
      adjustLeft();
      checkObstacles();
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
}

void adjustRight(){
  Serial.println("Adjusting Right");
  motorControl.setRobotDirection(RIGHT);
  motorControl.setRobotSpeed(100);

  // Wait until the adjustment is completed using encoder feedback
  waitForEncoderSteps(200);
  motorControl.setRobotDirection(FORWARD);
  motorControl.setRobotSpeed(150);
}

void adjustLeft(){
  Serial.println("Adjusting Left");
  motorControl.setRobotDirection(LEFT);
  motorControl.setRobotSpeed(100);

  // Wait until the adjustment is completed using encoder feedback
  waitForEncoderSteps(200);
  motorControl.setRobotDirection(FORWARD);
  motorControl.setRobotSpeed(150);
}

void adjustBackwards(){
  Serial.println("Ajusting Backwards");
  motorControl.setRobotDirection(BACKWARD);
  motorControl.setRobotSpeed(100);

  // Wait until the adjustment is completed using encoder feedback
  waitForEncoderSteps(200);
  motorControl.setRobotDirection(FORWARD);
  motorControl.setRobotSpeed(150);
}

void adjustForward(){
  Serial.println("Adjusting Forward");
  motorControl.setRobotDirection(FORWARD);
  motorControl.setRobotSpeed(150);
}

// Wait for a specific number of encoder steps
void waitForEncoderSteps(int steps) {
  int initialStepsA = MotorControl::stepsA;
  int initialStepsB = MotorControl::stepsB;

  while(abs(MotorControl::stepsA - initialStepsA) < steps &&
        abs(MotorControl::stepsB - initialStepsA) < steps) {
      motorControl.setSpeedWithControl(100); 
      delay(10);
   }
}

// Check for obstacles and change state accordingly
void checkObstacles() {
  if(obstacleInFront()){
    currentState = ADJUSTBACKWARDS;
  } else if(obstacleInBack()){
    currentState = ADJUSTFORWARD;
  } else if(wallOnTheRight()){
    currentState = ADJUSTLEFT;
  } else if(!wallOnTheLeft()){
    currentState = ADJUSTLEFT;
  } else if(leftWallClose()){
    currentState = ADJUSTRIGHT;
  } else {
    currentState = MOVEFORWARD;
  }
}

// Sensor check functions
bool obstacleInFront(){
  int frontUsDistance = ultrasonicSensors.readDistance(FRONT);
  int frontIrRightDistance = frontRightIR.readIR();
  int frontIrLeftDistance = frontLeftIR.readIR();


  if(frontUsDistance < 0 || frontUsDistance > 400) { // Check for valid range
    Serial.println("Error: Invalid front ultrasonic sensor reading");
    return false; // If invalid, assume no obstacle to avoid unnecessary stops
  }

  if((frontIrLeftDistance < 0 || frontIrLeftDistance > 100) &&
     (frontIrRightDistance < 0 || frontIrRightDistance > 100)) {// Check for valid ranges
    Serial.println("Error: Invalid Ir sensor reading on left or right");
    return false; // If either reading is invalid, skip adjustment
  }
  
  // Check for obstacle in front
  bool frontObstacle = (frontUsDistance <= wallFrontThreshold);
  bool frontleftObstacle = (frontIrLeftDistance <= frontLeftIrThreshold);
  bool frontrightObstacle = (frontIrRightDistance <= frontRightIrThreshold);

  return frontObstacle || frontleftObstacle || frontrightObstacle;
}

bool wallOnTheRight(){
  int rightIrDistance = rightIR.readIR();

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

bool wallOnTheLeft(){
  int leftIrDistance = leftIR.readIR();
  int rightIrDistance = rightIR.readIR();

  if((leftIrDistance < 0 || leftIrDistance > 100) ||
     (rightIrDistance < 0 || rightIrDistance > 100)) {// Check for valid ranges
    Serial.println("Error: Invalid Ir sensor reading on left or right");
    return false; // If either reading is invalid, skip adjustment
  }

  // Check if the left wall is within a reasonable distance
  if (leftIrDistance > lowerLeftWallThreshold && leftIrDistance <= upperLeftWallThreshold) {
    Serial.println("Wall detected on the left within safe range.");
    return true; // Wall is on the left
  } else {
    Serial.println("No wall detected within range on the left.");
    return false;
  }
}

bool leftWallClose() {
  int leftIrDistance = leftIR.readIR();
  
  // Validate sensor reading
  if (leftIrDistance < 0 || leftIrDistance > 100) {
    Serial.println("Error: Invalid IR sensor reading on left");
    return false;
  }

  // Check if the left wall is too close
  if (leftIrDistance <= lowerLeftWallThreshold) {
    Serial.println("Left wall is too close!");
    return true;
  }

  return false; // Left wall is not too close
}

bool leftWallFar() {
  int leftIrDistance = leftIR.readIR();
  int rightIrDistance = rightIR.readIR();

  if((leftIrDistance < 0 || leftIrDistance > 100) ||
     (rightIrDistance < 0 || rightIrDistance > 100)) {// Check for valid ranges
    Serial.println("Error: Invalid Ir sensor reading on left or right");
    return false; // If either reading is invalid, skip adjustment
  }

  // Check if the left wall is too far
  if (leftIrDistance > upperLeftWallThreshold && rightIrDistance <= rightWallThreshold ) {
    Serial.println("Left wall is too far!");
    return true;
  }

  return false; // Left wall is not too far
}

bool obstacleInBack() {
  int backUsDistance = ultrasonicSensors.readDistance(BACK);
  int backLeftUsDistance = ultrasonicSensors.readDistance(BACKLEFT);
  int backRightUsDistance = ultrasonicSensors.readDistance(BACKRIGHT);

  if (backUsDistance < 0 || backUsDistance > 400) {
    Serial.println("Error: Invalid back ultrasonic sensor reading");
    return false;
  }

  if (backLeftUsDistance < 0 || backLeftUsDistance > 400) {
    Serial.println("Error: Invalid back left ultrasonic sensor reading");
    return false;
  }

  if (backRightUsDistance < 0 || backRightUsDistance > 400) {
    Serial.println("Error: Invalid back right ultrasonic sensor reading");
    return false;
  }

  if (backUsDistance <= backUsThreshold || backLeftUsDistance <= backLeftUsThreshold || backRightUsDistance <= backRightUsThreshold) {
    Serial.println("Obstacle detected at the back!");
    return true;
  }
  return false;
}
