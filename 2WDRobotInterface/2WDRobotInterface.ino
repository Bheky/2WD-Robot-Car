/* First draft for interface.ino file
*/

#include <ARB.h>
#include <Wire.h>

#include "InfraredSensors.h"
#include "UltrasonicSensors.h"
#include "MotorControl.h"
#include "Arduino.h"

int wallFrontThreshold = 15;
int rightWallThreshold = 7;
int lowerLeftWallThreshold = 5;
int upperLeftWallThreshold = 14;
int middleLeftWallThreshold = 19;

// Thresholds for the side range sensors
int frontRightIrThreshold = 10;
int frontLeftIrThreshold = 10;
int backRightUsThreshold = 8;
int backLeftUsThreshold = 8; 

// Define robot states
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

void setup() {
  currentState = START;

  // Setup ARB and Serial Communication
  ARBSetup(true); 
  Serial.begin(9600);

  // Initialize sensors and motor control
  motorControl.initialize(); 
  rightIR.initialize();
  leftIR.initialize();
  frontLeftIR.initialize();
  frontRightIR.initialize();

  Serial.println("Setup complete.");
  delay(2000); // 2-second delay for stabilization
}

// Loop runs the robot component sequencially
void loop() {

  // Read ultrasonic sensors
  int frontUsDistance = ultrasonicSensors.readDistance(FRONT);
  int backUsDistance = ultrasonicSensors.readDistance(BACK);
  int backLeftUsDistance = ultrasonicSensors.readDistance(BACKLEFT);
  int backRightUsDistance = ultrasonicSensors.readDistance(BACKRIGHT);

  // Appply PID control
  motorControl.applyPD();

  //Update encoder values and print debug info
  motorControl.updateEncoders();
  motorControl.printDebugInfo();

  // State machine for robot behavior
  switch(currentState){
    case START:
      start();
      currentState = MOVEFORWARD;
      break;

    case MOVEFORWARD:
      moveForward();

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
      break;

    case ADJUSTBACKWARDS:
      adjustBackwards();

      if(obstacleInFront()){
        currentState = ADJUSTBACKWARDS;
      } else if(obstacleInBack()){
        currentState = ADJUSTFORWARD;
      } else if(wallOnTheRight()){
        currentState = ADJUSTLEFT;
      } else if(!wallOnTheLeft()){
        currentState = MOVEFORWARD;
      } else if(leftWallClose()){
        currentState = ADJUSTRIGHT;
      } else if(leftWallFar()){
        currentState = ADJUSTLEFT;
      } else {
        currentState = MOVEFORWARD;
      }
      break;

    case ADJUSTFORWARD:
      adjustForward();

      if(obstacleInFront()){
        currentState = ADJUSTBACKWARDS;
      } else if (obstacleInBack()){
        currentState = ADJUSTFORWARD;
      } else if(wallOnTheRight()){
        currentState = ADJUSTLEFT;
      } else if(!wallOnTheLeft()){
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

    if (obstacleInFront()) {
      currentState = ADJUSTBACKWARDS;
    } else if (obstacleInBack()) {
      currentState = ADJUSTFORWARD;
    } else if (wallOnTheRight()) {
      currentState = ADJUSTLEFT;
    } else if (!wallOnTheLeft()) {
      currentState = MOVEFORWARD;
    } else if (leftWallClose()) {
      currentState = ADJUSTRIGHT;
    } else if (leftWallFar()) {
      currentState = ADJUSTLEFT;
    } else {
      currentState = MOVEFORWARD;
    }
    break;


    case ADJUSTLEFT:
      adjustLeft();

      if(obstacleInFront()){
        currentState = ADJUSTBACKWARDS;
      } else if(obstacleInBack()){
        currentState = ADJUSTFORWARD;
      } else if(wallOnTheRight()){ 
        currentState = ADJUSTLEFT;
      } else if(!wallOnTheLeft()){
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

// State action functions

void start(){
  Serial.println("Starting Robot..");
}

void moveForward(){
  Serial.println("Moving Forward");
  motorControl.setRobotSpeed(150, 150);
  motorControl.setRobotDirection(FORWARD);
  delay(500);
}

void adjustRight(){
  Serial.println("Adjusting Right.");
  motorControl.setRobotDirection(FORWARD);
  motorControl.setRobotSpeed(150, 150);
  delay(500);
}

void adjustLeft(){
  Serial.println("Adjusting Left.");
  motorControl.setRobotDirection(FORWARD);
  motorControl.setRobotSpeed(150, 150); 
  delay(500);
}

void adjustBackwards(){
  Serial.println("Ajusting Backwards.");
  motorControl.setRobotDirection(BACKWARD);
  motorControl.setRobotSpeed(100, 100);
  delay(100);
}

void adjustForward() {
  Serial.println("Adjust Forward");
  motorControl.setRobotDirection(FORWARD);
  motorControl.setRobotSpeed(150, 150);
  delay(500);
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

  if((frontIrLeftDistance < 0 || frontIrLeftDistance > 100) ||
     (frontIrRightDistance < 0 || frontIrRightDistance > 100)) {// Check for valid ranges
    Serial.println("Error: Invalid Ir sensor reading on left or right");
    return false; // If either reading is invalid, skip adjustment
  }
  
  // Check for obstacle in front
  bool frontObstacle = (frontUsDistance <= wallFrontThreshold);
  bool frontleftObstacle = (frontIrLeftDistance <= frontLeftIrThreshold);
  bool frontrightObstacle = (frontIrRightDistance <= frontRightIrThreshold);

  if(frontObstacle || frontleftObstacle || frontrightObstacle) {
    Serial.println("Obstacle detected in front!");
    return true;
  }
  else{
    return false;
  }
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
  if (leftIrDistance > upperLeftWallThreshold || rightIrDistance <= rightWallThreshold ) {
    Serial.println("Left wall is too far!");
    return true;
  }

  return false; // Left wall is not too far
}

bool obstacleInBack() {
  int backLeftUsDistance = ultrasonicSensors.readDistance(BACKLEFT);
  int backRightUsDistance = ultrasonicSensors.readDistance(BACKRIGHT);

  if (backLeftUsDistance < 0 || backLeftUsDistance > 400) {
    Serial.println("Error: Invalid back left ultrasonic sensor reading");
    return false;
  }

  if (backRightUsDistance < 0 || backRightUsDistance > 400) {
    Serial.println("Error: Invalid back right ultrasonic sensor reading");
    return false;
  }

  if (backLeftUsDistance <= backLeftUsThreshold || backRightUsDistance <= backRightUsThreshold) {
    Serial.println("Obstacle detected at the back!");
    return true;
  }
  return false;
}

