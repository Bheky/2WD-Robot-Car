#include "MotorControl.h"
#include <ARB.h>

// Constants for encoder-based distance calculation
const float WHEEL_CIRCUMFERENCE_CM = 46.0; // Wheel circumference in cm
const int ENCODER_TICKS_PER_REVOLUTION = 12; // Encoder ticks per revolution

// Initialize static variables
volatile int MotorControl::stepsA = 0;
volatile int MotorControl::stepsB = 0;
volatile int MotorControl::lastStateA = 0;
volatile int MotorControl::lastStateB = 0;
Direction MotorControl::dirA = CW;
Direction MotorControl::dirB = CW;

// Constructor
MotorControl::MotorControl() : currentSpeedLeft(0), currentSpeedRight(0), distanceA(0), distanceB(0) {} 

// Initialization
void MotorControl::initialize(){
  // Set PinModes
  pinMode(MOTOR_DIRA, OUTPUT);
  pinMode(MOTOR_DIRB, OUTPUT);
  pinMode(MOTOR_PWMA, OUTPUT);
  pinMode(MOTOR_PWMB, OUTPUT);
  pinMode(MOTOR_ENCA, OUTPUT);
  pinMode(MOTOR_ENCB, OUTPUT);

  // Reset encoder variables
  stepsA = 0;
  stepsB = 0;
  lastStateA = digitalRead(MOTOR_ENCA);
  lastStateB = digitalRead(MOTOR_ENCB);

  // Attach Interrupts
  attachInterrupt(digitalPinToInterrupt(MOTOR_ENCA), ENCA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_ENCB), ENCB_ISR, CHANGE);

  Serial.println("MotorControl initialized.");
}

void MotorControl::setMotorDirection(Motor motor, Direction dir) {
    if (motor == A) {
        digitalWrite(MOTOR_DIRA, dir); // Write out the direction, 0 = CW, 1 = CCW
        dirA = dir; // Update the direction variable
    } else if (motor == B) {
        digitalWrite(MOTOR_DIRB, dir);
        dirB = dir;
    }
}

void MotorControl::setRobotSpeed(int leftMotorSpeed, int rightMotorSpeed){
  Serial.print("Setting robot speed to - Left Motor PWM: ");
  Serial.println(leftMotorSpeed);
  Serial.print(", Right Motor PWM");
  Serial.println(rightMotorSpeed);

  analogWrite(MOTOR_PWMA, leftMotorSpeed);
  analogWrite(MOTOR_PWMB, rightMotorSpeed);
}

void MotorControl::setRobotDirection(RobotDirection t_robotDirection) {
  Direction wheelDirectionA = (t_robotDirection == FORWARD || t_robotDirection == RIGHT) ? CW : CCW;
  Direction wheelDirectionB = (t_robotDirection == BACKWARD || t_robotDirection == RIGHT) ? CW : CCW;

  setMotorDirection(A, wheelDirectionA);
  setMotorDirection(B, wheelDirectionB);
}

void MotorControl::applyPD() {
  // Calculate errors for left and right
  errorLeft = targetSpeedLeft - currentSpeedLeft;
  errorRight = targetSpeedRight - currentSpeedRight;

  // PD calculations for left and right
  float derivativeLeft = errorLeft - previousErrorLeft;
  float derivativeRight = errorRight - previousErrorRight;

  int pdOutputLeft = kp * errorLeft + kd * derivativeLeft;
  int pdOutputRight = kp * errorRight + kd * derivativeRight;

  // Unifieed speed when moving forward or backward
  int unifiedSpeed = (targetSpeedLeft + targetSpeedRight) / 2;

  // Apply logic: if turning or adjusting, use PD outputs; otherwise, maintain unified speed
  int leftMotorSpeed = (targetSpeedLeft == targetSpeedRight)
                       ? constrain(unifiedSpeed, 0, 255)
                       : constrain(pdOutputLeft, 0, 255);
  int rightMotorSpeed = (targetSpeedLeft == targetSpeedRight)
                       ? constrain(unifiedSpeed, 0, 255)
                       : constrain(pdOutputRight, 0, 255);

  // Set motor speeds
  analogWrite(MOTOR_PWMA, leftMotorSpeed);
  analogWrite(MOTOR_PWMB, rightMotorSpeed);

  // Update previous errors
  previousErrorLeft = errorLeft;
  previousErrorRight = errorRight;
}

void MotorControl::updateEncoders() {

}

float MotorControl::getDistanceTraveled() {
  float distanceA = (stepsA / (float)ENCODER_TICKS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE_CM;
  float distanceB = (stepsB / (float)ENCODER_TICKS_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE_CM;
  return (distanceA + distanceB) / 2.0;
}

void MotorControl::printDebugInfo() {
  Serial.print("Encoder Steps A: ");
  Serial.println(stepsA);
  Serial.print("Encoder Steps B: ");
  Serial.println(stepsB);
  Serial.print("Distance Traveled: ");
  Serial.println(getDistanceTraveled());
  Serial.println("cm");
}

void MotorControl::ENCA_ISR() {
  int currentState = digitalRead(MOTOR_ENCA);
  if(currentState != lastStateA) {
    stepsA += (digitalRead(MOTOR_ENCB) != currentState) ? 1 : -1;
  }
  lastStateA = currentState;
}

void MotorControl::ENCB_ISR() {
  int currentState = digitalRead(MOTOR_ENCB);
  if(currentState != lastStateB) {
    stepsB += (digitalRead(MOTOR_ENCA) != currentState) ? 1 : -1;
  }
  lastStateB = currentState;
}

