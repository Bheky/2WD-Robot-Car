#include "MotorControl.h"
#include <ARB.h>

// Initialize static variables
volatile int MotorControl::stepsA = 0;
volatile int MotorControl::stepsB = 0;
Direction MotorControl::dirA = CW;
Direction MotorControl::dirB = CW;

MotorControl::MotorControl()
    : pdControllerA(0.47, 0.2), // Kp & Kd for motor A
      pdControllerB(0.47, 0.2)  // Kp & Kd for motor B
{}

void MotorControl::initialize(){
  // Set PinModes
  pinMode(MOTOR_DIRA, OUTPUT);
  pinMode(MOTOR_DIRB, OUTPUT);
  pinMode(MOTOR_PWMA, OUTPUT);
  pinMode(MOTOR_PWMB, OUTPUT);
  pinMode(MOTOR_ENCA, OUTPUT);
  pinMode(MOTOR_ENCB, OUTPUT);

  // Attach Interrupts
  attachInterrupt(digitalPinToInterrupt(MOTOR_ENCA), ENCA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR_ENCB), ENCB_ISR, CHANGE);
}

void MotorControl::update() {
    // Print the current number of steps recorded
    Serial.print("A steps: ");
    Serial.println(stepsA);
    Serial.print("B steps: ");
    Serial.println(stepsB);
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

void MotorControl::setRobotSpeed(int t_pwmValue){
  analogWrite(MOTOR_PWMA, t_pwmValue);
  analogWrite(MOTOR_PWMB, t_pwmValue);
}

void MotorControl::setSpeedWithControl(float desiredSpeed) {
  float speedA = pdControllerA.computeControlSignal(desiredSpeed, stepsA);
  float speedB = pdControllerB.computeControlSignal(desiredSpeed, stepsB);

  setRobotSpeed(speedA);
  // Adjust speedB based on the difference
  if (stepsA > stepsB) {
    speedB += 10; // Compensate for slower motor
  } else if (stepsB > stepsA) {
    speedA -= 10; // Compensate for faster motor
  }
  analogWrite(MOTOR_PWMB, speedB);
}

void MotorControl::setRobotDirection(RobotDirection t_robotDirection) {
  Direction wheelDirectionA = (t_robotDirection == FORWARD || t_robotDirection == RIGHT) ? CW : CCW;
  Direction wheelDirectionB = (t_robotDirection == BACKWARD || t_robotDirection == RIGHT) ? CW : CCW;

  setMotorDirection(A, wheelDirectionA);
  setMotorDirection(B, wheelDirectionB);
}

// ISRs for reading encoder counts
void MotorControl::ENCA_ISR() {
    if (dirA == CW) {
        stepsA++;
    } else {
        stepsA--;
    }
}

void MotorControl::ENCB_ISR() {
    if (dirB == CW) {
        stepsB++;
    } else {
        stepsB--;
    }
}