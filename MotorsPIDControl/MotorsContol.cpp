/*
  MotorsControl.cpp - 2WD Robot Car based on ARB
  Test Code v1
  PID motor control
  Author: Bheki Ndlovu
*/

#include "MotorsControl.h"
#include <ARB.h>

// Initialize static variables
volatile int MotorsControl::stepsA = 0;
volatile int MotorsControl::stepsB = 0;
Direction MotorsControl::dirA = CW;
Direction MotorsControl::dirB = CCW;


// CONSTRUCTOR

MotorsControl::MotorsControl() {
}

// PUBLIC METHODS

void MotorsControl::initialize() {
  m_setPinModes();
  m_attachInterrupts(); // Attaches interrupts
}

void MotorsControl::runMotorsControl() {

  if(m_pidControlMode == 0){
    m_readPidTunningSettings();
    m_readSetpoints();
    m_readOdometrySettings();
    m_readPidSignal();

    if(m_pidSignal == 0 || m_pidSignal == 1){
      m_computePID(m_setpointA, m_setpointB, true);
    }
  }
  else (
    // General movement
    m_readSpeedLevelValue();
    m_readDirectionInput();
  )
}

// PRIVATE MEMBERS

void MotorsControl::m_setPinModes() {
  // Set relevant pin modes
  pinMode(MOTOR_DIRA, OUTPUT);
  pinMode(MOTOR_DIRB, OUTPUT);
  pinMode(MOTOR_PWMA, OUTPUT);
  pinMode(MOTOR_PWMB, OUTPUT);
  pinMode(MOTOR_ENCA, INPUT);
  pinMode(MOTOR_ENCB, INPUT);
}

void MotorsControl::m_attachInterrupts(){
  // Attach interrupts to the motor encoder inputs
  attachInterrupt(digitalPinToInterrupt(MOTOR_ENCA), m_ENCA_ISR, CHANGE); // Pin number put converted to interrupt number
  attachInterrupt(digitalPinToInterrupt(MOTOR_ENCB), m_ENCB_ISR, CHANGE); //
}

int Motors::m_computePID(double t_setpointA, double t_setpointB, bool stopAtGoal){
  int exitCode = 0;
  
  double stepsA_cm = m_stepsToCentimetres(m_stepsA);
  double stepsB_cm = m_stepsToCentimetres(m_stepsB);

  double errorA = t_setpointA - stepsA_cm; // m_stepsA_cm -> input
  double errorB = t_setpointB - stepsB_cm; // m_stepsB_cm -> input

  Serial.print("KP: ");
  Serial.println(m_kpA);

  Serial.print("KI: ");
  Serial.println(m_kiA);

  Serial.print("KD: ");
  Serial.println(m_kdA);

  Serial.print("Goal Margin: ");
  Serial.println(m_goalMargin);

  if((errorA > -m_goalMargin && errorA < m_goalMargin && stopAtGoal == true) || (m_pidSignal == 1)){
    m_setMotorSpeed(A, 0);
    m_resetErrors(A);
    exitCode += 1;
  }
  else {
    m_errorSumA += (errorA * m_timeChange);
    double errorDiffA = (errorA - m_lastErrorA) / m_timeChange;
    
    double proportionalA = m_kpA * errorA;
    double integralA = m_kiA * m_errorSumA;
    double derivativeA = m_kdA * errorDiffA;
  
    double m_speedA = proportionalA + integralA + derivativeA; // m_speedA -> output
  
    m_setMotorSpeed(A, m_speedA);
  
    m_lastErrorA = errorA;
  }

  if((errorB > -m_goalMargin && errorB < m_goalMargin && stopAtGoal == true) || (m_pidSignal == 1)){
    m_setMotorSpeed(B, 0);
    m_resetErrors(B);
    exitCode += 1;
  }
  else {
    m_errorSumB += (errorB * m_timeChange);
    double errorDiffB = (errorB - m_lastErrorB) / m_timeChange;
    
    double proportionalB = m_kpB * errorB;
    double integralB = m_kiB * m_errorSumB;
    double derivativeB = m_kdB * errorDiffB;
  
    double m_speedB = proportionalB + integralB + derivativeB; // m_speedA -> output
  
    m_setMotorSpeed(B, m_speedB);
  
    m_lastErrorB = errorB;
  }

  if(exitCode >= 2){
    m_calculateCurrentPose(stepsA_cm, stepsB_cm);
    m_sendRobotPose(m_pose);
    m_stepsA = 0;
    m_stepsB = 0;
    m_pidSignal = -1;
  }

  delay(m_timeChange);
  
  return exitCode;
}

// Converts encoder steps to centimetres
double MotorsControl::m_stepsToCentimeters(int t_steps) { 
  double fullRotation = 298 * 6;
  double wheelRadius = m_wheelDiameter / 2;
  double perimeter = 2 * PI * wheelRadius;
  double distance = perimeter * (t_steps/fullRotation);
  return distance;
}

void MotorsControl::m_resetErrors(m_Motor t_motor) { 
  if(t_motor == A){
    m_errorSumA = 0;
    m_lastErrorA = 0;
  }
  else {
    m_errorSumB = 0;
    m_lastErrorB = 0;
  }
}

void MotorsControl::m_setMotorSpeed(m_Motor t_motor, double t_speedPWM) { 

  t_speedPWM = t_speedPWM >= 255 ? 255 : (t_speedPWM) <= -225 ? -255 : t_speed

  int pwmPin = t_motor == A ? MOTOR_PWMA : MOTOR_PWMB;
  int dirPin = t_motor == A ? MOTOR_DIRA : MOTOR_DIRB;

  m_wheelDirection wheelDirection = t_speedPWM < 0 ? (t_motor == A ? CCW : CW);

  if (t_motor == A) {
    m_dirA = wheelDirection;
  }
  else {
    m_dirB = wheelDirection;
  }

  digitalWrite(dirPin, wheelDirection);
  analogWrite(pwmPin, abs(t_speedPWM));
}

// Calculates the next pose based on calculated travelled distance of each wheel and store it in m_pose[]
void MotorsControl::m_calculateCurrentPose(double t_distanceA, double t_distanceB) { 
  double radiusB = (t_distanceA != t_distanceB) ? (t_distanceB * m_distanceBetweenWheels)/(t_distanceA - t_distanceB) : 10000000000000000000; // Large value instead of infinity if distances are equal.
  double angleChange = t_distanceB/radiusB;
  double radius = radiusB + m_distanceBetweenWheels/2;
  double distanceChange = 2 * radius * sin(angleChange/2);

  m_pose[0] = m_pose[0] + distanceChange * cos(m_pose[2]+ (angleChange/2));
  m_pose[1] = m_pose[1] + distanceChange * sin(m_pose[2]+ (angleChange/2));
  m_pose[2] = m_pose[2] + angleChange;

}

void MotorsControl::m_djustSpeed(int t_speedLevel) {
  // Variables for motor speed
  int speedAPWM = 0; // Motor A
  int speedBPWM = 0; // Motor B
  int baseSpeed = 255 / 9;

  // If the speed level is between 0 and 9
  if(t_speedLevel >= 0 && t_speedLevel <= 9){
    // Set the speed to the desired level
    speedAPWM = baseSpeed * t_speedLevel;
    speedBPWM = baseSpeed * t_speedLevel;
  }
  else {
    // Stop motors
    speedAPWM = 0;
    speedBPWM = 0;
  }

  // Send speed to motors
  analogWrite(MOTOR_PWMA, speedAPWM);
  analogWrite(MOTOR_PWMB, speedBPWM);
}

void Motors::m_motorSetDir(m_Motor t_motor, m_wheelDirection t_dir){
  if(t_motor == A){
    digitalWrite(MOTOR_DIRA, t_dir); // Write out the direction, 0 = CW, 1 = CCW
    m_dirA = t_dir; // Update the direction variable
  }
  else if(t_motor == B){
    digitalWrite(MOTOR_DIRB, t_dir);
    m_dirB = t_dir;
  }
}

// Moves the robot forwards
void Motors::m_moveForward(){
  Serial.println("Moving forwards.");
  m_motorSetDir(A, CW);
  m_motorSetDir(B, CCW);
}

// Moves the robot backwards
void Motors::m_moveBackward(){
  Serial.println("Moving backwards.");
  m_motorSetDir(A, CCW);
  m_motorSetDir(B, CW);
}

// Moves the robot left
void Motors::m_moveLeft(){
  Serial.println("Moving left.");
  m_motorSetDir(A, CCW);
  m_motorSetDir(B, CCW);
}

// Moves the robot right
void Motors::m_moveRight(){
  Serial.println("Moving right.");
  m_motorSetDir(A, CW);
  m_motorSetDir(B, CW);
}

// Stops the robot
void Motors::m_stopRobot(){
  Serial.println("Stopping robot.");
  m_adjustSpeed(0);
}

void Motors::m_rotateRobot(double t_angle_radians){
  double distance = t_angle_radians * m_distanceBetweenWheels / 2;
  int exitCode = 0;

  while(exitCode < 2){
    exitCode = m_computePID(distance, -distance, true);
  }
}

void Motors::m_advanceRobot(double t_distance_cm){
  int exitCode = 0;
  
  while(exitCode < 2){
    exitCode = m_computePID(t_distance_cm, t_distance_cm, true);
  }
}

void Motors::m_moveRobot(m_robotDirection t_robotDirection, int t_duration){
  int currentTime = 0;
 
  while(currentTime < t_duration){

    m_aimRobot(t_robotDirection);
    
    currentTime += m_timeChange;
  }
}

void Motors::m_aimRobot(m_robotDirection t_robotDirection){
  double setpointA = (t_robotDirection == FORWARD || t_robotDirection == RIGHT) ? m_stepDistance_cm : -m_stepDistance_cm;
  double setpointB = (t_robotDirection == FORWARD || t_robotDirection == LEFT) ? m_stepDistance_cm : -m_stepDistance_cm;
  
  m_computePID(setpointA, setpointB, true);
}

//ISR for reading MOTOR_ENCA
void MotorsControl::m_ENCA_ISR(){
  if(m_dirA == CW){
    m_stepsA++;
  }
  else if(m_dirA == CCW){
    m_stepsA--;
  }
}

//ISR for reading MOTOR_ENCB
void MotorsControl::m_ENCB_ISR(){
  if(m_dirB == CCW){
    m_stepsB++;
  }
  else if(m_dirB == CW){
    m_stepsB--;
  }
}




