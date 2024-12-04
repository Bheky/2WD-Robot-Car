#ifndef MOTORSCONTROL_H
#define MOTORSCONTROL_H

#include "Arduino.h"
#include <ARB.h>
#include <Wire.h>

#define PI 3.1415926535897932384626433832795

// Enum for motor direction and motors
typedef enum {CW,CCW} m_Direction; // CW = 0, CCW = 1
typedef enum {A,B} m_Motor; // A = 0, B =1
enum m_robotDirection { FORWARD, BACKWARD, LEFT, RIGHT };


class MotorsControl {
  public:
    MotorsControl();

    void initialize();
    void update();
    void setRobotSpeed(int t_pwmValue); // Set robot's speed using PWM
    void setRobotDirection(RobotDirection t_robotDirection); // Set robot's movement direction

    // void updateEncoders(); // Update encoder readings
    // void getDistanceTraveled() // Get the average distance traveled (cm)
    // void applyPID(); // Apply PID control to motors
    // void runMotorsControl();

    // Debugging info
    void printDegugInfo(); // Print debug information for motors and encoders


  private:
  // Method declarations
  void setMotorDirection(Motor motor, Direction dir);
  static Direction dirA;
  static Direction dirB;
  float distanceA, distanceB;
  float currentSpeedLeft;
  float currentSpeedRight;

  // Robot pose
  float m_robotpose[3] = {0.0, 0.0, 0.0};

  // PID
  int m_controlModeInputPrev;
  bool m_pidControlMode = 0;

  // Motor A tunings
  double m_kpA = 0.8;
  double m_kiA = 0.2;
  double m_kdA = 0.5;

  // Motor B tunings
  double m_kpB = 0.8;
  double m_kiB = 0.2;
  double m_kdB = 0.5;

  double m_goalMargin = 0.01;

  double m_errorSumA = 0;
  double m_errorSumB = 0;
  double m_lastErrorA = 0;
  double m_lastErrorB = 0;

  double m_setpointA, m_setpointB; // Motor goals as distance (setpoint)

  int m_timeChange = 1; // 1 millisecond delay per PID compute
  int m_stepDistance_cm = 5; // Distance per step

  int m_pidSignal = 0; // PID data signal to control PID computing

  // ODOMETRY

  int m_distanceBetweenWheels = 9; // In cm
  double m_wheelDiameter = 4.60;

  volatile int m_speedValuePrev = 0;
  volatile int m_inputPrev = 0;

  // MEMBER METHODS

  // Initialisation methods
  void m_setPinModes(); // Sets pins and pin modes of all components
  void m_attachInterrupts(); // Attaches interrupts
  void m_initializeSerialRegisters(); // Initialize serial registers
    
  // PID functions
  void m_readControlModeRegister();
  void m_readPidTunningSettings();
  void m_readSetpoints();
  void m_readOdometrySettings();
  void m_readPidSignal();
  
  int m_computePID(double t_setpointA, double t_setpointB, bool stopAtGoal);

  double m_stepsToCentimetres(int t_steps);
  void m_resetErrors(m_Motor t_motor);

  void m_setMotorSpeed(m_Motor t_motor, double t_speedPWM);
  void m_calculateCurrentPose(double t_distanceA, double t_distanceB);

  // General movement
  void m_readSpeedLevelValue();
  void m_readDirectionInput();
  void m_adjustSpeed(int t_speedLevel);
  void m_motorSetDir(m_Motor t_motor, m_wheelDirection t_dir);

  // Direction control no PID
  void m_moveForward();
  void m_moveBackward();
  void m_moveLeft();
  void m_moveRight();
  void m_stopRobot();

  // Direction control PID
  void m_rotateRobot(double t_angle_radians);
  void m_advanceRobot(double t_distance_cm);
  void m_moveRobot(m_robotDirection t_robotDirection, int t_duration);
  void m_aimRobot(m_robotDirection t_robotDirection);
  void m_sendRobotPose(double t_pose[3]);


  // Encoder variables
  volatile static int m_stepsA; // Variables to store step count from motor A
  volatile static int m_stepsB; // Variables to store step count from motor B

  // These values will not be reset unless a signal is received
  // volatile static int m_absoluteStepsA;
  // volatile static int m_absoluteStepsB;
  
  // Variables to store distance count from motors
  // double m_distanceA = 0;
  // double m_distanceB = 0;
  
  // int m_speedAPWM_prev = 0;
  // int m_speedBPWM_prev = 0;
  
  // Encoder interrupts
  static void m_ENCA_ISR();
  static void m_ENCB_ISR();
};

#endif

