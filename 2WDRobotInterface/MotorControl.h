#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>
#include <ARB.h>
#include <Wire.h>

// Define motor and direction enums
typedef enum {CW, CCW} Direction; // CW = 0, CCW = 1
typedef enum {A, B} Motor;  // Motor identifiers

typedef enum {FORWARD, BACKWARD, LEFT, RIGHT} RobotDirection;

class MotorControl {
public:
    MotorControl();

    void initialize(); // Initialize motor control pins
    void update();  
    void setRobotSpeed(int leftMotorSpeed, int rightMotorSpeed); // Set robot's speed using PWM
    void setRobotDirection(RobotDirection t_robotDirection); // Set robot's movement direction

    void updateEncoders();  // Update encoder readings
    float getDistanceTraveled();  // Get the average distance traveled (cm)
    void applyPD(); // Apply PID control to motors

    // Debugging info
    void printDebugInfo(); // Print debug information for motors and encoders

private:
    // Method declarations
    void setMotorDirection(Motor motor, Direction dir);
    static Direction dirA;
    static Direction dirB;
    float distanceA, distanceB;
    float currentSpeedLeft;
    float currentSpeedRight;

    // Encoder variables
    static volatile int stepsA;        // Steps for motor A
    static volatile int stepsB;        // Steps for motor B
    static volatile int lastStateA;    // Last state for motor A encoder
    static volatile int lastStateB;    // Last state for motor B encoder

    // PID control variables
    float kp = 0.47, kd = 0.2;  // PD constants
    int targetSpeedLeft = 100, targetSpeedRight = 100; // Target speeds
    float errorLeft = 0, errorRight = 0;
    float integralLeft = 0, integralRight = 0;
    float previousErrorLeft = 0, previousErrorRight = 0;

    // Distance tracking
    float stepsToCm = 0.0349;  // Conversion factor: steps to cm

    // Encoder interrupt service routines
    static void ENCA_ISR();
    static void ENCB_ISR();
};

#endif
