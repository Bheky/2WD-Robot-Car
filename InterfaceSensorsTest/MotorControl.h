#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>
#include <ARB.h>

// Define motor and direction enums
typedef enum {CW, CCW} Direction; // CW = 0, CCW = 1
typedef enum {A, B} Motor;

class MotorControl {
public:
    MotorControl();
    void initialize();
    void update();

    // Make the variables accessible to the ISR
    static volatile int stepsA;
    static volatile int stepsB;
    static Direction dirA;
    static Direction dirB;

private:
    // Motor control pins
    const int motorDirA = 9; // Motor A direction pin
    const int motorPWMA = 10; // Motor A PWM pin
    const int motorDirB = 11; // Motor B direction pin
    const int motorPWMB = 12; // Motor B PWM pin
    const int encoderA = 2; // Encoder pin for motor A
    const int encoderB = 3; // Encoder pin for motor B
    const int triggerFront = 4; // Trigger pin for front ultrasonic sensor
    const int echoFront = 5;    // Echo pin for front ultrasonic sensor
    const int triggerBack = 6; // Trigger pin for back ultrasonic sensor
    const int echoBack = 7;    // Echo pin for back ultrasonic sensor
    const int irSensorLeft = 8; // IR sensor left pin
    const int irSensorRight = 13; // IR sensor right pin

    // State machine states
    enum State {
        FORWARD,
        TURN_LEFT,
        TURN_RIGHT,
        STOP
    };
    State currentState = FORWARD;

    // Method declarations
    void readSensors();
    void controlMotors();
    void setMotorDirection(Motor motor, Direction dir);
    long getDistance(int triggerPin, int echoPin);
};

// Encoder interrupt service routines
void ENCA_ISR();
void ENCB_ISR();

#endif
