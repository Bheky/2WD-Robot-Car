#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>
#include <ARB.h>
#include "PDController.h"

// Define motor and direction enums
typedef enum {CW, CCW} Direction; // CW = 0, CCW = 1
typedef enum {A, B} Motor;

typedef enum {FORWARD, BACKWARD, LEFT, RIGHT} RobotDirection;

class MotorControl {
public:
    MotorControl();
    void initialize();
    void update();
    void setRobotSpeed(int t_pwmValue);
    void setRobotDirection(RobotDirection t_robotDirection);
    void setSpeedWithControl(float desiredSpeed);

    // Make the variables accessible to the ISR
    static volatile int stepsA;
    static volatile int stepsB;
    static Direction dirA;
    static Direction dirB;

private:
    // Method declarations
    void setMotorDirection(Motor motor, Direction dir);

    // Encoder interrupt service routines
    static void ENCA_ISR();
    static void ENCB_ISR();

    PDController pdControllerA;
    PDController pdControllerB;
};


#endif
