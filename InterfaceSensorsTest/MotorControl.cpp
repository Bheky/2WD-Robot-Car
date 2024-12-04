#include "MotorControl.h"
#include <ARB.h>

// Initialize static variables
volatile int MotorControl::stepsA = 0;
volatile int MotorControl::stepsB = 0;
Direction MotorControl::dirA = CW;
Direction MotorControl::dirB = CW;

MotorControl::MotorControl() {}

void MotorControl::initialize() {
    // Set pin modes for motors and encoders
    pinMode(motorDirA, OUTPUT);
    pinMode(motorPWMA, OUTPUT);
    pinMode(motorDirB, OUTPUT);
    pinMode(motorPWMB, OUTPUT);
    pinMode(encoderA, INPUT);
    pinMode(encoderB, INPUT);
    pinMode(triggerFront, OUTPUT);
    pinMode(echoFront, INPUT);
    pinMode(triggerBack, OUTPUT);
    pinMode(echoBack, INPUT);
    pinMode(irSensorLeft, INPUT);
    pinMode(irSensorRight, INPUT);

    // Set motors off by default
    setMotorDirection(A, CW);
    setMotorDirection(B, CW);
    analogWrite(motorPWMA, 0);
    analogWrite(motorPWMB, 0);

    // Attach interrupts to motor encoder inputs
    attachInterrupt(digitalPinToInterrupt(encoderA), ENCA_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderB), ENCB_ISR, CHANGE);

    Serial.begin(9600); // Start serial for debugging
}

void MotorControl::update() {
    readSensors();
    controlMotors();

    // Print the current number of steps recorded
    Serial.print("A steps: ");
    Serial.println(stepsA);
    Serial.print("B steps: ");
    Serial.println(stepsB);
}

void MotorControl::readSensors() {
    // Read distances from ultrasonic sensors
    long frontDistance = getDistance(triggerFront, echoFront);
    long backDistance = getDistance(triggerBack, echoBack);

    // Read IR sensor values
    bool leftObstacle = digitalRead(irSensorLeft);
    bool rightObstacle = digitalRead(irSensorRight);

    // Logic to determine state based on sensor readings
    if (leftObstacle) {
        currentState = TURN_RIGHT; // Turn right if left IR detects an obstacle
    } else if (rightObstacle) {
        currentState = TURN_LEFT; // Turn left if right IR detects an obstacle
    } else if (frontDistance < 20) { // Change 20 to your desired distance threshold
        currentState = TURN_RIGHT; // Turn if front ultrasonic detects an obstacle
    } else {
        currentState = FORWARD; // Move forward otherwise
    }
}

void MotorControl::controlMotors() {
    switch (currentState) {
        case FORWARD:
            setMotorDirection(A, CW);
            setMotorDirection(B, CW);
            analogWrite(motorPWMA, 255); // Full speed
            analogWrite(motorPWMB, 255);
            break;
        case TURN_LEFT:
            setMotorDirection(A, CCW);
            setMotorDirection(B, CW);
            analogWrite(motorPWMA, 255); // Full speed
            analogWrite(motorPWMB, 255);
            break;
        case TURN_RIGHT:
            setMotorDirection(A, CW);
            setMotorDirection(B, CCW);
            analogWrite(motorPWMA, 255); // Full speed
            analogWrite(motorPWMB, 255);
            break;
        case STOP:
            analogWrite(motorPWMA, 0); // Stop motor A
            analogWrite(motorPWMB, 0); // Stop motor B
            break;
    }
}

void MotorControl::setMotorDirection(Motor motor, Direction dir) {
    if (motor == A) {
        digitalWrite(motorDirA, dir); // Write out the direction, 0 = CW, 1 = CCW
        dirA = dir; // Update the direction variable
    } else if (motor == B) {
        digitalWrite(motorDirB, dir);
        dirB = dir;
    }
}

long MotorControl::getDistance(int triggerPin, int echoPin) {
    // Trigger the ultrasonic sensor
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);
    
    // Read the echo
    return pulseIn(echoPin, HIGH) * 0.034 / 2; // Convert to cm
}

// ISRs for reading encoder counts
void ENCA_ISR() {
    if (MotorControl::dirA == CW) {
        MotorControl::stepsA++;
    } else {
        MotorControl::stepsA--;
    }
}

void ENCB_ISR() {
    if (MotorControl::dirB == CW) {
        MotorControl::stepsB++;
    } else {
        MotorControl::stepsB--;
    }
}