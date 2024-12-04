#ifndef ULTRASONIC_SENSORS_H
#define ULTRASONIC_SENSORS_H

#include "Arduino.h"
#include <ARB.h>
#include <Wire.h>

typedef enum {FRONT, BACK, BACKLEFT, BACKRIGHT} Orientation;

class UltrasonicSensors {
  public:
    UltrasonicSensors();
    int readDistance(int sensorNumber);
    
  private:
    int m_sensorNumber;

    // Ultrasound variables
    int m_distances[4] = {0,0,0,0}; // Setup variable for results
    void m_getUltrasoundDistances();
};

#endif // ULTRASONIC_SENSORS_H












/*#ifndef ultrasonicsensors_h
#define ultrasonicsensors_h

#include "Arduino.h"
#include <ARB.h>
#include <Wire.h>

//#include "mbed.h"

//Register definitions for sending ultrasound data
#define REG_SEND_DATA_ULTRASOUND_1 20 // Register for Ultrasound 1
#define REG_SEND_DATA_ULTRASOUND_2 21 // Register for Ultrasound 2

// Pin definitions for ultrasonic sensors
#define USONIC1 P0_23 // Pin for ultrasonic sensor 1
#define USONIC2 P1_14 // Pin for ultrasonic sensor 2

class UltrasonicSensors {
  public:
    UltrasonicSensors(); // Constructor
    void initialize();
    void readUltrasonicSensors();

  private:
    // Ultrasonic variables
    int m_distances[2] = {0,0}; // Array to store current distances
    int m_distancesPrev[2] = {0,0}; // Array to store previous distances

    //DigitalInOut sensor1; // DigitalInOut for ultrasonic sensor 1
    //DigitalInOut sensor2; // DigitalInOut for ultrasonic sensor 2

    void m_initializeSerialRegisters();
    void m_getUltrasoundDistances();
    int uSecToCM(long microseconds); // Function to convert microseconds to cm

};

#endif */