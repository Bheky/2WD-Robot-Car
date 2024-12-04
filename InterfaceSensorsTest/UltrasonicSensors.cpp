#include "UltrasonicSensors.h"
#include <ARB.h>
#include <Wire.h>

// CONSTRUCTOR

UltrasonicSensors::UltrasonicSensors(){
}

// PUBLIC METHODS

int UltrasonicSensors::readDistance(int sensorNumber){
  m_sensorNumber = sensorNumber;

  // Read ultrasonic sensors
  m_getUltrasoundDistances();

  return m_distances[sensorNumber];
}

// PRIVATE MEMBERS

void UltrasonicSensors::m_getUltrasoundDistances(){
  int duration, cm; // Setup variables for results

  pin_size_t uSonicPin = m_sensorNumber == 0 ? USONIC1 : (m_sensorNumber == 1 ? USONIC2 : (m_sensorNumber == 2 ? USONIC3 : USONIC4));

  // Set the pin to output, bring it low, then high, then low to generate pulse
  pinMode(uSonicPin, OUTPUT);
  digitalWrite(uSonicPin, LOW);
  delayMicroseconds(2);
  digitalWrite(uSonicPin, HIGH);
  delayMicroseconds(15);
  digitalWrite(uSonicPin, LOW);

  // The same pin is used to read back the returning signal, so must be set back to input
  pinMode(uSonicPin, INPUT);
  duration = pulseIn(uSonicPin, HIGH);

  // Convert to cm using helper function
  cm = uSecToCM(duration);

  m_distances[m_sensorNumber] = cm;
}



// #include "UltrasonicSensors.h"
// #include <ARB.h>
// #include <Wire.h>


// UltrasonicSensors::UltrasonicSensors(int pin1, int pin2) : _pin1(pin1), _pin2(pin2) {}

// void UltrasonicSensors::begin() {
//     pinMode(_pin1, OUTPUT);
//     pinMode(_pin2, OUTPUT);
// }

// void UltrasonicSensors::readDistances(int* dist1, int* dist2) {
//     // Generate pulse for the first sensor
//     digitalWrite(_pin1, LOW);
//     delayMicroseconds(2);
//     digitalWrite(_pin1, HIGH);
//     delayMicroseconds(15);
//     digitalWrite(_pin1, LOW);
    
//     // Switch to input and read the duration
//     pinMode(_pin1, INPUT);
//     int duration1 = pulseIn(_pin1, HIGH);

//     // Convert to cm
//     *dist1 = uSecToCM(duration1);

//     // Generate pulse for the second sensor
//     digitalWrite(_pin2, LOW);
//     delayMicroseconds(2);
//     digitalWrite(_pin2, HIGH);
//     delayMicroseconds(15);
//     digitalWrite(_pin2, LOW);
    
//     // Switch to input and read the duration
//     pinMode(_pin2, INPUT);
//     int duration2 = pulseIn(_pin2, HIGH);

//     // Convert to cm
//     *dist2 = uSecToCM(duration2);
// }

// int UltrasonicSensors::uSecToCM(int microseconds) {
//     // Calculate the distance in cm
//     return microseconds / 29 / 2; // Convert to cm
// }



















/* #include "UltrasonicSensors.h"
#include <ARB.h>
#include <Wire.h>

// CONSTRUCTOR

UltrasonicSensors::UltrasonicSensors(){
  //m_distances[0] = 0;
  //m_distances[1] = 0;
  // m_distancesPrev[0] = 0;
  // m_distancesPrev[1] =0;
}

// PUBLIC METHODS

void UltrasonicSensors::initialize(){
  m_initializeSerialRegisters(); // Initialize serial registers
}

void UltrasonicSensors::readUltrasonicSensors(){
  // Read ultrasonic sensors
  m_getUltrasoundDistances();

  // Print out distances to the Serial Monitor
  Serial.print("Ultrasonic Sensor 1: ");
  Serial.print(m_distances[0]);
  Serial.println(" cm");

  Serial.print("Ultrasonic Sensor 2: ");
  Serial.print(m_distances[1]);
  Serial.println(" cm");

  // Update registers only if distances have changed
  if(m_distances[0] != m_distancesPrev[0]){
    putRegister(REG_SEND_DATA_ULTRASOUND_1, m_distances[0]);
    m_distancesPrev[0] = m_distances[0];
  }

  if(m_distances[1] != m_distancesPrev[1]){
    putRegister(REG_SEND_DATA_ULTRASOUND_2, m_distances[1]);
    m_distancesPrev[1] = m_distances[1];
  }
}

// PRIVATE MEMBERS

void UltrasonicSensors::m_initializeSerialRegisters(){
  // Ultrasound
  putRegister(REG_SEND_DATA_ULTRASOUND_1, 0);
  putRegister(REG_SEND_DATA_ULTRASOUND_2, 0);
} 

void UltrasonicSensors::m_getUltrasoundDistances(){
  long duration[2]; // Array to store pulse duration
  int cm[2];        // Declare cm array to store distances in centimeters

  // Trigger Ultrasonic Sensor 1
    sensor1.output(); // Set sensor1 as OUTPUT
    sensor1 = 0;      // Set pin low
    delayMicroseconds(2);       // Wait 2 microseconds
    sensor1 = 1;      // Set pin high
    delayMicroseconds(15);      // Wait 15 microseconds
    sensor1 = 0;      // Set pin low

    // Set sensor1 as INPUT and read duration
    sensor1.input();
    duration[0] = pulseIn(sensor1, HIGH);

    // Trigger Ultrasonic Sensor 2
    sensor2.output(); // Set sensor2 as OUTPUT
    sensor2 = 0;      // Set pin low
    delayMicroseconds(2);       // Wait 2 microseconds
    sensor2 = 1;      // Set pin high
    delayMicroseconds(15);      // Wait 15 microseconds
    sensor2 = 0;      // Set pin low

    // Set sensor2 as INPUT and read duration
    sensor2.input();
    duration[1] = pulseIn(sensor2, HIGH);

    // Convert duration to centimeters
    m_distances[0] = uSecToCM(duration[0]);
    m_distances[1] = uSecToCM(duration[1]); 


  // Set the pin to output, bring it low, then high, then low to generate pulse
  pinMode(USONIC1, OUTPUT);
  digitalWrite(USONIC1, LOW);
  delayMicroseconds(2);
  digitalWrite(USONIC1, HIGH);
  delayMicroseconds(15);
  digitalWrite(USONIC1, LOW);

  // Use same pin to read back the returning signal, so must be set back to input
  pinMode(USONIC1, INPUT);
  duration[0] = pulseIn(USONIC1, HIGH);

  // Set the pin to output, bring it low, then high, then low to generate pulse
  pinMode(USONIC2, OUTPUT);
  digitalWrite(USONIC2, LOW);
  delayMicroseconds(2);
  digitalWrite(USONIC2, HIGH);
  delayMicroseconds(15);
  digitalWrite(USONIC2, LOW);

  // Use same pin to read back the returning signal, so must be set back to input
  pinMode(USONIC2, INPUT);
  duration[1] = pulseIn(USONIC2, HIGH);

  // Convert to cm using helper function
  cm[0] = uSecToCM(duration[0]);
  cm[1] = uSecToCM(duration[1]);

  m_distances[0] = cm[0];
  m_distances[1] = cm[1]; 

  // Print out distances to the Serial Monitor
  Serial.print("Ultrasonic Sensor 1: ");
  Serial.print(cm[0]);
  Serial.println(" cm");

  Serial.print("Ultrasonic Sensor 2: ");
  Serial.print(cm[1]);
  Serial.println(" cm");

}

// Helper function to convert microseconds to centimeters
int UltrasonicSensors::uSecToCM(long microseconds) {
  return microseconds / 29 / 2; // Speed of sound is approx. 34300 cm/s
} */







