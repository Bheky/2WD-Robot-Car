/* This is a first draft of the code to use I2C multiplexer on the ARB board
to get data using same address of different I2C buses. 

Here the four range sensors to be used are GP2Y0E02B IR sensors which will be
connected to the first two I2C buses.

Note: the structure of the code is based on the foundation taken from the example
code in the ARB folder, however it has been modified to suit this project specifications.
*/

#include "InfraredSensors.h"
#include <ARB.h>
#include <Wire.h>

// CONSTRUCTOR

InfraredSensors::InfraredSensors(int t_irBusNumber)
  : m_irBusNumber(t_irBusNumber), m_infraredsensorsDistancePrev(0), m_shift(0) {}


// PUBLIC METHODS

void InfraredSensors::initialize(){
  m_setupI2C(m_irBusNumber);
}

int InfraredSensors::readIR(){
  // Read the IR sensor
  int infraredsensorsDistance = m_readI2CSensor(m_irBusNumber);

  return infraredsensorsDistance;
}

// PRIVATE METHODS

// I2C setup function
void InfraredSensors::m_setupI2C(int t_bus_number) {
  setI2CBus(t_bus_number); // Set which bus we are reading from
  // Write to the sensor to tell it to read from shift register
  Wire.beginTransmission(IR_SENSOR_ADDRESS);
  Wire.write(IR_SENSOR_SHIFT_REG);
  Wire.endTransmission();

  // Request 1 byte of data from the sensor to read the shift register
  Wire.requestFrom(IR_SENSOR_ADDRESS, 1);

  while(Wire.available() == 0){
    // Print a message to tell the user if the sketch is waiting for a particular sensor
    // If sensor does not reply, it may be a sign of a faulty or disconnected sensor
    Serial.print("Waiting for sensor ");
    Serial.print(t_bus_number);
    Serial.println();
  }

  // Once the data become available in the Wire buffer, put it into the shift array
  m_shift = Wire.read();

}

int InfraredSensors::m_readI2CSensor(int t_bus_number){
  setI2CBus(t_bus_number); // Set bus we are accessing

  // Write to sensor to tell it we're reading from the distance register
  Wire.beginTransmission(IR_SENSOR_ADDRESS);
  Wire.write(IR_SENSOR_DISTANCE_REG);
  Wire.endTransmission();

  // Request two bytes of data from the sensor
  Wire.requestFrom(IR_SENSOR_ADDRESS, 2);

  // Wait until bytes are received in the buffer
  while(Wire.available() <2);

  // Temporarily store the bytes read. Store high and low byte read
  byte high = Wire.read();
  byte low = Wire.read();

  // Calculate the distance in cm
  int distance = (high * 16 + low) / (16 * (int)pow(2, m_shift));  // Stores calculated distance

  return distance;
}

