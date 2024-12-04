#include "PDController.h"
#include "ARB.h"

PDController::PDController(float Kp, float Kd)
    : m_Kp(Kp), m_Kd(Kd), m_previousError(0.0) {}

float PDController::computeControlSignal(float setpoint, float currentPosition) {

  float error = setpoint - currentPosition;
  float derivative = error - m_previousError;

  float output = m_Kp * error + m_Kd * derivative;
  m_previousError = error;

  return output;

}