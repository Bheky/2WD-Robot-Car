#ifndef PDCONTROLLER_H
#define PDCONTROLLER_H

class PDController {
  public:
    PDController(float Kp, float Kd);
    float computeControlSignal(float setpoint, float currentPosition);

  private:
    float m_Kp; // Proportional gain
    float m_Kd; // Derivative gain
    float m_previousError; // Previous error value
};

#endif