#pragma once

#include <cmath>

typedef struct VA VA;
struct VA
{
  double m_speed;
  double m_acceleration;
  double m_jerk;
};

class KineticToVoltage
{

  // k_lut[MoteurIndex][ForwardBackward][Kv, Ka, Kintersept]
  double k_lut[4][2][3];

public:
  void SetMotorCoefficients(int motorID, int isBackward, double kv, double ka, double vintersept);
  double getVoltage(int motorID, const VA *pva);
};