// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <rev/CANSparkMax.h>
#include <frc/Encoder.h>
#include <assert.h>
#include "lib/CSVLogFile.h"
#include "lib/N/NMath.h"
#include "lib/N/NType.h"
#include "lib/N/NFlags.h"
#include <networktables/NetworkTableEntry.h>
#include <frc/shuffleboard/Shuffleboard.h>
// #include <units/units.h>
#include <iostream>

typedef struct TestSpecs TestSpecs;
struct TestSpecs
{
  unsigned long m_flags;
  double m_voltage;
  double m_ramp;
};

class NLCharacterization_Tests
{

public:
  enum class State
  {
    Stopped = 0,
    AskForStart = 1,
    Started = 2,
    AskForStop = 3
  };
  NLCharacterization_Tests(rev::CANSparkMax *leftMotor,
                           rev::CANSparkMax *leftMotorFollower,
                           rev::CANSparkMax *rightMotor,
                           rev::CANSparkMax *rightMotorFollower,
                           frc::Encoder *externalEncoderLeft,
                           frc::Encoder *externalEncoderRight,
                           long nbTestLow,
                           double endVoltageLow,
                           long nbTestMedium,
                           double endVoltageMedium,
                           long nbTestHigh,
                           double endVoltageHigh,
                           double rampValue,
                           double rampVoltage);
  ~NLCharacterization_Tests();
  void nextTest();
  void previousTest();
  void setCurrentTest(int8_t testId);
  void start();
  void stop();
  void fastLoop();
  State getState();
  int8_t getCurrentTestId();
  char *getCurrentFileLogName(char *pbuffer, int size);
  char *getCurrentTestDescription(char *pmessage, int size_terminated_null_char_included);
  int getTestsCounter();
  int areAllTestsDone();

private:
  rev::CANSparkMax *m_rightMotor;
  rev::CANSparkMax *m_rightMotorFollower;
  rev::CANSparkMax *m_leftMotor;
  rev::CANSparkMax *m_leftMotorFollower;

  frc::Encoder *m_externalEncoderRight;
  frc::Encoder *m_externalEncoderLeft;

  TestSpecs *TestData;
  State m_state = State::Stopped;
  int8_t m_CurrentTestID = 0;
  double m_oldRamp;
  int8_t m_nbTotalTest;

  CSVLogFile *m_LogFile;

  double m_ramp = 0;
  double m_time0;
};
