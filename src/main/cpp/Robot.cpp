/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Constants.h"
#include "Robot.h"
#include "lib/NL/NLOdometry.h"
#include <iostream>
#include <frc/shuffleboard/Shuffleboard.h>
#include <time.h>
// #include <units/units.h>

#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/SmallString.h>

double Deadband(double value, double deadband = 0.1)
{
    if (std::abs(value) < deadband)
        return 0;
    else
        return value < 0 ? (value + deadband) / (1.0 - deadband) : (value - deadband) / (1.0 - deadband);
}

void Robot::Drive(double forward, double turn)
{
    // cout<<forward<<std::endl;
    forward = Deadband(forward, 0.1);
    turn = Deadband(turn, 0.2);

    /*
    double c = 0.35 * (turn * 5.0 * (abs(turn) + 1) / (abs(forward) + 1));
    if (turn < 0.0) {
        m_drivetrain->Drive(forward * ((c + 1) / (1 - c)), forward);
    } else {
        m_drivetrain->Drive(forward, forward * ((1 - c) / (c + 1)));
    }*/

    double v = forward * VMAX;
    double w = turn * WMAX * m_turnAdjustFactor;

    // w = m_drivetrain->CalculateTurn(forward, w);

    double lwheel = v + (w * HALF_TRACKWIDTH);
    double rwheel = v - (w * HALF_TRACKWIDTH);

    double k;
    k = 1.0 / (NMAX(VMAX, NMAX(NABS(lwheel), NABS(rwheel))));
    lwheel *= k;
    rwheel *= k;

    // cout<<lwheel<<std::endl;

    m_moteurGauche.Set(lwheel);
    m_moteurGaucheFollower.Set(lwheel);
    m_moteurGaucheFollower2.Set(lwheel);
    m_moteurDroite.Set(rwheel);
    m_moteurDroiteFollower.Set(rwheel);
    m_moteurDroiteFollower2.Set(rwheel);
}

void Robot::RobotInit()
{
    std::cout << "RobotInit Step 1" << std::endl;
    m_moteurDroite.RestoreFactoryDefaults();
    m_moteurGauche.RestoreFactoryDefaults();
    m_moteurGaucheFollower2.RestoreFactoryDefaults();
    m_moteurDroiteFollower2.RestoreFactoryDefaults();
    m_moteurDroiteFollower.RestoreFactoryDefaults();
    m_moteurGaucheFollower.RestoreFactoryDefaults();

    m_moteurDroite.SetSmartCurrentLimit(40);
    m_moteurDroiteFollower.SetSmartCurrentLimit(40);
    m_moteurDroiteFollower2.SetSmartCurrentLimit(40);
    m_moteurGauche.SetSmartCurrentLimit(40);
    m_moteurGaucheFollower.SetSmartCurrentLimit(40);
    m_moteurGaucheFollower2.SetSmartCurrentLimit(40);

#ifdef TIME_RAMP
    m_moteurDroite.SetOpenLoopRampRate(TIME_RAMP);
    m_moteurGauche.SetOpenLoopRampRate(TIME_RAMP);
    m_moteurDroiteFollower.SetOpenLoopRampRate(TIME_RAMP);
    m_moteurGaucheFollower.SetOpenLoopRampRate(TIME_RAMP);
    m_moteurDroiteFollower2.SetOpenLoopRampRate(TIME_RAMP);
    m_moteurGaucheFollower2.SetOpenLoopRampRate(TIME_RAMP);
#else
    m_moteurDroite.SetOpenLoopRampRate(0);
    m_moteurGauche.SetOpenLoopRampRate(0);
    m_moteurDroiteFollower.SetOpenLoopRampRate(0);
    m_moteurGaucheFollower.SetOpenLoopRampRate(0);
    m_moteurDroiteFollower2.SetOpenLoopRampRate(0);
    m_moteurGaucheFollower2.SetOpenLoopRampRate(0);
#endif
#if VOLTAGE_COMPENSATION
    m_moteurDroite.EnableVoltageCompensation(VOLTAGE_COMPENSATION_VALUE);
    m_moteurGauche.EnableVoltageCompensation(VOLTAGE_COMPENSATION_VALUE);
    m_moteurDroiteFollower.EnableVoltageCompensation(VOLTAGE_COMPENSATION_VALUE);
    m_moteurGaucheFollower.EnableVoltageCompensation(VOLTAGE_COMPENSATION_VALUE);
    m_moteurDroiteFollower2.EnableVoltageCompensation(VOLTAGE_COMPENSATION_VALUE);
    m_moteurGaucheFollower2.EnableVoltageCompensation(VOLTAGE_COMPENSATION_VALUE);
#else
    m_moteurGauche.DisableVoltageCompensation();
    m_moteurGaucheFollower.DisableVoltageCompensation();
    m_moteurDroite.DisableVoltageCompensation();
    m_moteurDroiteFollower.DisableVoltageCompensation();
    m_moteurGaucheFollower2.DisableVoltageCompensation();
    m_moteurDroiteFollower2.DisableVoltageCompensation();
    µ
#endif
    m_moteurDroite.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_moteurGauche.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_moteurDroiteFollower.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_moteurGaucheFollower.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_moteurDroiteFollower2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_moteurGaucheFollower2.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    std::cout << "RobotInit Step 2" << std::endl;
    m_PowerEntry = frc::Shuffleboard::GetTab("voltage").Add("Voltage", 0.0).WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
    m_LogFileName = frc::Shuffleboard::GetTab("voltage").Add("Logfile Name", "").WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
    m_customEntry = frc::Shuffleboard::GetTab("voltage").Add("Data", "").WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
#if IMU
    m_speedY = frc::Shuffleboard::GetTab("voltage").Add("speedY", 0.0).WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
    m_speedX = frc::Shuffleboard::GetTab("voltage").Add("speedX", 0.0).WithWidget(frc::BuiltInWidgets::kTextView).GetEntry();
#endif
    std::cout << "RobotInit Step 3" << std::endl;
    m_encodeurExterneDroite.SetReverseDirection(true);
    m_encodeurExterneGauche.SetReverseDirection(false);

    m_encodeurExterneDroite.SetDistancePerPulse(1);
    m_encodeurExterneGauche.SetDistancePerPulse(1);

    m_encodeurExterneDroite.SetSamplesToAverage(65);
    m_encodeurExterneGauche.SetSamplesToAverage(65);
    std::cout << "RobotInit Step 4" << std::endl;

    m_moteurDroite.SetInverted(false);
    m_moteurDroiteFollower.SetInverted(false);
    m_moteurDroiteFollower2.SetInverted(false);

    m_moteurGauche.SetInverted(true);
    m_moteurGaucheFollower.SetInverted(true);
    m_moteurGaucheFollower2.SetInverted(true);

    m_moteurDroite.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 4);
    m_moteurDroite.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 4);
    m_moteurDroite.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 50);

    m_moteurDroiteFollower.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 4);
    m_moteurDroiteFollower.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 4);
    m_moteurDroiteFollower.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 50);

    m_moteurDroiteFollower2.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 4);
    m_moteurDroiteFollower2.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 4);
    m_moteurDroiteFollower2.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 50);

    m_moteurGauche.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 4);
    m_moteurGauche.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 4);
    m_moteurGauche.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 50);

    m_moteurGaucheFollower.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 4);
    m_moteurGaucheFollower.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 4);
    m_moteurGaucheFollower.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 50);

    m_moteurGaucheFollower2.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 4);
    m_moteurGaucheFollower2.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 4);
    m_moteurGaucheFollower2.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 50);

    std::cout << "RobotInit Step 5" << std::endl;
    Robot::AddPeriodic([&]()
                       { m_motorCharacterizationTests.fastLoop(); },
                       2_ms, 1_ms);
    std::cout << "RobotInit Step 6" << std::endl;
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
    m_encodeurExterneDroite.Reset();
    m_encodeurExterneGauche.Reset();

    m_isLogging = 0;
    m_isPathFollowing = 0;

    // reset robot path
    m_dsLeftWheel = 0.0f;
    m_dsRightWheel = 0.0f;

    m_currrentSState.null();

    m_refLeftS = 0.0f;
    m_refRightS = 0.0f;
    m_prevK = 0.0f;
    m_prevS = 0.0f;

    m_errorLeft.reset();
    m_errorRight.reset();

#if IMU
    init_x = m_imu.GetAccelInstantX();
    init_y = m_imu.GetAccelInstantY();
#endif
}

void Robot::TeleopPeriodic()
{
    // int errorLeftState = m_moteurGauche.GetStickyFaults();
    // int errorLeftFollowerState = m_moteurGaucheFollower.GetStickyFaults();
    // int errorLeftFollowe2rState = m_moteurGaucheFollower2.GetStickyFaults();

    // int errorRightState = m_moteurDroite.GetStickyFaults();
    // int errorRightFollowerState = m_moteurDroiteFollower.GetStickyFaults();
    // int errorRightFollower2State = m_moteurDroiteFollower2.GetStickyFaults();

    // frc::Shuffleboard::GetTab("voltage").Add("errorLeftState", errorLeftState).WithWidget(frc::BuiltInWidgets::kTextView);
    // frc::Shuffleboard::GetTab("voltage").Add("errorLeftFollowerState", errorLeftFollowerState).WithWidget(frc::BuiltInWidgets::kTextView);
    // frc::Shuffleboard::GetTab("voltage").Add("errorLeftFollower2State", errorLeftFollowe2rState).WithWidget(frc::BuiltInWidgets::kTextView);
    // frc::Shuffleboard::GetTab("voltage").Add("errorRightState", errorRightState).WithWidget(frc::BuiltInWidgets::kTextView);
    // frc::Shuffleboard::GetTab("voltage").Add("errorRightFollowerState", errorRightFollowerState).WithWidget(frc::BuiltInWidgets::kTextView);
    // frc::Shuffleboard::GetTab("voltage").Add("errorRightFollower2State", errorRightFollower2State).WithWidget(frc::BuiltInWidgets::kTextView);

    char infos[256];
    char desc[256];
#if IMU
    m_speedY.SetDouble(filterY.Calculate(m_imu.GetAccelInstantY() - init_y));
    m_speedX.SetDouble(filterX.Calculate(m_imu.GetAccelInstantX() - init_x));
#endif
    if (m_motorCharacterizationTests.getState() == NLCharacterization_Tests::State::Stopped)
    {
        Drive(-m_driverController.GetLeftY(), m_driverController.GetRightX());
    }

    if (m_driverController.GetBButtonPressed())
    {
        if (m_motorCharacterizationTests.getState() == NLCharacterization_Tests::State::Stopped)
        {
            m_motorCharacterizationTests.nextTest();
            sprintf(infos, "%s En Attente ... Appuyer sur A pour Démarrer.", m_motorCharacterizationTests.getCurrentTestDescription(desc, 256));
            m_customEntry->SetString(infos);
        }
    }

    if (m_driverController.GetXButtonPressed())
    {
        if (m_motorCharacterizationTests.getState() == NLCharacterization_Tests::State::Stopped)
        {

            m_motorCharacterizationTests.previousTest();
            sprintf(infos, "%s En Attente ... Appuyer sur A pour Démarrer.", m_motorCharacterizationTests.getCurrentTestDescription(desc, 256));
            m_customEntry->SetString(infos);
        }
    }

    if (m_driverController.GetAButtonPressed())
    {
        if (m_motorCharacterizationTests.getState() == NLCharacterization_Tests::State::Started)
        {
            m_motorCharacterizationTests.stop();
            m_motorCharacterizationTests.nextTest();
            sprintf(infos, "%s En Attente ... Appuyer sur A pour Démarrer.", m_motorCharacterizationTests.getCurrentTestDescription(desc, 256));
            m_customEntry->SetString(infos);
        }
        else if (m_motorCharacterizationTests.getState() == NLCharacterization_Tests::State::Stopped)
        {
            m_motorCharacterizationTests.start();
            sprintf(infos, "%s En Cours ... Appuyer sur A pour Arrêter.", m_motorCharacterizationTests.getCurrentTestDescription(desc, 256));
            m_customEntry->SetString(infos);
        }
    }

    m_LogFileName->SetString(m_motorCharacterizationTests.getCurrentFileLogName(infos, 256));
}

void Robot::TestInit()
{
}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif