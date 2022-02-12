// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


#pragma once

#include <frc/TimedRobot.h>

#include "Robotmap.hpp"
#include "auto/AutoPrograms.hpp"

#include <frc/smartdashboard/SmartDashboard.h>

#include "lib/Logging.h"
#include "lib/DataLogUtils.h"

class Robot : public frc::TimedRobot
{
public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void SimulationInit() override;
  void SimulationPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

private:

  double deadband(double val, double min = 0.10, double max = 1.0);

  Robotmap IO;
  AutoPrograms autoprograms{IO};

#ifdef LOGGER
  rj::DataLogUtils dataLogUtils{};
#endif

  frc::Timer shotTimer;
  frc::Timer brakeTimer;
  frc::Timer intakeTimer;

  enum class ClimberShooterMode:uint8_t
  {
    Shooter = 0,
    Climber
  } m_csmode;

  // Smartdash
  nt::NetworkTableEntry ntRobotName = frc::SmartDashboard::GetEntry("robot/RobotName");

  static constexpr double kVisionAngleTolDefault = 0.75;
  nt::NetworkTableEntry ntVisionAngleTol = frc::SmartDashboard::GetEntry("robot/visionAngleTol");
  
  static constexpr double kShooterRPMDefault = 3500;
  nt::NetworkTableEntry ntShooterRPM = frc::SmartDashboard::GetEntry("robot/shooterRPM");
  
  static constexpr double kShooterTopRPMDefault = 2000;
  nt::NetworkTableEntry ntShooterTopRPM = frc::SmartDashboard::GetEntry("robot/shooterTopRPM");

  static constexpr double kFeederVoltageDefault = 8.0;
  nt::NetworkTableEntry ntFeederVoltage = frc::SmartDashboard::GetEntry("robot/feederVoltage");

  static constexpr double kIndexerVoltageDefault = 8.0;
  nt::NetworkTableEntry ntIndexerVoltage = frc::SmartDashboard::GetEntry("robot/indexerVoltage");

};
