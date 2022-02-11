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
  Robotmap IO;
#ifdef LOGGER
  rj::DataLogUtils dataLogUtils{};
#endif
  AutoPrograms autoprograms{IO};

  double shooterPresetVal = 0.0;
  double hoodPresetVal = 0.0;
  double feederPresetVal = 0.0;

  nt::NetworkTableEntry lockShooterVoltage = frc::SmartDashboard::GetEntry("/lockShooterVoltage");
  nt::NetworkTableEntry targetShooterVoltage = frc::SmartDashboard::GetEntry("/targetShooterVoltage");
  nt::NetworkTableEntry lockHoodVoltage = frc::SmartDashboard::GetEntry("/lockHoodVoltage");
  nt::NetworkTableEntry targetHoodVoltage = frc::SmartDashboard::GetEntry("/targetHoodVoltage");
  nt::NetworkTableEntry lockFeederVoltage = frc::SmartDashboard::GetEntry("/lockFeederVoltage");
  nt::NetworkTableEntry targetFeederVoltage = frc::SmartDashboard::GetEntry("/targetFeederVoltage");
  nt::NetworkTableEntry targetIntakePercent = frc::SmartDashboard::GetEntry("/targetIntakePercent");

  const double deadbandVal = 0.1;

  double deadband(double val, double min = 0.1, double max = 1.0);

  frc::Timer shotTimer;
  frc::Timer brakeTimer;
  frc::Timer intakeTimer;

  enum class ClimberShooterMode:uint8_t
  {
    Shooter = 0,
    Climber
  } m_csmode;

  Shooter::State shotStats;

  bool climberMode = true;
};
