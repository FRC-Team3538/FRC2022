// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>

#include "Robotmap.hpp"
#include "auto/AutoPrograms.hpp"

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/PS4Controller.h>


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

  void TestInit() override;
  void TestPeriodic() override;

private:
  Robotmap IO;

  AutoPrograms autoprograms{IO};

  nt::NetworkTableEntry lockShooterVoltage = frc::SmartDashboard::GetEntry("/lockShooterVoltage");
  nt::NetworkTableEntry targetShooterVoltage = frc::SmartDashboard::GetEntry("/targetShooterVoltage");

  nt::NetworkTableEntry lockHoodVoltage = frc::SmartDashboard::GetEntry("/lockHoodVoltage");
  nt::NetworkTableEntry targetHoodVoltage = frc::SmartDashboard::GetEntry("/targetHoodVoltage");

  nt::NetworkTableEntry lockFeederVoltage = frc::SmartDashboard::GetEntry("/lockFeederVoltage");
  nt::NetworkTableEntry targetFeederVoltage = frc::SmartDashboard::GetEntry("/targetFeederVoltage");
};
