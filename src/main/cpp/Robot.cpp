// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <pathplanner/lib/PathPlanner.h>

using namespace pathplanner;

void Robot::RobotInit()
{
  IO.ConfigureMotors();
}

void Robot::RobotPeriodic()
{
  IO.UpdateSmartDash();
  IO.drivetrain.Periodic();
  autoprograms.SmartDash();
}

void Robot::AutonomousInit()
{
  autoprograms.Init();
}

void Robot::AutonomousPeriodic()
{
  autoprograms.Run();
}

void Robot::TeleopInit() {
  lockShooterVoltage.SetBoolean(true);
  targetShooterVoltage.SetDouble(0.0);
}

void Robot::TeleopPeriodic() {
  {
    double shooterVoltage;

    if (lockShooterVoltage.GetBoolean(false)) {
      shooterVoltage = targetShooterVoltage.GetDouble(0.0);
    } else if (IO.mainController.IsConnected()) {
      shooterVoltage = IO.mainController.GetR2Axis()* IO.pdp.GetVoltage();
    } else {
      shooterVoltage = 0.0;
    }

    IO.shooter.SetShooter(units::volt_t(shooterVoltage));
  }

  {
    double hoodVoltage;

    if (lockHoodVoltage.GetBoolean(false)) {
      hoodVoltage = targetHoodVoltage.GetDouble(0.0);
    } else if (IO.mainController.IsConnected()) {
      hoodVoltage = IO.mainController.GetL2Axis()* IO.pdp.GetVoltage();
    } else {
      hoodVoltage = 0.0;
    }

    IO.shooter.SetHood(units::volt_t(hoodVoltage));
  }

  {
    double feederVoltage;

    if (lockFeederVoltage.GetBoolean(false)) {
      feederVoltage = targetFeederVoltage.GetDouble(0.0);
    } else if (IO.mainController.IsConnected()) {
      feederVoltage = IO.mainController.GetLeftX() * IO.pdp.GetVoltage();
    } else {
      feederVoltage = 0.0;
    }

    IO.shooter.SetFeeder(units::volt_t(feederVoltage));
  }
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
