// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

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

  frc::SmartDashboard::PutBoolean("/lockShooterVoltage", true);
  frc::SmartDashboard::PutNumber("/targetShooterVoltage", 0.0);
  frc::SmartDashboard::PutBoolean("/lockHoodVoltage", true);
  frc::SmartDashboard::PutNumber("/targetHoodVoltage", 0.0);
  frc::SmartDashboard::PutBoolean("/lockFeederVoltage", true);
  frc::SmartDashboard::PutNumber("/targetFeederVoltage", 0.0);

  // lockShooterVoltage.SetBoolean(true);
  // targetShooterVoltage.SetDouble(0.0);

  // lockFeederVoltage.SetBoolean(true);
  // targetFeederVoltage.SetDouble(0.0);

  // lockHoodVoltage.SetBoolean(true);
  // targetHoodVoltage.SetDouble(0.0);
}

double deadband(double min, double max, double val) {
  if (val < 0) {
    if (val > -min) {
      return 0;
    }
    return (val + min) / (max - min);
  } else {
    if (val < min) {
      return 0;
    }
    return (val - min) / (max - min);
  }
}

void Robot::TeleopPeriodic() {
  if (IO.mainController.IsConnected()) {
    double fwd = -deadband(0.05, 1, IO.mainController.GetLeftY());
    double rot = -deadband(0.05, 1, IO.mainController.GetRightX());

    IO.drivetrain.Arcade(fwd, rot);

    double intake = deadband(0.05, 1, IO.mainController.GetR2Axis() / 2) 
                  - deadband(0.05, 1, IO.mainController.GetL2Axis() / 2);
    IO.shooter.SetFeeder(units::volt_t(intake) * IO.pdp.GetVoltage());

  }

  {
    double shooterVoltage;

    if (frc::SmartDashboard::GetBoolean("/lockShooterVoltage", false)) {
      shooterVoltage = frc::SmartDashboard::GetNumber("/targetShooterVoltage", 0);
    } else {
      shooterVoltage = 0.0;
    }

    IO.shooter.SetShooter(units::volt_t(shooterVoltage));
  }

  {
    double hoodVoltage;

    if (frc::SmartDashboard::GetBoolean("/lockHoodVoltage", false)) {
      hoodVoltage = frc::SmartDashboard::GetNumber("/targetHoodVoltage", 0);
    } else {
      hoodVoltage = 0.0;
    }

    IO.shooter.SetHood(units::volt_t(hoodVoltage));
  }

  {
    double feederVoltage;

    if (frc::SmartDashboard::GetBoolean("/lockFeederVoltage", false)) {
      feederVoltage = frc::SmartDashboard::GetNumber("/targetFeederVoltage", 0);
    } else {
      feederVoltage = 0.0;
    }

    // IO.shooter.SetFeeder(units::volt_t(feederVoltage));
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
