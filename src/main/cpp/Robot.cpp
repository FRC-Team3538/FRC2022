// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::RobotInit()
{
  frc::SmartDashboard::PutNumber("Target Feeder", 0.0);
  frc::SmartDashboard::PutNumber("Target Shooter", 0.0);
  frc::SmartDashboard::PutNumber("Target Hood", 0.0);

  IO.ConfigureMotors();
}

void Robot::RobotPeriodic()
{
  IO.UpdateSmartDash();
  IO.drivetrain.Periodic();
  IO.rjVision.Periodic();
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

void Robot::TeleopInit() {}

double deadband(double min, double max, double val)
{
  if (val < 0)
  {
    if (val > -min)
    {
      return 0;
    }
    return (val + min) / (max - min);
  }
  else
  {
    if (val < min)
    {
      return 0;
    }
    return (val - min) / (max - min);
  }
}

void Robot::TeleopPeriodic()
{
  if (IO.mainController.IsConnected())
  {
    double fwd = -deadband(0.05, 1, IO.mainController.GetLeftY());
    double rot = -deadband(0.05, 1, IO.mainController.GetRightX());

    IO.drivetrain.Arcade(fwd, rot);

    double intake = deadband(0.05, 1, IO.mainController.GetR2Axis() / 2) - deadband(0.05, 1, IO.mainController.GetL2Axis() / 2);
    IO.shooter.SetFeeder(units::volt_t(intake) * IO.pdp.GetVoltage());

    double intakePercent;
    if (IO.mainController.GetL1Button()) {
      intakePercent = -targetIntakePercent.GetDouble(0.0);
    } else if (IO.mainController.GetR1Button()) {
      intakePercent = targetIntakePercent.GetDouble(0.0);
    } else {
      intakePercent = 0.0;
    }

    IO.shooter.SetIntake(units::volt_t(intakePercent) * IO.pdp.GetVoltage());
  }

  {
    double shooterVoltage;


    if (lockShooterVoltage.GetBoolean(false))
    {
      shooterVoltage = frc::SmartDashboard::GetNumber("Target Shooter", 0.0);//targetShooterVoltage.GetDouble(0);
    }
    else
    {
      shooterVoltage = 0.0;
    }

    IO.shooter.SetShooter(units::volt_t(shooterVoltage));
  }

  {
    double hoodVoltage;

    if (lockHoodVoltage.GetBoolean(false))
    {
      //hoodVoltage = targetHoodVoltage.GetDouble(0);
      hoodVoltage = frc::SmartDashboard::GetNumber("Target Hood", 0.0);//targetShooterVoltage.GetDouble(0);
    }
    else
    {
      hoodVoltage = 0.0;
    }

    IO.shooter.SetHood(units::volt_t(hoodVoltage));
  }

  {
    double feederVoltage;

    if (lockFeederVoltage.GetBoolean(false))
    {
      feederVoltage = frc::SmartDashboard::GetNumber("Target Feeder", 0.0);//targetFeederVoltage.GetDouble(0);
    }
    else
    {
      feederVoltage = 0.0;
    }

    IO.shooter.SetFeeder(units::volt_t(feederVoltage));
  }
}

void Robot::DisabledInit()
{
  lockShooterVoltage.SetBoolean(true);
  // targetShooterVoltage.SetDouble(0.0);

  lockFeederVoltage.SetBoolean(true);
  // targetFeederVoltage.SetDouble(0.0);

  lockHoodVoltage.SetBoolean(true);
  targetHoodVoltage.SetDouble(0.0);
  targetIntakePercent.SetDouble(0.6);
}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
