// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <pathplanner/lib/PathPlanner.h>

using namespace pathplanner;

double Robot::deadband(double val, double deadband)
{
  if (val > 1.0)
    return 1.0;
  if (val < -1.0)
    return -1.0;
  if (std::abs(val) < deadband)
    return 0.0;
  return val;
}

void Robot::RobotInit()
{
  IO.ConfigureMotors();
  frc::SmartDashboard::PutNumber("Feeder Voltage", 0.0);
  frc::SmartDashboard::PutNumber("Shooter Voltage", 0.0);
  frc::SmartDashboard::PutNumber("Hood Wheel Voltage", 0.0);
}

void Robot::RobotPeriodic()
{
  IO.UpdateSmartDash();
  IO.drivetrain.Periodic();
  autoprograms.SmartDash();
  IO.rjVision.Periodic();
}

void Robot::AutonomousInit()
{
  autoprograms.Init();
}

void Robot::AutonomousPeriodic()
{
  autoprograms.Run();
}

void Robot::TeleopInit()
{
  lockShooterVoltage.SetBoolean(true);
  targetShooterVoltage.SetDouble(0.0);
}

void Robot::TeleopPeriodic()
{
  IO.drivetrain.Arcade(deadband(-IO.mainController.GetLeftY(), deadbandVal), deadband(-IO.mainController.GetRightX(), deadbandVal));

  {
    double shooterVoltage;

    if (IO.mainController.IsConnected())
    {
      shooterVoltage = (IO.mainController.GetR1Button() || IO.secondaryController.GetR1Button()) ? frc::SmartDashboard::GetNumber("Shooter Voltage", 0.0) : 0.0;
      std::cout << shooterVoltage << std::endl;
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
      hoodVoltage = targetHoodVoltage.GetDouble(0.0);
    }
    else if (IO.mainController.IsConnected())
    {
      hoodVoltage = (IO.mainController.GetR1Button() || IO.secondaryController.GetR1Button()) ? frc::SmartDashboard::GetNumber("Hood Wheel Voltage", 0.0) : 0.0;
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
      feederVoltage = targetFeederVoltage.GetDouble(0.0);
    }
    else if (IO.mainController.IsConnected())
    {
      feederVoltage = (IO.mainController.GetR1Button() || IO.secondaryController.GetR1Button()) ? frc::SmartDashboard::GetNumber("Feeder Voltage", 0.0) : 0.0;
    }
    else
    {
      feederVoltage = 0.0;
    }

    IO.shooter.SetFeeder(units::volt_t(feederVoltage));
  }

  {
    double intakeVoltage;

    if (IO.mainController.IsConnected())
    {
      if (IO.mainController.GetR1ButtonPressed() || IO.secondaryController.GetR1ButtonPressed())
      {
        shotTimer.Reset();
        shotTimer.Start();
      }

      if (IO.secondaryController.GetR1Button() || IO.mainController.GetR1Button())
      {
        intakeVoltage = shotTimer.Get() > 0.25_s ? 10.0 : 0.0; 
      }
      else
      {
        intakeVoltage = (deadband((IO.secondaryController.GetR2Axis() + 1.0) / 2.0, deadbandVal)) * 13.0;
      }
    }
    else
    {
      intakeVoltage = 0.0;
    }

    IO.shooter.SetIntake(units::volt_t{intakeVoltage});
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
