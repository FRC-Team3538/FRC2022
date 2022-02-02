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
  IO.watchdog.Disable();
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
  IO.drivetrain.SetBrakeMode();
}

void Robot::AutonomousPeriodic()
{
  autoprograms.Run();
}

void Robot::TeleopInit()
{
  lockShooterVoltage.SetBoolean(true);
  targetShooterVoltage.SetDouble(0.0);
  IO.drivetrain.SetBrakeMode();
}

void Robot::TeleopPeriodic()
{
  if (IO.mainController.GetTriangleButton())
  {
    vision::RJVisionPipeline::visionData data = IO.rjVision.Run();
    std::cout << data.distance.value() << std::endl;
    if (data.filled)
    {
      if (IO.drivetrain.TurnRel(0.0, data.angle, 1_deg))
      {
        Shooter::State shotStats = IO.shooter.CalculateShot(data.distance);
        IO.shooter.SetShooterRPM(shotStats.shooterVelocity);
        IO.shooter.SetHoodRPM(shotStats.hoodVelocity);
        if (IO.shooter.TempUpToSpeed())
        {
          IO.shooter.SetFeeder(10_V);
          IO.shooter.SetIntake(10_V);
        }
      }
    }
  }
  else
  {
    IO.drivetrain.Arcade(deadband(IO.mainController.GetLeftY(), deadbandVal), deadband(IO.mainController.GetRightX(), deadbandVal));
    IO.shooter.SetFeeder(0_V);
    double intakeVoltage = IO.mainController.IsConnected() ? (((deadband((IO.mainController.GetR2Axis() + 1.0) / 2.0, deadbandVal)) - (deadband((IO.mainController.GetL2Axis() + 1.0) / 2.0, deadbandVal))) * 13.0) : 0.0;
    IO.shooter.SetIntake(units::volt_t{intakeVoltage});
    IO.shooter.SetHoodRPM(units::revolutions_per_minute_t{0});
    IO.shooter.SetShooterRPM(units::revolutions_per_minute_t{0});
  }

  {
    double shooterVoltage;

    if (IO.mainController.IsConnected())
    {
      shooterVoltage = (IO.mainController.GetR1Button() || IO.secondaryController.GetR1Button()) ? frc::SmartDashboard::GetNumber("Shooter Voltage", 0.0) : 0.0;
      // std::cout << shooterVoltage << std::endl;
    }
    else
    {
      shooterVoltage = 0.0;
    }

    //IO.shooter.SetShooterRPM(units::revolutions_per_minute_t(shooterVoltage));
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

    //IO.shooter.SetHoodRPM(units::revolutions_per_minute_t(hoodVoltage));
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

    //IO.shooter.SetFeeder(units::volt_t(feederVoltage));
  }

  {
    double intakeVoltage = 0.0;

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
        intakeVoltage += IO.secondaryController.IsConnected() ? (((deadband((IO.secondaryController.GetR2Axis() + 1.0) / 2.0, deadbandVal)) - (deadband((IO.secondaryController.GetL2Axis() + 1.0) / 2.0, deadbandVal))) * 13.0) : 0.0;
        intakeVoltage += IO.mainController.IsConnected() ? (((deadband((IO.mainController.GetR2Axis() + 1.0) / 2.0, deadbandVal)) - (deadband((IO.mainController.GetL2Axis() + 1.0) / 2.0, deadbandVal))) * 13.0) : 0.0;
      }
    }
    else
    {
      intakeVoltage = 0.0;
    }

    //IO.shooter.SetIntake(units::volt_t{intakeVoltage});
  }



}

void Robot::DisabledInit() {
  brakeTimer.Reset();
  brakeTimer.Start();
}
void Robot::DisabledPeriodic() {
  frc::SmartDashboard::PutNumber("Driver FWD/REV (FWD +)", -IO.mainController.GetLeftY());
  frc::SmartDashboard::PutNumber("Driver LEFT/RIGHT (LEFT +)", -IO.mainController.GetRightX());
  frc::SmartDashboard::PutNumber("DT Gyro (CCW +)", IO.drivetrain.GetYaw().Radians().value());
  if (brakeTimer.Get() > 3.0_s)
  {
    IO.drivetrain.SetCoastMode();
    }
  }
  // steps afterward:
  // + voltage => fwd motion
  // fwd motion => + encoder count


void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
