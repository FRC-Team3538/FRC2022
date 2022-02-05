// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <lib/pathplanner/PathPlanner.h>

using namespace pathplanner;

double Robot::deadband(double val, double min = 0.1, double max = 1.0)
{
  if (val > max)
  {
    return max;
  }
  else if (val < -max)
  {
    return -max;
  }
  else if (std::abs(val) < min)
  {
    return 0.0;
  }
  else
  {
    double sgn = val / std::abs(val);
    return sgn * (std::abs(val) - min) / (max - min) * max;
  }
}

void Robot::RobotInit()
{
  IO.watchdog.Disable();
  IO.ConfigureSystem();
  IO.drivetrain.SetCoastMode();

  frc::SmartDashboard::PutData("DriveBase", &IO.drivetrain);

  dataLogUtils.EnableNTConnectionLogging();
  dataLogUtils.EnableNTEntryLogging();
  frc::SmartDashboard::PutNumber("Feeder Voltage", 0.0);
  frc::SmartDashboard::PutNumber("Shooter Voltage", 0.0);
  frc::SmartDashboard::PutNumber("Hood Wheel Voltage", 0.0);
}

void Robot::RobotPeriodic()
{
  IO.UpdateSmartDash();
  IO.drivetrain.Periodic();
  autoprograms.SmartDash();
  // IO.rjVision.Periodic();
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
  double intakeSpd = 0.0;
  double feederSpd = 0.0;

  // *** INTAKE DEPLOY ***

  if (IO.mainController.GetR1Button() || IO.secondaryController.GetR1Button())
  {
    IO.shooter.SetIntakeState(Shooter::Position::Deployed);
  }

  if (IO.mainController.GetL1Button() || IO.secondaryController.GetL1Button())
  {
    IO.shooter.SetIntakeState(Shooter::Position::Stowed);
  }

  // *** PRESETS ***

  switch (IO.secondaryController.GetPOV())
  {
  case 0:
  {
    // FENDER
    auto kShooterRPM = 0.0_rpm;
    auto kHoodRPM = 0.0_rpm;

    shotStats = {kShooterRPM, kHoodRPM, 0.0_deg};
    break;
  }

  case 90:
  {
    // MIDFIELD
    auto kShooterRPM = 0.0_rpm;
    auto kHoodRPM = 0.0_rpm;

    shotStats = {kShooterRPM, kHoodRPM, 0.0_deg};
    break;
  }

  case 180:
  {
    // LAUNCHPAD
    auto kShooterRPM = 0.0_rpm;
    auto kHoodRPM = 0.0_rpm;

    shotStats = {kShooterRPM, kHoodRPM, 0.0_deg};
    break;
  }

  case 270:
  {
    // TARMAC
    auto kShooterRPM = 0.0_rpm;
    auto kHoodRPM = 0.0_rpm;

    shotStats = {kShooterRPM, kHoodRPM, 0.0_deg};
    break;
  }

  default:
  {
  }
  }

  // *** MANUAL SHOOTING ***

  if (IO.secondaryController.GetCircleButtonPressed())
    IO.shooter.SetShooterState(shotStats);
  else if (IO.secondaryController.GetCrossButtonPressed())
    IO.shooter.SetShooterState({0.0_rpm, 0.0_rpm, 0.0_deg});

  if (!climberMode)
  {
    // HOOD CODE
  }

  // *** VISION AND DRIVING ***

  if (IO.mainController.GetR2Button())
  {
    vision::RJVisionPipeline::visionData data = IO.rjVision.Run();
    if (data.filled)
    {
      if (IO.drivetrain.TurnRel(0.0, data.angle, 0.75_deg))
      {
        Shooter::State shotStat = IO.shooter.CalculateShot(data.distance);
        IO.shooter.SetShooterState(shotStat);
        if (IO.shooter.TempUpToSpeed())
        {
          feederSpd = 10.0;
          intakeSpd = 10.0;
        }
      }
    }
  }
  else
  {
    double fwd = deadband(IO.mainController.GetLeftY());
    double rot = deadband(IO.mainController.GetRightX());
    IO.drivetrain.Arcade(fwd, rot);
  }

  // *** SETTING VALUES FOR FEEDER AND INTAKE ***

  if (IO.mainController.GetSquareButton())
    feederSpd = -10.0;
  else if (IO.secondaryController.GetTriangleButton())
    feederSpd = 10.0;
  else
    feederSpd = 0.0;

  if (intakeSpd == 0.0 || IO.mainController.GetTriangleButton())
  {
    intakeSpd += IO.secondaryController.IsConnected() ? (((deadband((IO.secondaryController.GetR2Axis() + 1.0) / 2.0)) - (deadband((IO.secondaryController.GetL2Axis() + 1.0) / 2.0))) * 13.0) : 0.0;
    intakeSpd += IO.mainController.IsConnected() ? (((deadband((IO.mainController.GetL2Axis() + 1.0) / 2.0)) - IO.mainController.GetTriangleButton()) * 13.0) : 0.0;
  }

  IO.shooter.SetFeeder(units::volt_t{feederSpd});
  IO.shooter.SetIntake(units::volt_t{intakeSpd});

  // *** CLIMBER CODE ***

  if (IO.secondaryController.GetPSButtonPressed())
    climberMode = !climberMode;

  if (climberMode)
  {
    IO.climber.SetClimber(deadband(IO.secondaryController.GetLeftY()));

    // CLIMB ANGLE SETTING
  }
  else
  {
    IO.climber.SetClimber(0.0);
    IO.climber.SetClimberState(Climber::ClimbState::Down);
  }
}

void Robot::DisabledInit()
{
  brakeTimer.Reset();
  brakeTimer.Start();
}

void Robot::DisabledPeriodic()
{
  // Automatic Coast Mode
  if (brakeTimer.Get() > 3.0_s)
  {
    IO.drivetrain.SetCoastMode();
  }
}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic()
{
  IO.drivetrain.SimulationPeriodic();
}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
