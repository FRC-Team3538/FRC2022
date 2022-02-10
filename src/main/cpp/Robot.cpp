// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <lib/pathplanner/PathPlanner.h>
#include <cmath>

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

  frc::SmartDashboard::PutData("Power", &IO.pdp);
  frc::SmartDashboard::PutData("Drivebase", &IO.drivetrain);
  frc::SmartDashboard::PutData("Gamepad_Dr", &IO.mainController);
  frc::SmartDashboard::PutData("Gamepad_Op", &IO.secondaryController);
  frc::SmartDashboard::PutData("Shooter", &IO.shooter);

#ifdef LOGGER
  dataLogUtils.EnableNTConnectionLogging();
  dataLogUtils.EnableNTEntryLogging();
  // arg bool - log joystick data if true
  dataLogUtils.InitDSLogging(true);
#endif // LOGGER
  
  frc::SmartDashboard::PutNumber("Feeder Voltage", 7.0);
  // frc::SmartDashboard::PutNumber("Multiplier", 1.0);
  frc::SmartDashboard::PutNumber("Shooter RPM", 3000.0);
  frc::SmartDashboard::PutNumber("Hood Wheel RPM", 4000.0);
}

void Robot::RobotPeriodic()
{
#ifdef LOGGER
  dataLogUtils.LogDSState();
#endif // LOGGER
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
  double indexerSpd = 0.0;
  // *** PRESETS ***

  switch (IO.secondaryController.GetPOV())
  {
  case 0:
  {
    // FENDER
    auto kShooterRPM = units::revolutions_per_minute_t{frc::SmartDashboard::GetNumber("Shooter RPM", 0.0)};
    auto kHoodRPM = units::revolutions_per_minute_t{frc::SmartDashboard::GetNumber("Hood Wheel RPM", 0.0)};

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

  if (IO.mainController.GetR1Button())
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
          indexerSpd = 10.0;
        }
      }
    }
  }
  else
  {
    double fwd = -deadband(IO.mainController.GetLeftY());
    double rot = -deadband(IO.mainController.GetRightX());
    IO.drivetrain.Arcade(fwd, rot);
    // Can neg rot if want to be lazy
  }

  if (IO.secondaryController.GetShareButton())
  {
    //ClimberShooterMode(ClimberShooterMode::Shooter);
    m_csmode = ClimberShooterMode::Shooter;
  }

  if (IO.secondaryController.GetOptionsButton())
  {
    //ClimberShooterMode(ClimberShooterMode::Climber);
    m_csmode = ClimberShooterMode::Climber;

  }

  if (m_csmode == ClimberShooterMode::Climber)
  {
    if (IO.secondaryController.GetR1Button())
    {
     IO.climber.SetClimberState(Climber::ClimbState::Up);
    }
  } 

  // *** SETTING VALUES FOR FEEDER AND INTAKE ***

  if (IO.mainController.GetSquareButton())
    feederSpd = -10.0;
  else if (IO.secondaryController.GetTriangleButton())
    feederSpd = frc::SmartDashboard::GetNumber("Feeder Voltage", 0.0);
  else
    feederSpd = 0.0;

  if (intakeSpd == 0.0)
  {
    intakeSpd += IO.secondaryController.IsConnected() ? (((deadband((IO.secondaryController.GetR2Axis() + 1.0) / 2.0)) - (deadband((IO.secondaryController.GetL2Axis() + 1.0) / 2.0))) * 13.0) : 0.0;
    intakeSpd += IO.mainController.IsConnected() ? (((deadband((IO.mainController.GetR2Axis() + 1.0) / 2.0)) - (deadband((IO.mainController.GetL2Axis() + 1.0) / 2.0))) * 13.0) : 0.0;
    
  }

  if (indexerSpd == 0.0)
  {
    indexerSpd += IO.secondaryController.IsConnected() ? (((deadband((IO.secondaryController.GetR2Axis() + 1.0) / 2.0)) - (deadband((IO.secondaryController.GetL2Axis() + 1.0) / 2.0))) * 13.0) : 0.0;
    indexerSpd += IO.mainController.IsConnected() ? (((deadband((IO.mainController.GetR2Axis() + 1.0) / 2.0)) - (deadband((IO.mainController.GetL2Axis() + 1.0) / 2.0))) * 13.0) : 0.0;
    
  }

  if (IO.secondaryController.GetTriangleButton())
  {
    IO.shooter.SetFeeder(units::volt_t{feederSpd});
    IO.shooter.SetIntake(units::volt_t{feederSpd});
    IO.shooter.SetIndexer(7.0_V);
  }
  else
  {
    IO.shooter.SetIntake(units::volt_t{intakeSpd});
    IO.shooter.SetIndexer(7.0_V);
    IO.shooter.SetFeeder(-2.0_V);
    //IO.shooter.SetIndexer(0_V);
    //IO.shooter.SetFeeder(0_V);
  }

  // *** INTAKE DEPOY CODE ***

  if (intakeSpd != 0.0)
  {
    IO.shooter.SetIntakeState(Shooter::Position::Deployed);
    intakeTimer.Start();
    intakeTimer.Reset();
  }
  else if (intakeTimer.Get() > 0.5_s)
  {
    IO.shooter.SetIntakeState(Shooter::Position::Stowed);
  }
  

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
