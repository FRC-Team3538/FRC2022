// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <lib/pathplanner/PathPlanner.h>
#include <frc/livewindow/LiveWindow.h>
#include <cmath>

using namespace pathplanner;

void Robot::RobotInit()
{
  // Disable Live Window Stuff, we don't use it...
  frc::LiveWindow::DisableAllTelemetry();
  frc::LiveWindow::SetEnabled(false);
  
  // Unsure if we need this stuff or not, leaving here as a reminder that it's an option.
  // SetNetworkTablesFlushEnabled(false);

  // System Setup Stuff
  IO.watchdog.Disable();
  IO.ConfigureSystem();
  IO.drivetrain.SetCoastMode();

  // Smartdash Stuff
  frc::SmartDashboard::PutData("Power", &IO.pdp);
  frc::SmartDashboard::PutData("Dr", &IO.mainController);
  frc::SmartDashboard::PutData("Op", &IO.secondaryController);
  frc::SmartDashboard::PutData("Drive", &IO.drivetrain);
  frc::SmartDashboard::PutData("Shooter", &IO.shooter);
  // TODO: Climber, Vision, PDH

  ntRobotName.ForceSetString(ntRobotName.GetString("UnnamedRobot"));
  ntRobotName.SetPersistent();

  ntVisionAngleTol.ForceSetDouble(ntVisionAngleTol.GetDouble(kVisionAngleTolDefault));
  ntVisionAngleTol.SetPersistent();
  
  // Logging Stuff
#ifdef LOGGER
  dataLogUtils.EnableNTConnectionLogging();
  dataLogUtils.EnableNTEntryLogging();
  // arg bool - log joystick data if true
  dataLogUtils.InitDSLogging(true);
#endif // LOGGER
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
  IO.drivetrain.SetBrakeMode();
}

void Robot::TeleopPeriodic()
{
  bool shoot = false;

  //
  // *** VISION AND DRIVING ***
  //
  if (IO.mainController.GetR1Button() || IO.secondaryController.GetTriangleButtonPressed())
  {
    vision::RJVisionPipeline::visionData data = IO.rjVision.Run();
    if (data.filled)
    { 
      // Start Shooter
      Shooter::State shotStat = IO.shooter.CalculateShot(data.distance);
      IO.shooter.SetShooterRPM(shotStat.shooterRPM);
      IO.shooter.SetShooterTopRPM(shotStat.shooterTopRPM);

      // Shoot
      units::degree_t tol{ntVisionAngleTol.GetDouble(kVisionAngleTolDefault)};
      shoot = IO.drivetrain.TurnRel(0.0, data.angle, tol);
    }
  }
  else
  {
    double fwd = -deadband(IO.mainController.GetLeftY());
    double rot = -deadband(IO.mainController.GetRightX());
    IO.drivetrain.Arcade(fwd, rot);
  }  

  //
  // *** SHOOTER ***
  //
  switch (IO.secondaryController.GetPOV())
  {
  case 0:
    // FENDER
    IO.shooter.SetShooterRPM(0_rpm);
    IO.shooter.SetShooterTopRPM(0_rpm);
    m_csmode = ClimberShooterMode::Shooter;
    break;

  case 90:
    // MIDFIELD
    IO.shooter.SetShooterRPM(0_rpm);
    IO.shooter.SetShooterTopRPM(0_rpm);
    m_csmode = ClimberShooterMode::Shooter;
    break;

  case 180:
    // LAUNCHPAD
    IO.shooter.SetShooterRPM(0_rpm);
    IO.shooter.SetShooterTopRPM(0_rpm);
    m_csmode = ClimberShooterMode::Shooter;
    break;

  case 270:
    // TARMAC
    IO.shooter.SetShooterRPM(0_rpm);
    IO.shooter.SetShooterTopRPM(0_rpm);
    m_csmode = ClimberShooterMode::Shooter;
    break;
  }

  // Shoot
  if (shoot || IO.secondaryController.GetSquareButtonPressed())
  {
    m_csmode = ClimberShooterMode::Shooter;
    IO.shooter.Shoot();
    shoot = true;
  }

  // Stop Shooter
  if (IO.secondaryController.GetCrossButtonPressed())
  {
    IO.shooter.SetShooter(0_V);
    IO.shooter.SetShooterTop(0_V);
  }

  //
  // *** INTAKE, INDEXER, & FEEDER ***  
  //
  double drL2 = IO.mainController.IsConnected() ? deadband((IO.mainController.GetL2Axis() + 1.0) / 2.0) : 0.0;
  double drR2 = IO.mainController.IsConnected() ? deadband((IO.mainController.GetR2Axis() + 1.0) / 2.0) : 0.0;
  double opL2 = IO.secondaryController.IsConnected() ? deadband((IO.secondaryController.GetL2Axis() + 1.0) / 2.0) : 0.0;
  double opR2 = IO.secondaryController.IsConnected() ? deadband((IO.secondaryController.GetR2Axis() + 1.0) / 2.0) : 0.0;

  auto intakeCmd = (opR2 - opL2 + drR2 - drL2) * 13.0_V;
  IO.shooter.SetIntake(intakeCmd);

  if (units::math::abs(intakeCmd) > 0.0_V)
  {
    IO.shooter.SetIntakeState(Shooter::Position::Deployed);
    intakeTimer.Start();
    intakeTimer.Reset();

    IO.shooter.SetIndexer(intakeCmd);
  } 
  else if(shoot) 
  {
    IO.shooter.SetIndexer(10_V);
  } 
  else 
  {
    IO.shooter.SetIndexer(0_V);
    IO.shooter.SetFeeder(0_V);
  }

  // Retract Intake Automatically
  if (intakeTimer.Get() > 0.5_s)
  {
    IO.shooter.SetIntakeState(Shooter::Position::Stowed);
  }

 
  //
  // *** Climber ***
  //
  if (IO.secondaryController.GetR1Button())
  {
    IO.climber.SetClimberState(Climber::ClimbState::Up);
    m_csmode = ClimberShooterMode::Climber;
  }

  if (IO.secondaryController.GetL1Button())
  {
    IO.climber.SetClimberState(Climber::ClimbState::Down);
    m_csmode = ClimberShooterMode::Climber;
  }

  if (m_csmode == ClimberShooterMode::Climber)
  {
    IO.climber.SetClimber(deadband(IO.secondaryController.GetLeftY()));
  }
  else
  {
    IO.climber.SetClimber(0.0);
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

double Robot::deadband(double val, double min, double max)
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

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
