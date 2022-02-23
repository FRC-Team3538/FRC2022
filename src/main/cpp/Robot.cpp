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
  // frc::SmartDashboard::PutData("Dr", &IO.mainController);
  // frc::SmartDashboard::PutData("Op", &IO.secondaryController);
  frc::SmartDashboard::PutData("Drive", &IO.drivetrain);
  frc::SmartDashboard::PutData("Shooter", &IO.shooter);
  frc::SmartDashboard::PutData("Climber", &IO.climber);
  frc::SmartDashboard::PutData("Ph", &IO.ph);
  // TODO: Climber, Vision, PDH
  // TODO: Move to a separate table so it doesn't fill smartdash automatically?
  // TODO: Canbus utlization

  ntRobotName.ForceSetString(ntRobotName.GetString("UnnamedRobot"));
  ntRobotName.SetPersistent();

  ntVisionAngleTol.ForceSetDouble(ntVisionAngleTol.GetDouble(kVisionAngleTolDefault));
  ntVisionAngleTol.SetPersistent();

  ntShooterRPM.ForceSetDouble(ntShooterRPM.GetDouble(kShooterRPMDefault));
  ntShooterRPM.SetPersistent();
  ntFeederVoltage.ForceSetDouble(ntFeederVoltage.GetDouble(kFeederVoltageDefault));
  ntFeederVoltage.SetPersistent();
  ntIndexerVoltage.ForceSetDouble(ntIndexerVoltage.GetDouble(kIndexerVoltageDefault));
  ntIndexerVoltage.SetPersistent();

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
  if (IO.mainController.GetR1Button() || IO.secondaryController.GetTriangleButton())
  {
    vision::RJVisionPipeline::visionData data = IO.rjVision.Run();
    if (data.filled)
    {
      // Start Shooter
      Shooter::State shotStat = IO.shooter.CalculateShot(data.distance);
      IO.shooter.SetShooterRPM(shotStat.shooterRPM);

      // Shoot
      units::degree_t tol{ntVisionAngleTol.GetDouble(kVisionAngleTolDefault)};
      shoot = IO.drivetrain.TurnRel(0.0, data.angle, tol);
    }
  }
  else
  {
    double fwd = -deadband(IO.mainController.GetLeftY());
    double rot = -deadband(IO.mainController.GetRightX());

    // Sniper Mode
    if (IO.mainController.GetR3Button())
    {
      fwd *= 0.3;
      rot *= 0.3;
    }

    IO.drivetrain.Arcade(fwd, rot);
  }

  //
  // *** SHOOTER ***
  //
  auto s = units::revolutions_per_minute_t{ntShooterRPM.GetDouble(kShooterRPMDefault)};

  switch (IO.secondaryController.GetPOV())
  {
  case -1:
    hoodOS = false;
    break;
  case 0:
    // FENDER
    IO.shooter.SetShooterRPM(s);
    m_csmode = ClimberShooterMode::Shooter;
    if (!hoodOS)
    {
      IO.shooter.SetHoodAngle(Shooter::HoodPosition::Bottom);
      hoodOS = true;
    }
    break;

  case 90:
    // MIDFIELD
    IO.shooter.SetShooterRPM(s);
    m_csmode = ClimberShooterMode::Shooter;
    if (!hoodOS)
    {
      IO.shooter.SetHoodAngle(Shooter::HoodPosition::Middle);
      hoodOS = true;
    }
    break;

  case 180:
    // LAUNCHPAD
    IO.shooter.SetShooterRPM(s);
    m_csmode = ClimberShooterMode::Shooter;
    if (!hoodOS)
    {
      IO.shooter.SetHoodAngle(Shooter::HoodPosition::Top);
      hoodOS = true;
    }
    break;

  case 270:
    // TARMAC
    IO.shooter.SetShooterRPM(0_rpm);
    m_csmode = ClimberShooterMode::Shooter;
    if (!hoodOS)
    {
      IO.shooter.SetHoodAngle(Shooter::HoodPosition::Middle);
      hoodOS = true;
    }
    break;
  }

  IO.shooter.SetHoodAngle();

  // Shoot
  if (shoot || IO.secondaryController.GetSquareButton() || IO.mainController.GetSquareButton())
  {
    m_csmode = ClimberShooterMode::Shooter;
    IO.shooter.Shoot();
    shoot = true;
  }

  // Stop Shooter
  if (IO.secondaryController.GetCrossButtonPressed() || IO.mainController.GetCrossButtonPressed())
  {
    IO.shooter.SetShooter(0_V);
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

  // Deploy Intake
  if (units::math::abs(intakeCmd) > 0.0_V)
  {
    IO.shooter.SetIntakeState(Shooter::Position::Deployed);
    intakeTimer.Start();
    intakeTimer.Reset();
  }

  // Retract Intake after a short delay
  if (intakeTimer.Get() > 0.5_s)
  {
    IO.shooter.SetIntakeState(Shooter::Position::Stowed);
  }

  // Indexer is shared between Intake and Shooter
  if (intakeCmd > 0.0_V || shoot)
  {
    IO.shooter.SetIndexer(units::volt_t{ntIndexerVoltage.GetDouble(kIndexerVoltageDefault)});
  }
  else if (intakeCmd < 0.0_V)
  {
    IO.shooter.SetIndexer(units::volt_t{(-1) * ntIndexerVoltage.GetDouble(kIndexerVoltageDefault)});
  }
  else
  {
    IO.shooter.SetIndexer(0_V);
  }

  // Feeder
  if (shoot)
  {
    IO.shooter.SetFeeder(units::volt_t{ntFeederVoltage.GetDouble(kFeederVoltageDefault)});
  }
  else if (IO.shooter.GetShooterRPM() > 10.0_rpm || 
      units::math::abs(intakeCmd) > 0.0_V)
  {
    IO.shooter.SetFeeder(-2_V);
  }
  else
  {
    IO.shooter.SetFeeder(0_V);
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
    IO.climber.SetClimber(units::volt_t{deadband(IO.secondaryController.GetLeftY()) * 13.0});
  }
  else
  {
    IO.climber.SetClimber(0.0_V);
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
    // return sgn * (std::abs(val) - min) / (max - min) * max;
    return sgn * (std::abs(val) - min) / (max - min) * max;
  }
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
