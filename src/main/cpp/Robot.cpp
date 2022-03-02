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
  // frc::SmartDashboard::PutData("Power", &IO.pdp);
  // // frc::SmartDashboard::PutData("Dr", &IO.mainController);
  // // frc::SmartDashboard::PutData("Op", &IO.secondaryController);
  // frc::SmartDashboard::PutData("Drive", &IO.drivetrain);
  // frc::SmartDashboard::PutData("Shooter", &IO.shooter);
  // frc::SmartDashboard::PutData("Climber", &IO.climber);
  // frc::SmartDashboard::PutData("Ph", &IO.ph);
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
  ntTurretTargetAng.ForceSetDouble(ntTurretTargetAng.GetDouble(kTurretTargetAngDefault));
  ntTurretTargetAng.SetPersistent();

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
  IO.rjVision.Periodic();
}

void Robot::AutonomousInit()
{
  autoprograms.Init();
  IO.drivetrain.SetBrakeMode();
  IO.rjVision.SetLED(false);
}

void Robot::AutonomousPeriodic()
{
  autoprograms.Run();
}

void Robot::TeleopInit()
{
  IO.drivetrain.SetBrakeMode();
  IO.rjVision.SetLED(false);
}

void Robot::TeleopPeriodic()
{
  bool shoot = false;

  //
  // *** VISION AND DRIVING ***
  //
  if (IO.mainController.GetR1Button()) // || IO.secondaryController.GetTriangleButton())
  {
    IO.shooter.SetIntakeState(Shooter::Position::Deployed);
    intakeTimer.Start();
    intakeTimer.Reset();

    vision::RJVisionPipeline::visionData data = IO.rjVision.Run();
    if (data.filled)
    {
      // Adjust for Movement
      VectorMath shotVector = VectorMath{data.distance, (IO.shooter.GetTurretAngle() - data.angle)}; // Plane Relative to Robot Direction, where back of the robot is 0 deg
      // *** NEED ROBOT VELOCITY DATA ***
      VectorMath robotMoveVector = VectorMath{0.0_in, 180.0_deg}; // Assuming about a 1 sec shot time. Maybe add a graph?
      VectorMath adjustedShotVector = shotVector - robotMoveVector;

      // Set Turret
      units::degree_t tol{ntVisionAngleTol.GetDouble(kVisionAngleTolDefault)};

      //  bool turretAtAngle = IO.shooter.SetTurretAngle(adjustedShotVector.GetTheta(), tol);
      bool turretAtAngle = IO.shooter.SetTurretAngle((IO.shooter.GetTurretAngle() + data.angle), tol);

      // Start Shooter
      Shooter::State shotStat = IO.shooter.CalculateShot(adjustedShotVector.GetMagnitude()); // Magnitude from adjusted vector gets us distance
      // IO.shooter.SetShooterRPM(shotStat.shooterRPM);
      IO.shooter.SetShooterRPM(2750_rpm);

      // Set Hood

      if (!hoodOS2)
      {
        // IO.shooter.SetHoodAngle(shotStat.hoodAngle);
        IO.shooter.SetHoodAngle(Shooter::HoodPosition::Middle);
        hoodOS2 = true;
      }

      // Shoot Maybe
      shoot = turretAtAngle;

      double fwd = -0.5 * deadband(IO.mainController.GetLeftY());

      IO.drivetrain.Arcade(fwd, 0.0);
    }
  }
  else
  {
    IO.rjVision.SetLED(false);

    VectorMath hubLocation = VectorMath{324.5_in, 159.5_in};
    VectorMath robotLocation = VectorMath{IO.drivetrain.GetPose().Translation()};
    units::degree_t robotAngle = IO.drivetrain.GetPose().Rotation().Degrees();

    units::degree_t turretAngle = -((robotAngle - 180.0_deg) - (hubLocation - robotLocation).GetTheta()); // Get the Estimated Turret Angle

    if (turretAngle > 180_deg)
    {
      turretAngle -= 360_deg;
    }

    if (turretAngle < -180_deg)
    {
      turretAngle += 360_deg;
    }

    if (turretAngle > units::degree_t{Shooter::kTurretMax})
    {
      turretAngle = units::degree_t{Shooter::kTurretMax};
    }
    else if (turretAngle < units::degree_t{Shooter::kTurretMin})
    {
      turretAngle = units::degree_t{Shooter::kTurretMin};
    }

    //IO.shooter.SetTurret(units::volt_t{-13.0 * deadband(IO.mainController.GetRightX())});

    // IO.shooter.SetTurretAngle(turretAngle, 0.25_deg);

    // IO.shooter.SetTurretAngle(units::degree_t{ntTurretTargetAng.GetDouble(kTurretTargetAngDefault)}, units::degree_t{0.5});

    hoodOS2 = false;

    double fwd = -deadband(IO.mainController.GetLeftY());
    //double rot = 0.0;
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
    IO.shooter.SetShooterRPM(2700_rpm);
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

  auto intakeCmd = ((opR2 - opL2 + drR2 - drL2) + (double)IO.mainController.GetR1Button()) * 13.0_V;
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
    IO.climber.SetClimber(units::volt_t{-deadband(IO.secondaryController.GetLeftY()) * 13.0});
  }
  else
  {
    IO.climber.SetClimber(0.0_V);
  }

  if (IO.secondaryController.GetShareButtonPressed())
    IO.climber.SetSensorOverride(!IO.climber.GetSensorOverride());
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
    IO.rjVision.SetLED(false);
  }
}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic()
{
  IO.drivetrain.SimulationPeriodic();
}

void Robot::TestInit()
{
  IO.pdp.ClearStickyFaults();
}
void Robot::TestPeriodic()
{
  IO.rjVision.SetLED(true);
}

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
