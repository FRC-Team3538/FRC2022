// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/livewindow/LiveWindow.h>
#include <cmath>                                    // for abs, fabs
#include <exception>                                // for exception
#include <string>                                   // for string
#include "frc/Errors.h"                             // for RuntimeError
#include "lib/PS4Controller.h"                    // for PS4Controller
#include "lib/PneumaticHub.h"                     // for PneumaticHub
#include "lib/pathplanner/PathPlannerTrajectory.h"  // for pathplanner
#include "subsystems/Shooter.h"                   // for Shooter, Shooter:...
#include "units/base.h"                             // for unit_t, operator-
#include "units/math.h"                             // for abs
#include "units/time.h"
#include <iostream>
#include <math.h>

using namespace pathplanner;

void Robot::RobotInit()
{
  // Disable Live Window Stuff, we don't use it...
  frc::LiveWindow::DisableAllTelemetry();
  frc::LiveWindow::SetEnabled(false);

  // System Setup Stuff
  IO.ConfigureSystem();

  ntRobotName.ForceSetString(ntRobotName.GetString("UnnamedRobot"));
  ntRobotName.SetPersistent();
  frc::SmartDashboard::PutData("Drivetrain", &IO.drivetrain);

  // Logging Stuff
#ifdef LOGGER
  frc::DataLogManager::LogNetworkTables(true);
  IO.RegisterDataEntries(log);
  // arg bool - log joystick data if true
  frc::DriverStation::StartDataLog(log, true);
#endif // LOGGER

  seedEncoderTimer.Start();
}

void Robot::RobotPeriodic()
{
  IO.drivetrain.UpdateOdometry();

  IO.UpdateSmartDash();
  autoprograms.SmartDash();

  // Logging Stuff
#ifdef LOGGER
  IO.LogDataEntries(log);
#endif // LOGGER

  if (!IO.drivetrain.Active() && seedEncoderTimer.Get() > 5_s) {
      frc::SmartDashboard::PutNumber("Drivetrain/SeedEncoderLastResult", IO.drivetrain.SeedEncoders());
      seedEncoderTimer.Reset();
  }
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

}

void Robot::TeleopPeriodic()
{
    // DRIVE CODE
    auto forward = -deadband(m_driver.GetLeftY(), 0.1, 1.0) * Drivetrain::kMaxSpeedLinear;
    auto strafe = -deadband(m_driver.GetLeftX(), 0.1, 1.0) * Drivetrain::kMaxSpeedLinear;
    auto rotate = -deadband(m_driver.GetRightX(), 0.1, 1.0) * Drivetrain::kMaxSpeedAngular * 0.75;

    //std::cout << forward << ", " << strafe << ", " << rotate << std::endl;

    IO.drivetrain.Drive(forward, strafe, rotate, false, false);
}

void Robot::DisabledInit()
{
  IO.drivetrain.Stop();
  
  brakeTimer.Reset();
  brakeTimer.Start();
}

void Robot::DisabledPeriodic()
{

}

void Robot::SimulationInit() {
  IO.SimInit();
}

void Robot::SimulationPeriodic()
{
  IO.SimPeriodic();
}

void Robot::TestInit()
{
  IO.pdp.ClearStickyFaults();

  //IO.drivetrain.ResetOdometry(frc::Pose2d{frc::Translation2d{4.1148_m, 4.1148_m}, frc::Rotation2d{0_deg}});

  m_testTimer.Reset();
  m_testTimer.Start();
}

void Robot::TestPeriodic()
{

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
