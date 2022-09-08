// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"


#include "frc/Errors.h"                             // for RuntimeError
#include "frc/PowerDistribution.h"                  // for PowerDistribution
#include "frc/RobotBase.h"                          // for StartRobot
#include "frc/smartdashboard/SmartDashboard.h"      // for SmartDashboard
#include "frc/util/Color.h"                         // for Color
#include "frc/Watchdog.h"                           // for Watchdog
#include "networktables/NetworkTableEntry.inc"      // for NetworkTableEntry...
#include "rev/ColorSensorV3.h"                      // for ColorSensorV3
#include "units/angular_velocity.h"                 // for operator""_rpm
#include "units/base.h"                             // for unit_t, operator-
#include "units/math.h"                             // for abs
#include "units/pressure.h"                         // for pounds_per_square...
#include "units/time.h"                             // for operator""_s, sec...
#include "units/velocity.h"                         // for meters_per_second_t
#include "units/voltage.h"                          // for operator""_V, volt_t
#include <cmath>                                    // for abs, fabs
#include <exception>                                // for exception
#include <frc/DriverStation.h>                      // for DriverStation
#include <frc/livewindow/LiveWindow.h>              // for LiveWindow
#include <iostream>
#include <math.h>
#include <photonlib/PhotonUtils.h>
#include <string>                                   // for string
#include <wpi/timestamp.h>


void Robot::RobotInit()
{
  // Disable Live Window Stuff, we don't use it...
  frc::LiveWindow::DisableAllTelemetry();
  frc::LiveWindow::SetEnabled(false);

  // System Setup Stuff
  IO.ConfigureSystem();

  frc::SmartDashboard::PutData("Drivetrain", &IO.drivetrain);
}

void Robot::RobotPeriodic()
{
}

void Robot::AutonomousInit()
{
}

void Robot::AutonomousPeriodic()
{
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

    IO.drivetrain.Drive(forward, strafe, rotate, true);
}

void Robot::DisabledInit()
{
  IO.drivetrain.Stop();
}

void Robot::DisabledPeriodic()
{

}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic()
{
  IO.SimPeriodic();
}

void Robot::TestInit()
{
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
