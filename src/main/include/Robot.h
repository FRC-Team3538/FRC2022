// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


#pragma once

#include <frc/TimedRobot.h>

#include "Robotmap.hpp"
#include "auto/AutoPrograms.hpp"

#include <frc/smartdashboard/SmartDashboard.h>

#include "lib/Logging.h"
#include "lib/VectorMath.hpp"

#include <frc/DataLogManager.h>

#include <rev/ColorSensorV3.h>
#include <rev/CIEColor.h>
#include <rev/ColorMatch.h>

#include <frc/SerialPort.h>

class Robot : public frc::TimedRobot
{
public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void SimulationInit() override;
  void SimulationPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

private:

  double deadband(double val, double min = 0.10, double max = 1.0);

  Robotmap IO;
  AutoPrograms autoprograms{IO};

#ifdef LOGGER
  wpi::log::DataLog &log = frc::DataLogManager::GetLog();
#endif

  frc::Timer shotTimer;
  frc::Timer brakeTimer;
  frc::Timer intakeTimer;
  frc::Timer climberTimer; // Rhyme points
  bool climberTimerOS = false;

  bool hoodOS = false;
  bool hoodOS2 = false;
  bool manualJog = false;
  
  enum class ClimberShooterMode:uint8_t
  {
    Shooter = 0,
    Climber
  } m_csmode;

  // Smartdash
  nt::NetworkTableEntry ntRobotName = frc::SmartDashboard::GetEntry("robot/RobotName");

  static constexpr double kVisionAngleTolDefault = 0.5;
  nt::NetworkTableEntry ntVisionAngleTol = frc::SmartDashboard::GetEntry("robot/visionAngleTol");
  
  static constexpr double kShooterRPMDefault = 3500;
  nt::NetworkTableEntry ntShooterRPM = frc::SmartDashboard::GetEntry("robot/shooterRPM");

  static constexpr double kFeederVoltageDefault = 2.0;
  nt::NetworkTableEntry ntFeederVoltage = frc::SmartDashboard::GetEntry("robot/feederVoltage");

  static constexpr double kIndexerVoltageDefault = 3.0;
  nt::NetworkTableEntry ntIndexerVoltage = frc::SmartDashboard::GetEntry("robot/indexerVoltage");

  static constexpr double kTurretTargetAngDefault = 0.0;
  nt::NetworkTableEntry ntTurretTargetAng = frc::SmartDashboard::GetEntry("robot/turretTargetAng");

  //----------- rev color sensor stuff ----------
  rev::ColorSensorV3 colorSensor{frc::I2C::Port::kMXP};

  frc::SerialPort led1{9600, frc::SerialPort::Port::kUSB};
  frc::SerialPort led2{9600, frc::SerialPort::Port::kUSB1};
  frc::SerialPort led3{9600, frc::SerialPort::Port::kUSB2};
  frc::SerialPort led4{9600, frc::SerialPort::Port::kOnboard};
  frc::SerialPort led5{9600, frc::SerialPort::Port::kMXP};

};
