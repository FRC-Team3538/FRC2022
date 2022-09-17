// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


#pragma once

#include <frc/TimedRobot.h>                     // for TimedRobot
#include <frc/smartdashboard/SmartDashboard.h>  // for SmartDashboard
#include <rev/ColorSensorV3.h>                  // for ColorSensorV3
#include <stdint.h>                             // for uint8_t
#include "Robotmap.h"                         // for Robotmap
#include "auto/AutoPrograms.h"                // for AutoPrograms
#include "frc/I2C.h"                            // for I2C, I2C::Port, I2C::...
#include "frc/Timer.h"                          // for Timer
#include "frc/geometry/Pose2d.h"                // for Pose2d
#include "frc/geometry/Rotation2d.h"            // for Rotation2d
#include "networktables/NetworkTableEntry.h"    // for NetworkTableEntry
#include "units/angle.h"                        // for operator""_deg
#include "units/length.h"                       // for operator""_in, operat...
#include "lib/Logging.h"
#include "frc/DataLogManager.h"

// Keep this around, these are the units lib base units, in order in units::base_unit:
// METERS
// KILOGRAMS
// SECONDS
// RADIANS
// AMPERES
// KELVIN
// MOLE
// CANDELA
// BYTE

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

  frc::Timer seedEncoderTimer;
  vision::RJVisionPipeline::visionData prevData;

  nt::NetworkTableEntry entry = frc::SmartDashboard::GetEntry("Recalibrate");



  bool manualJog = false;

  // Smartdash

  static constexpr double kVisionAngleTolDefault = 0.5;
  nt::NetworkTableEntry ntVisionAngleTol = frc::SmartDashboard::GetEntry("robot/visionAngleTol");

  static constexpr double kShooterRPMDefault = 2950;
  nt::NetworkTableEntry ntShooterRPM = frc::SmartDashboard::GetEntry("robot/shooterRPM");

  static constexpr double kShooterRatioDefault = 1.3;
  nt::NetworkTableEntry ntShooterRatio = frc::SmartDashboard::GetEntry("robot/ShooterRatio");

  static constexpr double kFeederVoltageDefault = 6.0;
  nt::NetworkTableEntry ntFeederVoltage = frc::SmartDashboard::GetEntry("robot/feederVoltage");

  static constexpr double kIndexerVoltageDefault = 6.0;
  nt::NetworkTableEntry ntIndexerVoltage = frc::SmartDashboard::GetEntry("robot/indexerVoltage");

  static constexpr double kTurretTargetAngDefault = 0.0;
  nt::NetworkTableEntry ntTurretTargetAng = frc::SmartDashboard::GetEntry("robot/turretTargetAng");

  // Presets
  static constexpr double kPresetLeftDefault = 4500.0;
  nt::NetworkTableEntry ntPresetLeft = frc::SmartDashboard::GetEntry("robot/Preset/Wall");
  static constexpr double kPresetRightDefault = 2950.0;
  nt::NetworkTableEntry ntPresetRight = frc::SmartDashboard::GetEntry("robot/Preset/Tarmac");
  static constexpr double kPresetDownDefault = 1000.0;
  nt::NetworkTableEntry ntPresetDown = frc::SmartDashboard::GetEntry("robot/Preset/Fender");
  static constexpr double kPresetUpDefault = 3500.0;
  nt::NetworkTableEntry ntPresetUp = frc::SmartDashboard::GetEntry("robot/Preset/Launchpad");

  //----------- rev color sensor stuff ----------
  //rev::ColorSensorV3 colorSensor{frc::I2C::Port::kMXP};

  frc::Timer autoEjectTimer;

  static constexpr auto camera_height = 40.63_in;
  static constexpr auto target_elevation = 103.25_in;
  static constexpr auto camera_pitch = 33_deg;

  static constexpr auto camera_to_center_turret_distance = 2.89_in;
  static constexpr auto turret_to_center_robot_distance = 2_in;

  frc::Pose2d center_hub = frc::Pose2d{8.2296_m, 4.1148_m, 0_deg};
  static constexpr auto hub_upper_radius = 2_ft;

  // nt::NetworkTableEntry localization_flag_entry = frc::SmartDashboard::GetEntry("flags/alternate_localization");

  frc::Timer m_testTimer;
  // bool m_enablePassiveTurret = true;
};
