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
#include <frc/DriverStation.h>
#include <photonlib/PhotonUtils.h>

using namespace pathplanner;

void Robot::RobotInit()
{
  // Disable Live Window Stuff, we don't use it...
  frc::LiveWindow::DisableAllTelemetry();
  frc::LiveWindow::SetEnabled(false);

  // System Setup Stuff
  IO.ConfigureSystem();

  frc::SmartDashboard::PutData("Drivetrain", &IO.drivetrain);

  // Logging Stuff
#ifdef LOGGER
  frc::DataLogManager::LogNetworkTables(true);
  IO.RegisterDataEntries(log);
  // arg bool - log joystick data if true
  frc::DriverStation::StartDataLog(log, true);
#endif // LOGGER

  seedEncoderTimer.Start();

  ntVisionAngleTol.ForceSetDouble(ntVisionAngleTol.GetDouble(kVisionAngleTolDefault));
  ntVisionAngleTol.SetPersistent();

  ntShooterRPM.ForceSetDouble(ntShooterRPM.GetDouble(kShooterRPMDefault));
  ntShooterRPM.SetPersistent();
  ntShooterRatio.ForceSetDouble(ntShooterRatio.GetDouble(kShooterRatioDefault));
  ntShooterRatio.SetPersistent();
  ntFeederVoltage.ForceSetDouble(ntFeederVoltage.GetDouble(kFeederVoltageDefault));
  ntFeederVoltage.SetPersistent();
  ntIndexerVoltage.ForceSetDouble(ntIndexerVoltage.GetDouble(kIndexerVoltageDefault));
  ntIndexerVoltage.SetPersistent();
  ntTurretTargetAng.ForceSetDouble(ntTurretTargetAng.GetDouble(kTurretTargetAngDefault));
  ntTurretTargetAng.SetPersistent();
  
  // Presets
  ntPresetLeft.ForceSetDouble(ntPresetLeft.GetDouble(kPresetLeftDefault));
  ntPresetLeft.SetPersistent();
  ntPresetRight.ForceSetDouble(ntPresetRight.GetDouble(kPresetRightDefault));
  ntPresetRight.SetPersistent();
  ntPresetDown.ForceSetDouble(ntPresetDown.GetDouble(kPresetDownDefault));
  ntPresetDown.SetPersistent();
  ntPresetUp.ForceSetDouble(ntPresetUp.GetDouble(kPresetUpDefault));
  ntPresetUp.SetPersistent();

  if (IO.shooter.GetTurretSwitch())
  {
    IO.shooter.ZeroTurret();
  }

  // localization_flag_entry.SetDefaultBoolean(false);
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
      std::cout << "seeding encoders" << std::endl;
      frc::SmartDashboard::PutNumber("Drivetrain/SeedEncoderLastResult", IO.drivetrain.SeedEncoders());
      seedEncoderTimer.Reset();
  }

  IO.vision.SetTurretAngle(IO.shooter.GetTurretAngle());
  IO.vision.Periodic();
  
  frc::SmartDashboard::PutNumber("robot/MatchTime", frc::DriverStation::GetMatchTime());
  frc::SmartDashboard::PutNumber("robot/PressureHigh", IO.ph.GetPressure(0).value());
  frc::SmartDashboard::PutNumber("robot/ShooterRPM", IO.shooter.GetShooterRPM().value());
  if (!IO.shooter.zeroed)
    IO.shooter.SetBlinkyZeroThing();

  

  // touchpad to toggle between old odometry & limelight and new pose estimation / photon vision / turret fun
  // if (IO.secondaryController.IsConnected() && IO.secondaryController.GetTouchpadPressed())
  // {
  //   auto new_flag = !localization_flag_entry.GetBoolean(false);
  //   localization_flag_entry.SetBoolean(new_flag);
  // }

  IO.shooter.SetShooterRatio(ntShooterRatio.GetDouble(kShooterRatioDefault));
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
  IO.shooter.SetTurretBrakeMode();
  IO.vision.SetLED(false);
}

void Robot::TeleopPeriodic()
{
    // DRIVE CODE
    auto forward = -deadband(m_driver.GetLeftY(), 0.1, 1.0) * Drivetrain::kMaxSpeedLinear;
    auto strafe = -deadband(m_driver.GetLeftX(), 0.1, 1.0) * Drivetrain::kMaxSpeedLinear;
    auto rotate = -deadband(m_driver.GetRightX(), 0.1, 1.0) * Drivetrain::kMaxSpeedAngular;

    //std::cout << forward << ", " << strafe << ", " << rotate << std::endl;

    IO.drivetrain.Drive(forward, strafe, rotate, true, false);

    bool shoot = false;
  // Shooter Presets
  auto s = units::revolutions_per_minute_t{ntShooterRPM.GetDouble(kShooterRPMDefault)};
  auto rpmUp = units::revolutions_per_minute_t{ntPresetUp.GetDouble(kPresetUpDefault)};
  auto rpmRight = units::revolutions_per_minute_t{ntPresetRight.GetDouble(kPresetRightDefault)};
  auto rpmDown = units::revolutions_per_minute_t{ntPresetDown.GetDouble(kPresetDownDefault)};
  auto rpmLeft = units::revolutions_per_minute_t{ntPresetLeft.GetDouble(kPresetLeftDefault)};
  
  auto turretOK = false;    //turret flag used for autoaim
  auto flywheelOK = false;  //flywheel flag used for autoaim
  // *** VISION AND DRIVING ***
  //
  if (IO.mainController.GetR1Button() || IO.secondaryController.GetCircleButton()) // || IO.secondaryController.GetTriangleButton())
  {
    // *** Automatic Aim and Shoot ***
  
    // If you forget to turn on the shooter, pick the default preset and shoot.
    if (IO.shooter.GetShooterRPM() < 250.0_rpm)
    {
      IO.shooter.SetShooterRPM(2975_rpm); // Tarmac
    }

    manualJog = false;

    vision::RJVisionPipeline::visionData data = IO.vision.Run();
    prevData = data;

    if (data.filled)
    {
      // Calculate Turret
     turretOK = IO.shooter.SetTurretAngle(data.turretAngle, 1.0_deg);
     flywheelOK = IO.shooter.AtRPM(50_rpm);
     shoot = turretOK && flywheelOK;
    }

    vision::RJVisionPipeline::photonVisionResult result = IO.vision.RunPhotonVision();
    if (result.base_result.HasTargets())
    {
      auto target = result.base_result.GetBestTarget();
      /* ---------- vision data ----------- */
      auto target_pitch = units::degree_t{target.GetPitch()};
      // negated because photonvision + limelight are CW-positive while world is CCW-positive
      auto target_yaw = -units::degree_t{target.GetYaw()};

      /* ---------- robot state data ------------ */

      // TODO which one?
      // auto robot_heading = IO.drivetrain.GetPose().Rotation().Degrees();
      auto robot_heading = IO.drivetrain.GetYaw().Degrees();

      auto turret_heading = IO.shooter.GetTurretAngle();
      auto turret_facing = robot_heading + turret_heading + 180_deg;

      // this gets the point on the rim of the hub closest to the robot.
      // this is the point that we're targeting in our pipeline
      // negative brings the edge closer to us, not further away
      auto hub_edge_transform = frc::Transform2d{frc::Translation2d{}, frc::Rotation2d{180_deg + turret_facing}} + frc::Transform2d{frc::Translation2d{hub_upper_radius, 0_ft}, frc::Rotation2d{}};
      auto hub_edge_facing_robot = center_hub + hub_edge_transform;

      // negative because we're going backward
      auto camera_to_turret = frc::Transform2d{frc::Translation2d{-camera_to_center_turret_distance, 0_in}, frc::Rotation2d{}};
      auto turret_to_facing_robot_north = frc::Transform2d{frc::Translation2d{}, frc::Rotation2d{180_deg - turret_heading}};
      // positive because we're facing forward on the robot
      auto turret_to_robot = frc::Transform2d{frc::Translation2d{turret_to_center_robot_distance, 0_in}, frc::Rotation2d{}};
      auto camera_to_robot = camera_to_turret + turret_to_facing_robot_north + turret_to_robot;

      auto distance = photonlib::PhotonUtils::CalculateDistanceToTarget(camera_height, target_elevation, camera_pitch, target_pitch);
      auto camera_to_target_translation = photonlib::PhotonUtils::EstimateCameraToTargetTranslation(distance, target_yaw);
      // I don't know exactly why turret_facing needs to be negative but it does.
      auto camera_to_target_transform = photonlib::PhotonUtils::EstimateCameraToTarget(camera_to_target_translation, hub_edge_facing_robot, frc::Rotation2d{-turret_facing});

      auto camera_pose = IO.drivetrain.GetPose() + camera_to_robot.Inverse();
      center_hub = camera_pose + camera_to_target_transform + hub_edge_transform.Inverse();

      // m_enablePassiveTurret = true;
    }
   // frc::SmartDashboard::PutBoolean("flags/turretOk", turretOK); //outputting if we locked on turret
   // frc::SmartDashboard::PutBoolean("flags/flywheelOK", flywheelOK); //outputting if we are in margin of 50 rpm of target
  }
  else if (IO.secondaryController.GetSquareButton())
  {
    // *** Automatic Aim but no shoot ***

    manualJog = false;
    vision::RJVisionPipeline::visionData data = IO.vision.Run();
    if (data.filled)
    {
        IO.shooter.SetTurretAngle(data.turretAngle, 1.0_deg);
    }

    vision::RJVisionPipeline::photonVisionResult result = IO.vision.RunPhotonVision();
    if (result.base_result.HasTargets())
    {
      auto target = result.base_result.GetBestTarget();
      /* ---------- vision data ----------- */
      auto target_pitch = units::degree_t{target.GetPitch()};
      // negated because photonvision + limelight are CW-positive while world is CCW-positive
      auto target_yaw = -units::degree_t{target.GetYaw()};

      /* ---------- robot state data ------------ */

      // TODO which one?
      // auto robot_heading = IO.drivetrain.GetPose().Rotation().Degrees();
      auto robot_heading = IO.drivetrain.GetYaw().Degrees();

      auto turret_heading = IO.shooter.GetTurretAngle();
      auto turret_facing = robot_heading + turret_heading + 180_deg;

      // this gets the point on the rim of the hub closest to the robot.
      // this is the point that we're targeting in our pipeline
      // negative brings the edge closer to us, not further away
      auto hub_edge_transform = frc::Transform2d{frc::Translation2d{}, frc::Rotation2d{180_deg + turret_facing}} + frc::Transform2d{frc::Translation2d{hub_upper_radius, 0_ft}, frc::Rotation2d{}};
      auto hub_edge_facing_robot = center_hub + hub_edge_transform;

      // negative because we're going backward
      auto camera_to_turret = frc::Transform2d{frc::Translation2d{-camera_to_center_turret_distance, 0_in}, frc::Rotation2d{}};
      auto turret_to_facing_robot_north = frc::Transform2d{frc::Translation2d{}, frc::Rotation2d{180_deg - turret_heading}};
      // positive because we're facing forward on the robot
      auto turret_to_robot = frc::Transform2d{frc::Translation2d{turret_to_center_robot_distance, 0_in}, frc::Rotation2d{}};
      auto camera_to_robot = camera_to_turret + turret_to_facing_robot_north + turret_to_robot;

      auto distance = photonlib::PhotonUtils::CalculateDistanceToTarget(camera_height, target_elevation, camera_pitch, target_pitch);
      auto camera_to_target_translation = photonlib::PhotonUtils::EstimateCameraToTargetTranslation(distance, target_yaw);
      // I don't know exactly why turret_facing needs to be negative but it does.
      auto camera_to_target_transform = photonlib::PhotonUtils::EstimateCameraToTarget(camera_to_target_translation, hub_edge_facing_robot, frc::Rotation2d{-turret_facing});

      auto camera_pose = IO.drivetrain.GetPose() + camera_to_robot.Inverse();
      center_hub = camera_pose + camera_to_target_transform + hub_edge_transform.Inverse();

      // m_enablePassiveTurret = true;
    }
  }
  else
  {
    IO.vision.SetLED(false);
    IO.vision.Reset();

    if (!IO.shooter.zeroed)
    {
      if (IO.secondaryController.GetPSButton())
      {
        auto turn_cmd = -1.5_V * deadband(IO.secondaryController.GetRightX());
        IO.shooter.SetTurret(turn_cmd);
        if (IO.shooter.GetTurretSwitch())
        {
          IO.shooter.ZeroTurret(turn_cmd > 0.0_V);
        }
      }
      else
      {
        IO.shooter.SetTurret(0.0_V);
      }
    }
    else if (IO.secondaryController.GetPSButton())
    {
      IO.shooter.SetTurretAngle(0.0_deg, 1_deg);
      manualJog = false;
    }
    else if (std::abs(deadband(IO.secondaryController.GetRightX())) > 0.0 || manualJog)
    {
      manualJog = true;
      IO.shooter.SetTurret(units::volt_t{-3.0 * deadband(IO.secondaryController.GetRightX())});
      // m_enablePassiveTurret = false;
    }
    else
    {
      IO.shooter.SetTurretAngle(0.0_deg, 1_deg);
    }

    // IO.shooter.SetTurretAngle(units::degree_t{ntTurretTargetAng.GetDouble(kTurretTargetAngDefault)}, units::degree_t{0.5});
  }

  //
  // *** SHOOTER ***
  //

  switch (IO.secondaryController.GetPOV())
  {
  case -1:
    break;

  case 0:
    // Launchpad
    IO.shooter.SetShooterRPM(rpmUp);
    break;

  case 90:
    // Tarmac 
    IO.shooter.SetShooterRPM(2975_rpm);
    break;

  case 180:
    // Fender
    IO.shooter.SetShooterRPM(rpmDown);
    break;

  case 270:
    // Wall
    IO.shooter.SetShooterRPM(3200_rpm);
    break;
  }

  // Custom Preset
  if (IO.secondaryController.GetOptionsButtonPressed())
  {
    IO.shooter.SetShooterRPM(s);
  }

  // Shoot
  if (shoot || IO.secondaryController.GetTriangleButton() || IO.mainController.GetL1Button())
  {
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

  auto intakeCmd = ((opR2 - opL2 + drR2 - drL2)) * 13.0_V; // + (double)IO.mainController.GetR1Button()) * 13.0_V;
  IO.shooter.SetIntake(intakeCmd);

  // Deploy Intake
  if (units::math::abs(intakeCmd) > 0.0_V)
  {
    IO.shooter.SetIntakeState(Shooter::Position::Deployed);
    intakeTimer.Start();
    intakeTimer.Reset();
  }

  //
  // Automatic Ball Ejection
  //

  // if intaking balls and color sensor is connected
  // if (false)//(units::math::abs(intakeCmd) > 0.0_V && colorSensor.IsConnected())
  // {

  //   // If a ball is present
  //   frc::Color ballColor = colorSensor.GetColor();
  //   if (fabs(ballColor.blue - ballColor.red) > 0.2)
  //   {
  //     // Check for ball color missmatch
  //     auto blueAlliance = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue;
  //     auto redAlliance = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed;
  //     auto blueBall = ballColor.blue > ballColor.red;
  //     auto redBall = !blueBall;
  //     if ((blueAlliance && redBall) || (redAlliance && blueBall))
  //     {
  //       autoEjectTimer.Start();
  //     }

  //     if ((blueAlliance && blueBall) || (redAlliance && redBall))
  //     {
  //       autoEjectTimer.Stop();
  //       autoEjectTimer.Reset();
  //       IO.shooter.SetShooter(0_V);
  //     }
  //   }

  //   if (autoEjectTimer.Get() > 0.25_s)
  //   {
  //     IO.shooter.SetShooterRPM(800_rpm);
  //     if (IO.shooter.SetTurretAngle(0.0_deg, 10_deg))
  //     {
  //       m_csmode = ClimberShooterMode::Shooter;
  //       IO.shooter.Shoot();
  //       shoot = true;
  //     }
  //   }
  // }

  // // Attempt to eject this ball for x seconds
  // if (autoEjectTimer.Get() > 2.0_s)
  // {
  //   autoEjectTimer.Stop();
  //   autoEjectTimer.Reset();
  //   IO.shooter.SetShooter(0_V);
  // }

  // Retract Intake after a short delay
  if (intakeTimer.Get() > 0.25_s)
  {
    IO.shooter.SetIntakeState(Shooter::Position::Stowed);
  }

  // Indexer is shared between Intake and Shooter
  if (intakeCmd > 0.0_V || shoot)
  {
    IO.shooter.SetIndexer(units::volt_t{ntIndexerVoltage.GetDouble(kIndexerVoltageDefault)});
  }
  else if (intakeCmd < -0.8 * 13_V)
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

    frc::SmartDashboard::PutBoolean("flags/turretOk", turretOK); //outputting if we locked on turret
    frc::SmartDashboard::PutBoolean("flags/flywheelOK", flywheelOK); //outputting if we are in margin of 50 rpm of target
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
