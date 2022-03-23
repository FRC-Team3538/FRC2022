// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <lib/pathplanner/PathPlanner.h>
#include <frc/livewindow/LiveWindow.h>
#include <cmath>

#include <frc/DriverStation.h>

#include <wpi/timestamp.h>

#include <photonlib/PhotonUtils.h>

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
  frc::SmartDashboard::PutData("Drive", &IO.drivetrain);
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
  frc::DataLogManager::LogNetworkTables(true);
  // arg bool - log joystick data if true
  frc::DriverStation::StartDataLog(log, true);
#endif // LOGGER

  if (IO.shooter.GetTurretSwitch())
  {
    IO.shooter.ZeroTurret();
  }

  // led1.SetWriteBufferMode(frc::SerialPort::WriteBufferMode::kFlushOnAccess);
  // led2.SetWriteBufferMode(frc::SerialPort::WriteBufferMode::kFlushOnAccess);
  // led3.SetWriteBufferMode(frc::SerialPort::WriteBufferMode::kFlushOnAccess);
  // led4.SetWriteBufferMode(frc::SerialPort::WriteBufferMode::kFlushOnAccess);
  // led5.SetWriteBufferMode(frc::SerialPort::WriteBufferMode::kFlushOnAccess);

  // led1.SetWriteBufferSize(64);
  // led2.SetWriteBufferSize(64);
  // led3.SetWriteBufferSize(64);
  // led4.SetWriteBufferSize(64);
  // led5.SetWriteBufferSize(64);

  // led1.EnableTermination();
  // led2.EnableTermination();
  // led3.EnableTermination();
  // led4.EnableTermination();
  // led5.EnableTermination();

  localization_flag_entry.SetDefaultBoolean(false);
}

void Robot::RobotPeriodic()
{
  IO.UpdateSmartDash();
  IO.drivetrain.Periodic();
  autoprograms.SmartDash();
  IO.rjVision.Periodic();
  frc::SmartDashboard::PutNumber("robot/MatchTime", frc::DriverStation::GetMatchTime());
  frc::SmartDashboard::PutNumber("robot/PressureHigh", IO.ph.GetPressure(0).value());
  if (!IO.shooter.zeroed)
    IO.shooter.SetBlinkyZeroThing();\

  // touchpad to toggle between old odometry & limelight and new pose estimation / photon vision / turret fun
  if (IO.secondaryController.IsConnected() && IO.secondaryController.GetTouchpadPressed())
  {
    auto new_flag = !localization_flag_entry.GetBoolean(false);
    localization_flag_entry.SetBoolean(new_flag);

    IO.rjVision.SetLED(new_flag);
  }

  if (localization_flag_entry.GetBoolean(false))
  {
    
    auto result = IO.rjVision.RunPhotonVision();
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

      auto turret_forward_to_robot_forward = frc::Transform2d{frc::Translation2d{}, frc::Rotation2d{180_deg - turret_heading}};
      auto camera_to_robot = camera_to_turret + turret_forward_to_robot_forward + turret_to_robot;

      auto robot_pose = IO.drivetrain.GetPose();
      auto turret_pose = robot_pose + camera_to_robot.Inverse();

      auto hub_to_turret = units::math::atan2(turret_pose.Y() - center_hub.Y(),turret_pose.X() - center_hub.X());


      // this gets the point on the rim of the hub closest to the robot.
      // this is the point that we're targeting in our pipeline
      auto hub_edge_facing_turret = center_hub 
        // rotate toward us
        + frc::Transform2d{frc::Translation2d{}, frc::Rotation2d{hub_to_turret}} 
        // move 2 ft to hub radius
        + frc::Transform2d{frc::Translation2d{hub_upper_radius, 0_ft}, frc::Rotation2d{}};


      auto distance = photonlib::PhotonUtils::CalculateDistanceToTarget(camera_height, target_elevation, camera_pitch, target_pitch);
      auto camera_to_target_translation = photonlib::PhotonUtils::EstimateCameraToTargetTranslation(distance, target_yaw);
      // I don't know exactly why turret_facing needs to be negative but it does.
      auto camera_to_target_transform = photonlib::PhotonUtils::EstimateCameraToTarget(camera_to_target_translation, hub_edge_facing_turret, frc::Rotation2d{-turret_facing});
      // debug use
      // auto estimated_camera_pose = photonlib::PhotonUtils::EstimateFieldToCamera(camera_to_target_transform, hub_edge_facing_turret);
      auto estimated_robot_pose = photonlib::PhotonUtils::EstimateFieldToRobot(camera_to_target_transform, hub_edge_facing_turret, camera_to_robot);

      IO.drivetrain.UpdateOdometryWithGlobalEstimate(estimated_robot_pose, result.read_time - result.base_result.GetLatency());

      // now we set our turret angle to the angle needed to face the center of the hub
      auto turret_position = IO.drivetrain.GetPose()
        // from the turret's frame of reference, facing foward on the robot
        + turret_to_robot.Inverse() 
        // rotating to face backward; ie center of turret
        + frc::Transform2d{frc::Translation2d{}, frc::Rotation2d{180_deg}};
      auto global_angle_to_hub = units::math::atan2(center_hub.Y() - turret_position.Y(), center_hub.X() - turret_position.X());
      auto turret_angle_to_hub = global_angle_to_hub - turret_position.Rotation().Radians();
      IO.shooter.SetTurretAngle(turret_angle_to_hub, 0.75_deg);

      // distance because we can
      Shooter::State shooter_state = IO.shooter.CalculateShot(distance);
      IO.shooter.SetShooterRPM(shooter_state.shooterRPM);
    }
  }
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
  if (IO.mainController.GetR1Button() || IO.secondaryController.GetCircleButton()) // || IO.secondaryController.GetTriangleButton())
  {
    if (IO.shooter.GetShooterRPM() < 250.0_rpm)
    {
      IO.shooter.SetShooterRPM(2800_rpm);
    }

    climberTimerOS = false;
    manualJog = false;
    IO.shooter.SetIntakeState(Shooter::Position::Deployed);
    intakeTimer.Start();
    intakeTimer.Reset();

    if (!hoodOS2)
    {
      IO.shooter.SetHoodAngle(Shooter::HoodPosition::Middle);
      hoodOS2 = true;
    }

    vision::RJVisionPipeline::visionData data = IO.rjVision.Run();
    if (data.filled)
    {
      units::meters_per_second_t driveVel = IO.drivetrain.GetVelocity();

      // vision::RJVisionPipeline::visionData lookAhead = IO.rjVision.LookAhead(data, prevData);
      // if (lookAhead.filled)
      // {
        // Adjust for Movement
        VectorMath shotVector = VectorMath{data.distance, data.turretAngle}; // Plane Relative to Robot Direction, where back of the robot is 0 deg
        VectorMath robotMoveVector = VectorMath{(driveVel * 1_s), 180.0_deg};          // Assuming about a 1 sec shot time. Maybe add a graph?
        VectorMath adjustedShotVector = shotVector - robotMoveVector;                  // - robotMoveVector because you want to shoot opposite of movement

        // Calculate Turret
        //std::cout << adjustedShotVector.GetTheta().value() << std::endl;
        bool turretAtAngle = IO.shooter.SetTurretAngle(adjustedShotVector.GetTheta(), 1.0_deg);
        // bool turretAtAngle = IO.shooter.SetTurretAngle(data.turretAngle, 1.0_deg);

        // Calculate Shooter
        Shooter::State shotStat = IO.shooter.CalculateShot(adjustedShotVector.GetMagnitude()); // Magnitude from adjusted vector gets us distance

        IO.shooter.SetShooterRPM(shotStat.shooterRPM);

        // Shoot Maybe
        if(turretAtAngle)
          elShoot = true;
        shoot = elShoot;
      //}
    }
    double fwd = -deadband(IO.mainController.GetLeftY());
    double rot = -deadband(IO.mainController.GetRightX());

    IO.drivetrain.Arcade(fwd, rot);

    prevData = data;
  }
  else if (IO.secondaryController.GetSquareButton())
  {
    climberTimerOS = false;
    manualJog = false;
    vision::RJVisionPipeline::visionData data = IO.rjVision.Run();
    if (data.filled)
    {
      // Set Turret
      units::degree_t tol{ntVisionAngleTol.GetDouble(kVisionAngleTolDefault)};

      IO.shooter.SetTurretAngle(data.turretAngle, 0.75_deg);
    }
    double fwd = -deadband(IO.mainController.GetLeftY());
    double rot = -deadband(IO.mainController.GetRightX());

    IO.drivetrain.Arcade(fwd, rot);
  }
  else
  {

    elShoot = false;

    IO.rjVision.SetLED(false);
    IO.rjVision.Reset();

    // Hub tracking is based off odometry throughout the match, so might need to make it able to zero
    // Or make sure the auto is started where the robot thinks it is

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

    // IO.shooter.SetTurret(units::volt_t{-13.0 * deadband(IO.mainController.GetRightX())});

    // IO.shooter.SetTurretAngle(turretAngle, 0.25_deg);

    if (!IO.shooter.zeroed)
    {
      if (IO.secondaryController.GetPSButton())
      {
        double turn = -1.5 * deadband(IO.secondaryController.GetRightX());
        IO.shooter.SetTurret(units::volt_t{turn});
        bool negative = turn > 0.0;
        if (IO.shooter.GetTurretSwitch())
          IO.shooter.ZeroTurret(negative);
      }
      else
        IO.shooter.SetTurret(0.0_V);
    }
    else if (IO.secondaryController.GetPSButton())
    {
      climberTimerOS = false;
      IO.shooter.SetTurretAngle(0.0_deg, 1_deg);
      m_csmode = ClimberShooterMode::Shooter;
      manualJog = false;
    }
    else if (std::abs(deadband(IO.secondaryController.GetRightX())) > 0.0 || manualJog)
    {
      climberTimerOS = false;
      manualJog = true;
      IO.shooter.SetTurret(units::volt_t{-3.0 * deadband(IO.secondaryController.GetRightX())});
    }
    else if (m_csmode == ClimberShooterMode::Climber)
    {
      IO.shooter.SetTurretAngle(90.0_deg, 1_deg);
    }
    else
    {
      IO.shooter.SetTurretAngle(0.0_deg, 1_deg);
      climberTimerOS = false;
    }

    // IO.shooter.SetTurretAngle(units::degree_t{ntTurretTargetAng.GetDouble(kTurretTargetAngDefault)}, units::degree_t{0.5});

    hoodOS2 = false;

    double fwd = -deadband(IO.mainController.GetLeftY());
    // double rot = 0.0;
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
    IO.shooter.SetShooterRPM(3500_rpm);
    m_csmode = ClimberShooterMode::Shooter;
    if (!hoodOS)
    {
      IO.shooter.SetHoodAngle(Shooter::HoodPosition::Bottom);
      hoodOS = true;
    }
    break;

  case 90:
    // MIDFIELD
    IO.shooter.SetShooterRPM(2750_rpm);
    m_csmode = ClimberShooterMode::Shooter;
    if (!hoodOS)
    {
      IO.shooter.SetHoodAngle(Shooter::HoodPosition::Middle);
      hoodOS = true;
    }
    break;

  case 180:
    // LAUNCHPAD
    IO.shooter.SetShooterRPM(1000_rpm);
    m_csmode = ClimberShooterMode::Shooter;
    if (!hoodOS)
    {
      IO.shooter.SetHoodAngle(Shooter::HoodPosition::Middle);
      hoodOS = true;
    }
    break;

  case 270:
    // TARMAC
    IO.shooter.SetShooterRPM(2950_rpm);
    m_csmode = ClimberShooterMode::Shooter;
    if (!hoodOS)
    {
      IO.shooter.SetHoodAngle(Shooter::HoodPosition::Middle);
      hoodOS = true;
    }
    break;
  }

  if (IO.secondaryController.GetOptionsButtonPressed())
  {
    IO.shooter.SetShooterRPM(s);
    m_csmode = ClimberShooterMode::Shooter;
  }

  IO.shooter.SetHoodAngle();

  // Shoot
  if (shoot || IO.secondaryController.GetTriangleButton() || IO.mainController.GetL1Button())
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

  //
  // Automatic Ball Ejection
  //

  // if intaking balls and color sensor is connected
  if (units::math::abs(intakeCmd) > 0.0_V && colorSensor.IsConnected())
  {

    // If a ball is present
    frc::Color ballColor = colorSensor.GetColor();
    if(fabs(ballColor.blue - ballColor.red) > 0.2)
    {
      // Check for ball color missmatch
      auto blueAlliance = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue;
      auto redAlliance = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed;
      auto blueBall = ballColor.blue > ballColor.red;
      auto redBall = !blueBall;
      if((blueAlliance && redBall) || (redAlliance && blueBall))
      {
        autoEjectTimer.Start();
      }

      if((blueAlliance && blueBall) || (redAlliance && redBall))
      {
        autoEjectTimer.Stop();
        autoEjectTimer.Reset();
        IO.shooter.SetShooter(0_V);
      }
    }

    if(autoEjectTimer.Get() > 0.25_s)
    {
      IO.shooter.SetShooterRPM(800_rpm);
      if(IO.shooter.SetTurretAngle(0.0_deg, 10_deg))
      {
        m_csmode = ClimberShooterMode::Shooter;
        IO.shooter.Shoot();
        shoot = true;
      }
    }
  }

  // Attempt to eject this ball for x seconds
  if(autoEjectTimer.Get() > 2.0_s)
  {
    autoEjectTimer.Stop();
    autoEjectTimer.Reset();
    IO.shooter.SetShooter(0_V);
  }

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

  //
  // *** Climber ***
  //
  if (IO.secondaryController.GetR1Button())
  {
    manualJog = false;
    if (!climberTimerOS)
    {
      climberTimer.Reset();
      climberTimer.Start();
      climberTimerOS = true;
    }

    if (climberTimer.Get() > 0.5_s)
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

void Robot::SimulationInit() {
  // Add sim targets, 8 placed around the hub?
  IO.rjVision.AddSimulationTarget(
    center_hub + frc::Transform2d{frc::Translation2d{}, frc::Rotation2d{0_deg}} 
      + frc::Transform2d{frc::Translation2d{hub_upper_radius, 0_ft}, frc::Rotation2d{}}, 
    target_elevation, 40_in, 3_in);
  IO.rjVision.AddSimulationTarget(
    center_hub + frc::Transform2d{frc::Translation2d{}, frc::Rotation2d{45_deg}} 
      + frc::Transform2d{frc::Translation2d{hub_upper_radius, 0_ft}, frc::Rotation2d{}}, 
    target_elevation, 40_in, 3_in);
  IO.rjVision.AddSimulationTarget(
    center_hub + frc::Transform2d{frc::Translation2d{}, frc::Rotation2d{90_deg}} 
      + frc::Transform2d{frc::Translation2d{hub_upper_radius, 0_ft}, frc::Rotation2d{}}, 
    target_elevation, 40_in, 3_in);
  IO.rjVision.AddSimulationTarget(
    center_hub + frc::Transform2d{frc::Translation2d{}, frc::Rotation2d{135_deg}} 
      + frc::Transform2d{frc::Translation2d{hub_upper_radius, 0_ft}, frc::Rotation2d{}}, 
    target_elevation, 40_in, 3_in);
  IO.rjVision.AddSimulationTarget(
    center_hub + frc::Transform2d{frc::Translation2d{}, frc::Rotation2d{180_deg}} 
      + frc::Transform2d{frc::Translation2d{hub_upper_radius, 0_ft}, frc::Rotation2d{}}, 
    target_elevation, 40_in, 3_in);
  IO.rjVision.AddSimulationTarget(
    center_hub + frc::Transform2d{frc::Translation2d{}, frc::Rotation2d{225_deg}} 
      + frc::Transform2d{frc::Translation2d{hub_upper_radius, 0_ft}, frc::Rotation2d{}}, 
    target_elevation, 40_in, 3_in);
  IO.rjVision.AddSimulationTarget(
    center_hub + frc::Transform2d{frc::Translation2d{}, frc::Rotation2d{270_deg}} 
      + frc::Transform2d{frc::Translation2d{hub_upper_radius, 0_ft}, frc::Rotation2d{}}, 
    target_elevation, 40_in, 3_in);
  IO.rjVision.AddSimulationTarget(
    center_hub + frc::Transform2d{frc::Translation2d{}, frc::Rotation2d{315_deg}} 
      + frc::Transform2d{frc::Translation2d{hub_upper_radius, 0_ft}, frc::Rotation2d{}}, 
    target_elevation, 40_in, 3_in);
}

void Robot::SimulationPeriodic()
{
  IO.drivetrain.SimulationPeriodic();

  auto turret_heading = IO.shooter.GetTurretAngle();

  // negative because we're going backward
  auto camera_to_turret = frc::Transform2d{frc::Translation2d{-camera_to_center_turret_distance, 0_in}, frc::Rotation2d{}};
  auto turret_to_facing_robot_north = frc::Transform2d{frc::Translation2d{}, frc::Rotation2d{180_deg - turret_heading}};

  auto camera_to_robot = camera_to_turret + turret_to_facing_robot_north + turret_to_robot;
  IO.rjVision.Simulate(camera_to_robot.Inverse(), IO.drivetrain.GetPose());
}

void Robot::TestInit()
{
  IO.pdp.ClearStickyFaults();
}
void Robot::TestPeriodic()
{
  IO.shooter.SetTurret(0_V);
  IO.rjVision.SetLED(true);

  if (IO.mainController.GetPSButton())
  {
    IO.rjVision.TakeSnapshot(1);
  }
  else
    IO.rjVision.TakeSnapshot(0);

  double fwd = -0.5 * deadband(IO.mainController.GetLeftY());
  double rot = -0.5 * deadband(IO.mainController.GetRightX());

  IO.drivetrain.Arcade(fwd, rot);
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
