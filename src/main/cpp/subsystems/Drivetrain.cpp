#include "subsystems/Drivetrain.h"
#include <frc/smartdashboard/SmartDashboard.h>

Drivetrain::Drivetrain()
{

}

void Drivetrain::UpdateTelemetry()
{

}

void Drivetrain::RegisterDataEntries(wpi::log::DataLog &log)
{

}

void Drivetrain::LogDataEntries(wpi::log::DataLog &log)
{

}

void Drivetrain::ConfigureSystem()
{
  m_imu.ZeroGyroBiasNow();
  ResetYaw();

  m_yawLockPID.EnableContinuousInput(-180, 180);

  // Display Robot position on field
  frc::SmartDashboard::PutData("Field", &m_fieldDisplay);

  m_frontLeft.ConfigureSystem();
  m_frontRight.ConfigureSystem();
  m_backLeft.ConfigureSystem();
  m_backRight.ConfigureSystem();
}

void Drivetrain::Drive(frc::Trajectory::State trajectoryState, units::radian_t yaw)
{
  const auto command = m_trajectoryController.Calculate(
      m_poseEstimator.GetEstimatedPosition(),
      trajectoryState,
      yaw);

  Drive(command.vx, command.vy, command.omega, false);
}

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot,
                       bool fieldRelative,
                       bool openLoop)
{
  // Remember the last operating mode, for smartdash display
  m_fieldRelative = fieldRelative;

  // Heading Lock
  constexpr auto noRotThreshold = 0.1_deg_per_s;
  if (units::math::abs(rot) > noRotThreshold)
  {
    // Disable YawLock as soon as any rotation command is received
    m_YawLockActive = false;
  }
  else if (units::math::abs(rot) < noRotThreshold && units::math::abs(m_robotVelocity.omega) < 10_deg_per_s)
  {
    // Wait for the robot to stop spinning to enable Yaw Lock
    m_YawLockActive = true; //true
  }

  if (m_YawLockActive)
  {
    // Robot will automatically maintain current yaw
    auto r = m_yawLockPID.Calculate(GetYaw().Degrees().value());
    rot = units::degrees_per_second_t{r};
  }
  else
  {
    // Manual control, save the current yaw.
    m_yawLockPID.SetSetpoint(GetYaw().Degrees().value());
    m_yawLockPID.Reset();
  }

  // Transform Field Oriented command to a Robot Relative Command
  if (fieldRelative)
  {
    m_command = frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, GetYaw());
  }
  else
  {
    m_command = frc::ChassisSpeeds{xSpeed, ySpeed, rot};
  }

  // Calculate desired swerve states
  auto states = m_kinematics.ToSwerveModuleStates(m_command);
  m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeedLinear);
  // m_altKinematics.NormalizeWheelSpeeds(&states, m_command);

  m_command = m_kinematics.ToChassisSpeeds(states);

  // Set State of Each Module
  auto [fl, fr, bl, br] = states;
  m_frontLeft.SetModule(fl, openLoop);
  m_frontRight.SetModule(fr, openLoop);
  m_backLeft.SetModule(bl, openLoop);
  m_backRight.SetModule(br, openLoop);
}

void Drivetrain::Stop()
{
  m_frontLeft.Stop();
  m_frontRight.Stop();
  m_backLeft.Stop();
  m_backRight.Stop();
}

frc::Rotation2d Drivetrain::GetYaw()
{
  return frc::Rotation2d{units::degree_t{m_imu.GetYaw()}};
}

void Drivetrain::UpdateOdometry()
{
//   auto odoPose = m_odometry.Update(GetYaw(),
//                     m_frontLeft.GetState(),
//                     m_frontRight.GetState(),
//                     m_backLeft.GetState(),
//                     m_backRight.GetState());

  auto estPose = m_poseEstimator.Update(GetYaw(),
                         m_frontLeft.GetState(),
                         m_frontRight.GetState(),
                         m_backLeft.GetState(),
                         m_backRight.GetState());
  
  m_robotVelocity = m_kinematics.ToChassisSpeeds({m_frontLeft.GetState(),
                                                  m_frontRight.GetState(),
                                                  m_backLeft.GetState(),
                                                  m_backRight.GetState()});

  // auto p = m_odometry.GetPose();
  // frc::Pose2d fliperoo = {-p.Y(), p.X(), p.Rotation().RotateBy(90_deg)}; // Driver Station PoV
  // m_odometryPose->SetPose(odoPose);
  m_estimatedPose->SetPose(estPose);
}

void Drivetrain::ResetYaw()
{
  m_imu.SetYaw(0.0);

  m_yawLockPID.SetSetpoint(GetYaw().Degrees().value());
  m_yawLockPID.Reset();
}

void Drivetrain::ResetOdometry(const frc::Pose2d &pose)
{
  //m_odometry.ResetPosition(pose, GetYaw());
  m_poseEstimator.ResetPosition(pose, GetYaw());
  m_yawLockPID.SetSetpoint(GetYaw().Degrees().value());
  m_yawLockPID.Reset();
}

frc::ChassisSpeeds Drivetrain::GetChassisSpeeds()
{
  return m_robotVelocity;
}

void Drivetrain::InitSendable(wpi::SendableBuilder &builder)
{
  builder.SetSmartDashboardType("DriveBase");
  builder.SetActuator(true);

  // Modules
  m_frontLeft.InitSendable(builder);
  m_frontRight.InitSendable(builder);
  m_backLeft.InitSendable(builder);
  m_backRight.InitSendable(builder);

  builder.AddDoubleProperty("gyro", [this] { return m_imu.GetYaw(); }, nullptr);
  
  // Pose
  builder.AddDoubleProperty(
      "poseEstimator/x", [this] { return m_poseEstimator.GetEstimatedPosition().X().value(); }, nullptr);
  builder.AddDoubleProperty(
      "poseEstimator/y", [this] { return m_poseEstimator.GetEstimatedPosition().Y().value(); }, nullptr);
  builder.AddDoubleProperty(
      "poseEstimator/yaw", [this] { return m_poseEstimator.GetEstimatedPosition().Rotation().Degrees().value(); }, nullptr);

  // builder.AddDoubleProperty(
  //     "odometry/x", [this] { return m_odometry.GetPose().X().value(); }, nullptr);
  // builder.AddDoubleProperty(
  //     "odometry/y", [this] { return m_odometry.GetPose().Y().value(); }, nullptr);
  // builder.AddDoubleProperty(
  //     "odometry/yaw", [this] { return m_odometry.GetPose().Rotation().Degrees().value(); }, nullptr);

  // Velocity
  builder.AddDoubleProperty(
      "vel/x", [this] { return m_robotVelocity.vx.value(); }, nullptr);
  builder.AddDoubleProperty(
      "vel/y", [this] { return m_robotVelocity.vy.value(); }, nullptr);
  builder.AddDoubleProperty(
      "vel/yaw", [this] { return units::degrees_per_second_t(m_robotVelocity.omega).value(); }, nullptr);

  // Command
  builder.AddDoubleProperty(
      "cmd/x", [this] { return m_command.vx / 1_mps; }, nullptr);
  builder.AddDoubleProperty(
      "cmd/y", [this] { return m_command.vy / 1_mps; }, nullptr);
  builder.AddDoubleProperty(
      "cmd/yaw", [this] { return m_command.omega / 1_rad_per_s; }, nullptr);

  // State
    builder.AddDoubleProperty(
      "state/x", [this] { return m_robotVelocity.vx / 1_mps; }, nullptr);
  builder.AddDoubleProperty(
      "state/y", [this] { return m_robotVelocity.vy / 1_mps; }, nullptr);
  builder.AddDoubleProperty(
      "state/yaw", [this] { return m_robotVelocity.omega / 1_rad_per_s; }, nullptr);

  // Operating Mode
  builder.AddBooleanProperty(
      "cmd/fieldRelative", [this] { return m_fieldRelative; }, nullptr);
}

units::ampere_t Drivetrain::SimPeriodic(units::volt_t battery)
{

  // Simulated IMU
  m_imuSimCollection.AddHeading(GetChassisSpeeds().omega * 20_ms / 1_deg);

  return m_frontLeft.SimPeriodic(battery) +
    m_frontRight.SimPeriodic(battery) +
    m_backLeft.SimPeriodic(battery) +
    m_backRight.SimPeriodic(battery);
}

void Drivetrain::SimInit()
{
  m_frontLeft.SimInit();
  m_frontRight.SimInit();
  m_backLeft.SimInit();
  m_backRight.SimInit();
}

ErrorCode Drivetrain::SeedEncoders() {
  auto error = m_frontLeft.SeedTurnMotor();
  if (error != ErrorCode::OK) {
    return error;
  }

  error = m_frontRight.SeedTurnMotor();
  if (error != ErrorCode::OK) {
    return error;
  }

  error = m_backLeft.SeedTurnMotor();
  if (error != ErrorCode::OK) {
    return error;
  }

  return m_backRight.SeedTurnMotor();
}

bool Drivetrain::Active()
{
  return units::math::abs(m_frontLeft.AngularVelocity()) > 0.01_rad_per_s
    || units::math::abs(m_frontRight.AngularVelocity()) > 0.01_rad_per_s
    || units::math::abs(m_backLeft.AngularVelocity()) > 0.01_rad_per_s
    || units::math::abs(m_backRight.AngularVelocity()) > 0.01_rad_per_s;
}

frc::Pose2d Drivetrain::GetPose()
{
  return m_poseEstimator.GetEstimatedPosition();
}