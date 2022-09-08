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
  frc::SmartDashboard::PutNumber("Front Left Ang", m_frontLeft.GetAngle().Degrees().value());
  // frc::SmartDashboard::PutNumber("Front Right Ang", m_frontRight.GetAngle().Degrees().value());
  // frc::SmartDashboard::PutNumber("Back Left Ang", m_backLeft.GetAngle().Degrees().value());
  // frc::SmartDashboard::PutNumber("Back Right Ang", m_backRight.GetAngle().Degrees().value());

  frc::SmartDashboard::PutNumber("Yaw", GetYaw().Degrees().value());

  frc::SmartDashboard::PutNumber("Odometry X", m_odometry.GetPose().X().value());
}

void Drivetrain::ConfigureSystem()
{
  m_imu.ZeroGyroBiasNow();
  ResetYaw();

  // Display Robot position on field
  frc::SmartDashboard::PutData("Field", &m_fieldDisplay);

  m_frontLeft.ConfigureSystem();
  // m_frontRight.ConfigureSystem();
  // m_backLeft.ConfigureSystem();
  // m_backRight.ConfigureSystem();
}

void Drivetrain::Drive(frc::Trajectory::State trajectoryState, units::radian_t yaw)
{
  const auto command = m_trajectoryController.Calculate(
      m_odometry.GetPose(),
      trajectoryState,
      yaw);

  Drive(command.vx, command.vy, command.omega, false);
}

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rot,
                       bool fieldRelative)
{
  // Remember the last operating mode, for smartdash display
  m_fieldRelative = fieldRelative;

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

  // Set State of Each Module
  auto [fl, fr, bl, br] = states;
  m_frontLeft.SetModule(fl);
  // m_frontRight.SetModule(fr);
  // m_backLeft.SetModule(bl);
  // m_backRight.SetModule(br);
}

void Drivetrain::Stop()
{
  m_frontLeft.Stop();
  // m_frontRight.Stop();
  // m_backLeft.Stop();
  // m_backRight.Stop();
}

frc::Rotation2d Drivetrain::GetYaw()
{
  return frc::Rotation2d{units::degree_t{m_imu.GetYaw()}};
}

void Drivetrain::UpdateOdometry()
{
  // m_odometry.Update(GetYaw(),
  //                   m_frontLeft.GetState(),
  //                   m_frontRight.GetState(),
  //                   m_backLeft.GetState(),
  //                   m_backRight.GetState());

  // m_poseEstimator.Update(GetYaw(),
  //                        m_frontLeft.GetState(),
  //                        m_frontRight.GetState(),
  //                        m_backLeft.GetState(),
  //                        m_backRight.GetState());

  // m_robotVelocity = m_kinematics.ToChassisSpeeds({m_frontLeft.GetState(),
  //                                                 m_frontRight.GetState(),
  //                                                 m_backLeft.GetState(),
  //                                                 m_backRight.GetState()});

  auto p = m_odometry.GetPose();
  frc::Pose2d fliperoo = {-p.Y(), p.X(), p.Rotation().RotateBy(90_deg)}; // Driver Station PoV
  m_fieldDisplay.SetRobotPose(fliperoo);
}

void Drivetrain::ResetYaw()
{
  m_imu.SetYaw(0.0);
}

void Drivetrain::ResetOdometry(const frc::Pose2d &pose)
{
  m_odometry.ResetPosition(pose, GetYaw());
  m_poseEstimator.ResetPosition(pose, GetYaw());
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
  // m_frontRight.InitSendable(builder);
  // m_backLeft.InitSendable(builder);
  // m_backRight.InitSendable(builder);


  builder.AddDoubleProperty("gyro", [this] { return m_imu.GetYaw(); }, nullptr);
  
  // Pose
  builder.AddDoubleProperty(
      "poseEstimator/x", [this] { return m_poseEstimator.GetEstimatedPosition().X().value(); }, nullptr);
  builder.AddDoubleProperty(
      "poseEstimator/y", [this] { return m_poseEstimator.GetEstimatedPosition().Y().value(); }, nullptr);
  builder.AddDoubleProperty(
      "poseEstimator/yaw", [this] { return m_poseEstimator.GetEstimatedPosition().Rotation().Degrees().value(); }, nullptr);

  builder.AddDoubleProperty(
      "odometry/x", [this] { return m_odometry.GetPose().X().value(); }, nullptr);
  builder.AddDoubleProperty(
      "odometry/y", [this] { return m_odometry.GetPose().Y().value(); }, nullptr);
  builder.AddDoubleProperty(
      "odometry/yaw", [this] { return m_odometry.GetPose().Rotation().Degrees().value(); }, nullptr);

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

  // Operating Mode
  builder.AddBooleanProperty(
      "cmd/fieldRelative", [this] { return m_fieldRelative; }, nullptr);
}

units::ampere_t Drivetrain::SimPeriodic(units::volt_t battery)
{

  // Simulated IMU
  // TODO
  m_imuSimCollection.SetRawHeading(m_odometry.GetPose().Rotation().Degrees() / 1_deg);

  return m_frontLeft.SimPeriodic(battery); // +
    // m_frontRight.SimPeriodic(battery) +
    // m_backLeft.SimPeriodic(battery) +
    // m_backRight.SimPeriodic(battery);
}