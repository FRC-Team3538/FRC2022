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
  frc::SmartDashboard::PutNumber("Front Right Ang", m_frontRight.GetAngle().Degrees().value());
  frc::SmartDashboard::PutNumber("Back Left Ang", m_backLeft.GetAngle().Degrees().value());
  frc::SmartDashboard::PutNumber("Back Right Ang", m_backRight.GetAngle().Degrees().value());

  frc::SmartDashboard::PutNumber("Yaw", GetYaw().Degrees().value());
  frc::SmartDashboard::PutBoolean("Yaw Lock", m_YawLockActive);

  yawLock.SetDefaultOption("Enabled", "Enabled");
  yawLock.AddOption("Disabled", "Disabled");
  if (yawLock.GetSelected() == "Enabled")
  {
    yawLockEnabled = true;
  }
  else if (yawLock.GetSelected() == "Disabled")
  {
    yawLockEnabled = false;
  }

  frc::SmartDashboard::PutData("Yaw Lock PID", &yawLock);
  std::string name = yawLock.GetSelected();
  frc::SmartDashboard::PutString("Yaw Lock PID", name);

  frc::SmartDashboard::PutNumber("Odometry X", m_odometry.GetPose().X().value());
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

  // Heading Lock
  constexpr auto noRotThreshold = 0.1_deg_per_s;
  if (units::math::abs(rot) > noRotThreshold)
  {
    // Disable YawLock as soon as any rotation command is received
    m_YawLockActive = false;
  }
  else if (units::math::abs(rot) < noRotThreshold && units::math::abs(m_robotVelocity.omega) < 30_deg_per_s)
  {
    // Wait for the robot to stop spinning to enable Yaw Lock
    m_YawLockActive = yawLockEnabled; //true
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

  // Set State of Each Module
  auto [fl, fr, bl, br] = states;
  m_frontLeft.SetModule(fl);
  m_frontRight.SetModule(fr);
  m_backLeft.SetModule(bl);
  m_backRight.SetModule(br);
}

void Drivetrain::Stop()
{
  m_frontLeft.Stop();
  m_frontRight.Stop();
  m_backLeft.Stop();
  m_backRight.Stop();
}

void Drivetrain::Test(double y, double x)
{
  frc::SwerveModuleState f1;
  f1.speed = std::sqrt(std::pow(y, 2) + std::pow(x, 2)) * kMaxSpeedLinear;
  f1.angle = frc::Rotation2d(units::radian_t(std::atan2(-x, y)));
  m_frontLeft.SetModule(f1);
  //std::cout << f1.angle.Degrees().value() << std::endl;
}

frc::Rotation2d Drivetrain::GetYaw()
{
  return frc::Rotation2d{units::degree_t{m_imu.GetYaw()}};
}

void Drivetrain::UpdateOdometry()
{
  m_odometry.Update(GetYaw(),
                    m_frontLeft.GetState(),
                    m_frontRight.GetState(),
                    m_backLeft.GetState(),
                    m_backRight.GetState());

  m_poseEstimator.Update(GetYaw(),
                         m_frontLeft.GetState(),
                         m_frontRight.GetState(),
                         m_backLeft.GetState(),
                         m_backRight.GetState());

  m_robotVelocity = m_kinematics.ToChassisSpeeds({m_frontLeft.GetState(),
                                                  m_frontRight.GetState(),
                                                  m_backLeft.GetState(),
                                                  m_backRight.GetState()});

  auto p = m_odometry.GetPose();
  frc::Pose2d fliperoo = {-p.Y(), p.X(), p.Rotation().RotateBy(90_deg)}; // Driver Station PoV
  m_fieldDisplay.SetRobotPose(fliperoo);
}

void Drivetrain::ResetYaw()
{
  m_imu.SetYaw(0.0);

  m_yawLockPID.SetSetpoint(GetYaw().Degrees().value());
  m_yawLockPID.Reset();
}

void Drivetrain::ResetOdometry(const frc::Pose2d &pose)
{
  m_odometry.ResetPosition(pose, GetYaw());
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

  // m_yawLockPID.InitSendable(builder);

  builder.AddDoubleProperty("gyro", [this] { return m_imu.GetYaw(); }, nullptr);
  //builder.AddDoubleProperty("pigeon/yaw", [this] { double ypr[3]; alt_imu.GetYawPitchRoll(ypr); return ypr[0]; }, nullptr);
  //builder.AddDoubleProperty("pigeon/pitch", [this] { double ypr[3]; alt_imu.GetYawPitchRoll(ypr); return ypr[1]; }, nullptr);
  //builder.AddDoubleProperty("pigeon/roll", [this] { double ypr[3]; alt_imu.GetYawPitchRoll(ypr); return ypr[2]; }, nullptr);
  
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
      "cmd/x", [this] { return m_command.vx.value(); }, nullptr);
  builder.AddDoubleProperty(
      "cmd/y", [this] { return m_command.vy.value(); }, nullptr);
  builder.AddDoubleProperty(
      "cmd/yaw", [this] { return units::degrees_per_second_t(m_command.omega).value(); }, nullptr);

  // Heading Lock
  builder.AddDoubleProperty(
      "YawPID/kP", [this] { return m_yawLockPID.GetP(); }, [this](double value) { m_yawLockPID.SetP(value); });
  builder.AddDoubleProperty(
      "YawPID/kI", [this] { return m_yawLockPID.GetI(); }, [this](double value) { m_yawLockPID.SetI(value); });
  builder.AddDoubleProperty(
      "YawPID/kD", [this] { return m_yawLockPID.GetD(); }, [this](double value) { m_yawLockPID.SetD(value); });
  builder.AddDoubleProperty(
      "YawPID/SP",
      [this] { return units::degree_t(m_yawLockPID.GetSetpoint()).value(); }, nullptr);

  // Operating Mode
  builder.AddBooleanProperty(
      "cmd/fieldRelative", [this] { return m_fieldRelative; }, nullptr);
}

units::ampere_t Drivetrain::SimPeriodic(units::volt_t battery)
{

  // Simulated IMU
  m_imuSimCollection.SetRawHeading(m_odometry.GetPose().Rotation().Degrees() / 1_deg);

  return m_frontLeft.SimPeriodic(battery) +
    m_frontRight.SimPeriodic(battery) +
    m_backLeft.SimPeriodic(battery) +
    m_backRight.SimPeriodic(battery);
}