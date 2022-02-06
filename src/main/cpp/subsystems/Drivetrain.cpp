// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.hpp"

#include <frc/RobotController.h>

#include <iostream>

Drivetrain::Drivetrain()
{
    // Motor Configuration
    m_driveL0.ConfigFactoryDefault();
    m_driveL1.ConfigFactoryDefault();
    m_driveL2.ConfigFactoryDefault();
    m_driveR0.ConfigFactoryDefault();
    m_driveR1.ConfigFactoryDefault();
    m_driveR2.ConfigFactoryDefault();

    m_driveL0.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
    m_driveL0.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 18);

    m_driveR0.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
    m_driveR0.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 18);

    m_rightGroup.SetInverted(false);
    m_leftGroup.SetInverted(true);

    m_driveL0.SetSelectedSensorPosition(0.0);
    m_driveR0.SetSelectedSensorPosition(0.0);

    // Motor Current Limits
    // SupplyCurrentLimitConfiguration config{true, 30.0, 40.0, 0.0};
    // m_driveL0.ConfigSupplyCurrentLimit(config);
    // m_driveL1.ConfigSupplyCurrentLimit(config);
    // m_driveL2.ConfigSupplyCurrentLimit(config);
    // m_driveR0.ConfigSupplyCurrentLimit(config);
    // m_driveR1.ConfigSupplyCurrentLimit(config);
    // m_driveR2.ConfigSupplyCurrentLimit(config);

    // IMU / Gyro / AHRS
    // m_imu.Reset();
    m_imu.ConfigFactoryDefault();

    // Yaw PID
    m_yawPID.EnableContinuousInput(-180, 180);
    //m_yawPID.SetIntegratorRange(0.05, 0.5);

    // Robot Pose Display
    frc::SmartDashboard::PutData("Field", &m_fieldSim);
}

// Teleop Control
void Drivetrain::Arcade(double forward, double rotate)
{
    cmd_fwd = forward;
    cmd_rot = rotate;

    m_leftGroup.Set(forward - rotate);
    m_rightGroup.Set(forward + rotate);
}

void Drivetrain::SetSpeeds(const frc::DifferentialDriveWheelSpeeds &speeds)
{
    cmd_vl = speeds.left.value();
    cmd_vr = speeds.right.value();

    auto leftFeedforward = m_feedforward.Calculate(speeds.left);
    auto rightFeedforward = m_feedforward.Calculate(speeds.right);

    auto leftRate = - m_driveL0.GetSelectedSensorVelocity() * kDPP * 10.0;
    auto rightRate = m_driveR0.GetSelectedSensorVelocity() * kDPP * 10.0;

    double leftOutput = m_leftPIDController.Calculate(
        leftRate.value(),
        speeds.left.to<double>());

    double rightOutput = m_rightPIDController.Calculate(
        rightRate.value(),
        speeds.right.to<double>());

    m_leftGroup.SetVoltage(units::volt_t{leftOutput} + leftFeedforward);
    m_rightGroup.SetVoltage(units::volt_t{rightOutput} + rightFeedforward);
}

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::radians_per_second_t rot)
{
    SetSpeeds(m_kinematics.ToWheelSpeeds({xSpeed, 0_mps, rot}));
}

void Drivetrain::Drive(const frc::Trajectory::State& target)
{
    auto pose = GetPose();
    auto speeds = m_ramsete.Calculate(pose, target);

    cmd_vx = speeds.vx.value();
    cmd_vw = speeds.omega.value();

    Drive(speeds.vx, speeds.omega);
}

frc::Pose2d Drivetrain::GetPose() const 
{ 
    return m_odometry.GetPose(); 
}

void Drivetrain::UpdateOdometry()
{
    // left is inverted, so sensor position is inverted
    auto left = -m_driveL0.GetSelectedSensorPosition(0);
    auto right = m_driveR0.GetSelectedSensorPosition(0);
    auto yaw = GetYaw();

    m_odometry.Update(yaw,
                      units::meter_t(left * kDPP),
                      units::meter_t(right * kDPP));
                   
    m_fieldSim.SetRobotPose(GetPose());  // TEMP DEBUG OFFSET
}

void Drivetrain::ResetOdometry(const frc::Pose2d &pose)
{
    m_driveL0.SetSelectedSensorPosition(0);
    m_driveR0.SetSelectedSensorPosition(0);

    // auto res = m_imu.SetFusedHeading(pose.Rotation().Degrees().value(), 50);
    // if (res != 0) {
    //     std::cout << "ERROR: Fused Heading Set Failed: " << res << std::endl;
    // }
    m_odometry.ResetPosition(pose, pose.Rotation());



    m_drivetrainSimulator.SetPose(pose);
}

frc::Rotation2d Drivetrain::GetYaw()
{
    //ctre::phoenix::sensors::PigeonIMU::FusionStatus status;
    //m_imu.GetFusedHeading(status);
    //double heading = status.heading;

    // std::cout << status.bIsFusing << std::endl;
    // std::cout << status.bIsValid << std::endl;
    // std::cout << status.description << std::endl;
    // std::cout << status.lastError << std::endl;

    // TODO(Dereck): Verify the sensor data is valid...
    return m_imu.GetRotation2d();
}

bool Drivetrain::TurnRel(double forward, units::degree_t target, units::degree_t tolerance)
{
    m_yawPID.SetSetpoint((target + GetYaw().Degrees()).value());
    m_yawPID.SetTolerance(tolerance.value());
    double rotate = m_yawPID.Calculate(GetYaw().Degrees().value());

    if (units::math::abs(target) < tolerance)
    {
        Arcade(0.0, 0.0);
        return true;
    }
    else
    {
        Arcade(forward, rotate);
        return false;
    }
}

void Drivetrain::SimulationPeriodic()
{
    // To update our simulation, we set motor `tage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    m_drivetrainSimulator.SetInputs(units::volt_t{m_driveL0.GetMotorOutputVoltage()},
                                    units::volt_t{-m_driveR0.GetMotorOutputVoltage()});
    m_drivetrainSimulator.Update(20_ms);

    // TODO(Dereck): I don't trust this yet...
    //auto vbatt = frc::RobotController::GetInputVoltage() + ( m_drivetrainSimulator.GetCurrentDraw().value() * kBatteryResistance);

    // Left Motors
    auto left_sim = m_driveL0.GetSimCollection();
    auto left_pos = m_drivetrainSimulator.GetLeftPosition().to<double>();
    auto left_vel = m_drivetrainSimulator.GetLeftVelocity().to<double>();
    //left_sim.SetIntegratedSensorRawPosition(left_pos / kDPP.value());
    left_sim.SetIntegratedSensorVelocity(left_vel / kDPP.value() / 10);
    m_driveL0.SetSelectedSensorPosition(left_pos / kDPP.value());
    //left_sim.SetBusVoltage(vbatt);

    // Right Motors
    auto right_sim = m_driveR0.GetSimCollection();
    auto right_pos = m_drivetrainSimulator.GetRightPosition().to<double>();
    auto right_vel = m_drivetrainSimulator.GetRightVelocity().to<double>();
    //right_sim.SetIntegratedSensorRawPosition(right_pos / kDPP.value());
    right_sim.SetIntegratedSensorVelocity(right_vel / kDPP.value() / 10);
    m_driveR0.SetSelectedSensorPosition(right_pos / kDPP.value());
    //right_sim.SetBusVoltage(vbatt);
    
    // IMU 
    // TODO(Dereck): CHECK DIRECTION WRT REAL SENSOR
    auto imu_sim = m_imu.GetSimCollection();
    imu_sim.SetRawHeading(-m_drivetrainSimulator.GetHeading().Degrees().to<double>());
}

void Drivetrain::Periodic()
{
    UpdateOdometry();
}

void Drivetrain::SetBrakeMode()
{
  m_driveL0.SetNeutralMode(NeutralMode::Brake);
  m_driveL1.SetNeutralMode(NeutralMode::Brake);
  m_driveL2.SetNeutralMode(NeutralMode::Brake);
  m_driveR0.SetNeutralMode(NeutralMode::Brake);
  m_driveR1.SetNeutralMode(NeutralMode::Brake);
  m_driveR2.SetNeutralMode(NeutralMode::Brake);
}

void Drivetrain::SetCoastMode()
{
  m_driveL0.SetNeutralMode(NeutralMode::Coast);
  m_driveL1.SetNeutralMode(NeutralMode::Coast);
  m_driveL2.SetNeutralMode(NeutralMode::Coast);
  m_driveR0.SetNeutralMode(NeutralMode::Coast);
  m_driveR1.SetNeutralMode(NeutralMode::Coast);
  m_driveR2.SetNeutralMode(NeutralMode::Coast);
}

frc::Field2d& Drivetrain::GetField() {
    return m_fieldSim;
}

void Drivetrain::UpdateTelemetry() { }

void Drivetrain::ConfigureSystem() { }

void Drivetrain::InitSendable(wpi::SendableBuilder &builder)
{
    builder.SetSmartDashboardType("DriveBase");
    builder.SetActuator(true);

    // Commands
    builder.AddDoubleProperty(
        "cmd/fwd", [this] { return cmd_fwd; }, nullptr);
    builder.AddDoubleProperty(
        "cmd/rot", [this] { return cmd_rot; }, nullptr);
    builder.AddDoubleProperty(
        "cmd/vx", [this] { return cmd_vx; }, nullptr);
    builder.AddDoubleProperty(
        "cmd/vw", [this] { return cmd_vw; }, nullptr);
    builder.AddDoubleProperty(
        "cmd/vl", [this] { return cmd_vl; }, nullptr);
    builder.AddDoubleProperty(
        "cmd/vr", [this] { return cmd_vr; }, nullptr);

    // Left Motors
    builder.AddDoubleProperty(
        "left/percent", [this] { return m_driveL0.Get(); }, nullptr);
    builder.AddDoubleProperty(
        "left/voltage", [this] { return m_driveL0.GetMotorOutputVoltage(); }, nullptr);
    builder.AddDoubleProperty(
        "left/position", [this] { return - m_driveL0.GetSelectedSensorPosition() * kDPP.value(); }, nullptr);
    builder.AddDoubleProperty(
        "left/velocity", [this] { return - m_driveL0.GetSelectedSensorVelocity() * kDPP.value() * 10; }, nullptr);

    // Right Motors
    builder.AddDoubleProperty(
        "right/percent", [this] { return m_driveR0.Get(); }, nullptr);
    builder.AddDoubleProperty(
        "right/voltage", [this] { return m_driveR0.GetMotorOutputVoltage(); }, nullptr);
    builder.AddDoubleProperty(
        "right/position", [this] { return m_driveR0.GetSelectedSensorPosition() * kDPP.value(); }, nullptr);
    builder.AddDoubleProperty(
        "right/velocity", [this] { return m_driveR0.GetSelectedSensorVelocity() * kDPP.value() * 10; }, nullptr);

    // IMU
    builder.AddDoubleProperty(
        "IMU/yaw", [this] { return GetYaw().Degrees().value(); }, nullptr);

    // Pose
    builder.AddDoubleProperty(
        "pose/x", [this] { return GetPose().X().value(); }, nullptr);
    builder.AddDoubleProperty(
        "pose/y", [this] { return GetPose().Y().value(); }, nullptr);
    builder.AddDoubleProperty(
        "pose/yaw", [this] { return GetPose().Rotation().Radians().value(); }, nullptr);

    // Yaw PID
    builder.AddDoubleProperty(
        "YawPID/kP", [this] { return m_yawPID.GetP(); }, [this](double value) { m_yawPID.SetP(value); });
    builder.AddDoubleProperty(
        "YawPID/kI", [this] { return m_yawPID.GetI(); }, [this](double value) { m_yawPID.SetI(value); });
    builder.AddDoubleProperty(
        "YawPID/kD", [this] { return m_yawPID.GetD(); }, [this](double value) { m_yawPID.SetD(value); });
    builder.AddDoubleProperty(
        "YawPID/SP",
        [this] { return units::degree_t(m_yawPID.GetSetpoint()).value(); }, nullptr);

}