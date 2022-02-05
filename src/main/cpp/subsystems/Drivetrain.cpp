// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.hpp"

#include <frc/RobotController.h>

#include <iostream>

Drivetrain::Drivetrain(bool isSimulation)
{
    m_isSimulation = isSimulation;

    m_driveL0.ConfigFactoryDefault();
    m_driveL1.ConfigFactoryDefault();
    m_driveL2.ConfigFactoryDefault();
    m_driveR0.ConfigFactoryDefault();
    m_driveR1.ConfigFactoryDefault();
    m_driveR2.ConfigFactoryDefault();

    double pidIdx = 0;
    m_driveL0.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, pidIdx);
    m_driveL0.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 18);

    m_driveR0.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, pidIdx);
    m_driveR0.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 18);

    m_gyro.Reset();
    // m_imu.Reset();
    m_imu.ConfigFactoryDefault();

    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    // 0.4787_m / 1_rot
    // 2048 * 10.71 ticks / rot
    // 2.182807 * 10^-5 ticks/m
    // 0.0000218
    // auto dpp = empiricalDist / 188960.5; // 218325.5;//128173.5;//((2 * wpi::numbers::pi * kWheelRadius) / kEncoderResolution);
    // TODO: really measure this, but do we need to do it for AR?
    auto dpp = 0.0000218_m;
    m_leftEncoder.SetDistancePerPulse(dpp.value());
    m_rightEncoder.SetDistancePerPulse(-dpp.value());

    m_leftEncoder.Reset();
    m_rightEncoder.Reset();

    m_rightGroup.SetInverted(false);
    m_leftGroup.SetInverted(true);

    m_driveL0.SetSelectedSensorPosition(0.0);
    m_driveR0.SetSelectedSensorPosition(0.0);

    
    // impel.SetNeutralMode(NeutralMode::Coast);
    // impel2.SetNeutralMode(NeutralMode::Coast);

    frc::SmartDashboard::PutData("Field", &m_fieldSim);

    SupplyCurrentLimitConfiguration config{true, 30.0, 40.0, 0.0};
    // impel.ConfigSupplyCurrentLimit(config);
    // impel2.ConfigSupplyCurrentLimit(config);
}

void Drivetrain::SetSpeeds(const frc::DifferentialDriveWheelSpeeds &speeds)
{
    auto leftFeedforward = m_feedforward.Calculate(speeds.left);
    auto rightFeedforward = m_feedforward.Calculate(speeds.right);

    double leftRate = 0.0;
    double rightRate = 0.0;

    if (m_isSimulation)
    {
        leftRate = m_leftEncoder.GetRate();
        rightRate = m_rightEncoder.GetRate();
    }
    else
    {
        leftRate = m_driveL0.GetSelectedSensorVelocity(0) * m_leftEncoder.GetDistancePerPulse() * 10.0;
        rightRate = m_driveR0.GetSelectedSensorVelocity(0) * m_leftEncoder.GetDistancePerPulse() * 10.0;
    }

    double leftOutput = m_leftPIDController.Calculate(
        leftRate,
        speeds.left.to<double>());

    double rightOutput = m_rightPIDController.Calculate(
        rightRate,
        speeds.right.to<double>());

    m_leftGroup.SetVoltage(units::volt_t{leftOutput} + leftFeedforward);
    m_rightGroup.SetVoltage(units::volt_t{rightOutput} + rightFeedforward);

    // m_leftGroup.SetVoltage(leftFeedforward);
    // m_rightGroup.SetVoltage(rightFeedforward);
}

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::radians_per_second_t rot)
{
    SetSpeeds(m_kinematics.ToWheelSpeeds({xSpeed, 0_mps, rot}));
}

void Drivetrain::Arcade(double forward, double rotate)
{
    m_leftGroup.SetVoltage((forward - rotate) * 13_V);
    m_rightGroup.SetVoltage((forward + rotate) * 13_V);
}

void Drivetrain::UpdateOdometry()
{
    auto left = m_driveL0.GetSelectedSensorPosition(0) * m_leftEncoder.GetDistancePerPulse();
    auto right = m_driveR0.GetSelectedSensorPosition(0) * m_rightEncoder.GetDistancePerPulse();

    m_odometry.Update(GetYaw(),
                      units::meter_t(left),
                      units::meter_t(right));
}

void Drivetrain::ResetOdometry(const frc::Pose2d &pose)
{
    m_leftEncoder.Reset();
    m_rightEncoder.Reset();

    m_driveL0.SetSelectedSensorPosition(0);
    m_driveR0.SetSelectedSensorPosition(0);

    m_drivetrainSimulator.SetPose(pose);
    m_imu.SetFusedHeading(pose.Rotation().Degrees().value(), 50);
    m_odometry.ResetPosition(pose, pose.Rotation());
}

frc::Rotation2d Drivetrain::GetYaw()
{
    ctre::phoenix::sensors::PigeonIMU::FusionStatus status;
    m_imu.GetFusedHeading(status);
    double heading = status.heading;
    // std::cout << status.bIsFusing << std::endl;
    // std::cout << status.bIsValid << std::endl;
    // std::cout << status.description << std::endl;
    // std::cout << status.lastError << std::endl;
    if (heading > 180)
    {
        while (heading > 180)
            heading -= 360;
    }

    if (heading < -180)
    {
        while (heading < -180)
            heading += 360;
    }
    return frc::Rotation2d{units::degree_t{heading}};
}

bool Drivetrain::TurnRel(double forward, units::degree_t target, units::degree_t tolerance)
{
    //std::cout << target.value() << std::endl;
    yawController.SetSetpoint((target + GetYaw().Degrees()).value());
    yawController.SetTolerance(tolerance.value());
    double rotate = yawController.Calculate(GetYaw().Degrees().value());

    //std::cout << rotate << std::endl;
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
    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    m_drivetrainSimulator.SetInputs(units::volt_t{m_leftLeader.Get()} *
                                        frc::RobotController::GetInputVoltage(),
                                    units::volt_t{-m_rightLeader.Get()} *
                                        frc::RobotController::GetInputVoltage());
    m_drivetrainSimulator.Update(20_ms);

    m_leftEncoderSim.SetDistance(
        m_drivetrainSimulator.GetLeftPosition().to<double>());
    m_leftEncoderSim.SetRate(
        m_drivetrainSimulator.GetLeftVelocity().to<double>());
    m_rightEncoderSim.SetDistance(
        m_drivetrainSimulator.GetRightPosition().to<double>());
    m_rightEncoderSim.SetRate(
        m_drivetrainSimulator.GetRightVelocity().to<double>());
    m_gyroSim.SetAngle(
        -m_drivetrainSimulator.GetHeading().Degrees().to<double>());
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

void Drivetrain::UpdateTelemetry()
{
    m_fieldSim.SetRobotPose(m_odometry.GetPose());
    double angle = GetYaw().Radians().value();
    double distL = (m_driveL0.GetSelectedSensorPosition(0) * m_leftEncoder.GetDistancePerPulse());
    double distR = (m_driveR0.GetSelectedSensorPosition(0) * m_rightEncoder.GetDistancePerPulse());
    frc::SmartDashboard::PutNumber("Dist L", distL);
    frc::SmartDashboard::PutNumber("Dist R", distR);
    frc::SmartDashboard::PutNumber("Angle", angle);

    yawController.SetPID(frc::SmartDashboard::GetNumber("P", 0.0), frc::SmartDashboard::GetNumber("I", 0.0), frc::SmartDashboard::GetNumber("D", 0.0));
    frc::SmartDashboard::PutNumber("P FROM THE P", yawController.GetP());
    // frc::SmartDashboard::PutNumber("DISTANCEL", m_driveL0.GetSelectedSensorPosition(0));
    // frc::SmartDashboard::PutNumber("DISTANCER", m_driveR0.GetSelectedSensorPosition(0));
}

void Drivetrain::ConfigureSystem()
{
    frc::SmartDashboard::PutNumber("P", 0.0);
    frc::SmartDashboard::PutNumber("I", 0.0);
    frc::SmartDashboard::PutNumber("D", 0.0);
    yawController.EnableContinuousInput(-180, 180);
    //yawController.SetIntegratorRange(0.05, 0.5);
}