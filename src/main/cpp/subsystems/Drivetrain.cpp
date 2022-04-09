// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.hpp"

#include <frc/Timer.h>                                       // for Timer

#include "frc/controller/PIDController.h"                    // for PIDContr...
#include "frc/controller/RamseteController.h"                // for RamseteC...
#include "frc/controller/SimpleMotorFeedforward.h"           // for SimpleMo...
#include "frc/geometry/Translation2d.h"                      // for Translat...
#include "frc/kinematics/ChassisSpeeds.h"                    // for ChassisS...
#include "frc/kinematics/DifferentialDriveKinematics.h"      // for Differen...
#include "frc/kinematics/DifferentialDriveOdometry.h"        // for Differen...
#include "frc/kinematics/DifferentialDriveWheelSpeeds.h"     // for Differen...
#include "frc/motorcontrol/MotorControllerGroup.h"           // for MotorCon...
#include "frc/simulation/DifferentialDrivetrainSim.h"        // for Differen...
#include "frc/smartdashboard/Field2d.h"                      // for Field2d
#include "frc/smartdashboard/FieldObject2d.h"                // for FieldObj...
#include "frc/smartdashboard/SmartDashboard.h"               // for SmartDas...
#include "frc/trajectory/Trajectory.h"                       // for Trajecto...
#include "networktables/NetworkTableEntry.inc"               // for NetworkT...
#include "units/angle.h"                                     // for degree_t
#include "units/angular_velocity.h"                          // for radians_...
#include "units/curvature.h"                                 // for curvature_t
#include "units/length.h"                                    // for meter_t
#include "units/math.h"                                      // for abs
#include "units/velocity.h"                                  // for meters_p...
#include "units/voltage.h"                                   // for volt_t
#include "wpi/sendable/SendableBuilder.h"                    // for Sendable...
#include "ctre/phoenix/motorcontrol/ControlMode.h"
#include "ctre/phoenix/motorcontrol/FeedbackDevice.h"
#include "ctre/phoenix/motorcontrol/NeutralMode.h"
#include "ctre/phoenix/motorcontrol/StatusFrame.h"
#include "ctre/phoenix/motorcontrol/TalonFXSimCollection.h"
#include "ctre/phoenix/sensors/BasePigeonSimCollection.h"

using namespace ctre::phoenix::motorcontrol;

#include <wpi/DataLog.h>

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
    // m_driveL0.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 18);

    m_driveR0.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
    // m_driveR0.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 18);

    // SetInverted on individual motors rather than the group so that
    // Encoders and LED's are also inverted as expected.
    m_driveL0.SetInverted(true);
    m_driveL1.SetInverted(true);
    m_driveL2.SetInverted(true);
    m_driveR0.SetInverted(false);
    m_driveR1.SetInverted(false);
    m_driveR2.SetInverted(false);

    // Break / Coast mode (Not affected by ConfigFactoryDefault)
    SetCoastMode();

    // Reset Encoder positions
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
    // m_yawPID.SetIntegratorRange(0.05, 0.5);

    // Robot Pose Display
    frc::SmartDashboard::PutData("Field", &m_fieldSim);

    // Set Status Frames but without spaces
    SetStatusFrames(m_driveL0, 250);
    SetStatusFrames(m_driveL1, 250);
    SetStatusFrames(m_driveL2, 250);
    SetStatusFrames(m_driveR0, 250);
    SetStatusFrames(m_driveR1, 250);
    SetStatusFrames(m_driveR2, 250);

    m_driveL0.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General, 20, 50);
    m_driveR0.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_1_General, 20, 50);

    m_driveL0.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0, 20, 50);
    m_driveR0.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_2_Feedback0, 20, 50);

    m_driveL0.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 20, 50);
    m_driveR0.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 20, 50);

    m_driveL0.ConfigVelocityMeasurementPeriod(ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_50Ms, 50);
    m_driveL1.ConfigVelocityMeasurementPeriod(ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_50Ms, 50);
    m_driveL2.ConfigVelocityMeasurementPeriod(ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_50Ms, 50);
    m_driveR0.ConfigVelocityMeasurementPeriod(ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_50Ms, 50);
    m_driveR1.ConfigVelocityMeasurementPeriod(ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_50Ms, 50);
    m_driveR2.ConfigVelocityMeasurementPeriod(ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_50Ms, 50);

    m_driveL0.ConfigVelocityMeasurementWindow(4, 50);
    m_driveL1.ConfigVelocityMeasurementWindow(4, 50);
    m_driveL2.ConfigVelocityMeasurementWindow(4, 50);
    m_driveR0.ConfigVelocityMeasurementWindow(4, 50);
    m_driveR1.ConfigVelocityMeasurementWindow(4, 50);
    m_driveR2.ConfigVelocityMeasurementWindow(4, 50);

    m_driveL0.Config_kP(0, 0.043815, 50);
    m_driveL1.Config_kP(0, 0.043815, 50);
    m_driveL2.Config_kP(0, 0.043815, 50);
    m_driveR0.Config_kP(0, 0.043815, 50);
    m_driveR1.Config_kP(0, 0.043815, 50);
    m_driveR2.Config_kP(0, 0.043815, 50);

    referencePose = m_fieldSim.GetObject("reference");
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

    auto leftRate = -m_driveL0.GetSelectedSensorVelocity() * kDPP / 0.1_s;
    auto rightRate = m_driveR0.GetSelectedSensorVelocity() * kDPP / 0.1_s;

    auto current_speeds_vec = Eigen::Vector2<double>{leftRate.value(), rightRate.value()};
    auto goal_speeds_vec = Eigen::Vector2<double>{speeds.left.value(), speeds.right.value()};

    auto ff = m_drivetrainFeedForward.Calculate(current_speeds_vec, goal_speeds_vec);

    auto leftFeedforward = ff[0];
    auto rightFeedforward = ff[1];

    // auto leftFeedforward = m_feedforward.Calculate(speeds.left);
    // auto rightFeedforward = m_feedforward.Calculate(speeds.right);

    // convert from meters per second to ticks per 100ms
    auto leftSpeed = speeds.left * (0.1_s / kDPP);
    auto rightSpeed = speeds.right * (0.1_s / kDPP);

    auto left_ff = leftFeedforward / 12_V;
    auto right_ff = rightFeedforward / 12_V;

    m_driveL0.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Velocity, leftSpeed.value(), ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward, left_ff.value());
    m_driveL1.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Velocity, leftSpeed.value(), ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward, left_ff.value());
    m_driveL2.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Velocity, leftSpeed.value(), ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward, left_ff.value());
    m_driveR0.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Velocity, rightSpeed.value(), ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward, right_ff.value());
    m_driveR1.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Velocity, rightSpeed.value(), ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward, right_ff.value());
    m_driveR2.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Velocity, rightSpeed.value(), ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward, right_ff.value());
}

void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::radians_per_second_t rot)
{
    SetSpeeds(m_kinematics.ToWheelSpeeds({xSpeed, 0_mps, rot}));
}

void Drivetrain::Drive(frc::ChassisSpeeds &speeds)
{
    SetSpeeds(m_kinematics.ToWheelSpeeds(speeds));
}

void Drivetrain::Drive(const frc::Trajectory::State &target)
{
    reference = target;
    referencePose->SetPose(reference.pose);

    auto pose = GetPose();
    // std::cout << "Heading @ " << target.t.value() << "s: " << pose.Rotation().Radians().value() << std::endl;
    auto speeds = m_ramsete.Calculate(pose, target);

    cmd_vx = speeds.vx.value();
    cmd_vw = speeds.omega.value();

    Drive(speeds);
}

frc::Pose2d Drivetrain::GetPose() const
{
    if (localization_flag_entry.GetBoolean(false) )
    {
        // return m_poseEstimator.GetEstimatedPosition();
        return frc::Pose2d{};
    }
    else 
    {  
        return m_odometry.GetPose();
    }
}

void Drivetrain::UpdateOdometryWithGlobalEstimate(frc::Pose2d globalEstimate, units::second_t estimateTime)
{
    // m_poseEstimator.AddVisionMeasurement(globalEstimate, estimateTime);
}

void Drivetrain::UpdateOdometry()
{
#ifdef __FRC_ROBORIO__
    auto imuYaw = m_imu.GetRotation2d();
    auto left = m_driveL0.GetSelectedSensorPosition(0) * kDPP;
    auto right = m_driveR0.GetSelectedSensorPosition(0) * kDPP;

    // auto leftV = m_driveL0.GetSelectedSensorVelocity(0) * kDPP / 100.0_ms;
    // auto rightV = m_driveR0.GetSelectedSensorVelocity(0) * kDPP / 100.0_ms;
#else
    auto imuYaw = m_imu.GetRotation2d();
    // auto imuYaw = -m_drivetrainSimulator.GetHeading();
    auto left = m_drivetrainSimulator.GetLeftPosition();
    auto right = m_drivetrainSimulator.GetRightPosition();

    // auto leftV = m_drivetrainSimulator.GetLeftVelocity();
    // auto rightV = m_drivetrainSimulator.GetRightVelocity();
#endif

    m_odometry.Update(imuYaw, left, right);

    // m_poseEstimator.Update(GetYaw(), frc::DifferentialDriveWheelSpeeds{leftV, rightV}, left, right);

    m_fieldSim.SetRobotPose(GetPose()); // TEMP DEBUG OFFSET
}

void Drivetrain::ResetOdometry(const frc::Pose2d &pose)
{
    m_driveL0.SetSelectedSensorPosition(0);
    m_driveR0.SetSelectedSensorPosition(0);

    // Delay for Falcons to reset
    frc::Timer t;
    t.Start();
    while (t.Get() < 20_ms)
        ;

    // auto res = m_imu.SetFusedHeading(pose.Rotation().Degrees().value(), 50);
    // if (res != 0) {
    //     std::cout << "ERROR: Fused Heading Set Failed: " << res << std::endl;
    // }

    // std::cout << "Resetting heading to: " << pose.Rotation().Radians().value() << std::endl;
    m_odometry.ResetPosition(pose, GetYaw());
    // std::cout << "Heading after reset: " << m_odometry.GetPose().Rotation().Radians().value() << std::endl;

    // m_poseEstimator.ResetPosition(pose, GetYaw());

#ifndef __FRC_ROBORIO__
    // Simulator
    // Reset the pose of the robot but do not reset the Simulated IMU.
    m_drivetrainSimulator.SetPose({pose.Translation(), GetYaw()});
#endif // __FRC_ROBORIO__
}

units::meters_per_second_t Drivetrain::GetVelocity()
{
    units::meters_per_second_t left = (m_driveL0.GetSelectedSensorVelocity() * kDPP * 10.0) / units::second_t{1.0};
    units::meters_per_second_t right = (m_driveR0.GetSelectedSensorVelocity() * kDPP * 10.0) / units::second_t{1.0};

    return ((left + right) / 2.0);
}

frc::Rotation2d Drivetrain::GetYaw()
{
    // ctre::phoenix::sensors::PigeonIMU::FusionStatus status;
    //  this can probably be reverted, but see TODO
    return m_imu.GetRotation2d();
    // double heading = status.heading;

    // std::cout << status.bIsFusing << std::endl;
    // std::cout << status.bIsValid << std::endl;
    // std::cout << status.description << std::endl;
    // std::cout << status.lastError << std::endl;

    // TODO(Dereck): Verify the sensor data is valid...
    // return m_imu.GetRotation2d();
}

bool Drivetrain::TurnRel(double forward, units::degree_t target, units::degree_t tolerance)
{
    // TODO: Settle Time?

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
#ifndef __FRC_ROBORIO__
    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    m_drivetrainSimulator.SetInputs(units::volt_t{m_driveL0.GetMotorOutputVoltage()},
                                    units::volt_t{m_driveR0.GetMotorOutputVoltage()});
    m_drivetrainSimulator.Update(20_ms);

    // TODO(Dereck): I don't trust this yet...
    // auto vbatt = frc::RobotController::GetInputVoltage() + ( m_drivetrainSimulator.GetCurrentDraw().value() * kBatteryResistance);

    // Left Motors
    auto left_sim = m_driveL0.GetSimCollection();
    auto left_pos = m_drivetrainSimulator.GetLeftPosition().to<double>();
    auto left_vel = m_drivetrainSimulator.GetLeftVelocity().to<double>();
    // left_sim.SetIntegratedSensorRawPosition(left_pos / kDPP.value());
    left_sim.SetIntegratedSensorVelocity(left_vel / kDPP.value() / 10);
    m_driveL0.SetSelectedSensorPosition(left_pos / kDPP.value());
    // left_sim.SetBusVoltage(vbatt);

    // Right Motors
    auto right_sim = m_driveR0.GetSimCollection();
    auto right_pos = m_drivetrainSimulator.GetRightPosition().to<double>();
    auto right_vel = m_drivetrainSimulator.GetRightVelocity().to<double>();
    // right_sim.SetIntegratedSensorRawPosition(right_pos / kDPP.value());
    right_sim.SetIntegratedSensorVelocity(right_vel / kDPP.value() / 10);
    m_driveR0.SetSelectedSensorPosition(right_pos / kDPP.value());
    // right_sim.SetBusVoltage(vbatt);

    // IMU
    // TODO(Dereck): CHECK DIRECTION WRT REAL SENSOR
    auto imu_sim = m_imu.GetSimCollection();
    imu_sim.SetRawHeading(m_drivetrainSimulator.GetHeading().Degrees().to<double>());
#endif // __FRC_ROBORIO__
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

frc::Field2d &Drivetrain::GetField()
{
    return m_fieldSim;
}

void Drivetrain::UpdateTelemetry() {}

void Drivetrain::ConfigureSystem() {}

void Drivetrain::InitSendable(wpi::SendableBuilder &builder)
{
    builder.SetSmartDashboardType("DriveBase");
    builder.SetActuator(true);

    // Commands
    builder.AddDoubleProperty(
        "cmd/fwd", [this]
        { return cmd_fwd; },
        nullptr);
    builder.AddDoubleProperty(
        "cmd/rot", [this]
        { return cmd_rot; },
        nullptr);
    builder.AddDoubleProperty(
        "cmd/vx", [this]
        { return cmd_vx; },
        nullptr);
    builder.AddDoubleProperty(
        "cmd/vw", [this]
        { return cmd_vw; },
        nullptr);
    builder.AddDoubleProperty(
        "cmd/vl", [this]
        { return cmd_vl; },
        nullptr);
    builder.AddDoubleProperty(
        "cmd/vr", [this]
        { return cmd_vr; },
        nullptr);

    // Left Motors
    builder.AddDoubleProperty(
        "left/percent", [this]
        { return m_driveL0.Get(); },
        nullptr);
    builder.AddDoubleProperty(
        "left/voltage", [this]
        { return m_driveL0.GetMotorOutputVoltage(); },
        nullptr);
    builder.AddDoubleProperty(
        "left/position", [this]
        { return m_driveL0.GetSelectedSensorPosition() * kDPP.value(); },
        nullptr);
    builder.AddDoubleProperty(
        "left/velocity", [this]
        { return m_driveL0.GetSelectedSensorVelocity() * kDPP.value() * 10; },
        nullptr);

    // Right Motors
    builder.AddDoubleProperty(
        "right/percent", [this]
        { return m_driveR0.Get(); },
        nullptr);
    builder.AddDoubleProperty(
        "right/voltage", [this]
        { return m_driveR0.GetMotorOutputVoltage(); },
        nullptr);
    builder.AddDoubleProperty(
        "right/position", [this]
        { return m_driveR0.GetSelectedSensorPosition() * kDPP.value(); },
        nullptr);
    builder.AddDoubleProperty(
        "right/velocity", [this]
        { return m_driveR0.GetSelectedSensorVelocity() * kDPP.value() * 10; },
        nullptr);

    // IMU
    // doing this because I want yaw within -pi, pi
    builder.AddDoubleProperty(
        "IMU/yaw", [this]
        { 
            auto yaw =  GetYaw();
            return frc::Rotation2d(yaw.Cos(), yaw.Sin()).Radians().value(); },
        nullptr);

    // Pose
    builder.AddDoubleProperty(
        "pose/x", [this]
        { return GetPose().X().value(); },
        nullptr);
    builder.AddDoubleProperty(
        "pose/y", [this]
        { return GetPose().Y().value(); },
        nullptr);
    builder.AddDoubleProperty(
        "pose/yaw", [this]
        { return GetPose().Rotation().Radians().value(); },
        nullptr);

    // Yaw PID
    builder.AddDoubleProperty(
        "YawPID/kP", [this]
        { return m_yawPID.GetP(); },
        [this](double value)
        { m_yawPID.SetP(value); });
    builder.AddDoubleProperty(
        "YawPID/kI", [this]
        { return m_yawPID.GetI(); },
        [this](double value)
        { m_yawPID.SetI(value); });
    builder.AddDoubleProperty(
        "YawPID/kD", [this]
        { return m_yawPID.GetD(); },
        [this](double value)
        { m_yawPID.SetD(value); });
    builder.AddDoubleProperty(
        "YawPID/SP",
        [this]
        { return units::degree_t(m_yawPID.GetSetpoint()).value(); },
        nullptr);

    builder.AddDoubleProperty(
        "DrivePID/kP", [this]
        { return m_leftPIDController.GetP(); },
        [this](double value)
        { m_leftPIDController.SetP(value);
          m_rightPIDController.SetP(value); });
    builder.AddDoubleProperty(
        "DrivePID/kI", [this]
        { return m_leftPIDController.GetI(); },
        [this](double value)
        { m_leftPIDController.SetI(value);
          m_rightPIDController.SetI(value); });
    builder.AddDoubleProperty(
        "DrivePID/kD", [this]
        { return m_leftPIDController.GetD(); },
        [this](double value)
        { m_leftPIDController.SetD(value);
          m_rightPIDController.SetD(value); });

    builder.AddDoubleProperty("traj/t", [this] { return reference.t.value(); }, nullptr);
    builder.AddDoubleProperty("traj/x", [this] { return reference.pose.Translation().X().value(); }, nullptr);
    builder.AddDoubleProperty("traj/y", [this] { return reference.pose.Translation().Y().value(); }, nullptr);
    builder.AddDoubleProperty("traj/theta", [this] { return reference.pose.Rotation().Radians().value(); }, nullptr);
    builder.AddDoubleProperty("traj/k", [this] { return reference.curvature.value(); }, nullptr);
    builder.AddDoubleProperty("traj/v", [this] { return reference.velocity.value(); }, nullptr);
    builder.AddDoubleProperty("traj/a", [this] { return reference.acceleration.value(); }, nullptr);

}

void Drivetrain::RegisterDataEntries(wpi::log::DataLog &log)
{
    RegisterDataEntry(log, "Drive/cmd/fwd", "double");
    RegisterDataEntry(log, "Drive/cmd/rot", "double");
    RegisterDataEntry(log, "Drive/cmd/vx", "double");
    RegisterDataEntry(log, "Drive/cmd/vw", "double");
    RegisterDataEntry(log, "Drive/cmd/vl", "double");
    RegisterDataEntry(log, "Drive/cmd/vr", "double");

    FalconEntryStartHelper(log, "Drive/left1");
    FalconEntryStartHelper(log, "Drive/left2");
    FalconEntryStartHelper(log, "Drive/left3");
    FalconEntryStartHelper(log, "Drive/right1");
    FalconEntryStartHelper(log, "Drive/right2");
    FalconEntryStartHelper(log, "Drive/right3");

    RegisterDataEntry(log, "Drive/IMU/yaw", "double");
    RegisterDataEntry(log, "Drive/pose/x", "double");
    RegisterDataEntry(log, "Drive/pose/y", "double");
    RegisterDataEntry(log, "Drive/pose/yaw", "double");

    RegisterDataEntry(log, "Drive/YawPID/kP", "double");
    RegisterDataEntry(log, "Drive/YawPID/kI", "double");
    RegisterDataEntry(log, "Drive/YawPID/kD", "double");
    RegisterDataEntry(log, "Drive/YawPID/kSP", "double");

    RegisterDataEntry(log, "Drive/DrivePID/kP", "double");
    RegisterDataEntry(log, "Drive/DrivePID/kI", "double");
    RegisterDataEntry(log, "Drive/DrivePID/kD", "double");

    RegisterDataEntry(log, "Drive/traj/t", "double");
    RegisterDataEntry(log, "Drive/traj/x", "double");
    RegisterDataEntry(log, "Drive/traj/y", "double");
    RegisterDataEntry(log, "Drive/traj/theta", "double");
    RegisterDataEntry(log, "Drive/traj/k", "double");
    RegisterDataEntry(log, "Drive/traj/v", "double");
    RegisterDataEntry(log, "Drive/traj/a", "double");
}

void Drivetrain::LogDataEntries(wpi::log::DataLog &log)
{
    log.AppendDouble(GetDataEntry("Drive/cmd/fwd"), cmd_fwd, 0);
    log.AppendDouble(GetDataEntry("Drive/cmd/rot"), cmd_rot, 0);

    log.AppendDouble(GetDataEntry("Drive/cmd/vx"), cmd_vx, 0);
    log.AppendDouble(GetDataEntry("Drive/cmd/vw"), cmd_vw, 0);

    log.AppendDouble(GetDataEntry("Drive/cmd/vl"), cmd_vl, 0);
    log.AppendDouble(GetDataEntry("Drive/cmd/vr"), cmd_vr, 0);

    FalconEntryHelper(log, m_driveL0, "Drive/left1");
    FalconEntryHelper(log, m_driveL1, "Drive/left2");
    FalconEntryHelper(log, m_driveL2, "Drive/left3");
    FalconEntryHelper(log, m_driveR0, "Drive/right1");
    FalconEntryHelper(log, m_driveR1, "Drive/right2");
    FalconEntryHelper(log, m_driveR2, "Drive/right3");

    auto yaw =  GetYaw();
    log.AppendDouble(GetDataEntry("Drive/IMU/yaw"), frc::Rotation2d(yaw.Cos(), yaw.Sin()).Radians().value(), 0);

    auto pose = GetPose();
    log.AppendDouble(GetDataEntry("Drive/pose/x"), pose.X().value(), 0);
    log.AppendDouble(GetDataEntry("Drive/pose/y"), pose.Y().value(), 0);
    log.AppendDouble(GetDataEntry("Drive/pose/yaw"), pose.Rotation().Radians().value(), 0);

    log.AppendDouble(GetDataEntry("Drive/YawPID/kP"), m_yawPID.GetP(), 0);
    log.AppendDouble(GetDataEntry("Drive/YawPID/kI"), m_yawPID.GetI(), 0);
    log.AppendDouble(GetDataEntry("Drive/YawPID/kD"), m_yawPID.GetD(), 0);
    log.AppendDouble(GetDataEntry("Drive/YawPID/kSP"), m_yawPID.GetSetpoint(), 0);

    log.AppendDouble(GetDataEntry("Drive/DrivePID/kP"), m_leftPIDController.GetP(), 0);
    log.AppendDouble(GetDataEntry("Drive/DrivePID/kI"), m_leftPIDController.GetI(), 0);
    log.AppendDouble(GetDataEntry("Drive/DrivePID/kD"), m_leftPIDController.GetD(), 0);

    log.AppendDouble(GetDataEntry("Drive/traj/t"), reference.t.value(), 0);
    log.AppendDouble(GetDataEntry("Drive/traj/x"), reference.pose.X().value(), 0);
    log.AppendDouble(GetDataEntry("Drive/traj/y"), reference.pose.Y().value(), 0);
    log.AppendDouble(GetDataEntry("Drive/traj/theta"), reference.pose.Rotation().Radians().value(), 0);
    log.AppendDouble(GetDataEntry("Drive/traj/k"), reference.curvature.value(), 0);
    log.AppendDouble(GetDataEntry("Drive/traj/v"), reference.velocity.value(), 0);
    log.AppendDouble(GetDataEntry("Drive/traj/a"), reference.acceleration.value(), 0);
}