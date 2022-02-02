// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/AnalogGyro.h>
#include <frc/Encoder.h>
#include <frc/motorcontrol/PWMVictorSPX.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/simulation/AnalogGyroSim.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/system/plant/LinearSystemId.h>
// #include <frc/ADIS16470_IMU.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/numbers>
#include <math.h>

#include "ctre/Phoenix.h"
#include "Subsystem.hpp"

/**
 * Represents a differential drive style drivetrain.
 */
class Drivetrain : public Subsystem
{
public:
    Drivetrain(bool isSimulation)
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

    void ConfigureSystem();
    void SetSpeeds(const frc::DifferentialDriveWheelSpeeds &speeds);
    void Drive(units::meters_per_second_t xSpeed,
               units::radians_per_second_t rot);
    void Arcade(double forward, double rotate);
    void UpdateOdometry();
    void UpdateTelemetry();
    void ResetOdometry(const frc::Pose2d &pose);
    frc::Rotation2d GetYaw();

    bool TurnRel(double forward, units::degree_t target, units::degree_t tolerance);

    frc::Pose2d GetPose() const { return m_odometry.GetPose(); }

    void SimulationPeriodic();
    void Periodic();
    void SetBrakeMode();
    void SetCoastMode();

private:
    /***************************************************************************/
    // CrossFire Characterization Values

    static constexpr units::meter_t kTrackWidth = 0.579_m; //.579
    static constexpr units::meter_t kWheelRadius = 3.0_in;
    static constexpr units::meter_t empiricalDist = 210_in;
    static constexpr double kGearRatio = 10.71;
    static constexpr int kEncoderResolution = 2048;
    static constexpr int kMotorCount = 2;

    decltype(1_V) kStatic{0.59481};                    //.706
    decltype(1_V / 1_mps) kVlinear{2.4226};            // 1.86
    decltype(1_V / 1_mps_sq) kAlinear{0.34258};       // 0.0917
    decltype(1_V / 1_rad_per_s) kVangular{1.96};     // 1.94
    decltype(1_V / 1_rad_per_s_sq) kAangular{0.077}; // 0.0716

    // Velocity Control PID (Is this really required ???)
    frc2::PIDController m_leftPIDController{0.0, 0.0, 0.0}; // 2.75
    frc2::PIDController m_rightPIDController{0.0, 0.0, 0.0};

public:
    // Teleop Values
    /// TODO(Dereck): Measure these too
    static constexpr units::feet_per_second_t kMaxSpeed{20.0};
    static constexpr units::degrees_per_second_t kMaxAngularSpeed{720.0};

    // WPI_TalonFX impel{6};
    // WPI_TalonFX impel2{7};

    frc::DifferentialDriveKinematics GetKinematics()
    {
        return m_kinematics;
    }

    frc::SimpleMotorFeedforward<units::meter> GetFeedForward()
    {
        return m_feedforward;
    }

    /***************************************************************************/

private:
    bool m_isSimulation = false;

    // Simulation motor controllers
    frc::PWMVictorSPX m_leftLeader{1};
    frc::PWMVictorSPX m_leftFollower{2};
    frc::PWMVictorSPX m_rightLeader{3};
    frc::PWMVictorSPX m_rightFollower{4};

    // Real motor Controllers
    WPI_TalonFX m_driveL0{0};
    WPI_TalonFX m_driveL1{1};
    WPI_TalonFX m_driveL2{2};
    WPI_TalonFX m_driveR0{3};
    WPI_TalonFX m_driveR1{4};
    WPI_TalonFX m_driveR2{5};

    // Controller Groups
    frc::MotorControllerGroup m_leftGroup{
        m_leftLeader,
        m_leftFollower,
        m_driveL0,
        m_driveL1,
        m_driveL2};

    frc::MotorControllerGroup m_rightGroup{
        m_rightLeader,
        m_rightFollower,
        m_driveR0,
        m_driveR1,
        m_driveR2};

    // Simulated Encoders
    frc::Encoder m_leftEncoder{0, 1};
    frc::Encoder m_rightEncoder{2, 3};

    // frc::ADIS16470_IMU m_imu{
    //     frc::ADIS16470_IMU::IMUAxis::kZ, //kZ
    //     frc::SPI::Port::kOnboardCS0,
    //     frc::ADIS16470_IMU::CalibrationTime::_2s};

    PigeonIMU m_imu{30};

    //
    // Dynamics
    //
    frc::DifferentialDriveKinematics m_kinematics{kTrackWidth};
    frc::DifferentialDriveOdometry m_odometry{frc::Rotation2d()};
    frc::SimpleMotorFeedforward<units::meters> m_feedforward{kStatic, kVlinear, kAlinear};

    //
    // Simulation
    //
    frc::LinearSystem<2, 2, 2> m_drivetrainSystem =
        frc::LinearSystemId::IdentifyDrivetrainSystem(
            kVlinear, kAlinear,
            kVangular, kAangular, kTrackWidth);

    frc::sim::DifferentialDrivetrainSim m_drivetrainSimulator{
        m_drivetrainSystem,
        kTrackWidth,
        frc::DCMotor::Falcon500(kMotorCount),
        kGearRatio,
        kWheelRadius};

    frc::AnalogGyro m_gyro{0};
    frc::sim::AnalogGyroSim m_gyroSim{m_gyro};
    frc::sim::EncoderSim m_leftEncoderSim{m_leftEncoder};
    frc::sim::EncoderSim m_rightEncoderSim{m_rightEncoder};
    frc::Field2d m_fieldSim;

    frc::PIDController yawController{0.25, 0.0, 0.05};
};
