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
    Drivetrain(bool isSimulation);
    
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
    // Characterization Values

    static constexpr units::meter_t kTrackWidth = 0.579_m;
    static constexpr units::meter_t kWheelRadius = 2.0_in;

    static constexpr double kGearRatio = 8.48;
    static constexpr int kEncoderResolution = 2048;
    static constexpr int kMotorCount = 3; // Per gearbox

    decltype(1_V) kStatic{0.59481};              
    decltype(1_V / 1_mps) kVlinear{2.4226};         
    decltype(1_V / 1_mps_sq) kAlinear{0.34258};      
    decltype(1_V / 1_rad_per_s) kVangular{1.96};    
    decltype(1_V / 1_rad_per_s_sq) kAangular{0.077};

    // Velocity Control PID (Is this really required ???)
    frc2::PIDController m_leftPIDController{0.0, 0.0, 0.0};
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
