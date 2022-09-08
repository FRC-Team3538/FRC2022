#pragma once

// Lower Level Robot Stuffs
#include "subsystems/SwerveModule.h"

// Utilities
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/controller/HolonomicDriveController.h>
#include "Subsystem.h"
#include <frc/geometry/Translation2d.h>
#include <cmath>
#include <frc/smartdashboard/SendableChooser.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableHelper.h>
#include <lib/AlternativeSwerveKinematics.h>
#include <units/angular_acceleration.h>

class Drivetrain : public Subsystem, 
                   public wpi::Sendable
{
public:
    Drivetrain();

    // Init Stuff
    void ConfigureSystem();

    // Telemetry
    void UpdateTelemetry();
    void RegisterDataEntries(wpi::log::DataLog &log);
    void LogDataEntries(wpi::log::DataLog &log);

    // Setters
    void Drive(units::meters_per_second_t xSpeed,
               units::meters_per_second_t ySpeed,
               units::radians_per_second_t rot,
               bool fieldRelative = true);
    void Drive(frc::Trajectory::State trajectoryState, units::radian_t yaw = 0_rad);
    void ResetYaw();
    void ResetOdometry(const frc::Pose2d &pose);
    void UpdateOdometry();
    void Stop();

    // Getters
    frc::Rotation2d GetYaw();
    frc::ChassisSpeeds GetChassisSpeeds();

    // Telemetry / Smartdash
    void InitSendable(wpi::SendableBuilder &builder) override;

    // Simulation
    units::ampere_t SimPeriodic(units::volt_t volts);

    // Public config values
    static constexpr units::meters_per_second_t kMaxSpeedLinear = 16_fps;
    static constexpr units::radians_per_second_t kMaxSpeedAngular = 360_deg_per_s;
    static constexpr units::meters_per_second_squared_t kMaxAccelerationLinear = units::feet_per_second_squared_t(20.0);
    static constexpr units::inch_t kWheelToWheel = 22_in;

private:
    bool m_fieldRelative;

    // Configuration
    static constexpr auto dist = kWheelToWheel / 2;
    frc::Translation2d frontLeftLocation{+dist, +dist};
    frc::Translation2d frontRightLocation{+dist, -dist};
    frc::Translation2d backLeftLocation{-dist, +dist};
    frc::Translation2d backRightLocation{-dist, -dist};

    
    ctre::phoenix::sensors::WPI_Pigeon2 m_imu{19};
    ctre::phoenix::sensors::BasePigeonSimCollection m_imuSimCollection = m_imu.GetSimCollection();

    // Odomoetry
    frc::Field2d m_fieldDisplay;

    // Control
    frc::ChassisSpeeds m_command;

    static constexpr auto kMaxModuleLinearAcceleration = 80.0_mps_sq;
    static constexpr auto kMaxModuleLinearJerk = 200.0_mps_sq / 1_s;

    static constexpr auto kMaxModuleAngularVelocity = 18_rad_per_s;
    static constexpr auto kMaxModuleAngularAcceleration = 200_rad_per_s_sq;

    SwerveModuleConfig m_frontLeftConfig{
        -123.135_deg,
        {
            1.89,
            0.0,
            0.0,
            {
                kMaxModuleLinearAcceleration,
                kMaxModuleLinearJerk
            }
        },
        {
            1.44,   // 2.5179,
            0.0,    // 0.0,
            0.0125, // 0.15272,
            {
                kMaxModuleAngularVelocity,
                kMaxModuleAngularAcceleration
            }
        },
        {
            0.607_V,
            2.2_V / 1_mps,
            0.199_V / 1_mps_sq
        },
        {
            0.776_V,
            0.232_V / 1_rad_per_s,
            0.004_V / 1_rad_per_s_sq
        }
    };

    // SwerveModuleConfig m_frontRightConfig{
    //     75.938_deg,
    //     {
    //         1.89,
    //         0.0,
    //         0.0,
    //         {
    //             kMaxModuleLinearAcceleration,
    //             kMaxModuleLinearJerk
    //         }
    //     },
    //     {
    //         1.44,   // 2.5179,
    //         0.0,    // 0.0,
    //         0.0125, // 0.15272,
    //         {
    //             kMaxModuleAngularVelocity,
    //             kMaxModuleAngularAcceleration
    //         }
    //     },
    //     {
    //         0.607_V,
    //         2.2_V / 1_mps,
    //         0.199_V / 1_mps_sq
    //     },
    //     {
    //         0.1_V,
    //         0.12_V / 1_rad_per_s,
    //         0.008_V / 1_rad_per_s_sq
    //     }
    // };

    // SwerveModuleConfig m_backLeftConfig{
    //     -2.549_deg,
    //     {
    //         1.89,
    //         0.0,
    //         0.0,
    //         {
    //             kMaxModuleLinearAcceleration,
    //             kMaxModuleLinearJerk
    //         }
    //     },
    //     {
    //         1.44,   // 2.5179,
    //         0.0,    // 0.0,
    //         0.0125, // 0.15272,
    //         {
    //             kMaxModuleAngularVelocity,
    //             kMaxModuleAngularAcceleration
    //         }
    //     },
    //     {
    //         0.607_V,
    //         2.2_V / 1_mps,
    //         0.199_V / 1_mps_sq
    //     },
    //     {
    //         0.1_V,
    //         0.12_V / 1_rad_per_s,
    //         0.008_V / 1_rad_per_s_sq
    //     }
    // };

    // SwerveModuleConfig m_backRightConfig{
    //     128.848_deg,
    //     {
    //         1.89,
    //         0.0,
    //         0.0,
    //         {
    //             kMaxModuleLinearAcceleration,
    //             kMaxModuleLinearJerk
    //         }
    //     },
    //     {
    //         1.44,   // 2.5179,
    //         0.0,    // 0.0,
    //         0.0125, // 0.15272,
    //         {
    //             kMaxModuleAngularVelocity,
    //             kMaxModuleAngularAcceleration
    //         }
    //     },
    //     {
    //         0.607_V,
    //         2.2_V / 1_mps,
    //         0.199_V / 1_mps_sq
    //     },
    //     {
    //         0.1_V,
    //         0.12_V / 1_rad_per_s,
    //         0.008_V / 1_rad_per_s_sq
    //     }
    // };

    // Odometry
    frc::SwerveDriveKinematics<4> m_kinematics{
        frontLeftLocation,
        frontRightLocation,
        backLeftLocation,
        backRightLocation};
    
    // AlternativeSwerveKinematics<4> m_altKinematics{
    //     kMaxSpeedLinear,
    //     kMaxSpeedAngular,
    //     kMaxSpeedLinear
    // };

    // frc::SwerveDriveOdometry<4> m_odometry{
    //     m_kinematics,
    //     frc::Rotation2d(),
    //     frc::Pose2d()};

    // frc::SwerveDrivePoseEstimator<4> m_poseEstimator{
    //     frc::Rotation2d(),
    //     frc::Pose2d(),
    //     m_kinematics,
    //     {0.5, 0.5, 0.05},
    //     {2.0},
    //     {0.0, 0.0, 0.0}};

    frc::ChassisSpeeds m_robotVelocity;

    // Swerve Modules
    SwerveModule m_frontLeft{"FL", 0, 1, 20}; //, m_frontLeftConfig};
    // SwerveModule m_frontRight{"FR", 2, 3, 21, m_frontRightConfig};
    // SwerveModule m_backLeft{"BL", 4, 5, 22, m_backLeftConfig};
    // SwerveModule m_backRight{"BR", 6, 7, 23, m_backRightConfig};

    // Trajectory Following
    // frc::HolonomicDriveController m_trajectoryController{
    //     {2.0, 0.0, 0.0}, // X-error
    //     {2.0, 0.0, 0.0}, // Y-error
    //     {2.0, 0.0, 0.0, {360_deg_per_s, 720_deg_per_s_sq}}}; // Rotation error
};