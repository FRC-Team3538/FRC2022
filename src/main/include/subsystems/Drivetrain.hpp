// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/PIDController.h>                   // for PIDContro...
#include <frc/controller/RamseteController.h>               // for RamseteCo...
#include <frc/controller/SimpleMotorFeedforward.h>          // for SimpleMot...
#include <frc/kinematics/DifferentialDriveKinematics.h>     // for Different...
#include <frc/kinematics/DifferentialDriveOdometry.h>       // for Different...
#include <frc/motorcontrol/MotorControllerGroup.h>          // for MotorCont...
#include <frc/simulation/DifferentialDrivetrainSim.h>       // for Different...
#include <frc/smartdashboard/Field2d.h>                     // for Field2d
#include <frc/smartdashboard/SmartDashboard.h>              // for SmartDash...
#include <frc/system/plant/LinearSystemId.h>                // for LinearSys...
#include <frc/trajectory/Trajectory.h>                      // for Trajectory
#include <math.h>                                           // for M_PI
#include <units/angle.h>                                    // for degree_t
#include <units/angular_velocity.h>                         // for radians_p...
#include <units/length.h>                                   // for meter_t
#include <units/velocity.h>                                 // for meters_pe...
#include <units/voltage.h>                                  // for volt_t
#include <wpi/sendable/SendableHelper.h>                    // for SendableH...

#include "Subsystem.hpp"                                    // for Subsystem
#include "frc/geometry/Pose2d.h"                            // for Pose2d
#include "frc/geometry/Rotation2d.h"                        // for Rotation2d
#include "frc/motorcontrol/MotorControllerGroup.inc"        // for MotorCont...
#include "frc/system/LinearSystem.h"                        // for LinearSystem
#include "frc/system/plant/DCMotor.h"                       // for DCMotor
#include "networktables/NetworkTableEntry.h"                // for NetworkTa...
#include "units/acceleration.h"                             // for meters_pe...
#include "units/angular_acceleration.h"                     // for radians_p...
#include "units/base.h"                                     // for unit_t
#include "units/time.h"                                     // for second_t
#include "wpi/sendable/Sendable.h"                          // for Sendable
#include "ctre/phoenix/motorcontrol/IMotorController.h"
#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"
#include "ctre/phoenix/sensors/SensorVelocityMeasPeriod.h"
#include "ctre/phoenix/sensors/WPI_Pigeon2.h"

namespace frc {
class FieldObject2d;
struct DifferentialDriveWheelSpeeds;
}  // namespace frc
namespace wpi {
class SendableBuilder;
}  // namespace wpi

using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::sensors;

// leave this for wandows
#ifndef M_PI
    #define M_PI    3.14159265358979323846
#endif

/**
 * Represents a differential drive style drivetrain.
 */
class Drivetrain : public Subsystem,
                   public wpi::Sendable,
                   public wpi::SendableHelper<Drivetrain>
{
public:
    Drivetrain();

    void InitSendable(wpi::SendableBuilder &builder) override;

    void ConfigureSystem() override;

    void Arcade(double forward, double rotate);
    void SetSpeeds(const frc::DifferentialDriveWheelSpeeds &speeds);
    void Drive(units::meters_per_second_t xSpeed,
               units::radians_per_second_t rot);
    void Drive(frc::ChassisSpeeds &speeds);
    bool TurnRel(double forward, units::degree_t target, units::degree_t tolerance);
    void Drive(const frc::Trajectory::State& target);

    void UpdateOdometryWithGlobalEstimate(frc::Pose2d globalEstimate, units::second_t estimateTime);
    void UpdateOdometry();
    void ResetOdometry(const frc::Pose2d &pose);
    frc::Rotation2d GetYaw();
    frc::Pose2d GetPose() const;
    frc::DifferentialDriveKinematics GetKinematics() { return m_kinematics; }
    frc::SimpleMotorFeedforward<units::meter> GetFeedForward() { return m_feedforward; }
    units::meters_per_second_t GetVelocity();

    void SimulationPeriodic();
    void Periodic();
    void UpdateTelemetry() override;
    void SetBrakeMode();
    void SetCoastMode();

    frc::Field2d& GetField();

private:
    /***************************************************************************/
    // Characterization Values

    static constexpr units::meter_t kTrackWidth = 0.78107_m;
    static constexpr units::meter_t kWheelRadius = 1.95_in;

    static constexpr double kGearRatio = (60.0 / 11.0) * (54.0 / 38.0);
    static constexpr int kEncoderResolution = 2048;
    static constexpr int kMotorCount = 3; // Per gearbox

    // Made these slightly more obscure to support use in calculating kMaxSpeedLinear/Angular at compile time.
    static constexpr auto kMaxVoltage = 12.0_V;
    static constexpr auto kStatic = 0.68798_V;
    static constexpr auto kVlinear = 2.643_V / 1_mps;
    static constexpr auto kAlinear = 0.16209_V / 1_mps_sq;
    static constexpr auto kVangular = 2.7351_V / 1_rad_per_s;
    static constexpr auto kAangular = 0.035228_V / 1_rad_per_s_sq;

    // Velocity Control PID (Is this really required ???)
    // these are updated w/ the latest sysid data, BUT are untested
    frc2::PIDController m_leftPIDController{5.7845, 0.0, 0.0}; //{0.89223, 0.0, 0.0};
    frc2::PIDController m_rightPIDController{5.7845, 0.0, 0.0}; //{0.89223, 0.0, 0.0};

    // Average Battery Resistance (Simulation)
    static constexpr auto kBatteryResistance = 0.03;

    // Distance Per Pulse (Scale Factor)
    static constexpr auto kDPP = (2 * kWheelRadius * M_PI) / kGearRatio / kEncoderResolution; 

public:
    // Teleop Values
    // CHECK: calculated based on sysid characterization
    static constexpr units::meters_per_second_t kMaxSpeedLinear = (kMaxVoltage - kStatic) / kVlinear;
    static constexpr units::radians_per_second_t kMaxSpeedAngular = (kMaxVoltage - kStatic) / kVangular;


    /***************************************************************************/

private:
    // Real motor Controllers
    WPI_TalonFX m_driveL0{0};
    WPI_TalonFX m_driveL1{1};
    WPI_TalonFX m_driveL2{2};
    WPI_TalonFX m_driveR0{3};
    WPI_TalonFX m_driveR1{4};
    WPI_TalonFX m_driveR2{5};

    // Controller Groups
    frc::MotorControllerGroup m_leftGroup{
        m_driveL0,
        m_driveL1,
        m_driveL2
    };

    frc::MotorControllerGroup m_rightGroup{
        m_driveR0,
        m_driveR1,
        m_driveR2
    };


    WPI_Pigeon2 m_imu{30};

    //
    // Dynamics
    //
    frc::DifferentialDriveKinematics m_kinematics{kTrackWidth};
    frc::DifferentialDriveOdometry m_odometry{GetYaw()};
    // frc::DifferentialDrivePoseEstimator m_poseEstimator{
    //     GetYaw(), 
    //     frc::Pose2d{}, 
    //     {0.002, 0.002, 0.0001, 0.5, 0.5},
    //     {0.005, 0.005, 0.0001},
    //     {0.2, 0.2, 0.1}};
    frc::SimpleMotorFeedforward<units::meters> m_feedforward{kStatic, kVlinear, kAlinear};
    frc::RamseteController m_ramsete{units::unit_t<frc::RamseteController::b_unit>{2.0},
                                     units::unit_t<frc::RamseteController::zeta_unit>{0.7}};


    //
    // Telemetry
    //
    double cmd_fwd = 0.0;
    double cmd_rot = 0.0;
    double cmd_vx = 0.0;
    double cmd_vw = 0.0;
    double cmd_vl = 0.0;
    double cmd_vr = 0.0;

#ifndef __FRC_ROBORIO__
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
#endif // __FRC_ROBORIO__

    frc::Field2d m_fieldSim;
    frc::FieldObject2d *referencePose;

    frc::PIDController m_yawPID{0.25, 0.0, 0.05};

    frc::Trajectory::State reference;

    nt::NetworkTableEntry localization_flag_entry = frc::SmartDashboard::GetEntry("flags/alternate_localization");
};
