#pragma once

// Units
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/velocity.h>
#include <units/current.h>
#include <units/acceleration.h>
#include <units/temperature.h>
#include <units/time.h>
#include <units/constants.h>

// Utilities
#include <cmath>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/Timer.h>
#include <frc/Preferences.h>
#include <string.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <iostream>

// Simulation
#include <frc/system/plant/LinearSystemId.h>
#include <frc/simulation/LinearSystemSim.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/simulation/ElevatorSim.h>

#include <subsystems/Subsystem.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableHelper.h>

#include <ctre/Phoenix.h>
#include <limits>
#include <lib/SwerveWheelSim.h>
#include <frc/simulation/DCMotorSim.h>
#include <lib/SwerveModuleConfig.h>

class SwerveModule : public Subsystem, 
                     public wpi::Sendable
{
public:
    SwerveModule(std::string moduleID, int driveMotorChannel, int turningMotorChannel, int turningEncoderChannel, SwerveModuleConfig config);
    SwerveModule() = delete; // Removes default constructor because why tf u using the default constructor, my G. SMH

    // Init Stuff
    void ConfigureSystem() override;

    // Telemetry
    void RegisterDataEntries(wpi::log::DataLog &log);
    void LogDataEntries(wpi::log::DataLog &log);
    void UpdateTelemetry() override;

    // Odometry
    frc::SwerveModuleState GetState();
    units::meters_per_second_t GetVelocity();
    frc::Rotation2d GetAngle();

    // Module Actions
    void SetModule(const frc::SwerveModuleState &state);
    void Stop();

    // Telemetry / Smartdash
    void InitSendable(wpi::SendableBuilder &builder) override;

    // Simulation
    units::ampere_t SimPeriodic(units::volt_t volts);

private:
    frc::SwerveModuleState currentState;

    frc::SwerveModuleState targetState;

    std::string moduleID;

    // Hardware
    ctre::phoenix::motorcontrol::can::WPI_TalonFX m_driveMotor;
    ctre::phoenix::motorcontrol::can::WPI_TalonFX m_turningMotor;
    CANCoder turningEncAbs;

    // Configuration
    static constexpr auto kWheelRadius = 2.0_in;
    static constexpr int kEncoderResolution = 2048;
    static constexpr double kDriveGearboxRatio = 6.75;
    static constexpr double kTurnGearboxRatio = 12.8;

    static constexpr auto kDriveScaleFactor =
        (2 * units::constants::pi * kWheelRadius) / (kDriveGearboxRatio * kEncoderResolution);

    static constexpr auto kTurningMotorVoltageNominal = 12.8_V;

    static constexpr auto kDriveMotorCurrentLimit = 55_A;
    static constexpr auto kTurningMotorCurrentLimit = 30_A;

    // Control
    frc::ProfiledPIDController<units::meters_per_second> m_drivePIDController;
    frc::ProfiledPIDController<units::radians> m_turningPIDController;

    frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward;
    frc::SimpleMotorFeedforward<units::radians> m_turnFeedforward;

    units::degree_t angle_offset;


    //
    // Simulation
    //
    units::volt_t m_driveVolts = 0_V;
    units::volt_t m_turnVolts = 0_V;

    // Drive
    frc::LinearSystem<1, 1, 1> m_drivePlant =
    frc::LinearSystemId::IdentifyVelocitySystem<units::meter>(
        m_driveFeedforward.kV, 
        m_driveFeedforward.kA);

    SwerveWheelSim m_driveSim{
        m_drivePlant,
        frc::DCMotor::Falcon500(),
        kDriveGearboxRatio,
        kWheelRadius,
        {0.1}};

    ctre::phoenix::motorcontrol::TalonFXSimCollection m_driveMotorSim = m_driveMotor.GetSimCollection();
    ctre::phoenix::motorcontrol::TalonFXSimCollection m_turnMotorSim = m_turningMotor.GetSimCollection();
    ctre::phoenix::sensors::CANCoderSimCollection m_encoderSim = turningEncAbs.GetSimCollection();

    // Turn
    // frc::LinearSystem<2, 1, 1> m_turnPlant =
    // frc::LinearSystemId::IdentifyPositionSystem<units::radian>(
    //     m_turnFeedforward.kV, 
    //     m_turnFeedforward.kA);

    // derived based on implementation of DCMotorSystem and PositionSystem
    /*
        1/kA = G * motor.Kt / (motor.R * J)
        -kV / kA = -1 * G * G * motor.Kt / (motor.R * J)
        -kV = -G # skip

        1 / kA = G * motor.Kt / (motor.R * J)
        J * motor.R = G * motor.kT * kA
        J = G * motor.Kt * kA / motor.R

        kA = 0.008_V / 1_rad_per_s_sq
        kV = 0.12_V / 1_rad_per_s
        G = 12.8
        motor.Kt = stallTorque / stallCurrent
        motor.kV = freeSpeed / (nominalVoltage - R * freeCurrent)
        motor.R = nominalVoltage / stallCurrent

        args:
        - nominalVoltage = 12_V
        - stallTorque = 4.69_Nm
        - stallCurrent = 257_A
        - freeCurrent = 1.5_A
        - freeSpeed = 6380
        - numMotors = 1

        motor.R = 12_V / 257_A
        motor.kV = 6380 / (12_V - motor.R * 1.5_A)
        motor.Kt = 4.69_Nm / 257_A

        J = G * motor.Kt * kA / motor.R
        J = 12.8 * 4.69_Nm / 257_A * 0.008_V / 1_rad_per_s_sq / (12_V / 257_A)
        = 12.8 * 4.69_Nm * 0.008_V * 257_A / 257_A / 1_rad_per_s_sq / 12_V
        = 12.8 * 4.69_Nm * 0.008_V / (12_V * 1_rad_per_s_sq) 
    */

    frc::LinearSystem<2, 1, 2> m_turnPlant =
    frc::LinearSystemId::DCMotorSystem(
        frc::DCMotor::Falcon500(),
        12.8 * 4.69_Nm * 0.008_V / (12_V * 1_rad_per_s_sq) * 1_rad,
        kTurnGearboxRatio);


    frc::sim::DCMotorSim m_turnSim{
        m_turnPlant,
        frc::DCMotor::Falcon500(),
        kTurnGearboxRatio,
        {0.01}};
    
    units::ampere_t SimDrive(units::volt_t battery);
    units::ampere_t SimTurn(units::volt_t battery);
};