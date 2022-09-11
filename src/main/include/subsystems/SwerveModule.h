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

#include <subsystems/Subsystem.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableHelper.h>

#include <ctre/Phoenix.h>
#include <limits>
#include <lib/SwerveWheelSim.h>
#include <frc/simulation/DCMotorSim.h>
#include <lib/SwerveModuleConfig.h>
#include <lib/TurretSim.h>

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
    frc::Rotation2d GetMotorAngle();

    // Module Actions
    void SetModule(const frc::SwerveModuleState &state, bool openLoop = false);
    void Stop();

    // Telemetry / Smartdash
    void InitSendable(wpi::SendableBuilder &builder) override;

    // Simulation
    void SimInit() override;
    units::ampere_t SimPeriodic(units::volt_t volts) override;

    ErrorCode SeedTurnMotor();

    units::radians_per_second_t AngularVelocity();

private:
    frc::SwerveModuleState currentState;

    frc::SwerveModuleState targetState;

    std::string moduleID;

    SwerveModuleConfig config;

    // Hardware
    ctre::phoenix::motorcontrol::can::WPI_TalonFX m_driveMotor;
    ctre::phoenix::motorcontrol::can::WPI_TalonFX m_turningMotor;
    CANCoder m_turnEncoder;

    // Configuration
    static constexpr auto kWheelRadius = 2.0_in;
    static constexpr int kEncoderResolution = 2048;
    static constexpr double kDriveGearboxRatio = 6.75;
    static constexpr double kTurnGearboxRatio = 12.8;

    static constexpr auto kDriveScaleFactor =
        (2 * units::constants::pi * kWheelRadius) / (1_tr * kDriveGearboxRatio);

    static constexpr auto kTurningMotorVoltageNominal = 12.8_V;

    static constexpr auto kDriveMotorCurrentLimit = 55_A;
    static constexpr auto kTurningMotorCurrentLimit = 30_A;

    // Control

    frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward;
    frc::SimpleMotorFeedforward<units::radians> m_turnFeedforward;

    units::degree_t angle_offset;

    SlotConfiguration driveSlotConfig = SlotConfiguration();
    SlotConfiguration turnSlotConfig = SlotConfiguration();

    //
    // Simulation
    //
    bool m_isSimulation = false;
    units::volt_t m_driveVolts = 0_V;
    units::volt_t m_turnVolts = 0_V;

    ctre::phoenix::motorcontrol::TalonFXSimCollection driveMotorSim = m_driveMotor.GetSimCollection();
    ctre::phoenix::motorcontrol::TalonFXSimCollection turningMotorSim = m_turningMotor.GetSimCollection();
    ctre::phoenix::sensors::CANCoderSimCollection encoderSim = m_turnEncoder.GetSimCollection();

    // Drive
    frc::LinearSystem<1, 1, 1> m_drivePlant =
    frc::LinearSystemId::IdentifyVelocitySystem<units::radian>(
        m_driveFeedforward.kV * kWheelRadius / 1_rad, 
        m_driveFeedforward.kA * kWheelRadius / 1_rad);

    SwerveWheelSim m_driveSim{
        m_drivePlant,
        frc::DCMotor::Falcon500(),
        kDriveGearboxRatio,
        kWheelRadius,
        {0.0}};

    // Turn
    frc::LinearSystem<2, 1, 1> m_turnPlant =
    frc::LinearSystemId::IdentifyPositionSystem<units::radian>(
        m_turnFeedforward.kV, 
        m_turnFeedforward.kA);

    
    TurretSim m_turnSim{
        m_turnPlant,
        frc::DCMotor::Falcon500(),
        kTurnGearboxRatio,
        {0.0}};

    units::ampere_t SimDrive(units::volt_t battery);
    units::ampere_t SimTurn(units::volt_t battery);

    void Drive(units::meters_per_second_t target, bool openLoop = false);
    void Turn(frc::Rotation2d target);
};