#pragma once

#include <units/angular_velocity.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/constants.h>

#include <wpi/numbers>

#include <frc/Solenoid.h>
#include <frc/controller/ProfiledPIDController.h>
#include <wpi/sendable/SendableRegistry.h>
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableHelper.h>

#include <ctre/Phoenix.h>

#include "Subsystem.hpp"

#include <iostream>

class Shooter : public Subsystem,
                public wpi::Sendable,
                public wpi::SendableHelper<Shooter>
{
public:
    struct State
    {
        units::revolutions_per_minute_t shooterVelocity = 0_rpm;

        units::revolutions_per_minute_t hoodVelocity = 0_rpm;

        units::degree_t turretAngle = 0_deg;

        bool operator==(const State &param)
        {
            return ((this->shooterVelocity == param.shooterVelocity) && (this->hoodVelocity == param.hoodVelocity) && (this->turretAngle == param.turretAngle));
        }
    };

    enum class Position : bool
    {
        Stowed = 0,
        Deployed = 1
    };

    // Constructor
    Shooter();

    // RJ::Subsystem Interface
    void ConfigureSystem();
    void UpdateTelemetry();

    // *** SETTERS ***
    void SetIntakeState(Position pos);
    void SetIntake(units::volt_t voltage);
    void SetIndexer(units::volt_t targetVoltage);
    void SetFeeder(units::volt_t targetVolts);

    void SetShooter(units::volt_t targetVolts);
    void SetShooterRPM(units::revolutions_per_minute_t targetRPM);

    void SetShooterTop(units::volt_t targetVolts);
    void SetShooterTopRPM(units::revolutions_per_minute_t targetRPM);
    
    // void SetTurret(units::volt_t targetVolts);
    // void SetTurretAngle(units::degree_t targetAngle);

    // void SetHood(units::volt_t targetVolts);
    // void SetHoodAngle(units::degree_t targetAngle);

    // *** GETTERS ***
    units::revolutions_per_minute_t GetShooterRPM();
    // units::degree_t GetTurretAngle();

    // Helpers
    State CalculateShot(units::inch_t distance);
    void SetShooterState(State shotStats);
    bool TempUpToSpeed();

    // Smartdash Sendable Interface
    void InitSendable(wpi::SendableBuilder &builder) override;

private:

    // Hardware
    WPI_TalonFX intake{10};
    WPI_TalonFX indexerA{11};
    // WPI_TalonFX indexerB{12};
    WPI_TalonFX feeder{13};
    WPI_TalonFX shooterA{14};
    WPI_TalonFX shooterB{15};
    WPI_TalonFX shooterTop{16};
    // WPI_TalonFX turret{17};
    // WPI_TalonFX hood{18};

    frc::Solenoid deployPiston{frc::PneumaticsModuleType::REVPH, 15};

    // Constants
    static constexpr double kScaleFactorTurret = 1.0;
    static constexpr double kScaleFactorFly = (1.0 / 2048);

    // Controllers
    frc::ProfiledPIDController<units::radian> turretPID{
        0.5, 0.0, 0.1, // Rotation-error
        frc::TrapezoidProfile<units::radian>::Constraints{
            180_deg_per_s,
            360_deg_per_s / 1_s}};

    // Command Inputs for telemetry
    int cmd_intake = 0.0;
    int cmd_indexer = 0.0;
    int cmd_feeder = 0.0;
    int cmd_shooter = 0.0;
    int cmd_shooterTop = 0.0;
    int cmd_turret = 0.0;
    int cmd_hood = 0.0;
};