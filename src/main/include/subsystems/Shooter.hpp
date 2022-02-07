#pragma once

#include <units/angular_velocity.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/constants.h>

#include <wpi/numbers>

#include <frc/Solenoid.h>
#include <frc/controller/ProfiledPIDController.h>

#include <ctre/Phoenix.h>

#include "Subsystem.hpp"

#include <iostream>

class Shooter : public Subsystem
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

    enum class Position : uint8_t
    {
        Stowed = 0,
        Deployed
    };

    Shooter();

    void ConfigureSystem();
    void UpdateTelemetry();

    // *** SETTERS ***

    void SetTurretAngle(units::degree_t targetAngle);
    void SetShooterRPM(units::revolutions_per_minute_t targetRPM);

    void SetTurret(units::volt_t targetVolts);
    void SetShooter(units::volt_t targetVolts);
    void SetShooterState(State shotStats);

    void SetFeeder(units::volt_t targetVolts);
    void SetHood(units::volt_t targetVolts);
    void SetHoodRPM(units::revolutions_per_minute_t targetRPM);
    void SetIndexer(double setValue);

    void SetIntakeState(Position pos);
    void SetIntake(units::volt_t voltage);

    // *** GETTERS ***

    units::degree_t GetTurretAngle();
    units::revolutions_per_minute_t GetShooterRPM();

    State CalculateShot(units::inch_t distance);

    bool TempUpToSpeed();

private:
    WPI_TalonFX intake{10};
    WPI_TalonFX indexerA{11};
    // WPI_TalonFX indexerB{12};
    WPI_TalonFX feeder{13};
    WPI_TalonFX shooterA{14};
    WPI_TalonFX shooterB{15};
    WPI_TalonFX hood{16};
    // WPI_TalonFX turret{17};

    // frc::Solenoid deployPiston{frc::PneumaticsModuleType::REVPH, 1};

    static constexpr double kScaleFactorTurret = 1.0;
    static constexpr double kScaleFactorFly = (1.0 / 2048);

    frc::ProfiledPIDController<units::radian> turretPID{
        0.5, 0.0, 0.1, // Rotation-error
        frc::TrapezoidProfile<units::radian>::Constraints{
            180_deg_per_s,
            360_deg_per_s / 1_s}};

    nt::NetworkTableEntry shooterVoltageEntry = frc::SmartDashboard::GetEntry("/shooter/Shooter_Voltage");
    nt::NetworkTableEntry shooterRPMEntry = frc::SmartDashboard::GetEntry("/shooter/Shooter_RPM");
    nt::NetworkTableEntry shooterSurfaceSpeedEntry = frc::SmartDashboard::GetEntry("/shooter/Shooter_Surface_Speed_FPS");
    nt::NetworkTableEntry feederVoltageEntry = frc::SmartDashboard::GetEntry("/shooter/Feeder_Voltage");
    nt::NetworkTableEntry feederRPMEntry = frc::SmartDashboard::GetEntry("/shooter/Feeder_RPM");
    nt::NetworkTableEntry hoodVoltageEntry = frc::SmartDashboard::GetEntry("/shooter/Hood_Voltage");
    nt::NetworkTableEntry hoodRPMEntry = frc::SmartDashboard::GetEntry("/shooter/Hood_RPM");
    nt::NetworkTableEntry hoodSurfaceSpeedEntry = frc::SmartDashboard::GetEntry("/shooter/Hood_Surface_Speed_FPS");
    nt::NetworkTableEntry shotEffortEntry = frc::SmartDashboard::GetEntry("/shooter/Shot_Effort_FPS");
    nt::NetworkTableEntry backspinEffortEntry = frc::SmartDashboard::GetEntry("/shooter/Backspin_Effort_FPS");
    nt::NetworkTableEntry impartedBackspinEntry = frc::SmartDashboard::GetEntry("/shooter/Imparted_Backspin_RPM");
};