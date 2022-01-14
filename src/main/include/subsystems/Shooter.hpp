#pragma once

#include <units/angular_velocity.h>
#include <units/angle.h>
#include <units/length.h>

#include <frc/Solenoid.h>

#include <ctre/Phoenix.h>

#include "Subsystem.hpp"

#include <iostream>

class Shooter : public Subsystem
{
public:
    struct State
    {
        units::revolutions_per_minute_t shooterVelocity = 0_rpm;

        units::degree_t turretAngle = 0_deg;
    };

    enum class Position:uint8_t
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

    void SetFeeder(double setValue);
    void SetIndexer(double setValue);

    void SetIntakeState(Position pos);
    void SetIntake(double setValue);

    // *** GETTERS ***

    units::degree_t GetTurretAngle();
    units::revolutions_per_minute_t GetShooterRPM();

    State CalculateShot(units::inch_t distance);

private:

    WPI_TalonFX intake{10};
    WPI_TalonFX indexerA{11};
    WPI_TalonFX indexerB{12};
    WPI_TalonFX feeder{13};
    WPI_TalonFX shooterA{14};
    WPI_TalonFX shooterB{15};
    WPI_TalonFX hood{16};
    WPI_TalonFX turret{17};

    frc::Solenoid deployPiston{frc::PneumaticsModuleType::REVPH, 1};

    static constexpr double kScaleFactorTurret = 1.0;
    static constexpr double kScaleFactorFly = 1.0;
};