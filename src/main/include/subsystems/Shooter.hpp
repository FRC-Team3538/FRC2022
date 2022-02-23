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
        units::revolutions_per_minute_t shooterRPM = 0_rpm;
        units::revolutions_per_minute_t shooterTopRPM = 0_rpm;
        // units::degree_t turretAngle = 0_deg;
        // units::degree_t hoodAngle = 0_deg;

        bool operator==(const State &param)
        {
            return ((this->shooterRPM == param.shooterRPM)
                    && (this->shooterTopRPM == param.shooterTopRPM) );
                    //&& (this->turretAngle == param.turretAngle) &&
                    //&& (this->hoodAngle == param.hoodAngle) );
        }
    };

    enum class Position : bool
    {
        Stowed = 0,
        Deployed = 1
    };

    enum class HoodPosition : uint8_t
    {
        Top = 0,
        Middle,
        Bottom
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
    void SetShooterRPM();

    void SetShooterTop(units::volt_t targetVolts);
    void SetShooterTopRPM(units::revolutions_per_minute_t targetRPM);
    void SetShooterTopRPM();
    
    // void SetTurret(units::volt_t targetVolts);
    // void SetTurretAngle(units::degree_t targetAngle);

    // void SetHood(units::volt_t targetVolts);
    void SetHoodAngle(HoodPosition pos);
    void SetHoodAngle();

    // *** GETTERS ***
    units::revolutions_per_minute_t GetShooterRPM();
    units::revolutions_per_minute_t GetShooterTopRPM();
    // units::degree_t GetTurretAngle();

    // Helpers
    bool Shoot();
    State CalculateShot(units::inch_t distance);
    void FalconSlotConfig(WPI_TalonFX& motor, int slot, SlotConfiguration& config);

    // Smartdash Sendable Interface
    void InitSendable(wpi::SendableBuilder &builder) override;
    void FalconSendableHelper(wpi::SendableBuilder &builder, WPI_TalonFX& motor, std::string name);

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

    frc::Solenoid deployPiston{frc::PneumaticsModuleType::REVPH, 0};

    frc::Solenoid hood{frc::PneumaticsModuleType::REVPH, 2};
    frc::Solenoid hoodStop{frc::PneumaticsModuleType::REVPH, 3};

    // Constants
    static constexpr double kScaleFactorTurret = 1.0;
    static constexpr double kTicks2RPM = (1.0 / (2048.0)) * 10.0 * 60.0;

    // Controllers
    SlotConfiguration shooterSlotConfig;

    frc::ProfiledPIDController<units::radian> turretPID{
        0.5, 0.0, 0.1, // Rotation-error
        frc::TrapezoidProfile<units::radian>::Constraints{
            180_deg_per_s,
            360_deg_per_s / 1_s}};

    // Current Command
    units::revolutions_per_minute_t cmd_shooterRPM{0_rpm};
    units::revolutions_per_minute_t cmd_shooterTopRPM{0_rpm};

    HoodPosition cmd_hoodPosition = HoodPosition::Bottom;

    bool hoodPosOS = false;
    frc::Timer hoodPosTimer;
};