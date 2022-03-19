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
#include <frc/DigitalInput.h>

#include "Subsystem.hpp"

#include <iostream>

class Shooter : public Subsystem,
                public wpi::Sendable,
                public wpi::SendableHelper<Shooter>
{
public:
    bool zeroed = false;

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

    struct State
    {
        units::revolutions_per_minute_t shooterRPM = 0_rpm;
        HoodPosition hoodAngle = HoodPosition::Top;

        bool operator==(const State &param)
        {
            return ((this->shooterRPM == param.shooterRPM) && (this->hoodAngle == param.hoodAngle));
        }
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

    void SetTurret(units::volt_t targetVolts);
    bool SetTurretAngle(units::degree_t targetAngle, units::degree_t tol);
    void ZeroTurret(bool negative);
    void ZeroTurret();

    void SetHoodAngle(HoodPosition pos);
    void SetHoodAngle();

    void SetBlinkyZeroThing();

    // *** GETTERS ***
    units::revolutions_per_minute_t GetShooterRPM();
    units::degree_t GetTurretAngle();

    bool GetTurretSwitch();

    // Helpers
    void ResetEdgeDetector();
    bool Shoot_EdgeDetector();
    bool Shoot(units::second_t settleTime = 1.5_s);
    State CalculateShot(units::inch_t distance);
    void FalconSlotConfig(WPI_TalonFX &motor, int slot, SlotConfiguration &config);

    // Smartdash Sendable Interface
    void InitSendable(wpi::SendableBuilder &builder) override;
    void FalconSendableHelper(wpi::SendableBuilder &builder, WPI_TalonFX &motor, std::string name);

    static constexpr double kTurretMax = 90.0; // Degrees
    static constexpr double kTurretMin = -90.0;

private:
    // Hardware
    WPI_TalonFX intake{10};
    WPI_TalonFX indexerA{11};
    // WPI_TalonFX indexerB{s12};
    WPI_TalonFX feeder{13};
    WPI_TalonFX shooterA{14};
    WPI_TalonFX shooterB{15};
    // WPI_TalonFX shooterTop{16};
    WPI_TalonFX turret{17};
    // WPI_TalonFX hood{18};

    frc::Solenoid deployPiston{frc::PneumaticsModuleType::REVPH, 0};

    frc::Solenoid hood{frc::PneumaticsModuleType::REVPH, 2};
    frc::Solenoid hoodStop{frc::PneumaticsModuleType::REVPH, 3};

    // Constants
    static constexpr double kScaleFactorTurret = 360.0 * (18.0 / 184.0) * (11.0 / 60.0) * (1.0 / 2048.0);// Angle Over Ticks
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

    HoodPosition cmd_hoodPosition = HoodPosition::Bottom;

    bool hoodPosOS = false;
    frc::Timer hoodPosTimer;

    frc::DigitalInput turretZeroSwitch {9};

    frc::Timer epilepsyTimer;
    bool blinkyZeroLight = false;
    bool m_upToSpeed = false;
};