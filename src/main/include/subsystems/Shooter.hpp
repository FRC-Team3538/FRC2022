#pragma once

#include <frc/DigitalInput.h>                                   // for Digit...
#include <frc/Solenoid.h>                                       // for Solenoid
#include <frc/controller/ProfiledPIDController.h>               // for Profi...
#include <frc/filter/LinearFilter.h>                            // for Linea...
#include <stdint.h>                                             // for uint8_t
#include <units/angle.h>                                        // for degree_t
#include <units/angular_velocity.h>                             // for revol...
#include <units/length.h>                                       // for inch_t
#include <wpi/sendable/SendableHelper.h>                        // for Senda...
#include <string>                                               // for string

#include "Subsystem.hpp"                                        // for Subsy...
#include "frc/PneumaticsModuleType.h"                           // for Pneum...
#include "frc/Timer.h"                                          // for Timer
#include "frc/trajectory/TrapezoidProfile.h"                    // for Trape...
#include "units/base.h"                                         // for unit_t
#include "units/time.h"                                         // for opera...
#include "units/voltage.h"                                      // for volt_t
#include "wpi/sendable/Sendable.h"                              // for Sendable
#include "ctre/phoenix/motorcontrol/IMotorController.h"
#include "ctre/phoenix/motorcontrol/can/BaseMotorController.h"
#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"

namespace wpi {
class SendableBuilder;
}  // namespace wpi

using namespace ctre::phoenix::motorcontrol::can;

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
    void ConfigureSystem() override;
    void UpdateTelemetry() override;

    void RegisterDataEntries(wpi::log::DataLog &log);
    void LogDataEntries(wpi::log::DataLog &log);

    void Periodic();

    // *** SETTERS ***
    void SetIntakeState(Position pos);
    void SetIntake(units::volt_t voltage);
    void SetIndexer(units::volt_t targetVoltage);
    void SetFeeder(units::volt_t targetVolts);

    void SetShooter(units::volt_t targetVolts);
    void SetShooterRPM(units::revolutions_per_minute_t targetRPM);
    void SetShooterRPM();
    void SetShooterRatio(double ratio);

    void SetTurretCoastMode();
    void SetTurretBrakeMode();
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

    static constexpr auto kTurretMax = 150_deg;
    static constexpr auto kTurretMin = 150_deg * -1;

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
    static constexpr auto kScaleFactorTurret = 360_deg * (18.0 / 184.0) * (11.0 / 60.0) * (1.0 / 2048.0); // Angle Over Ticks
    static constexpr double kTicks2RPM = (1.0 / (2048.0)) * 10.0 * 60.0;

    // static constexpr auto kSTurret = 1.0697_V;
    static constexpr auto kSTurret = 0.5_V;
    static constexpr auto kVTurret = 6.9796_V / 1_tps;
    static constexpr auto kATurret = 0.087289_V / 1_tps / 1_s;

    // Controllers
    SlotConfiguration shooterSlotConfig;

    frc::ProfiledPIDController<units::radian> turretPID{
        // Velocity PID 0.61116, 0.0, 0.0, // Rotation-error
        // Position PID 4.9295, 0.0, 0.67183, // Rotation-error
        0.61116, 0.0, 0.0, // Rotation-error
        frc::TrapezoidProfile<units::radian>::Constraints{
            90_deg_per_s,
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

    frc::LinearFilter<units::angular_velocity::revolutions_per_minute_t> filter = frc::LinearFilter<units::angular_velocity::revolutions_per_minute_t>::HighPass(0.1, 0.02_s);
    units::angular_velocity::revolutions_per_minute_t m_latestFilterResult;

    // Ratio between top and bottom wheels
    // note: this must be 1.0 if they are linked with belts !!!!
    double shooter_ratio = 1.0;

    units::degree_t goalAngle;
};