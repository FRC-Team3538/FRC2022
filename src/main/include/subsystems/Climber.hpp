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
#include <frc/Solenoid.h>
#include <frc/DigitalInput.h>

#include "Subsystem.hpp"

class Climber : public Subsystem,
                public wpi::Sendable,
                public wpi::SendableHelper<Climber>
{
public:

    enum class ClimbState:uint8_t
    {
        Up = 0,
        Down
    };

    

    Climber();

    void ConfigureSystem();
    void UpdateTelemetry();

    // *** SETTERS ***

    void SetClimber(units::volt_t targetVoltage);
    void SetClimberState(ClimbState climbPosition);
    void SetSensorOverride(bool override);

    // *** GETTERS ***

    ClimbState GetClimberState();

    bool GetSensorOverride();

    void InitSendable(wpi::SendableBuilder &builder) override;
    void FalconSendableHelper(wpi::SendableBuilder &builder, WPI_TalonFX& motor, std::string name);


private:
    bool sensorOverrode = true;

    WPI_TalonFX climberA{20};
    WPI_TalonFX climberB{21};

    frc::Solenoid tiltPiston{frc::PneumaticsModuleType::REVPH, 1};

    frc::DigitalInput bottomMagSwitch {9};
};