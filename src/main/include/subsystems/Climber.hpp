#pragma once

#include <frc/DigitalInput.h>                           // for DigitalInput
#include <frc/Solenoid.h>                               // for Solenoid
#include <stdint.h>                                     // for uint8_t
#include <wpi/sendable/SendableHelper.h>                // for SendableHelper
#include <string>                                       // for string

#include "Subsystem.hpp"                                // for Subsystem
#include "frc/PneumaticsModuleType.h"                   // for PneumaticsMod...
#include "units/voltage.h"                              // for volt_t
#include "wpi/sendable/Sendable.h"                      // for Sendable
#include "ctre/phoenix/motorcontrol/IMotorController.h"
#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"

namespace wpi {
class SendableBuilder;
}  // namespace wpi

using namespace ctre::phoenix::motorcontrol::can;

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

    void ConfigureSystem() override;
    void UpdateTelemetry() override;

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
    const double kScaleFactor = 1 / 124000.0; // Ticks over climber position (from 0 - 1)

    bool sensorOverrode = false;

    WPI_TalonFX climberA{20};
    WPI_TalonFX climberB{21};

    frc::Solenoid tiltPiston{frc::PneumaticsModuleType::REVPH, 1};

    frc::DigitalInput bottomMagSwitch {8};
};