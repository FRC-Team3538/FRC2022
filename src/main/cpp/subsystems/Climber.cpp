#pragma once
#include <frc/DigitalInput.h                         // for DigitalInput
#include <frc/Solenoid.h>                               // for Solenoid
#include <stdint.h>                                     // for uint8_t
#include <wpi/sendable/SendableHelper.h>                // for SendableHelper
#include <string>                                       // for string
#include <subsystems/Climber.hpp>
#include "frc/PneumaticsModuleType.h"                   // for PneumaticsMod...
#include "units/voltage.h"                              // for volt_t
#include "wpi/sendable/Sendable.h"                      // for Sendable
#include "ctre/phoenix/motorcontrol/IMotorController.h"
#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"

Climber::Climber()
{

}
void Climber::ConfigureSystem()
{
    ClimberA.ConfigFactoryDefault();
    ClimberB.ConfigFactoryDefault();

    ClimberA.SetInverted(true);
    ClimberB.SetInverted(true);
}

void Climber::SetClimberState(Climbstate climberstate)
{
    switch(climberstate) 
    {
    case Climbstate::Up:
        tiltPiston.Set(true);
    case Climbstate::Down:
        tiltPiston.Set(false);
     }
}
void Climber::SetClimber(units::volt_t targetVoltage)
{
    climberA.SetVoltage(targetVoltage);
    climberB.SetVoltage(targetVoltage);
}

void Climber::SetSensorOverride(bool override)
{
    if (sensorOverrode != override)
    {
        climberA.SetConfigS
    }
}
bool Climber::GetSensorOverride()
{
    sensorOverrode = 
}
