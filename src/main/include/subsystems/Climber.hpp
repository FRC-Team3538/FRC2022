#pragma once

#include <frc/Solenoid.h>

#include <ctre/Phoenix.h>

#include "Subsystem.hpp"

class Climber : public Subsystem
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

    void SetClimber(double setValue);
    void SetClimberState(ClimbState climbPosition);

    // *** GETTERS ***

    ClimbState GetClimberState();

private:
    WPI_TalonFX climberA{20};
    WPI_TalonFX climberB{21};

    frc::Solenoid tiltPiston{frc::PneumaticsModuleType::REVPH, 4};
};