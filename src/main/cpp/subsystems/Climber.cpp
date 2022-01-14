#include "subsystems/Climber.hpp"

Climber::Climber()
{
}

void Climber::ConfigureSystem()
{
    climberB.Follow(climberA);
}

void Climber::UpdateTelemetry()
{
}

void Climber::SetClimber(double setValue)
{
    climberA.SetVoltage(units::volt_t{setValue * 13.0});
}

void Climber::SetClimberState(ClimbState climbPosition)
{
    switch(climbPosition)
    {
        case ClimbState::Up:
        {
            tiltPiston.Set(true);
            break;
        }

        case ClimbState::Down:
        {
            tiltPiston.Set(false);
            break;
        }
    }
}

Climber::ClimbState Climber::GetClimberState()
{
    return tiltPiston.Get() ? ClimbState::Up : ClimbState::Down;
}