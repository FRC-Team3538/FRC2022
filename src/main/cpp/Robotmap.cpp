#include "Robotmap.hpp"

void Robotmap::UpdateSmartDash()
{
    subsystems[telemetryCt]->UpdateTelemetry();
    ++telemetryCt;
    if (telemetryCt == subsystems.size())
        telemetryCt = 0;
}

void Robotmap::ConfigureMotors()
{
    for (auto system : subsystems)
    {
        system->ConfigureSystem();
    }
}