#include "Robotmap.hpp"

/**
 * Ran periodically in Robot.cpp
 * Cycles through the systems (one system per loop)
 * and runs UpdateTelemetry()
 *
 */
void Robotmap::UpdateSmartDash()
{
    subsystems[telemetryCt]->UpdateTelemetry();

    ++telemetryCt;

    if (telemetryCt == subsystems.size()) {
        // frc::SmartDashboard::PutNumber("pdp/Voltage", pdp.GetVoltage());
        // frc::SmartDashboard::PutNumber("pdp/TotalCurrent", pdp.GetTotalCurrent());
        pdpVoltageEntry.SetDouble(pdp.GetVoltage());
        pdpTotalCurrentEntry.SetDouble(pdp.GetTotalCurrent());
        
        telemetryCt = 0;
    }
}

/**
 * Loops through subsystems and
 * runs ConfigureSystem()
 *
 */
void Robotmap::ConfigureMotors()
{
    for (auto system : subsystems)
        system->ConfigureSystem();
}