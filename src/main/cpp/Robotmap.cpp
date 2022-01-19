#include "Robotmap.hpp"

#include <wpi/timestamp.h>

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

        std::vector<double> currentVector;

        for(int i = 0; i < 16; i++) {
            currentVector.push_back(pdp.GetCurrent(i));
        }

        dataLog.AppendDouble(pdpVoltageDatalogEntry, pdp.GetVoltage(), wpi::Now());
        dataLog.AppendDoubleArray(pdpCurrentDatalogEntry, currentVector, wpi::Now());
        
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