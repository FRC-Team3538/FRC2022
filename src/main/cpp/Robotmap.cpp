#include "Robotmap.hpp"

#include <wpi/timestamp.h>

// Constructor
// *** ALSO PUT SUBSYSTEMS HERE ***
Robotmap::Robotmap()
{
    subsystems.push_back(&drivetrain);
    subsystems.push_back(&shooter);
    subsystems.push_back(&rjVision);
    subsystems.push_back(&climber);
}

/**
 * Ran periodically in Robot.cpp
 * Cycles through the systems (one system per loop)
 * and runs UpdateTelemetry()
 *
 */
void Robotmap::UpdateSmartDash()
{
    if (telemetryCt < subsystems.size())
    {
        subsystems[telemetryCt]->UpdateTelemetry();
    }
    else if (telemetryCt == subsystems.size())
    {
        pdpVoltageEntry.SetDouble(pdp.GetVoltage());
        pdpTotalCurrentEntry.SetDouble(pdp.GetTotalCurrent());

        std::vector<double> currentVector;

        for(int i = 0; i < 16; i++) {
            currentVector.push_back(pdp.GetCurrent(i));
        }

        frc::DataLogManager::GetLog().AppendDouble(pdpVoltageDatalogEntry, pdp.GetVoltage(), wpi::Now());
        frc::DataLogManager::GetLog().AppendDoubleArray(pdpCurrentDatalogEntry, currentVector, wpi::Now());
        
        // Restart the loop
        telemetryCt = 0;
    }

    ++telemetryCt;
}

/**
 * Loops through subsystems and
 * runs ConfigureSystem()
 *
 */
void Robotmap::ConfigureSystem()
{
    for (auto system : subsystems)
        system->ConfigureSystem();
}

void Robotmap::watchDog()
{
    std::cout << "SAD WATCHDOG" << std::endl;
}