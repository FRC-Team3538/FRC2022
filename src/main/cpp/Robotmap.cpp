#include "Robotmap.hpp"

#include <wpi/timestamp.h>

#include "lib/Logging.h"

// Constructor
// *** ALSO PUT SUBSYSTEMS HERE ***
Robotmap::Robotmap()
{
#ifdef LOGGER
    pdpVoltageDatalogEntry = frc::DataLogManager::GetLog().Start("pdp_voltage", "double");
    pdpCurrentDatalogEntry = frc::DataLogManager::GetLog().Start("pdp_current", "double[]");
#endif
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

        for(int i = 0; i < 20; i++) {
            currentVector.push_back(pdp.GetCurrent(i));
        }
#ifdef LOGGING
        frc::DataLogManager::GetLog().AppendDouble(pdpVoltageDatalogEntry, pdp.GetVoltage(), wpi::Now());
        frc::DataLogManager::GetLog().AppendDoubleArray(pdpCurrentDatalogEntry, currentVector, wpi::Now());
#endif
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