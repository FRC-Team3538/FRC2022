#include "Robotmap.h"
#include <iostream>                         // for operator<<, endl, basic_o...
#include "frc/PowerDistribution.h"          // for PowerDistribution
#include "subsystems/Subsystem.h"         // for Subsystem

#include <wpi/DataLog.h>

// Constructor
// *** ALSO PUT SUBSYSTEMS HERE ***
Robotmap::Robotmap()
{
    subsystems.emplace_back(&drivetrain);
    // subsystems.emplace_back(&vision);
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
    pdp.SetSwitchableChannel(true);
    for (auto system : subsystems)
        system->ConfigureSystem();
}

void Robotmap::RegisterDataEntries(wpi::log::DataLog &log)
{
}

void Robotmap::LogDataEntries(wpi::log::DataLog &log)
{
}

void Robotmap::SimPeriodic()
{
    units::ampere_t amps = 0_A;
    for (auto system : subsystems) {
        amps += system->SimPeriodic(battery_voltage);
    }
    battery_voltage = battery.Calculate({amps});
}