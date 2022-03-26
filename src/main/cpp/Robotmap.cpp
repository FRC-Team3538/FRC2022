#include "Robotmap.hpp"
#include <iostream>                         // for operator<<, endl, basic_o...
#include "frc/PowerDistribution.h"          // for PowerDistribution
#include "subsystems/Climber.hpp"           // for Climber
#include "subsystems/Drivetrain.hpp"        // for Drivetrain
#include "subsystems/RJVisionPipeline.hpp"  // for RJVisionPipeline
#include "subsystems/Shooter.hpp"           // for Shooter
#include "subsystems/Subsystem.hpp"         // for Subsystem

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

void Robotmap::watchDog()
{
    std::cout << "SAD WATCHDOG" << std::endl;
}