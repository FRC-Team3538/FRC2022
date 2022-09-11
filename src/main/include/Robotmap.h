#pragma once

#include <frc/PowerDistribution.h>          // for PowerDistribution
#include <stddef.h>                         // for size_t
#include <functional>                       // for _Bind_helper<>::type, bind
#include <lib/PS4Controller.h>              // for PS4Controller
#include <lib/PneumaticHub.h>               // for PneumaticHub
#include <vector>                           // for vector
#include "units/time.h"                     // for second_t
#include <vector>
#include <functional>

#include <lib/PS4Controller.h>

#include <lib/PneumaticHub.h>

#include <frc/PowerDistribution.h>
#include <frc/TimedRobot.h>
#include <unordered_map>
#include <wpi/DataLog.h>
#include <subsystems/Drivetrain.h>
#include <subsystems/RJVisionPipeline.h>

#include <frc/simulation/BatterySim.h>

class Subsystem;

class Robotmap
{
private:
    std::vector<Subsystem *> subsystems;

public:
    RJ::PS4Controller mainController{0};
    RJ::PS4Controller secondaryController{1};

    frc::PowerDistribution pdp;
    RJ::PneumaticHub ph;

    // *** PUT SUBSYSTEMS HERE ***
    Drivetrain drivetrain;
    vision::RJVisionPipeline vision;

    frc::sim::BatterySim battery;
    units::volt_t battery_voltage;

    Robotmap();

    void UpdateSmartDash();
    void ConfigureSystem();
    void RegisterDataEntries(wpi::log::DataLog &log);
    void LogDataEntries(wpi::log::DataLog &log);

    void SimInit();
    void SimPeriodic();

    // SmartDash Cycler
    size_t telemetryCt = 0;

    std::unordered_map<std::string_view, int> data_entries;
};