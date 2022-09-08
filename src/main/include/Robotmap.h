#pragma once

#include "frc/Watchdog.h"                   // for Watchdog
#include "units/time.h"                     // for second_t
#include <frc/PowerDistribution.h>          // for PowerDistribution
#include <frc/simulation/BatterySim.h>
#include <frc/TimedRobot.h>
#include <stddef.h>                         // for size_t
#include <subsystems/Drivetrain.h>
#include <unordered_map>
#include <vector>                           // for vector
#include <wpi/DataLog.h>
#include <frc/PowerDistribution.h>
#include <frc/PS4Controller.h>

class Subsystem;

class Robotmap
{
private:
    std::vector<Subsystem *> subsystems;

public:
    frc::PS4Controller mainController{0};
    frc::PS4Controller secondaryController{1};

    // *** PUT SUBSYSTEMS HERE ***
    Drivetrain drivetrain;

    frc::PowerDistribution pdp;

    frc::sim::BatterySim battery;
    units::volt_t battery_voltage;

    Robotmap();

    void UpdateSmartDash();
    void ConfigureSystem();
    void RegisterDataEntries(wpi::log::DataLog &log);
    void LogDataEntries(wpi::log::DataLog &log);

    void SimPeriodic();

    // SmartDash Cycler
    size_t telemetryCt = 0;

    std::unordered_map<std::string_view, int> data_entries;
};