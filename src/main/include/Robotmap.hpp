#pragma once

#include <subsystems/Drivetrain.hpp>
#include <subsystems/Shooter.hpp>
#include <subsystems/RJVisionPipeline.hpp>
#include <subsystems/Climber.hpp>
#include <vector>
#include <functional>

#include <lib/PS4Controller.hpp>

#include <frc/PowerDistribution.h>
#include <frc/TimedRobot.h>

#include "lib/wpi/DataLogManager.h"
#include "lib/wpi/DataLog.h"



class Robotmap
{
private:
    std::vector<Subsystem *> subsystems;

    int pdpVoltageDatalogEntry;
    int pdpCurrentDatalogEntry;

    nt::NetworkTableEntry pdpVoltageEntry = frc::SmartDashboard::GetEntry("/pdp/Voltage");
    nt::NetworkTableEntry pdpTotalCurrentEntry = frc::SmartDashboard::GetEntry("/pdp/TotalCurrent");

    std::function<void(void)> test = std::bind(&Robotmap::watchDog, this);

public:
    RJ::PS4Controller mainController{0};
    RJ::PS4Controller secondaryController{1};

    frc::PowerDistribution pdp;
    frc::Watchdog watchdog{units::second_t{0.02}, test};

    // *** PUT SUBSYSTEMS HERE ***
    Drivetrain drivetrain;
    Shooter shooter;
    vision::RJVisionPipeline rjVision;
    Climber climber;
  
    Robotmap();

    void UpdateSmartDash();
    void ConfigureSystem();
    void watchDog();

    // SmartDash Cycler
    size_t telemetryCt = 0;
};