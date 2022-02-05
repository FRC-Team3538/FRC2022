#pragma once

#include <subsystems/Drivetrain.hpp>
#include <subsystems/Shooter.hpp>
#include <subsystems/RJVisionPipeline.hpp>
#include <subsystems/Climber.hpp>
#include <vector>
#include <functional>

#include <frc/PS4Controller.h>

#include <frc/PowerDistribution.h>
#include <frc/TimedRobot.h>


#include "lib/wpi/DataLogManager.h"
#include "lib/wpi/DataLog.h"



class Robotmap
{
private:
    std::vector<Subsystem *> subsystems;
  
    // pdpVoltageDatalogEntry = frc::DataLogManager::GetLog().Start("pdp_voltage", "double");
    // pdpCurrentDatalogEntry = frc::DataLogManager::GetLog().Start("pdp_current", "double[]");
    int pdpVoltageDatalogEntry;
    int pdpCurrentDatalogEntry;

    nt::NetworkTableEntry pdpVoltageEntry = frc::SmartDashboard::GetEntry("/pdp/Voltage");
    nt::NetworkTableEntry pdpTotalCurrentEntry = frc::SmartDashboard::GetEntry("/pdp/TotalCurrent");

    std::function<void(void)> test = std::bind(&Robotmap::watchDog, this);

public:
    frc::PS4Controller mainController{0};
    frc::PS4Controller secondaryController{1};

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