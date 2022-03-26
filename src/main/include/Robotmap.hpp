#pragma once

#include <frc/PowerDistribution.h>          // for PowerDistribution
#include <stddef.h>                         // for size_t
#include <functional>                       // for _Bind_helper<>::type, bind
#include <lib/PS4Controller.hpp>            // for PS4Controller
#include <lib/PneumaticHub.hpp>             // for PneumaticHub
#include <subsystems/Climber.hpp>           // for Climber
#include <subsystems/Drivetrain.hpp>        // for Drivetrain
#include <subsystems/RJVisionPipeline.hpp>  // for RJVisionPipeline, RJVisio...
#include <subsystems/Shooter.hpp>           // for Shooter
#include <vector>                           // for vector
#include "frc/Watchdog.h"                   // for Watchdog
#include "units/time.h"                     // for second_t
class Subsystem;

class Robotmap
{
private:
    std::vector<Subsystem *> subsystems;

    std::function<void(void)> test = std::bind(&Robotmap::watchDog, this);

public:
    RJ::PS4Controller mainController{0};
    RJ::PS4Controller secondaryController{1};

    frc::PowerDistribution pdp;
    RJ::PneumaticHub ph;
    frc::Watchdog watchdog{units::second_t{0.02}, test};

    // *** PUT SUBSYSTEMS HERE ***
    Drivetrain drivetrain;
    Shooter shooter;
    vision::RJVisionPipeline rjVision {vision::RJVisionPipeline::FilterType::EMAWithSpinup};
    Climber climber;
  
    Robotmap();

    void UpdateSmartDash();
    void ConfigureSystem();
    void watchDog();

    // SmartDash Cycler
    size_t telemetryCt = 0;
};