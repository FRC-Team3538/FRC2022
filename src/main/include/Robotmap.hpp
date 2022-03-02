#pragma once

#include <subsystems/Drivetrain.hpp>
#include <subsystems/Shooter.hpp>
#include <subsystems/RJVisionPipeline.hpp>
#include <subsystems/Climber.hpp>
#include <vector>
#include <functional>

#include <lib/PS4Controller.hpp>

#include <lib/PneumaticHub.hpp>

#include <frc/PowerDistribution.h>
#include <frc/TimedRobot.h>

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
    vision::RJVisionPipeline rjVision;
    Climber climber;
  
    Robotmap();

    void UpdateSmartDash();
    void ConfigureSystem();
    void watchDog();

    // SmartDash Cycler
    size_t telemetryCt = 0;
};