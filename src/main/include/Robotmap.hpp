#pragma once

#include <subsystems/Drivetrain.hpp>
#include <subsystems/Shooter.hpp>
#include <subsystems/RJVisionPipeline.hpp>
#include <vector>
#include <functional>

#include <frc/PS4Controller.h>

#include <frc/PowerDistribution.h>
#include <frc/TimedRobot.h>


class Robotmap
{
private:
    std::vector<Subsystem *> subsystems;
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

    // *** ALSO PUT SUBSYSTEMS HERE ***
    Robotmap();

    void UpdateSmartDash();
    void ConfigureSystem();
    void watchDog();

    // SmartDash Cycler
    size_t telemetryCt = 0;
 
};