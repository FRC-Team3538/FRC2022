#pragma once

#include <subsystems/Drivetrain.hpp>
#include <subsystems/Shooter.hpp>
#include <subsystems/RJVisionPipeline.hpp>
#include <vector>
#include <frc/controller/RamseteController.h>
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
    void ConfigureMotors();
    void watchDog();

    // SmartDash Cycler
    size_t telemetryCt = 0;

    nt::NetworkTableEntry pdpVoltageEntry = frc::SmartDashboard::GetEntry("/pdp/Voltage");
    nt::NetworkTableEntry pdpTotalCurrentEntry = frc::SmartDashboard::GetEntry("/pdp/TotalCurrent");

    // Ramsete Controller
    frc::RamseteController m_ramsete{units::unit_t<frc::RamseteController::b_unit>{2.0},
                                   units::unit_t<frc::RamseteController::zeta_unit>{0.7}};

};