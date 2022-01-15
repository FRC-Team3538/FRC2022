#pragma once

#include <subsystems/Drivetrain.hpp>
#include <subsystems/Shooter.hpp>
#include <vector>
#include <frc/controller/RamseteController.h>

#include <frc/PS4Controller.h>

#include <frc/PowerDistribution.h>

class Robotmap
{
private:
    std::vector<Subsystem *> subsystems;

public:
    frc::PS4Controller mainController{0};
    frc::PS4Controller secondaryController{1};

    frc::PowerDistribution pdp;


    // *** PUT SUBSYSTEMS HERE ***
    Drivetrain drivetrain{false};
    Shooter shooter;

    // *** ALSO PUT SUBSYSTEMS HERE ***
    Robotmap()
    {
        subsystems.push_back(&drivetrain);
        subsystems.push_back(&shooter);
    }

    void UpdateSmartDash();
    void ConfigureMotors();

    // SmartDash Cycler
    size_t telemetryCt = 0;

    nt::NetworkTableEntry pdpVoltageEntry = frc::SmartDashboard::GetEntry("/pdp/Voltage");
    nt::NetworkTableEntry pdpTotalCurrentEntry = frc::SmartDashboard::GetEntry("/pdp/TotalCurrent");

    // Ramsete Controller
    frc::RamseteController m_ramsete{units::unit_t<frc::RamseteController::b_unit>{2.0},
                                   units::unit_t<frc::RamseteController::zeta_unit>{0.7}};
};