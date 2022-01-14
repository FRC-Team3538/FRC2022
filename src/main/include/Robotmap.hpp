#pragma once

#include <subsystems/Drivetrain.hpp>
#include <vector>
#include <frc/controller/RamseteController.h>

class Robotmap
{
private:
    std::vector<Subsystem *> subsystems;

public:
    // *** PUT SUBSYSTEMS HERE ***
    Drivetrain drivetrain{false};

    // *** ALSO PUT SUBSYSTEMS HERE ***
    Robotmap()
    {
        subsystems.push_back(&drivetrain);
    }

    void UpdateSmartDash();
    void ConfigureMotors();

    // SmartDash Cycler
    size_t telemetryCt = 0;

    // Ramsete Controller
    frc::RamseteController m_ramsete{units::unit_t<frc::RamseteController::b_unit>{2.0},
                                   units::unit_t<frc::RamseteController::zeta_unit>{0.7}};
};