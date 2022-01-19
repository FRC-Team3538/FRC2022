#pragma once

#include <subsystems/Drivetrain.hpp>
#include <subsystems/Shooter.hpp>
#include <subsystems/RJVisionPipeline.hpp>
#include <vector>
#include <frc/controller/RamseteController.h>

#include <frc/PS4Controller.h>

#include <frc/PowerDistribution.h>

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

public:
    frc::PS4Controller mainController{0};
    frc::PS4Controller secondaryController{1};

    frc::PowerDistribution pdp{0, frc::PowerDistribution::ModuleType::kCTRE};

    // *** PUT SUBSYSTEMS HERE ***
    Drivetrain drivetrain{false};
    Shooter shooter;
    vision::RJVisionPipeline rjVision;

    // *** ALSO PUT SUBSYSTEMS HERE ***
    Robotmap()
    {
        pdpVoltageDatalogEntry = frc::DataLogManager::GetLog().Start("pdp_voltage", "double");
        pdpCurrentDatalogEntry = frc::DataLogManager::GetLog().Start("pdp_current", "double[]");
        subsystems.push_back(&drivetrain);
        subsystems.push_back(&shooter);
        subsystems.push_back(&rjVision);
    }

    void UpdateSmartDash();
    void ConfigureMotors();

    // SmartDash Cycler
    size_t telemetryCt = 0;

    // Ramsete Controller
    frc::RamseteController m_ramsete{units::unit_t<frc::RamseteController::b_unit>{2.0},
                                   units::unit_t<frc::RamseteController::zeta_unit>{0.7}};
};