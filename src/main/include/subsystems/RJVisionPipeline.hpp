#pragma once

#include <iostream>
#include "frc/smartdashboard/SmartDashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include <cmath>
#include <frc/Timer.h>
#include <units/length.h>
#include <units/angle.h>
#include "Subsystem.hpp"

namespace vision
{

    /*
     * RJVisionPipeline class.
     */

    class RJVisionPipeline : public Subsystem
    {
    private:
        units::inch_t estDist = 0.0_in;

        std::shared_ptr<nt::NetworkTable> table;
        double dy, dx, tv;
        frc::Timer lightOn;

        bool pipeSwitchOS = false;
        int pipeSwitchCt = 0;
        frc::Timer pipeSwitch;

        // Angle of elevation of camera
        const units::degree_t cameraAngle = 16.0_deg;

        // Distance between camera lens and vision target midpoint
        const units::inch_t deltaH = 59.0_in;

    public:
        struct visionData
        {
            units::inch_t distance;
            units::degree_t angle;
            bool filled = false;
        };

        // Init Stuff
        RJVisionPipeline();
        void ConfigureSystem();

        // Periodic
        void Periodic();
        void UpdateTelemetry();

        // Setter
        RJVisionPipeline::visionData Run();
        units::inch_t DistEstimation();
        void Reset();
    };
} // namespace vision