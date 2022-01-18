#pragma once

#include <iostream>
#include "frc/smartdashboard/SmartDashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include <cmath>
#include <frc/Timer.h>
#include "Subsystem.hpp"

namespace vision
{

    /*
     * RJVisionPipeline class.
     */

    class RJVisionPipeline : public Subsystem
    {
    private:
        const double cameraAngle = 32;
        const double dh = 63.0; // distance between camera lens and quarter-way up the goal

        double estDist = 0.0;

        std::shared_ptr<nt::NetworkTable> table;
        double dy, dx, tv;
        frc::Timer lightOn;

        bool pipeSwitchOS = false;
        int pipeSwitchCt = 0;
        frc::Timer pipeSwitch;

    public:
        struct visionData
        {
            double distance, angle;
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
        double DistEstimation();
        void Reset();
    };
} // namespace vision