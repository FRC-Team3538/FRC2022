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
#include <list>

namespace vision
{

    /*
     * RJVisionPipeline class.
     */

    class RJVisionPipeline : public Subsystem
    {
    public:
        enum class FilterType : uint8_t
        {
            SampleAverage = 0, // Test to reduce jitter in steady state. Samples N times and returns values. Doesn't update after that
            EMANoSpinup,
            EMAWithSpinup,
            NoFilter
        };

        struct visionData
        {
            units::inch_t distance;
            units::degree_t angle;
            bool filled = false;
        };

        // Init Stuff
        RJVisionPipeline() = delete;
        RJVisionPipeline(FilterType filter = FilterType::NoFilter);
        void ConfigureSystem();

        // Periodic
        void Periodic();
        void UpdateTelemetry();

        // Setter
        RJVisionPipeline::visionData Run();
        units::inch_t DistEstimation(units::degree_t deltaY, units::degree_t deltaX);
        void Reset();
        void SetLED(bool enable);

    private:
        units::inch_t estDist = 0.0_in;

        std::shared_ptr<nt::NetworkTable> table;
        double dy, dx, tv;
        frc::Timer lightOn;

        bool pipeSwitchOS = false;
        int pipeSwitchCt = 0;
        frc::Timer pipeSwitch;

        // Angle of elevation of camera
        const units::degree_t cameraAngle = 33.0_deg;

        // Distance between camera lens and vision target midpoint
        const units::inch_t deltaH = 68.0_in;

        // Filter Stuff
        std::list<double> xList;
        std::list<double> yList;

        const double alpha = 0.125; // Weight
        const uint8_t N = 14;       // Note, EMA will have a spinup interval of about 20ms * N
        //units::second_t sampleSpinUpDelay = 0.15_s;

        double CalculateEMA(const std::list<double> &list);
        double CalculateAverage(const std::list<double> &list);

        FilterType filter;

        frc::Timer spinUpTimer;
        bool spinUpOS = false;
    };
} // namespace vision