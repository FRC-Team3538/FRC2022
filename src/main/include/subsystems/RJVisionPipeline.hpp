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
#include "subsystems/Shooter.hpp"

#include <photonlib/PhotonCamera.h>

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
            SMA,
            EMANoSpinup,
            EMAWithSpinup,
            MedianFilter,
            NoFilter
        };

        struct visionData
        {
            units::inch_t distance;
            units::degree_t deltaX;
            units::degree_t turretAngle;
            units::revolutions_per_minute_t calculatedShotRPM;
            bool filled = false;
        };

        struct photonVisionResult
        {
            photonlib::PhotonPipelineResult base_result;
            units::second_t read_time;
        };

        // Init Stuff
        RJVisionPipeline() = delete;
        RJVisionPipeline(Shooter &shooter, FilterType filter = FilterType::NoFilter);
        void ConfigureSystem();

        // Periodic
        void Periodic();
        void UpdateTelemetry();

        // Setter
        RJVisionPipeline::visionData Run();
        RJVisionPipeline::photonVisionResult RunPhotonVision();
        units::inch_t DistEstimation(units::degree_t deltaY, units::degree_t deltaX);
        void Reset();
        void SetLED(bool enable);
        void TakeSnapshot(uint8_t numberOfSnaps);
        void SetFilterType(FilterType setFilter);

    private:
        Shooter &shooter;

        units::inch_t estDist = 0.0_in;

        // TODO: ensure correctness @Jordan
        photonlib::PhotonCamera camera{""};
        std::shared_ptr<nt::NetworkTable> table;
        double dy, dx, tv;
        frc::Timer lightOn;

        bool turretAngleOS = false;
        units::degree_t savedTurretAngle;

        bool pipeSwitchOS = false;
        int pipeSwitchCt = 0;
        frc::Timer pipeSwitch;

        frc::Timer snapShotTimer;
        units::second_t snapTime;

        // Angle of elevation of camera
        const units::degree_t cameraAngle = 33.0_deg;

        // Distance between camera lens and vision target midpoint
        const units::inch_t deltaH = 68.0_in;

        // Filter Stuff
        std::list<double> xList;
        std::list<double> yList;

        double alpha = 0.125; // Weight
        uint8_t N = 8;       // Note, EMA will have a spinup interval of about 20ms * N
        units::second_t sampleSpinUpDelay = 0.3_s;

        double CalculateEMA(const std::list<double> &list);
        double CalculateAverage(const std::list<double> &list);

        FilterType filter;

        frc::Timer spinUpTimer;
        bool spinUpOS = false;
    };
} // namespace vision