#pragma once

#include <frc/Timer.h>                       // for Timer
#include <stdint.h>                          // for uint8_t
#include <units/angle.h>                     // for degree_t, operator""_deg
#include <units/angular_velocity.h>          // for revolutions_per_minute_t
#include <units/length.h>                    // for inch_t, operator""_in
#include <list>                              // for list
#include <memory>                            // for shared_ptr
#include "Subsystem.hpp"                     // for Subsystem
#include "photonlib/PhotonPipelineResult.h"  // for PhotonPipelineResult
#include "units/base.h"                      // for operator*
#include "units/time.h"                      // for second_t, millisecond_t
namespace nt { class NetworkTable; }

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
            NoFilter
        };

        struct visionData
        {
            units::inch_t distance;
            units::degree_t deltaX;
            units::degree_t deltaY;
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
        RJVisionPipeline(FilterType filter = FilterType::EMAWithSpinup);
        void ConfigureSystem() override;

        // Periodic
        void Periodic();
        void UpdateTelemetry() override;

        // Setter
        RJVisionPipeline::visionData Run();
        RJVisionPipeline::photonVisionResult RunPhotonVision();
        units::inch_t DistEstimation(units::degree_t deltaY, units::degree_t deltaX);
        void Reset();
        void SetLED(bool enable);
        void TakeSnapshot(uint8_t numberOfSnaps);
        void SetFilterType(FilterType setFilter);
        void SetTurretAngle(units::degree_t turretAngle);

    private:
        units::degree_t turretAngle;
        units::inch_t estDist = 0.0_in;

        // TODO: ensure correctness @Jordan
        // photonlib::PhotonCamera camera{""};
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
        std::list<double> turretList;

        double alpha = 0.2; // Weight
        uint8_t N = 10;        // Note, EMA will have a spinup interval of about 20ms * N
        units::second_t sampleSpinUpDelay = 0.3_s;
        double estimatedPhaseShift = (double)(N + 2) / 2.0;
        units::second_t estimatedPhaseShiftTime = 20_ms * ((double)(N + 1) / 2.0);

        double CalculateEMA(const std::list<double> &list);
        double CalculateAverage(const std::list<double> &list);

        FilterType filter;

        frc::Timer spinUpTimer;
        bool spinUpOS = false;
    };
} // namespace vision