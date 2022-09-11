#include "subsystems/RJVisionPipeline.h"


#include "frc/smartdashboard/SmartDashboard.h"     // for SmartDashboard
#include "frc/Timer.h"                             // for Timer
#include "networktables/NetworkTable.h"            // for NetworkTable
#include "networktables/NetworkTableInstance.h"    // for NetworkTableInstance
#include "networktables/NetworkTableInstance.inc"  // for NetworkTableInstan...
#include "units/angle.h"                           // for degree_t, radian_t
#include "units/length.h"                          // for inch_t, operator""_in
#include <cmath>                                   // for pow, tan
#include <wpi/DataLog.h>
#include <wpi/timestamp.h>                         // for Now

using namespace nt;

namespace vision
{

    RJVisionPipeline::RJVisionPipeline(FilterType filter) : filter(filter)
    {
        frc::SmartDashboard::PutNumber("ALPHA", 0.2);
        frc::SmartDashboard::PutNumber("N", 10);
    }

    void RJVisionPipeline::ConfigureSystem()
    {
        camera.SetPipelineIndex(0);
        camera.SetDriverMode(false);
    }

    void RJVisionPipeline::Periodic()
    {
        auto result = camera.GetLatestResult();
        tv = result.HasTargets();
        auto target = result.GetBestTarget();
        dx = -target.GetYaw();
        dy = target.GetPitch();

        alpha = frc::SmartDashboard::GetNumber("ALPHA", 0.2);
        N = frc::SmartDashboard::GetNumber("N", 10);
        estimatedPhaseShift = (double)(N + 1) / 2.0;
    }

    RJVisionPipeline::visionData RJVisionPipeline::Run()
    {
        SetLED(true);

        RJVisionPipeline::visionData telemetry;

        if (tv != 0.0)
        {
            switch (filter)
            {

            case FilterType::EMAWithSpinup:
            {
                if (xList.size() >= N)
                {
                    xList.pop_front();
                    xList.push_back(dx + turretAngle.value());

                    yList.pop_front();
                    yList.push_back(dy);

                    // telemetry.deltaX = units::degree_t{CalculateEMA(xList)};
                    telemetry.distance = DistEstimation(units::degree_t{CalculateEMA(yList)}, telemetry.deltaX);
                    // telemetry.turretAngle = telemetry.deltaX + turretAngle;
                    telemetry.turretAngle = units::degree_t{CalculateEMA(xList)};

                    telemetry.filled = true;
                }
                else
                {
                    //std::cout << dx << ", " << turretAngle.value() << std::endl;
                    xList.push_back(dx + turretAngle.value());
                    yList.push_back(dy);
                    telemetry.filled = false;
                }
            }
            break;

            case FilterType::SMA:
            {
                if (xList.size() >= N)
                {
                    turretList.pop_front();
                    turretList.push_back(dx + turretAngle.value());

                    xList.pop_front();
                    xList.push_back(dx);

                    yList.pop_front();
                    yList.push_back(dy);

                    telemetry.deltaX = units::degree_t{CalculateAverage(xList)};
                    telemetry.deltaY = units::degree_t{CalculateAverage(yList)};
                    telemetry.distance = DistEstimation(units::degree_t{CalculateAverage(yList)}, telemetry.deltaX);
                    telemetry.turretAngle = units::degree_t{CalculateAverage(turretList)};

                    telemetry.filled = true;
                }
                else
                {
                    turretList.push_back(dx + turretAngle.value());
                    xList.push_back(dx);
                    yList.push_back(dy);
                    telemetry.filled = false;
                }
            }
            break;

            case FilterType::EMANoSpinup:
            {
                if (xList.size() >= N)
                {
                    xList.pop_front();
                    yList.pop_front();
                }

                xList.push_back(dx + turretAngle.value());
                yList.push_back(dy);

                // telemetry.deltaX = units::degree_t{CalculateEMA(xList)};
                telemetry.distance = DistEstimation(units::degree_t{CalculateEMA(yList)}, telemetry.deltaX);
                telemetry.turretAngle = units::degree_t{CalculateEMA(xList)}; // telemetry.deltaX + turretAngle;

                telemetry.filled = true;
            }
            break;

            case FilterType::SampleAverage:
            {
                if (xList.size() >= N)
                {
                    telemetry.turretAngle = units::degree_t{CalculateAverage(xList)};
                    telemetry.distance = DistEstimation(units::degree_t{CalculateAverage(yList)}, telemetry.deltaX);

                    telemetry.filled = true;
                }
                else
                {

                    if (!spinUpOS)
                    {
                        spinUpTimer.Reset();
                        spinUpTimer.Start();
                        spinUpOS = true;
                    }

                    if (spinUpTimer.Get() > sampleSpinUpDelay)
                    {
                        xList.push_back((dx + turretAngle.value()));
                        yList.push_back(dy);
                        telemetry.filled = false;
                    }
                }
            }
            break;

            case FilterType::NoFilter:
            {
                telemetry.deltaX = units::degree_t{dx};
                telemetry.distance = DistEstimation(units::degree_t{dy}, telemetry.deltaX);
                telemetry.filled = true;
            }
            break;
            }

            return telemetry;

        }
        else
        {
            telemetry.filled = false;
            return telemetry;
        }
    }

    RJVisionPipeline::photonVisionResult RJVisionPipeline::RunPhotonVision()
    {
        RJVisionPipeline::photonVisionResult result;
        uint64_t change_time = rawVisionEntry.GetLastChange();
        result.read_time = units::microsecond_t{(double) change_time};
        result.base_result = camera.GetLatestResult();
        return result;
    }

    void RJVisionPipeline::SetFilterType(FilterType setFilter)
    {
        filter = setFilter;
    }

    double RJVisionPipeline::CalculateEMA(const std::list<double> &list)
    {
        // EXPONENTIAL MOVING AVERAGE. Maybe add an IQR Filter? Also maybe use an actual circular buffer?
        // Could also try using a Butterworth

        double value = *list.begin();

        if (list.size() <= 1)
        {
            return value;
        }
        else
        {
            //std::cout << value << ", ";

            for (auto i = ++list.begin(); i != list.end(); ++i)
            {
                value = (alpha * (*i)) + ((1 - alpha) * value);
                //std::cout << *i << ", ";
            }
        }

        //std::cout << std::endl;
        return value;
    }

    double RJVisionPipeline::CalculateAverage(const std::list<double> &list)
    {
        double value = 0.0;

        for (auto i : list)
        {
            value += i;
            // std::cout << value << ", ";
        }

        value /= list.size();

        // std::cout << value << std::endl;

        return value;
    }

    void RJVisionPipeline::UpdateTelemetry()
    {
        frc::SmartDashboard::PutNumber("dx", dx);
        frc::SmartDashboard::PutNumber("Vision Dist", estDist.value());
    }

    void RJVisionPipeline::Reset()
    {
        turretList.clear();
        xList.clear();
        yList.clear();
        spinUpOS = false;
    }

    units::inch_t RJVisionPipeline::DistEstimation(units::degree_t deltaY, units::degree_t deltaX)
    {
        // For now, the assumption is that the dy remains accurate despite the dx changing
        // Might need to change that for rootin, tootin, scootin, n shootin

        deltaX *= deltaX < 0.0_deg ? -1.0 : 1.0;

        deltaY += units::degree_t{-0.007 + (0.0564 * pow(deltaX.value(), 1)) + (-0.01538 * pow(deltaX.value(), 2)) + (0.0003375 * pow(deltaX.value(), 3))};

        units::inch_t dist = deltaH / (tan((deltaY.value() + cameraAngle.value()) * (3.1415 / 180.0)));
        estDist = dist + 17.0_in;
        return estDist;
    }

    void RJVisionPipeline::SetLED(bool enable)
    {
        if (enable)
        {
            camera.SetLEDMode(photonlib::LEDMode::kOn);
        }
        else
        {
            camera.SetLEDMode(photonlib::LEDMode::kOff);
        }
    }

    void RJVisionPipeline::TakeSnapshot(uint8_t numberOfSnaps)
    {
        if (numberOfSnaps > 0)
        {
            camera.TakeInputSnapshot();
            camera.TakeOutputSnapshot();
        }
    }

    void RJVisionPipeline::SetTurretAngle(units::degree_t angle)
    {
        turretAngle = angle;
    }

    void RJVisionPipeline::RegisterDataEntries(wpi::log::DataLog &log)
    {
        RegisterDataEntry(log, "dx", "double");
        RegisterDataEntry(log, "Vision Dist", "double");
    }

    void RJVisionPipeline::LogDataEntries(wpi::log::DataLog &log)
    {
        log.AppendDouble(GetDataEntry("dx"), dx, 0);
        log.AppendDouble(GetDataEntry("Vision Dist"), estDist.value(), 0);
    }

    void RJVisionPipeline::SimInit()
    {
        
    }

    units::ampere_t RJVisionPipeline::SimPeriodic(units::volt_t battery)
    {
        return 0_A;
    }

} // namespace vision