#include "subsystems/RJVisionPipeline.hpp"

#include <photonlib/PhotonUtils.h>

using namespace nt;

namespace vision
{

    RJVisionPipeline::RJVisionPipeline(Shooter &shooter, FilterType filter) : shooter(shooter), filter(filter)
    {
        frc::SmartDashboard::PutNumber("ALPHA", 0.125);
        frc::SmartDashboard::PutNumber("N", 8);
    }

    void RJVisionPipeline::ConfigureSystem()
    {
        table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
        // table->PutNumber("ledMode", 1.0);
        table->PutNumber("pipeline", 0.0);
        table->PutNumber("camMode", 0.0);
        camera.SetPipelineIndex(0);
        camera.SetDriverMode(false);
    }

    void RJVisionPipeline::Periodic()
    {
        auto result = camera.GetLatestResult();
        tv = result.HasTargets();
        if (result.HasTargets()) {
            auto target = result.GetBestTarget();
            dx = target.GetYaw();
            dy = target.GetPitch();
            tv = 1.0;
        }
        dx = -(table->GetNumber("tx", dx));
        dy = table->GetNumber("ty", dy);
        tv = table->GetNumber("tv", tv);

        alpha = frc::SmartDashboard::GetNumber("ALPHA", 0.125);
        N = frc::SmartDashboard::GetNumber("N", 8);
        // if (snapShotTimer.Get() > snapTime)
        // {
        //     table->PutNumber("snapshot", 0.0);
        // }
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
                    xList.push_back(dx + shooter.GetTurretAngle().value());

                    yList.pop_front();
                    yList.push_back(dy);

                    // telemetry.deltaX = units::degree_t{CalculateEMA(xList)};
                    telemetry.distance = DistEstimation(units::degree_t{CalculateEMA(yList)}, telemetry.deltaX);
                    // telemetry.turretAngle = telemetry.deltaX + shooter.GetTurretAngle();
                    telemetry.turretAngle = units::degree_t{CalculateEMA(xList)};

                    telemetry.filled = true;
                }
                else
                {
                    xList.push_back(dx + shooter.GetTurretAngle().value());
                    yList.push_back(dy);
                    telemetry.filled = false;
                }
            }
            break;

            case FilterType::SMA:
            {
                if (xList.size() >= N)
                {
                    xList.pop_front();
                    xList.push_back(dx + shooter.GetTurretAngle().value());

                    yList.pop_front();
                    yList.push_back(dy);

                    // telemetry.deltaX = units::degree_t{CalculateEMA(xList)};
                    telemetry.distance = DistEstimation(units::degree_t{CalculateAverage(yList)}, telemetry.deltaX);
                    // telemetry.turretAngle = telemetry.deltaX + shooter.GetTurretAngle();
                    telemetry.turretAngle = units::degree_t{CalculateAverage(xList)};

                    telemetry.filled = true;
                }
                else
                {
                    xList.push_back(dx + shooter.GetTurretAngle().value());
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

                xList.push_back(dx + shooter.GetTurretAngle().value());
                yList.push_back(dy);

                // telemetry.deltaX = units::degree_t{CalculateEMA(xList)};
                telemetry.distance = DistEstimation(units::degree_t{CalculateEMA(yList)}, telemetry.deltaX);
                telemetry.turretAngle = units::degree_t{CalculateEMA(xList)}; // telemetry.deltaX + shooter.GetTurretAngle();

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
                        xList.push_back((dx + shooter.GetTurretAngle().value()));
                        yList.push_back(dy);
                        telemetry.filled = false;
                    }
                }
            }
            break;

            // case FilterType::MedianFilter:
            // {
                
            // }

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
            // std::cout << value << ", ";

            for (auto i = ++list.begin(); i != list.end(); ++i)
            {
                value = (alpha * (*i)) + ((1 - alpha) * value);
                // std::cout << *i << ", ";
            }
            // std::cout << value << std::endl;
        }

        return value;
    }

    double RJVisionPipeline::CalculateAverage(const std::list<double> &list)
    {
        double value = 0.0;

        for (auto i = list.begin(); i != list.end(); ++i)
        {
            value += *i;
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
        xList.clear();
        yList.clear();
        spinUpOS = false;
    }

    units::inch_t RJVisionPipeline::DistEstimation(units::degree_t deltaY, units::degree_t deltaX)
    {
        // return photonlib::PhotonUtils::CalculateDistanceToTarget(32_in, 102_in, 33_deg, deltaY);
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
            table->PutNumber("ledMode", 3.0); // Force On
        }
        else
        {
            camera.SetLEDMode(photonlib::LEDMode::kOff);
            table->PutNumber("ledMode", 1.0); // Force Off
        }
    }

    void RJVisionPipeline::TakeSnapshot(uint8_t numberOfSnaps)
    {
        camera.TakeInputSnapshot();
        camera.TakeOutputSnapshot();
        table->PutNumber("snapshot", (double)numberOfSnaps);
        // snapTime = units::second_t{(double)numberOfSnaps / 2.0};
        // snapShotTimer.Reset();
        // snapShotTimer.Start();
    }
} // namespace vision