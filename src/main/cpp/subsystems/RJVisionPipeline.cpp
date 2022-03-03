#include "subsystems/RJVisionPipeline.hpp"

using namespace nt;

namespace vision
{

    RJVisionPipeline::RJVisionPipeline(FilterType filter) : filter(filter)
    {
    }

    void RJVisionPipeline::ConfigureSystem()
    {
        table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
        // table->PutNumber("ledMode", 1.0);
        table->PutNumber("pipeline", 0.0);
        table->PutNumber("camMode", 0.0);
    }

    void RJVisionPipeline::Periodic()
    {
        dx = -(table->GetNumber("tx", 0.0));
        dy = table->GetNumber("ty", 0.0);
        tv = table->GetNumber("tv", 0.0);
    }

    RJVisionPipeline::visionData RJVisionPipeline::Run()
    {
        SetLED(true);

        RJVisionPipeline::visionData telemetry;

        if (tv != 0.0)
        {
            switch (filter)
            {

            case FilterType::EMANoSpinup:
            {
                if (xList.size() >= N)
                {
                    xList.pop_front();
                    xList.push_back(dx);

                    yList.pop_front();
                    yList.push_back(dy);

                    telemetry.angle = units::degree_t{CalculateEMA(xList)};
                    telemetry.distance = DistEstimation(units::degree_t{CalculateEMA(yList)}, telemetry.angle);

                    telemetry.filled = true;
                }
                else
                {
                    xList.push_back(dx);
                    yList.push_back(dy);
                    telemetry.filled = false;
                }
            }
            break;

            case FilterType::EMAWithSpinup:
            {
                if (xList.size() >= N)
                {
                    xList.pop_front();
                    yList.pop_front();
                }

                xList.push_back(dx);
                yList.push_back(dy);

                telemetry.angle = units::degree_t{CalculateEMA(xList)};
                telemetry.distance = DistEstimation(units::degree_t{CalculateEMA(yList)}, telemetry.angle);

                telemetry.filled = true;
            }
            break;

            case FilterType::SampleAverage:
            {
                if (xList.size() >= N)
                {
                    telemetry.angle = units::degree_t{CalculateAverage(xList)};
                    telemetry.distance = DistEstimation(units::degree_t{CalculateAverage(yList)}, telemetry.angle);

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

                    if (spinUpTimer.Get() > 0.3_s)
                    {
                        xList.push_back(dx);
                        yList.push_back(dy);
                        telemetry.filled = false;
                    }
                }
            }
            break;

            case FilterType::NoFilter:
            {
                telemetry.angle = units::degree_t{dx};
                telemetry.distance = DistEstimation(units::degree_t{dy}, telemetry.angle);
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

    double RJVisionPipeline::CalculateEMA(const std::list<double> &list)
    {
        // EXPONENTIAL MOVING AVERAGE. Maybe add an IQR Filter? Also maybe use an actual circular buffer?
        // Could also try using a Butterworth

        double value = 0.0;

        for (auto i = list.begin(); i != list.end(); ++i)
        {
            value = (alpha * (*i)) + ((1 - alpha) * value);
        }

        return value;
    }

    double RJVisionPipeline::CalculateAverage(const std::list<double> &list)
    {
        double value = 0.0;

        for (auto i = list.begin(); i != list.end(); ++i)
        {
            value += *i;
        }

        value /= list.size();

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
        // For now, the assumption is that the dy remains accurate despite the dx changing
        // Might need to change that for rootin, tootin, scootin, n shootin

        deltaX *= deltaX < 0.0_deg ? -1.0 : 1.0;

        deltaY += units::degree_t{-0.007 + (0.0564 * pow(deltaX.value(), 1)) + (-0.01538 * pow(deltaX.value(), 2)) + (0.0003375 * pow(deltaX.value(), 3))};

        units::inch_t dist = deltaH / (tan((deltaY.value() + cameraAngle.value()) * (3.1415 / 180.0)));
        estDist = dist;
        return estDist;
    }

    void RJVisionPipeline::SetLED(bool enable)
    {
        if (enable)
        {
            table->PutNumber("ledMode", 3.0); // Force On
        }
        else
        {
            table->PutNumber("ledMode", 1.0); // Force Off
        }
    }
} // namespace vision