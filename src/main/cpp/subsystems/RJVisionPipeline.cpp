#include "subsystems/RJVisionPipeline.hpp"

using namespace nt;

namespace vision
{

    RJVisionPipeline::RJVisionPipeline()
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
            if (spinupInterval)
            {
                if (emaList.size() >= N)
                {
                    emaList.pop_front();
                    emaList.push_back(dx);

                    telemetry.angle = units::degree_t{CalculateEMA()};
                    telemetry.distance = DistEstimation();

                    telemetry.filled = true;
                }
                else
                {
                    emaList.push_back(dx);
                    telemetry.filled = false;
                }
            }
            else
            {
                if (emaList.size() >= N)
                    emaList.pop_front();

                emaList.push_back(dx);

                telemetry.angle = units::degree_t{CalculateEMA()};
                telemetry.distance = DistEstimation();

                // Maybe put back spinup interval for accuracy and smoothness on start?
                // Will cause about a 160ms delay currently tho

                telemetry.filled = true;
            }
        }
        else
        {
            // telemetry.angle = 420.0;
            // telemetry.distance = -1.0;
            telemetry.filled = false;
        }

        return telemetry;
    }

    double RJVisionPipeline::CalculateEMA()
    {
        // EXPONENTIAL MOVING AVERAGE. Maybe add an IQR Filter? Also maybe use an actual circular buffer?
        // Could also try using a Butterworth

        double value = 0.0;

        for (auto i = emaList.begin(); i != emaList.end(); ++i)
        {
            value = (alpha * (*i)) + ((1 - alpha) * value);
        }

        return value;
    }

    void RJVisionPipeline::UpdateTelemetry()
    {
        frc::SmartDashboard::PutNumber("dx", dx);
        frc::SmartDashboard::PutNumber("Vision Dist", estDist.value());
    }

    void RJVisionPipeline::Reset()
    {
        emaList.clear();
    }

    units::inch_t RJVisionPipeline::DistEstimation()
    {
        // For now, the assumption is that the dy remains accurate despite the dx changing
        // Might need to change that for rootin, tootin, scootin, n shootin

        units::inch_t dist = deltaH / (tan((dy + cameraAngle.value()) * (3.1415 / 180.0)));
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