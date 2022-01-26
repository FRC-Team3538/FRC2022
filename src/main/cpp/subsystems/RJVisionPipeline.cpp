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
        //table->PutNumber("ledMode", 1.0);
        table->PutNumber("pipeline", 0.0);
        table->PutNumber("camMode", 0.0);
    }

    void RJVisionPipeline::Periodic()
    {
        dx = (table->GetNumber("tx", 0.0));
        dy = table->GetNumber("ty", 0.0);
        tv = table->GetNumber("tv", 0.0);
    }

    RJVisionPipeline::visionData RJVisionPipeline::Run()
    {
        if (table->GetNumber("ledMode", 0.0) != 3.0)
        {
            table->PutNumber("ledMode", 3.0);
            lightOn.Reset();
            lightOn.Start();
        }

        RJVisionPipeline::visionData telemetry;

        if (lightOn.Get() > 0.1_s) 
        {
            if (tv == 1.0)
            {
                telemetry.angle = units::degree_t{dx};
                telemetry.distance = DistEstimation();
                telemetry.filled = true;
            }
            else
            {
                // telemetry.angle = 420.0;
                // telemetry.distance = -1.0;
                telemetry.filled = false;
            }
        }
        return telemetry;
    }

    void RJVisionPipeline::UpdateTelemetry()
    {
        frc::SmartDashboard::PutNumber("dx", dx);
        frc::SmartDashboard::PutNumber("Vision Dist", estDist.value());
    }

    void RJVisionPipeline::Reset()
    {
        //table->PutNumber("ledMode", 1.0);
        pipeSwitchOS = false;
        pipeSwitchCt = 0;
    }

    units::inch_t RJVisionPipeline::DistEstimation()
    {
        units::inch_t dist = Constants::deltaH / (tan((dy + Constants::cameraAngle.value()) * (3.1415 / 180.0)));
        estDist = dist;
        return estDist;
    }
} // namespace vision