#include <auto/DriveForwardAuto.h>
#include <frc/smartdashboard/SmartDashboard.h>

DriveForwardAuto::DriveForwardAuto(Robotmap &IO):
    IO(IO)
{}

DriveForwardAuto::~DriveForwardAuto()
{}

void DriveForwardAuto::Init()
{
    speed = units::feet_per_second_t{frc::SmartDashboard::GetNumber("auto/speed (feet per second)", 0)};
    heading = units::degree_t{frc::SmartDashboard::GetNumber("auto/heading (degrees)", 0)};
    duration = units::second_t{frc::SmartDashboard::GetNumber("auto/duration (seconds)", 0)};
    timer.Start();

    IO.drivetrain.SetFieldCentric(false);
    IO.drivetrain.ResetYaw(heading);
}

void DriveForwardAuto::Run()
{
    if (timer.Get() < duration)
    {
        IO.drivetrain.Drive(speed, 0_mps, 0_rad_per_s, false);
    }
    else
    {
        IO.drivetrain.Drive(0_mps, 0_mps, 0_rad_per_s, false);
    }
}

std::string DriveForwardAuto::GetName()
{
    return "Drive Forward (Networktables config)";
}

void DriveForwardAuto::UpdateSmartDash()
{
    return;
}