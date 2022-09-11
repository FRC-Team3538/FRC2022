#include <lib/TurretSim.h>

TurretSim::TurretSim(
    const frc::LinearSystem<2, 1, 1>& plant, 
    const frc::DCMotor& gearbox,
    double gearing, 
    const std::array<double, 1>& measurementStdDevs):
        ElevatorSim(
            plant, 
            gearbox, 
            gearing,
            1_m, 
            -1_m * std::numeric_limits<double>::infinity(), 
            1_m * std::numeric_limits<double>::infinity(), 
            false,
            measurementStdDevs)
{}

units::radian_t TurretSim::GetAngle() const
{
    return GetPosition() * 1_rad / 1_m;
}

units::radians_per_second_t TurretSim::GetAngularVelocity() const
{
    return GetVelocity() * 1_rad / 1_m;
}
