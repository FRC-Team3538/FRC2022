#pragma once 

#include <lib/wpi/ElevatorSim.h>

class TurretSim : public frc::sim::ElevatorSim
{
public:
    TurretSim(const frc::LinearSystem<2, 1, 1>& plant, const frc::DCMotor& gearbox,
              double gearing, const std::array<double, 1>& measurementStdDevs = {0.0});

    units::radian_t GetAngle() const;
    units::radians_per_second_t GetAngularVelocity() const;
};