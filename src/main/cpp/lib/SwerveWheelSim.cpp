#include <lib/SwerveWheelSim.h>

SwerveWheelSim::SwerveWheelSim(
        frc::LinearSystem<1, 1, 1> plant, 
        frc::DCMotor motor, 
        double gearboxRatio, 
        units::meter_t wheel_radius, 
        const std::array<double, 1>& measurementStdDevs):
    FlywheelSim(plant, motor, gearboxRatio, measurementStdDevs),
    wheel_radius(wheel_radius)
{}

units::meters_per_second_t SwerveWheelSim::GetLinearVelocity() const {
    return wheel_radius * GetAngularVelocity() / 1_rad;
}