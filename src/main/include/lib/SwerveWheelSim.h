#include <frc/simulation/FlywheelSim.h>
#include <units/velocity.h>

class SwerveWheelSim : public frc::sim::FlywheelSim
{
public:
    SwerveWheelSim(
        frc::LinearSystem<1, 1, 1> plant, 
        frc::DCMotor motor, 
        double gearboxRatio, 
        units::meter_t wheel_radius, 
        const std::array<double, 1>& measurementStdDevs = {0.0});

    units::meters_per_second_t GetLinearVelocity() const;

private:
    units::meter_t wheel_radius;
};