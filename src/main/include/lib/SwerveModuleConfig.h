#include <units/velocity.h>
#include <units/voltage.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/acceleration.h>

#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>


template <class Distance>
class PIDConfig
{
public:
    using Distance_t = units::unit_t<Distance>;
    using State = typename frc::TrapezoidProfile<Distance>::State;
    using Constraints = typename frc::TrapezoidProfile<Distance>::Constraints;
    constexpr PIDConfig(
        double kP,
        double kI,
        double kD,
        Constraints constraints):
            kP(kP),
            kI(kI),
            kD(kD),
            constraints(constraints)
    {}

    constexpr frc::ProfiledPIDController<Distance> GetPIDController()
    {
        return frc::ProfiledPIDController<Distance>(kP, kI, kD, constraints);
    }

    double kP;
    double kI;
    double kD;
    Constraints constraints;
};

template <class Distance>
class FFConfig
{
public:
    using Distance_t = units::unit_t<Distance>;
    using Velocity =
      units::compound_unit<Distance, units::inverse<units::seconds>>;
    using Velocity_t = units::unit_t<Velocity>;
    using Acceleration =
      units::compound_unit<Velocity, units::inverse<units::seconds>>;
    using Acceleration_t = units::unit_t<Acceleration>;

    using kV_unit = units::compound_unit<units::volt, units::inverse<Velocity>>;
    using kV_t = units::unit_t<kV_unit>;

    using kA_unit = units::compound_unit<units::volt, units::inverse<Acceleration>>;
    using kA_t = units::unit_t<kA_unit>;

    constexpr FFConfig(
        units::volt_t kS,
        kV_t kV,
        kA_t kA):
            kS(kS),
            kV(kV),
            kA(kA)
    {}

    constexpr frc::SimpleMotorFeedforward<Distance> GetFeedForward()
    {
        return frc::SimpleMotorFeedforward<Distance>(kS, kV, kA);
    }

    units::volt_t kS;
    kV_t kV;
    kA_t kA;
};

class SwerveModuleConfig
{
public:
    SwerveModuleConfig() = delete;
    SwerveModuleConfig(
        units::degree_t angleOffset,
        PIDConfig<units::meters_per_second> drivePID,
        PIDConfig<units::radian> turnPID,
        FFConfig<units::meter> driveFf,
        FFConfig<units::radian> turnFf):
            angleOffset(angleOffset),
            drivePID(drivePID),
            turnPID(turnPID),
            driveFf(driveFf),
            turnFf(turnFf)
    {}

    units::degree_t angleOffset;
    PIDConfig<units::meters_per_second> drivePID;
    PIDConfig<units::radian> turnPID;
    FFConfig<units::meter> driveFf;
    FFConfig<units::radian> turnFf;
};