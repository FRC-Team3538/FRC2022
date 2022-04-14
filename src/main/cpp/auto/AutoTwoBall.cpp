#include "auto/AutoTwoBall.hpp"
#include <frc/trajectory/constraint/CentripetalAccelerationConstraint.h>
#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include "Robotmap.hpp"
#include "frc/Timer.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc/trajectory/Trajectory.h"
#include "frc/trajectory/TrajectoryConfig.h"
#include "lib/AutoHelper.h"
#include "subsystems/Drivetrain.hpp"
#include "subsystems/Shooter.hpp"
#include "units/acceleration.h"
#include "units/angular_velocity.h"
#include "units/base.h"
#include "units/time.h"
#include "units/velocity.h"
#include "units/voltage.h"

// Name for Smart Dash Chooser
std::string AutoTwoBall::GetName()
{
    return "02 - Two Ball";
}

// Initialization
// Constructor requires a reference to the robot map
AutoTwoBall::AutoTwoBall(Robotmap &IO) : IO(IO)
{
    m_state = 0;
}

AutoTwoBall::~AutoTwoBall() {}

// State Machine
void AutoTwoBall::NextState()
{
    m_state++;
    IO.drivetrain.Arcade(0.0, 0.0);
    m_autoTimer.Reset();
    m_autoTimer.Start();
}

void AutoTwoBall::Init()
{
    units::feet_per_second_t maxLinearVel = 5_fps;
    // units::standard_gravity_t maxCentripetalAcc = 0.5_SG;
    units::feet_per_second_squared_t maxLinearAcc = 1_mps_sq;

    // frc::TrajectoryConfig config(Drivetrain::kMaxSpeedLinear, Drivetrain::kMaxAccelerationLinear);
    frc::TrajectoryConfig config(maxLinearVel, maxLinearAcc);
    config.AddConstraint(frc::CentripetalAccelerationConstraint{maxLinearAcc});
    config.AddConstraint(frc::DifferentialDriveVoltageConstraint{IO.drivetrain.GetFeedForward(), IO.drivetrain.GetKinematics(), 12_V});
    config.AddConstraint(frc::DifferentialDriveKinematicsConstraint{IO.drivetrain.GetKinematics(), maxLinearVel});
    config.SetReversed(false);

    m_trajectory = rj::AutoHelper::LoadTrajectorySplit("02 - Two Ball", &config)[0];

    m_autoTimer.Reset();
    m_autoTimer.Start();

    IO.drivetrain.ResetOdometry(m_trajectory.InitialPose());
}

// Execute the program
void AutoTwoBall::Run()
{
    switch (m_state)
    {
    case 0:
    {
        IO.drivetrain.Arcade(0.2, 0.0);

        IO.shooter.SetIntakeState(Shooter::Position::Deployed);
        IO.shooter.SetIntake(8_V);
        IO.shooter.SetShooterRPM(3100_rpm);
        IO.shooter.SetIndexer(3_V);
        IO.shooter.SetFeeder(-2_V);

        //  auto reference = m_trajectory.Sample(m_autoTimer.Get());
        //  IO.drivetrain.Drive(reference);
        //  if (m_autoTimer.Get() > m_trajectory.TotalTime() + 1.5_s)

        if (m_autoTimer.Get() > 2.0_s)
        {
            NextState();
        }

        break;
    }
    case 1:
    {
        IO.drivetrain.Arcade(0.0, 0.0);

        IO.shooter.SetFeeder(4_V);

        if (m_autoTimer.Get() > 4.0_s)
        {
            NextState();
        }
        break;
    }
    case 2:
    {
        IO.drivetrain.Arcade(0.0, 0.0);

        IO.shooter.SetIntakeState(Shooter::Position::Stowed);
        IO.shooter.SetIntake(0_V);
        IO.shooter.SetShooterRPM(0_rpm);
        IO.shooter.SetIndexer(0_V);
        IO.shooter.SetFeeder(0_V);

        NextState();
        break;
    }
    default:
    {
        // std::cout << "Done!" << std::endl;
        IO.drivetrain.Arcade(0.0, 0.0);
    }
    }
}

// Called Automagically by AutoPrograms (RobotPeriodic)
void AutoTwoBall::UpdateSmartDash()
{
    frc::SmartDashboard::PutNumber("Auto/State", m_state);
}