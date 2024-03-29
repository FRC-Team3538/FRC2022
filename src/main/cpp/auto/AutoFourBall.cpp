#include "auto/AutoFourBall.hpp"
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
std::string AutoFourBall::GetName()
{
    return "04 - Four Ball";
}

// Initialization
// Constructor requires a reference to the robot map
AutoFourBall::AutoFourBall(Robotmap &IO) : IO(IO)
{
    m_state = 0;
}

AutoFourBall::~AutoFourBall() {}

// State Machine
void AutoFourBall::NextState()
{
    m_state++;

    switch(m_state)
    {
        case 0:
        {
            break;
        }
        case 1:
        {
            IO.shooter.SetIntakeState(Shooter::Position::Deployed);
            IO.shooter.SetIntake(8_V);
            IO.shooter.SetShooterRPM(4000_rpm);
            IO.shooter.SetIndexer(8_V);
            IO.shooter.SetFeeder(-2_V);

            break;
        }
        case 2:
        {
            IO.drivetrain.Arcade(0.0, 0.0);
            IO.shooter.SetFeeder(8_V);

            break;
        }
        case 3:
        {
            IO.shooter.SetFeeder(-2_V);

            break;
        }
        case 4:
        {
            IO.drivetrain.Arcade(0.0, 0.0);
            IO.shooter.SetFeeder(8_V);

            break;
        }
        case 5:
        {
            IO.shooter.SetIntakeState(Shooter::Position::Stowed);
            IO.shooter.SetIntake(0_V);
            IO.shooter.SetShooterRPM(0_rpm);
            IO.shooter.SetIndexer(0_V);
            IO.shooter.SetFeeder(0_V);
            break;
        }
        default:
        {
            break;
        }
    }

    m_autoTimer.Reset();
    m_autoTimer.Start();
}

void AutoFourBall::Init()
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

    auto trajectories = rj::AutoHelper::LoadTrajectorySplit("04 - Four Ball", &config);

    m_trajectory_first = trajectories[0];
    m_trajectory_second = trajectories[1] + trajectories[2];

    IO.drivetrain.ResetOdometry(m_trajectory_first.InitialPose());

    m_autoTimer.Reset();
    m_autoTimer.Start();
}

// Execute the program
void AutoFourBall::Run()
{
    switch (m_state)
    {
        case 0:
        {        
            NextState();

            break;
        }
        case 1:
        {
            auto reference = m_trajectory_first.Sample(m_autoTimer.Get());

            IO.drivetrain.Drive(reference);

            if (m_autoTimer.Get() > m_trajectory_first.TotalTime() + 1.5_s)
            {
                NextState();
            }
            break;
        }
        case 2:
        {
            if (IO.shooter.Shoot())
            {
                NextState();
            }
            break;
        }
        case 3:
        {
            auto reference = m_trajectory_second.Sample(m_autoTimer.Get());

            IO.drivetrain.Drive(reference);

            if (m_autoTimer.Get() > m_trajectory_second.TotalTime())
            {
                NextState();
            }
            break;
        }
        case 4:
        {
            if (IO.shooter.Shoot())
            {
                NextState();
            }
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
void AutoFourBall::UpdateSmartDash()
{
    frc::SmartDashboard::PutNumber("Auto/State", m_state);
}