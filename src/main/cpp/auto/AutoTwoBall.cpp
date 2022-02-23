#include "auto/AutoTwoBall.hpp"

#include "lib/AutoHelper.h"

#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/trajectory/constraint/CentripetalAccelerationConstraint.h>

#include <iostream>

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
            IO.shooter.SetShooterRPM(3025_rpm);
            IO.shooter.SetIndexer(4_V);
            IO.shooter.SetFeeder(-2_V);

            break;
        }
        case 2:
        {
            IO.drivetrain.Arcade(0.0, 0.0);
            IO.shooter.SetFeeder(6_V);
            break;
        }
        case 3:
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

void AutoTwoBall::Init()
{
    units::feet_per_second_t maxLinearVel = 5_fps;
    // units::standard_gravity_t maxCentripetalAcc = 0.5_SG;
    units::feet_per_second_squared_t maxLinearAcc = 2_mps_sq;

    // frc::TrajectoryConfig config(Drivetrain::kMaxSpeedLinear, Drivetrain::kMaxAccelerationLinear);
    frc::TrajectoryConfig config(maxLinearVel, maxLinearAcc);
    config.AddConstraint(frc::CentripetalAccelerationConstraint{maxLinearAcc});
    config.AddConstraint(frc::DifferentialDriveVoltageConstraint{IO.drivetrain.GetFeedForward(), IO.drivetrain.GetKinematics(), 8_V});
    config.AddConstraint(frc::DifferentialDriveKinematicsConstraint{IO.drivetrain.GetKinematics(), maxLinearVel});
    config.SetReversed(false);

    m_trajectory = rj::AutoHelper::LoadTrajectory("02 - Two Ball", &config);

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
            NextState();

            break;
        }
        case 1:
        {
            auto reference = m_trajectory.Sample(m_autoTimer.Get());

            frc::SmartDashboard::PutNumber("traj/t", reference.t.value());
            frc::SmartDashboard::PutNumber("traj/x", reference.pose.Translation().X().value());
            frc::SmartDashboard::PutNumber("traj/y", reference.pose.Translation().Y().value());
            frc::SmartDashboard::PutNumber("traj/theta", reference.pose.Rotation().Radians().value());
            frc::SmartDashboard::PutNumber("traj/k", reference.curvature.value());
            frc::SmartDashboard::PutNumber("traj/v", reference.velocity.value());
            frc::SmartDashboard::PutNumber("traj/a", reference.acceleration.value());

            IO.drivetrain.Drive(reference);

            if ((m_autoTimer.Get() > m_trajectory.TotalTime()+1_s))
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