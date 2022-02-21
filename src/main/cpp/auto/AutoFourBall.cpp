#include "auto/AutoFourBall.hpp"

#include "lib/AutoHelper.h"

#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/trajectory/constraint/CentripetalAccelerationConstraint.h>

#include <iostream>

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

            //IO.drivetrain.ResetOdometry(m_trajectory_second.InitialPose());
            startPathTime = m_autoTimer.Get();
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
    units::feet_per_second_t maxLinearVel = 2_mps;
    // units::standard_gravity_t maxCentripetalAcc = 0.5_SG;
    units::feet_per_second_squared_t maxLinearAcc = 2_mps_sq;

    // frc::TrajectoryConfig config(Drivetrain::kMaxSpeedLinear, Drivetrain::kMaxAccelerationLinear);
    frc::TrajectoryConfig config(maxLinearVel, maxLinearAcc);
    config.AddConstraint(frc::CentripetalAccelerationConstraint{5_mps_sq});
    config.AddConstraint(frc::DifferentialDriveVoltageConstraint{IO.drivetrain.GetFeedForward(), IO.drivetrain.GetKinematics(), 10_V});
    config.AddConstraint(frc::DifferentialDriveKinematicsConstraint{IO.drivetrain.GetKinematics(), 2_mps});
    config.SetReversed(false);

    m_trajectory_first = rj::AutoHelper::LoadTrajectory("04 - Four Ball Pt 2", &config);

    config.SetReversed(false);
    m_trajectory_second = rj::AutoHelper::LoadTrajectory("04 - Four Ball Pt 2", &config);

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

            frc::SmartDashboard::PutNumber("traj/t", reference.t.value());
            frc::SmartDashboard::PutNumber("traj/x", reference.pose.Translation().X().value());
            frc::SmartDashboard::PutNumber("traj/y", reference.pose.Translation().Y().value());
            frc::SmartDashboard::PutNumber("traj/theta", reference.pose.Rotation().Radians().value());
            frc::SmartDashboard::PutNumber("traj/k", reference.curvature.value());
            frc::SmartDashboard::PutNumber("traj/v", reference.velocity.value());
            frc::SmartDashboard::PutNumber("traj/a", reference.acceleration.value());

            IO.drivetrain.Drive(reference);

            if ((m_autoTimer.Get() > m_trajectory_first.TotalTime()))
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
            auto reference = m_trajectory_second.Sample(m_autoTimer.Get() - startPathTime);

            frc::SmartDashboard::PutNumber("traj/t", reference.t.value());
            frc::SmartDashboard::PutNumber("traj/x", reference.pose.Translation().X().value());
            frc::SmartDashboard::PutNumber("traj/y", reference.pose.Translation().Y().value());
            frc::SmartDashboard::PutNumber("traj/theta", reference.pose.Rotation().Radians().value());
            frc::SmartDashboard::PutNumber("traj/k", reference.curvature.value());
            frc::SmartDashboard::PutNumber("traj/v", reference.velocity.value());
            frc::SmartDashboard::PutNumber("traj/a", reference.acceleration.value());

            IO.drivetrain.Drive(reference);

            if (((m_autoTimer.Get() - startPathTime) > m_trajectory_second.TotalTime()))
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