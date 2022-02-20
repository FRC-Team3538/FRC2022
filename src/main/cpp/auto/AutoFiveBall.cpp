#include "auto/AutoFiveBall.hpp"

#include "lib/AutoHelper.h"

#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/trajectory/constraint/CentripetalAccelerationConstraint.h>

#include <iostream>

// Name for Smart Dash Chooser
std::string AutoFiveBall::GetName()
{
    return "10 - Five Ball Wall Ball";
}

// Initialization
// Constructor requires a reference to the robot map
AutoFiveBall::AutoFiveBall(Robotmap &IO) : IO(IO)
{
    m_state = 0;
}

AutoFiveBall::~AutoFiveBall() {}

// State Machine
void AutoFiveBall::NextState()
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
            IO.shooter.SetShooterTopRPM(2400_rpm);
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
            IO.shooter.SetIntakeState(Shooter::Position::Deployed);
            IO.shooter.SetIntake(8_V);
            IO.shooter.SetShooterRPM(4000_rpm);
            IO.shooter.SetShooterTopRPM(2400_rpm);
            IO.shooter.SetIndexer(8_V);
            IO.shooter.SetFeeder(-2_V);

            IO.drivetrain.ResetOdometry(m_trajectory_second.InitialPose());

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
            IO.shooter.SetIntakeState(Shooter::Position::Deployed);
            IO.shooter.SetIntake(8_V);
            IO.shooter.SetShooterRPM(4000_rpm);
            IO.shooter.SetShooterTopRPM(2400_rpm);
            IO.shooter.SetIndexer(8_V);
            IO.shooter.SetFeeder(-2_V);
            break;
        }
        case 6:
        {
            IO.drivetrain.Arcade(0.0, 0.0);
            IO.shooter.SetFeeder(8_V);

            break;
        }
        case 7:
        {
            IO.drivetrain.Arcade(0.0, 0.0);
            IO.shooter.SetIntakeState(Shooter::Position::Stowed);
            IO.shooter.SetIntake(0_V);
            IO.shooter.SetShooterRPM(0_rpm);
            IO.shooter.SetShooterTopRPM(0_rpm);
            IO.shooter.SetIndexer(0_V);
            IO.shooter.SetFeeder(0_V);

            break;
        }
        default:
        {
            IO.drivetrain.Arcade(0.0, 0.0);
            break;
        }
    }

    m_autoTimer.Reset();
    m_autoTimer.Start();
}

void AutoFiveBall::Init()
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

    m_trajectory_first = rj::AutoHelper::LoadTrajectory("5 Ball Wall Ball 1", &config);

    config.SetReversed(true);
    m_trajectory_second = rj::AutoHelper::LoadTrajectory("5 Ball Wall Ball 2", &config);

    IO.drivetrain.ResetOdometry(m_trajectory_first.InitialPose());

    m_autoTimer.Reset();
    m_autoTimer.Start();
}

// Execute the program
void AutoFiveBall::Run()
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
            if (IO.shooter.Shoot())
            {
                NextState();
            }

            break;
        }
        case 2:
        {
            if (m_autoTimer.Get() > units::time::second_t(2.0))
            {
                NextState();
            }

            break;
        }
        case 3:
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

            if ((m_autoTimer.Get() > m_trajectory_first.TotalTime() && IO.shooter.Shoot()))
            {
                NextState();
            }

            break;
        }
        case 4:
        {
            m_autoTimer.Reset();
            m_autoTimer.Start();
            if (m_autoTimer.Get() > units::time::second_t(2.0))
            {
                NextState();
            }

            break;
        }
        case 5:
        {
            auto reference = m_trajectory_second.Sample(m_autoTimer.Get());

            frc::SmartDashboard::PutNumber("traj/t", reference.t.value());
            frc::SmartDashboard::PutNumber("traj/x", reference.pose.Translation().X().value());
            frc::SmartDashboard::PutNumber("traj/y", reference.pose.Translation().Y().value());
            frc::SmartDashboard::PutNumber("traj/theta", reference.pose.Rotation().Radians().value());
            frc::SmartDashboard::PutNumber("traj/k", reference.curvature.value());
            frc::SmartDashboard::PutNumber("traj/v", reference.velocity.value());
            frc::SmartDashboard::PutNumber("traj/a", reference.acceleration.value());

            IO.drivetrain.Drive(reference);

            if ((m_autoTimer.Get() > m_trajectory_second.TotalTime() && IO.shooter.Shoot()))
            {
                NextState();
            }

            break;
        }
        case 6:
        {
            if (m_autoTimer.Get() > units::time::second_t(2.0))
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
void AutoFiveBall::UpdateSmartDash()
{
    frc::SmartDashboard::PutNumber("Auto State", m_state);
}