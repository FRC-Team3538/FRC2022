#include "auto/Twoball.hpp"

#include "lib/AutoHelper.h"

#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/trajectory/constraint/CentripetalAccelerationConstraint.h>

#include <iostream>

// Name for Smart Dash Chooser
std::string Twoball::GetName()
{
    return "7 - Twoball";
}

// Initialization
// Constructor requires a reference to the robot map
Twoball::Twoball(Robotmap &IO) : IO(IO)
{
    m_state = 0;
}

Twoball::~Twoball() {}

// State Machine
void Twoball::NextState()
{
    m_state++;
    m_autoTimer.Reset();
    m_autoTimer.Start();
}

void Twoball::Init()
{

    units::feet_per_second_t maxLinearVel = 4_fps;
    // units::standard_gravity_t maxCentripetalAcc = 0.5_SG;
    units::feet_per_second_squared_t maxLinearAcc = 4_fps_sq;

    // frc::TrajectoryConfig config(Drivetrain::kMaxSpeedLinear, Drivetrain::kMaxAccelerationLinear);
    frc::TrajectoryConfig config(maxLinearVel, maxLinearAcc);
    // config.AddConstraint(frc::CentripetalAccelerationConstraint{5_mps_sq});
    // config.AddConstraint(frc::DifferentialDriveVoltageConstraint{IO.drivetrain.GetFeedForward(), IO.drivetrain.GetKinematics(), 5_V});
    // config.AddConstraint(frc::DifferentialDriveKinematicsConstraint{IO.drivetrain.GetKinematics(), 4_fps});
    config.SetReversed(false);

    m_trajectory = rj::AutoHelper::LoadTrajectory("2 Ball Blue", &config);

    m_autoTimer.Reset();
    m_autoTimer.Start();

    auto pose = m_trajectory.InitialPose();
    IO.drivetrain.ResetOdometry(pose);
    // IO.drivetrain.GetField().GetObject("traj")->SetTrajectory(m_trajectory);
    // std::cout << m_trajectory.States().size() << ", " << m_trajectory.TotalTime().value() << std::endl;
}

// Execute the program
void Twoball::Run()
{
    switch (m_state)
    {
    case 0:
    {
        IO.shooter.SetIntakeState(Shooter::Position::Deployed);
        IO.shooter.SetIntake(8_V);
        IO.shooter.SetShooterRPM(4000_rpm);
        IO.shooter.SetShooterTopRPM(2400_rpm);
        IO.shooter.SetFeeder(-2_V);
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

        if ((m_autoTimer.Get() > m_trajectory.TotalTime()))
        {
            NextState();
        }
        break;
    }

    case 2:

    {
        IO.drivetrain.Arcade(0.0, 0.0);

        if (IO.shooter.Shoot())
        {
            NextState();
        }
        break;
    }

    case 3:
    {
        IO.shooter.SetIntakeState(Shooter::Position::Stowed);
        IO.shooter.SetIntake(0_V);
        IO.shooter.SetShooterRPM(0_rpm);
        IO.shooter.SetShooterTopRPM(0_rpm);
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
void Twoball::UpdateSmartDash()
{
    frc::SmartDashboard::PutNumber("Auto State", m_state);
}