#include "auto/AutoLine.hpp"

#include "lib/AutoHelper.h"

#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/trajectory/constraint/CentripetalAccelerationConstraint.h>

#include <iostream>

// Name for Smart Dash Chooser
std::string AutoLine::GetName()
{
    return "01 - Line";
}

// Initialization
// Constructor requires a reference to the robot map
AutoLine::AutoLine(Robotmap &IO) : IO(IO)
{
    m_state = 0;
}

AutoLine::~AutoLine() {}

//State Machine
void AutoLine::NextState()
{
    m_state++;
    m_autoTimer.Reset();
    m_autoTimer.Start();
}

void AutoLine::Init()
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

    m_trajectory = rj::AutoHelper::LoadTrajectory("01 - Line", &config);

    m_autoTimer.Reset();
    m_autoTimer.Start();

    auto pose = m_trajectory.InitialPose();
    IO.drivetrain.ResetOdometry(pose);
    // IO.drivetrain.GetField().GetObject("traj")->SetTrajectory(m_trajectory);
    // std::cout << m_trajectory.States().size() << ", " << m_trajectory.TotalTime().value() << std::endl;
}

// Execute the program
void AutoLine::Run()
{
    switch (m_state)
    {
    case 0:
    {
        // std::cout << m_autoTimer.Get().value() << std::endl;
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
    default:
    {
        // std::cout << "Done!" << std::endl;
        IO.drivetrain.Arcade(0.0, 0.0);
    }
    }
}

// Called Automagically by AutoPrograms (RobotPeriodic)
void AutoLine::UpdateSmartDash()
{
    frc::SmartDashboard::PutNumber("Auto/State", m_state);
}