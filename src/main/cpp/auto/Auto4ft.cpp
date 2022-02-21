#include "auto/Auto4ft.hpp"

#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/trajectory/constraint/CentripetalAccelerationConstraint.h>

#include <lib/AutoHelper.h>

// Name for Smart Dash Chooser
std::string Auto4ft::GetName()
{
    return "90 - Auto4ft";
}

// Initialization
// Constructor requires a reference to the robot map
Auto4ft::Auto4ft(Robotmap &IO) : IO(IO)
{
    m_state = 0;
}

Auto4ft::~Auto4ft() {}

//State Machine
void Auto4ft::NextState()
{
    m_state++;
    m_autoTimer.Reset();
    m_autoTimer.Start();
}

void Auto4ft::Init()
{

    units::feet_per_second_t maxLinearVel = 2_fps;
    // units::standard_gravity_t maxCentripetalAcc = 0.5_SG;
    units::feet_per_second_squared_t maxLinearAcc = 2_fps_sq;

    // frc::TrajectoryConfig config(Drivetrain::kMaxSpeedLinear, Drivetrain::kMaxAccelerationLinear);
    frc::TrajectoryConfig config(maxLinearVel, maxLinearAcc);
    config.AddConstraint(frc::CentripetalAccelerationConstraint{3_mps_sq});
    config.AddConstraint(frc::DifferentialDriveVoltageConstraint{IO.drivetrain.GetFeedForward(), IO.drivetrain.GetKinematics(), 4_V});
    config.AddConstraint(frc::DifferentialDriveKinematicsConstraint{IO.drivetrain.GetKinematics(), 2_fps});
    config.SetReversed(false);

    m_trajectory = rj::AutoHelper::LoadTrajectory("90 - 4ft Straight", &config); 

    m_autoTimer.Reset();
    m_autoTimer.Start();

    IO.drivetrain.ResetOdometry(m_trajectory.InitialPose());
}

// Execute the program
void Auto4ft::Run()
{
    switch (m_state)
    {
    case 0:
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
    default:
    {
        IO.drivetrain.Arcade(0.0, 0.0);
    }
    }

    UpdateSmartDash();
}

void Auto4ft::UpdateSmartDash()
{
    frc::SmartDashboard::PutNumber("Auto/State", m_state);
}