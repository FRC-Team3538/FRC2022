#include "auto/AutoBackForward.hpp"

#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/trajectory/constraint/CentripetalAccelerationConstraint.h>

#include <lib/AutoHelper.h>

// Name for Smart Dash Chooser
std::string AutoBackForward::GetName()
{
    return "91 - BackForward";
}

// Initialization
// Constructor requires a reference to the robot map
AutoBackForward::AutoBackForward(Robotmap &IO) : IO(IO)
{
    m_state = 0;
}

AutoBackForward::~AutoBackForward() {}

//State Machine
void AutoBackForward::NextState()
{
    m_state++;
    m_autoTimer.Reset();
    m_autoTimer.Start();
}

void AutoBackForward::Init()
{

    units::feet_per_second_t maxLinearVel = 2_fps;
    // units::standard_gravity_t maxCentripetalAcc = 0.5_SG;
    units::feet_per_second_squared_t maxLinearAcc = 2_fps_sq;

    // frc::TrajectoryConfig config(Drivetrain::kMaxSpeedLinear, Drivetrain::kMaxAccelerationLinear);
    frc::TrajectoryConfig config(maxLinearVel, maxLinearAcc);
    config.AddConstraint(frc::CentripetalAccelerationConstraint{5_mps_sq});
    config.AddConstraint(frc::DifferentialDriveVoltageConstraint{IO.drivetrain.GetFeedForward(), IO.drivetrain.GetKinematics(), 5_V});
    config.AddConstraint(frc::DifferentialDriveKinematicsConstraint{IO.drivetrain.GetKinematics(), 4_fps});
    config.SetReversed(false);

    m_trajectory = rj::AutoHelper::LoadTrajectory("Testing Forward Back", &config);

    m_autoTimer.Reset();
    m_autoTimer.Start();

    IO.drivetrain.ResetOdometry(m_trajectory.InitialPose());
}

// Execute the program
void AutoBackForward::Run()
{
    switch (m_state)
    {
    case 0:
    {
        auto reference = m_trajectory.Sample(m_autoTimer.Get());
        frc::SmartDashboard::PutNumber("Drive State/X", reference.pose.Translation().X().value());
        frc::SmartDashboard::PutNumber("Drive State/Y", reference.pose.Translation().Y().value());
        frc::SmartDashboard::PutNumber("Drive State/Theta", reference.pose.Rotation().Radians().value());
        frc::SmartDashboard::PutNumber("Drive State/T", reference.t.value());
        frc::SmartDashboard::PutNumber("Drive State/V", reference.velocity.value());
        frc::SmartDashboard::PutNumber("Drive State/A", reference.acceleration.value());
        frc::SmartDashboard::PutNumber("Drive State/k", reference.curvature.value());

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

void AutoBackForward::UpdateSmartDash()
{
    frc::SmartDashboard::PutNumber("Auto/State", m_state);
}