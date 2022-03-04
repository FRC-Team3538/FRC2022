#include "auto/NinetyDegreeAngle.hpp"

#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/trajectory/constraint/CentripetalAccelerationConstraint.h>

#include <lib/AutoHelper.h>

// Name for Smart Dash Chooser
std::string NinetyDegreeAngle::GetName()
{
    return "91 - NinetyDegAngle";
}

// Initialization
// Constructor requires a reference to the robot map
NinetyDegreeAngle::NinetyDegreeAngle(Robotmap &IO) : IO(IO)
{
    m_state = 0;
}

NinetyDegreeAngle::~NinetyDegreeAngle() {}

//State Machine
void NinetyDegreeAngle::NextState()
{
    m_state++;
    m_autoTimer.Reset();
    m_autoTimer.Start();
}

void NinetyDegreeAngle::Init()
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

    m_trajectory = rj::AutoHelper::LoadTrajectory("91 - 90 Degree Angle", &config); 

    m_autoTimer.Reset();
    m_autoTimer.Start();

    IO.drivetrain.ResetOdometry(m_trajectory.InitialPose());
}

// Execute the program
void NinetyDegreeAngle::Run()
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

void NinetyDegreeAngle::UpdateSmartDash()
{
    frc::SmartDashboard::PutNumber("Auto/State", m_state);
}
