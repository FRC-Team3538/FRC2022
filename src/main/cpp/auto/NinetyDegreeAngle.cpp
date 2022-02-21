#include "auto/NinetyDegreeAngle.hpp"

#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/trajectory/constraint/CentripetalAccelerationConstraint.h>

#include <lib/AutoHelper.h>

// Name for Smart Dash Chooser
std::string NinetyDegreeAngle::GetName()
{
    return "93 - NinetyDegAngle";
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

    units::feet_per_second_t maxLinearVel = 2_fps;
    // units::standard_gravity_t maxCentripetalAcc = 0.5_SG;
    units::feet_per_second_squared_t maxLinearAcc = 2_fps_sq;

    // frc::TrajectoryConfig config(Drivetrain::kMaxSpeedLinear, Drivetrain::kMaxAccelerationLinear);
    frc::TrajectoryConfig config(maxLinearVel, maxLinearAcc);
    config.AddConstraint(frc::CentripetalAccelerationConstraint{3_mps_sq});
    config.AddConstraint(frc::DifferentialDriveVoltageConstraint{IO.drivetrain.GetFeedForward(), IO.drivetrain.GetKinematics(), 4_V});
    config.AddConstraint(frc::DifferentialDriveKinematicsConstraint{IO.drivetrain.GetKinematics(), 2_fps});
    config.SetReversed(true);

    m_trajectory = rj::AutoHelper::LoadTrajectory("90 Degree Angle", &config); 

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

void NinetyDegreeAngle::UpdateSmartDash()
{
    frc::SmartDashboard::PutNumber("Auto/State", m_state);
}
