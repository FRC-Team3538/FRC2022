#include "auto/AutoTurn.hpp"
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
#include "units/acceleration.h"
#include "units/base.h"
#include "units/velocity.h"
#include "units/voltage.h"

// Name for Smart Dash Chooser
std::string AutoTurn::GetName()
{
    return "92 - Turn";
}

// Initialization
// Constructor requires a reference to the robot map
AutoTurn::AutoTurn(Robotmap &IO) : IO(IO)
{
    m_state = 0;
}

AutoTurn::~AutoTurn() {}

//State Machine
void AutoTurn::NextState()
{
    m_state++;
    m_autoTimer.Reset();
    m_autoTimer.Start();
}

void AutoTurn::Init()
{
    units::feet_per_second_t maxLinearVel = 8_fps;
    units::meters_per_second_squared_t maxCentripetalAcc = 1.7_mps_sq;
    units::meters_per_second_squared_t maxLinearAcc = 1.7_mps_sq;

    frc::TrajectoryConfig config(maxLinearVel, maxLinearAcc);
    config.AddConstraint(frc::CentripetalAccelerationConstraint{maxCentripetalAcc});
    config.AddConstraint(frc::DifferentialDriveVoltageConstraint{IO.drivetrain.GetFeedForward(), IO.drivetrain.GetKinematics(), 12_V});
    config.AddConstraint(frc::DifferentialDriveKinematicsConstraint{IO.drivetrain.GetKinematics(), maxLinearVel});
    config.SetReversed(false);

    auto trajectories = rj::AutoHelper::LoadTrajectorySplit("92 - Turning Left and Right", &config);
    m_trajectory = trajectories[0] + trajectories[1];

    m_autoTimer.Reset();
    m_autoTimer.Start();

    IO.drivetrain.ResetOdometry(m_trajectory.InitialPose());
}

// Execute the program
void AutoTurn::Run()
{
    switch (m_state)
    {
    case 0:
    {
        // std::cout << m_autoTimer.Get().value() << std::endl;
        auto reference = m_trajectory.Sample(m_autoTimer.Get());

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

void AutoTurn::UpdateSmartDash()
{
    frc::SmartDashboard::PutNumber("Auto/State", m_state);
}