#include "auto/AutoFiveBallBlue.hpp"
#include <frc/trajectory/constraint/CentripetalAccelerationConstraint.h>
#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <units/acceleration.h>
#include <iostream>
#include "Robotmap.hpp"
#include "auto/AutoFiveBallSafe.hpp"
#include "frc/Timer.h"
#include "frc/trajectory/Trajectory.h"
#include "frc/trajectory/TrajectoryConfig.h"
#include "lib/AutoHelper.h"
#include "subsystems/Drivetrain.hpp"
#include "units/angular_velocity.h"
#include "units/time.h"
#include "units/velocity.h"
#include "units/voltage.h"

// Name for Smart Dash Chooser
std::string AutoFiveBallBlue::GetName()
{
    return "10 - Five Ball Safe Blue";
}

// Initialization
// Constructor requires a reference to the robot map
AutoFiveBallBlue::AutoFiveBallBlue(Robotmap &IO) : AutoFiveBallSafe(IO)
{
    
}

AutoFiveBallBlue::~AutoFiveBallBlue() {}


void AutoFiveBallBlue::Init()
{
    units::feet_per_second_t maxLinearVel = 8_fps;
    units::meters_per_second_squared_t maxCentripetalAcc = 1.7_mps_sq;

    units::meters_per_second_squared_t maxLinearAcc = 1.7_mps_sq;

    frc::TrajectoryConfig config(maxLinearVel, maxLinearAcc);
    config.AddConstraint(frc::CentripetalAccelerationConstraint{maxCentripetalAcc});
    config.AddConstraint(frc::DifferentialDriveVoltageConstraint{IO.drivetrain.GetFeedForward(), IO.drivetrain.GetKinematics(), 12_V});
    config.AddConstraint(frc::DifferentialDriveKinematicsConstraint{IO.drivetrain.GetKinematics(), maxLinearVel});
    config.SetReversed(false);

    m_trajectory_first = rj::AutoHelper::LoadTrajectory("09 - 5 Ball Safe Blue 1", &config);
    m_trajectory_second = rj::AutoHelper::LoadTrajectory("09 - 5 Ball Safe Blue 2", &config);
    m_trajectory_third = rj::AutoHelper::LoadTrajectory("09 - 5 Ball Safe Blue 3", &config);

    m_rpmShot1 = 3075_rpm;
    m_rpmShot2 = 2825_rpm;
    m_rpmShot3 = 2950_rpm;

    std::cout << m_trajectory_first.TotalTime().value() + m_trajectory_second.TotalTime().value() + m_trajectory_third.TotalTime().value() << std::endl;

    IO.drivetrain.ResetOdometry(m_trajectory_first.InitialPose());

    m_driveTimer.Reset();
    m_driveTimer.Start();
    m_shooterTimer.Reset();
    m_shooterTimer.Start();
    m_totalTimer.Reset();
    m_totalTimer.Start();
}
