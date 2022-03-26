#pragma once

#include <string>

#include <frc/Timer.h>

#include <units/velocity.h>

#include "AutoInterface.hpp"
#include "Robotmap.hpp"
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/constraint/CentripetalAccelerationConstraint.h>
#include <wpi/json.h>
#include <memory>
#include "lib/csv.h"

class AutoFiveBallRed : public AutoInterface
{
public:
    // Name of this program, used by SmartDash
    static std::string GetName();

private:
    // Get a referance to the robotmap
    Robotmap &IO;

    // State Variables
    int m_driveState;
    bool m_newDriveState;
    int m_shooterState;
    bool m_newShooterState;
    frc::Timer m_driveTimer;
    frc::Timer m_shooterTimer;
    frc::Timer m_totalTimer;

    void NextDriveState();
    void NextShooterState();

    bool FollowTrajectory(frc::Trajectory &trajectory);
    bool FindVisionTarget();

    frc::Trajectory m_trajectory_first;
    frc::Trajectory m_trajectory_second;
    frc::Trajectory m_trajectory_third;

    double kVisionAngleTolDefault = 0.5;
    nt::NetworkTableEntry ntVisionAngleTol = frc::SmartDashboard::GetEntry("robot/visionAngleTol");

    units::radian_t turretTarget;
    int m_shotCount;

    bool m_resetShooterState;
    bool m_resetDriveState;


public:
    // Constructor requires a reference to the RobotMap
    AutoFiveBallRed() = delete;
    AutoFiveBallRed(Robotmap &);
    ~AutoFiveBallRed();

    // Auto Program Logic
    void Init();
    void Run();
    void UpdateSmartDash();
};