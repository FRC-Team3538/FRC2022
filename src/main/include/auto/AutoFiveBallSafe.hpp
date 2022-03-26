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

class AutoFiveBallSafe : public AutoInterface
{
public:
    // Name of this program, used by SmartDash
    static std::string GetName();

protected:
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

    units::revolutions_per_minute_t m_rpmShot1 = 3075_rpm;
    units::revolutions_per_minute_t m_rpmShot2 = 2825_rpm;
    units::revolutions_per_minute_t m_rpmShot3 = 2950_rpm;


public:
    // Constructor requires a reference to the RobotMap
    AutoFiveBallSafe() = delete;
    AutoFiveBallSafe(Robotmap &);
    ~AutoFiveBallSafe();

    // Auto Program Logic
    void Init();
    void Run();
    void UpdateSmartDash();
};