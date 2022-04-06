#pragma once

#include <frc/Timer.h>                          // for Timer
#include <frc/trajectory/Trajectory.h>          // for Trajectory
#include <units/angular_velocity.h>             // for operator""_rpm, revol...
#include <string>                               // for string
#include "AutoInterface.hpp"                    // for AutoInterface
#include "frc/smartdashboard/SmartDashboard.h"  // for SmartDashboard
#include "networktables/NetworkTableEntry.h"    // for NetworkTableEntry
class Robotmap;  // lines 11-11

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

    int m_shotCount;

    bool m_resetShooterState;
    bool m_resetDriveState;

    units::revolutions_per_minute_t m_rpmShot1 = 4550_rpm;//3075_rpm;
    units::revolutions_per_minute_t m_rpmShot2 = 3950_rpm;//2825_rpm;
    units::revolutions_per_minute_t m_rpmShot3 = 3950_rpm;//2950_rpm;


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