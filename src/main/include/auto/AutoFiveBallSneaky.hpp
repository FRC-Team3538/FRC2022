#pragma once

#include <frc/Timer.h>                          // for Timer
#include <frc/trajectory/Trajectory.h>          // for Trajectory
#include <string>                               // for string
#include "AutoInterface.hpp"                    // for AutoInterface
#include "frc/smartdashboard/SmartDashboard.h"  // for SmartDashboard
#include "networktables/NetworkTableEntry.h"    // for NetworkTableEntry
class Robotmap;  // lines 10-10

class AutoFiveBallSneaky : public AutoInterface
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

    double kVisionAngleTolDefault = 0.5;
    nt::NetworkTableEntry ntVisionAngleTol = frc::SmartDashboard::GetEntry("robot/visionAngleTol");

    int m_shotCount;

    bool m_resetDriveState;
    bool m_resetShooterState;


public:
    // Constructor requires a reference to the RobotMap
    AutoFiveBallSneaky() = delete;
    AutoFiveBallSneaky(Robotmap &);
    ~AutoFiveBallSneaky();

    // Auto Program Logic
    void Init();
    void Run();
    void UpdateSmartDash();
};