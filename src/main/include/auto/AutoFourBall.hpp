#pragma once

#include <frc/Timer.h>                  // for Timer
#include <frc/trajectory/Trajectory.h>  // for Trajectory
#include <string>                       // for string
#include "AutoInterface.hpp"            // for AutoInterface
class Robotmap;  // lines 8-8

class AutoFourBall : public AutoInterface
{
public:
    // Name of this program, used by SmartDash
    static std::string GetName();

private:
    // Get a referance to the robotmap
    Robotmap &IO;

    // State Variables
    int m_state;
    frc::Timer m_autoTimer;

    void NextState();

    frc::Trajectory m_trajectory_first;
    frc::Trajectory m_trajectory_second;

public:
    // Constructor requires a reference to the RobotMap
    AutoFourBall() = delete;
    AutoFourBall(Robotmap &);
    ~AutoFourBall();

    // Auto Program Logic
    void Init();
    void Run();
    void UpdateSmartDash();
};