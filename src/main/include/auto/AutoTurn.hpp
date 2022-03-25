#pragma once

#include <frc/Timer.h>                  // for Timer
#include <frc/trajectory/Trajectory.h>  // for Trajectory
#include <string>                       // for string
#include "AutoInterface.hpp"            // for AutoInterface
class Robotmap;

class AutoTurn : public AutoInterface
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

    frc::Trajectory m_trajectory;

public:
    // Constructor requires a reference to the RobotMap
    AutoTurn() = delete;
    AutoTurn(Robotmap &);
    ~AutoTurn();

    // Auto Program Logic
    void Init();
    void Run();
    void UpdateSmartDash();
};