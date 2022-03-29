#pragma once

#include <frc/Timer.h>                  // for Timer
#include <frc/trajectory/Trajectory.h>  // for Trajectory
#include <string>                       // for string
#include "AutoInterface.hpp"            // for AutoInterface
class Robotmap;

class Billiard : public AutoInterface
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

public:
    // Constructor requires a reference to the RobotMap
    Billiard() = delete;
    Billiard(Robotmap &);
    ~Billiard();

    // Auto Program Logic
    void Init();
    void Run();
    void UpdateSmartDash();
};