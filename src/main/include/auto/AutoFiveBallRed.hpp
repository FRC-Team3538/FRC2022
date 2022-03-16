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
    int m_state;
    frc::Timer m_autoTimer;

    void NextState();

    frc::Trajectory m_trajectory_first;
    frc::Trajectory m_trajectory_second;

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