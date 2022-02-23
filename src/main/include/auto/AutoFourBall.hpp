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

    units::time::second_t startPathTime;

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