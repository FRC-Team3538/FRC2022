#include "auto/AutoBilliard.hpp"
#include <frc/trajectory/constraint/CentripetalAccelerationConstraint.h>
#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include "Robotmap.hpp"
#include "frc/Timer.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc/trajectory/Trajectory.h"
#include "frc/trajectory/TrajectoryConfig.h"
#include "lib/AutoHelper.h"
#include "subsystems/Drivetrain.hpp"
#include "subsystems/Shooter.hpp"
#include "units/acceleration.h"
#include "units/angular_velocity.h"
#include "units/base.h"
#include "units/time.h"
#include "units/velocity.h"
#include "units/voltage.h"

// Name for Smart Dash Chooser
std::string Billiard::GetName()
{
    return "88 - Billiard";
}

// Initialization
// Constructor requires a reference to the robot map
Billiard::Billiard(Robotmap &IO) : IO(IO)
{
    m_state = 0;
}

Billiard::~Billiard() {}

// State Machine
void Billiard::NextState()
{
    m_state++;
    IO.drivetrain.Arcade(0.0, 0.0);
    m_autoTimer.Reset();
    m_autoTimer.Start();
}

void Billiard::Init()
{
    m_autoTimer.Reset();
    m_autoTimer.Start();

    IO.drivetrain.ResetOdometry(frc::Pose2d());
}

// Execute the program
void Billiard::Run()
{
    switch (m_state)
    {
    case 0:
    {
        IO.drivetrain.Arcade(0.0, 0.0);
        IO.shooter.SetIntakeState(Shooter::Position::Deployed);

        if (m_autoTimer.Get() > 0.5_s)
        {
            NextState();
        }

        break;
    }
    case 1:
    {
        IO.drivetrain.Arcade(0.0, 0.0);

        IO.shooter.SetFeeder(-2_V);
        IO.shooter.SetIndexer(-3_V);
        IO.shooter.SetIntake(-6_V);

        if (m_autoTimer.Get() > 4.0_s)
        {
            NextState();
        }
        break;
    }
    case 2:
    {
        IO.drivetrain.Arcade(0.0, 0.0);

        IO.shooter.SetIntakeState(Shooter::Position::Stowed);
        IO.shooter.SetIntake(0_V);
        IO.shooter.SetShooterRPM(0_rpm);
        IO.shooter.SetIndexer(0_V);
        IO.shooter.SetFeeder(0_V);

        NextState();
        break;
    }
    default:
    {
        // std::cout << "Done!" << std::endl;
        IO.drivetrain.Arcade(0.0, 0.0);
    }
    }
}

// Called Automagically by AutoPrograms (RobotPeriodic)
void Billiard::UpdateSmartDash()
{
    frc::SmartDashboard::PutNumber("Auto/State", m_state);
}