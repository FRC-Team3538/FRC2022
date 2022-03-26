#include "auto/AutoFiveBallSneaky.hpp"
#include <frc/trajectory/constraint/CentripetalAccelerationConstraint.h>
#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <units/acceleration.h>
#include <iostream>
#include "Robotmap.hpp"
#include "frc/Timer.h"
#include "frc/trajectory/Trajectory.h"
#include "frc/trajectory/TrajectoryConfig.h"
#include "lib/AutoHelper.h"
#include "networktables/NetworkTableEntry.inc"
#include "subsystems/Drivetrain.hpp"
#include "subsystems/RJVisionPipeline.hpp"
#include "subsystems/Shooter.hpp"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/base.h"
#include "units/time.h"
#include "units/velocity.h"
#include "units/voltage.h"

// Name for Smart Dash Chooser
std::string AutoFiveBallSneaky::GetName()
{
    return "08 - Five Ball Sneaky";
}

// Initialization
// Constructor requires a reference to the robot map
AutoFiveBallSneaky::AutoFiveBallSneaky(Robotmap &IO) : IO(IO)
{
    m_driveState = 0;
    m_shooterState = 0;
    m_newDriveState = true;
    m_newShooterState = true;

    m_shotCount = 0;
}

AutoFiveBallSneaky::~AutoFiveBallSneaky() {}

// process
// 1 - grab second
// 2 - shoot both
// 3 - grab 3, 4
// 4 - shoot both
// 5 - grab 5
// 6 - shoot last

// State Machine
void AutoFiveBallSneaky::NextDriveState()
{
    m_driveState++;
    m_newDriveState = true;

    m_driveTimer.Reset();
    m_driveTimer.Start();
}

// State Machine
void AutoFiveBallSneaky::NextShooterState()
{
    m_shooterState++;
    m_newShooterState = true;

    m_shooterTimer.Reset();
    m_shooterTimer.Start();
}

void AutoFiveBallSneaky::Init()
{
    units::feet_per_second_t maxLinearVel = 8_fps;
    units::meters_per_second_squared_t maxCentripetalAcc = 1.7_mps_sq;

    units::meters_per_second_squared_t maxLinearAcc = 2_mps_sq;

    frc::TrajectoryConfig config(maxLinearVel, maxLinearAcc);
    config.AddConstraint(frc::CentripetalAccelerationConstraint{maxCentripetalAcc});
    config.AddConstraint(frc::DifferentialDriveVoltageConstraint{IO.drivetrain.GetFeedForward(), IO.drivetrain.GetKinematics(), 12_V});
    config.AddConstraint(frc::DifferentialDriveKinematicsConstraint{IO.drivetrain.GetKinematics(), maxLinearVel});
    config.SetReversed(false);

    m_trajectory_first = rj::AutoHelper::LoadTrajectory("07 - 5 Ball Sneaky 1", &config);
    m_trajectory_second = rj::AutoHelper::LoadTrajectory("07 - 5 Ball Sneaky 2", &config);

    std::cout << m_trajectory_first.TotalTime().value() + m_trajectory_second.TotalTime().value() << std::endl;

    IO.drivetrain.ResetOdometry(m_trajectory_first.InitialPose());

    m_driveTimer.Reset();
    m_driveTimer.Start();
    m_shooterTimer.Start();
    m_shooterTimer.Start();
    m_totalTimer.Reset();
    m_totalTimer.Start();
}

bool AutoFiveBallSneaky::FollowTrajectory(frc::Trajectory &trajectory)
{
    auto reference = trajectory.Sample(m_driveTimer.Get());

    IO.drivetrain.Drive(reference);

    return m_driveTimer.Get() > trajectory.TotalTime();
}

bool AutoFiveBallSneaky::FindVisionTarget()
{
    vision::RJVisionPipeline::visionData data = IO.rjVision.Run();
    return data.filled && IO.shooter.SetTurretAngle(data.turretAngle, 0.5_deg);
}

// Execute the program
void AutoFiveBallSneaky::Run()
{
    units::degree_t tol{ntVisionAngleTol.GetDouble(kVisionAngleTolDefault)};
    m_resetDriveState = m_newDriveState;
    m_resetShooterState = m_newShooterState;

    switch (m_driveState)
    {
    case 0:
    {
        // drive first path
        if (FollowTrajectory(m_trajectory_first)) {
            std::cout << "First path completed in  " << m_driveTimer.Get().value() << "s" << std::endl;
            NextDriveState();
            NextShooterState();
        }
        break;
    }
    case 1:
    {
        // wait until shooter is done
        if (m_newDriveState) {
            IO.drivetrain.Arcade(0.0, 0.0);
        }

        break;
    }
    case 2:
    {
        // run second path
        if (FollowTrajectory(m_trajectory_second)) {
            std::cout << "Second path completed in  " << m_driveTimer.Get().value() << "s" << std::endl;
            NextDriveState();
            NextShooterState();
        }
        break;
    }
    default:
    {
        // drivetrain is done
        if (m_newDriveState) {
            std::cout << "Paths completed in " << m_totalTimer.Get().value() << "s" << std::endl;
        }
        IO.drivetrain.Arcade(0.0, 0.0);
    }
    }

    switch (m_shooterState)
    {
    case 0:
    {
        // initialize everything at the beginning of auton
        // wait for first path
        if (m_newShooterState) {
            // state entry shit
            IO.shooter.SetIntakeState(Shooter::Position::Deployed);
            IO.shooter.SetIntake(8_V);
            IO.shooter.SetShooterRPM(3000_rpm);
            IO.shooter.SetIndexer(3_V);
            IO.shooter.SetFeeder(-2_V);

            IO.shooter.SetTurretAngle(20_deg, tol);

            IO.rjVision.SetLED(true);
        }

        break;
    }
    case 1:
    {
        // aim at target
        if (m_newShooterState) {
            IO.shooter.SetIndexer(0_V);
        }

        if (FindVisionTarget() || m_shooterTimer.Get() > 2_s)
        {
            std::cout << "Aim phase completed in  " << m_shooterTimer.Get().value() << "s" << std::endl;
            NextShooterState();
        }

        break;
    }
    case 2:
    {
        // shoot 2 balls
        if (m_newShooterState)
        {
            IO.shooter.ResetEdgeDetector();
            IO.shooter.SetFeeder(4_V);
            IO.shooter.SetIndexer(3_V);
        }

        FindVisionTarget();

        if (IO.shooter.Shoot_EdgeDetector())
        {
            m_shotCount += 1;
        }

        if (m_shotCount >= 2 || m_shooterTimer.Get() > 2_s) {
            std::cout << "Shot phase completed in  " << m_shooterTimer.Get().value() << "s" << std::endl;
            NextDriveState();
            NextShooterState();
        }

        break;
    }
    case 3:
    {
        // wait for second path
        if (m_newShooterState) {
            IO.shooter.SetFeeder(-2_V);

            IO.shooter.SetTurretAngle(-75_deg, tol);
        }

        break;
    }
    case 4:
    {
        // aim at target
        if (m_newShooterState) {
            IO.shooter.SetShooterRPM(3100_rpm);
            IO.shooter.SetIndexer(0_V);
        }

        if (FindVisionTarget() || m_shooterTimer.Get() > 2_s)
        {
            std::cout << "Aim phase completed in  " << m_shooterTimer.Get().value() << "s" << std::endl;
            NextShooterState();
        }

        break;
    }
    case 5:
    {
        // shoot 3 balls
        if (m_newShooterState)
        {
            IO.shooter.SetFeeder(4_V);
            IO.shooter.SetIndexer(3_V);
        }
    
        if (IO.shooter.Shoot_EdgeDetector())
        {
            m_shotCount += 1;
        }

        if (m_shotCount >= 5 || m_shooterTimer.Get() > 2_s) {
            std::cout << "Shot phase completed in  " << m_shooterTimer.Get().value() << "s" << std::endl;
            NextDriveState();
            NextShooterState();
        }

        break;
    }
    default:
    {
        // done - log it
        if (m_newShooterState) {
            IO.shooter.SetIntakeState(Shooter::Position::Stowed);
            IO.shooter.SetIntake(0_V);
            IO.shooter.SetShooterRPM(0_rpm);
            IO.shooter.SetIndexer(0_V);
            IO.shooter.SetFeeder(0_V);
            std::cout << "Auton completed in " << m_totalTimer.Get().value() << "s" << std::endl;
        }
    }
    }

    if (m_resetShooterState) {
        m_newShooterState = false;
    }

    if (m_resetDriveState) {
        m_resetDriveState = false;
    }
}

// Called Automagically by AutoPrograms (RobotPeriodic)
void AutoFiveBallSneaky::UpdateSmartDash()
{
    frc::SmartDashboard::PutNumber("Auto/DriveState", m_driveState);
    frc::SmartDashboard::PutNumber("Auto/ShooterState", m_shooterState);
}