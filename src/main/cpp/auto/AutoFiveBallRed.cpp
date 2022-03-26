#include "auto/AutoFiveBallRed.hpp"

#include "lib/AutoHelper.h"

#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/trajectory/constraint/CentripetalAccelerationConstraint.h>

#include <units/acceleration.h>

#include <iostream>

// Name for Smart Dash Chooser
std::string AutoFiveBallRed::GetName()
{
    return "10 - Five Ball Safe Red";
}

// Initialization
// Constructor requires a reference to the robot map
AutoFiveBallRed::AutoFiveBallRed(Robotmap &IO) : IO(IO)
{
    m_driveState = 0;
    m_shooterState = 0;
    m_newDriveState = true;
    m_newShooterState = true;

    m_shotCount = 0;
}

AutoFiveBallRed::~AutoFiveBallRed() {}

// process
// 1 - grab second
// 2 - shoot both
// 3 - grab 3, 4
// 4 - shoot both
// 5 - grab 5
// 6 - shoot last

// State Machine
void AutoFiveBallRed::NextDriveState()
{
    m_driveState++;
    m_newDriveState = true;

    m_driveTimer.Reset();
    m_driveTimer.Start();
}

// State Machine
void AutoFiveBallRed::NextShooterState()
{
    m_shooterState++;
    m_newShooterState = true;

    m_shooterTimer.Reset();
    m_shooterTimer.Start();
}

void AutoFiveBallRed::Init()
{
    units::feet_per_second_t maxLinearVel = 8_fps;
    units::meters_per_second_squared_t maxCentripetalAcc = 1.7_mps_sq;

    units::meters_per_second_squared_t maxLinearAcc = 1.7_mps_sq;

    frc::TrajectoryConfig config(maxLinearVel, maxLinearAcc);
    config.AddConstraint(frc::CentripetalAccelerationConstraint{maxCentripetalAcc});
    config.AddConstraint(frc::DifferentialDriveVoltageConstraint{IO.drivetrain.GetFeedForward(), IO.drivetrain.GetKinematics(), 12_V});
    config.AddConstraint(frc::DifferentialDriveKinematicsConstraint{IO.drivetrain.GetKinematics(), maxLinearVel});
    config.SetReversed(false);

    m_trajectory_first = rj::AutoHelper::LoadTrajectory("10 - 5 Ball Safe Red 1", &config);
    m_trajectory_second = rj::AutoHelper::LoadTrajectory("10 - 5 Ball Safe Red 2", &config);
    m_trajectory_third = rj::AutoHelper::LoadTrajectory("10 - 5 Ball Safe Red 3", &config);

    std::cout << m_trajectory_first.TotalTime().value() + m_trajectory_second.TotalTime().value() + m_trajectory_third.TotalTime().value() << std::endl;

    IO.drivetrain.ResetOdometry(m_trajectory_first.InitialPose());

    m_driveTimer.Reset();
    m_driveTimer.Start();
    m_shooterTimer.Reset();
    m_shooterTimer.Start();
    m_totalTimer.Reset();
    m_totalTimer.Start();
}

bool AutoFiveBallRed::FollowTrajectory(frc::Trajectory &trajectory)
{
    auto reference = trajectory.Sample(m_driveTimer.Get());

    IO.drivetrain.Drive(reference);

    return m_driveTimer.Get() > trajectory.TotalTime();
}

bool AutoFiveBallRed::FindVisionTarget()
{
    vision::RJVisionPipeline::visionData data = IO.rjVision.Run();
    return data.filled && IO.shooter.SetTurretAngle(data.turretAngle, 1_deg);
}

// Execute the program
void AutoFiveBallRed::Run()
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
    case 3:
    {
        // wait until shooter is done
        if (m_newDriveState) {
            IO.drivetrain.Arcade(0.0, 0.0);
        }
        
        break;
    }
    case 4:
    {
        // run third path
        if (FollowTrajectory(m_trajectory_third)) {
            std::cout << "Third path completed in  " << m_driveTimer.Get().value() << "s" << std::endl;
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
            IO.shooter.SetShooterRPM(3150_rpm);
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
            // IO.shooter.ResetEdgeDetector();
            IO.shooter.SetFeeder(4_V);
            IO.shooter.SetIndexer(3_V);
        }

        FindVisionTarget();

        // if (IO.shooter.Shoot_EdgeDetector())
        // {
        //     m_shotCount += 1;
        // }

        if (IO.shooter.Shoot() || m_shotCount >= 2 || m_shooterTimer.Get() > 2_s) {
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
            IO.shooter.SetShooterRPM(2825_rpm);
            IO.shooter.SetFeeder(-2_V);

            IO.shooter.SetTurretAngle(35_deg, tol);
        }

        break;
    }
    case 4:
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
    case 5:
    {
        // shoot 2 balls
        if (m_newShooterState)
        {
            IO.shooter.SetFeeder(4_V);
            IO.shooter.SetIndexer(3_V);
        }
    
        // if (IO.shooter.Shoot_EdgeDetector())
        // {
        //     m_shotCount += 1;
        // }

        if (IO.shooter.Shoot() || m_shotCount >= 4 || m_shooterTimer.Get() > 2_s) {
            std::cout << "Shot phase completed in  " << m_shooterTimer.Get().value() << "s" << std::endl;
            NextDriveState();
            NextShooterState();
        }

        break;
    }
    case 6:
    {
        // wait for third path
        if (m_newShooterState) {
            IO.shooter.SetShooterRPM(2950_rpm);
            IO.shooter.SetFeeder(-2_V);

            IO.shooter.SetTurretAngle(-35_deg, tol);
        }

        break;
    }
    case 7:
    {
        // aim at target
        if (m_newShooterState) {
            IO.shooter.SetShooterRPM(3000_rpm);

            IO.shooter.SetIndexer(0_V);
        }

        if (FindVisionTarget() || m_shooterTimer.Get() > 2_s)
        {
            std::cout << "Aim phase completed in  " << m_shooterTimer.Get().value() << "s" << std::endl;
            NextShooterState();
        }

        break;
    }
    case 8:
    {
        // shoot 1 ball
        if (m_newShooterState)
        {
            IO.shooter.SetFeeder(4_V);
            IO.shooter.SetIndexer(3_V);
        }

        // if (IO.shooter.Shoot_EdgeDetector())
        // {
        //     m_shotCount += 1;
        // }

        if (IO.shooter.Shoot() || m_shotCount >= 5 || m_shooterTimer.Get() > 2_s) {
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
void AutoFiveBallRed::UpdateSmartDash()
{
    frc::SmartDashboard::PutNumber("Auto/DriveState", m_driveState);
    frc::SmartDashboard::PutNumber("Auto/ShooterState", m_shooterState);
}