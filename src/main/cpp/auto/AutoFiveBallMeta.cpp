#include "auto/AutoFiveBallMeta.hpp"
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
#include <frc/DriverStation.h>

// Name for Smart Dash Chooser
std::string AutoFiveBallMeta::GetName()
{
    return "05 - Five Ball Meta";
}

// Initialization
// Constructor requires a reference to the robot map
AutoFiveBallMeta::AutoFiveBallMeta(Robotmap &IO) : IO(IO)
{
    m_driveState = 0;
    m_shooterState = 0;
    m_newDriveState = true;
    m_newShooterState = true;

    m_shotCount = 0;
}

AutoFiveBallMeta::~AutoFiveBallMeta() {}

// process
// 1 - grab second
// 2 - shoot both
// 3 - grab 3, 4
// 4 - shoot both
// 5 - grab 5
// 6 - shoot last

// State Machine
void AutoFiveBallMeta::NextDriveState()
{
    m_driveState++;
    m_newDriveState = true;

    m_driveTimer.Reset();
    m_driveTimer.Start();
}

// State Machine
void AutoFiveBallMeta::NextShooterState()
{
    m_shooterState++;
    m_newShooterState = true;

    m_shooterTimer.Reset();
    m_shooterTimer.Start();
}

void AutoFiveBallMeta::Init()
{
    units::feet_per_second_t maxLinearVel = 8_fps;
    units::meters_per_second_squared_t maxCentripetalAcc = 1.7_mps_sq;

    units::meters_per_second_squared_t maxLinearAcc = 2_mps_sq;

    frc::TrajectoryConfig config(maxLinearVel, maxLinearAcc);
    config.AddConstraint(frc::CentripetalAccelerationConstraint{maxCentripetalAcc});
    config.AddConstraint(frc::DifferentialDriveVoltageConstraint{IO.drivetrain.GetFeedForward(), IO.drivetrain.GetKinematics(), 12_V});
    config.AddConstraint(frc::DifferentialDriveKinematicsConstraint{IO.drivetrain.GetKinematics(), maxLinearVel});
    config.SetReversed(false);

    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue)
    {
        // TODO clone sneaky path
        auto trajectories = rj::AutoHelper::LoadTrajectorySplit("05 - 5 Ball Wall Ball Blue", &config);

        m_trajectory_first = trajectories[0];
        m_trajectory_second = trajectories[1];
        m_trajectory_third = trajectories[2];
    }
    else
    {
        auto trajectories = rj::AutoHelper::LoadTrajectorySplit("05 - 5 Ball Wall Ball Red", &config);

        m_trajectory_first = trajectories[0];
        m_trajectory_second = trajectories[1];
        m_trajectory_third = trajectories[2];
    }

    IO.drivetrain.ResetOdometry(m_trajectory_first.InitialPose());

    m_driveTimer.Reset();
    m_driveTimer.Start();
    m_shooterTimer.Start();
    m_shooterTimer.Start();
    m_totalTimer.Reset();
    m_totalTimer.Start();
}

bool AutoFiveBallMeta::FollowTrajectory(frc::Trajectory &trajectory)
{
    auto reference = trajectory.Sample(m_driveTimer.Get());

    IO.drivetrain.Drive(reference);

    return m_driveTimer.Get() > trajectory.TotalTime();
}

bool AutoFiveBallMeta::FindVisionTarget()
{
    vision::RJVisionPipeline::visionData data = IO.rjVision.Run();
    return data.filled && IO.shooter.SetTurretAngle(data.turretAngle, 0.5_deg);
}

void AutoFiveBallMeta::PassiveTurretAim()
{
    /* ---------- robot state data ------------ */
    auto turret_to_center_robot_distance = 2_in;
    frc::Pose2d center_hub = frc::Pose2d{8.2296_m, 4.1148_m, 0_deg};

    auto robot_heading = IO.drivetrain.GetPose().Rotation().Radians();

    // positive because we're facing forward on the robot
    auto turret_to_robot = frc::Transform2d{frc::Translation2d{turret_to_center_robot_distance, 0_in}, frc::Rotation2d{}};

    // now we set our turret angle to the angle needed to face the center of the hub
    auto turret_position = IO.drivetrain.GetPose() + turret_to_robot.Inverse() + frc::Transform2d{frc::Translation2d{}, frc::Rotation2d{180_deg}};
    auto global_angle_to_hub = units::math::atan2(center_hub.Y() - turret_position.Y(), center_hub.X() - turret_position.X());
    auto turret_angle_to_hub = global_angle_to_hub - turret_position.Rotation().Radians();
    IO.shooter.SetTurretAngleSmooth(turret_angle_to_hub, 0.75_deg);
}

// Execute the program
void AutoFiveBallMeta::Run()
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
        }
        break;
    }
    case 3:
    {
        // wait for a configurable period
        if (m_newDriveState) {
            IO.drivetrain.Arcade(0.0, 0.0);
        }

        if (m_driveTimer.Get() > m_hpWaitTime)
        {
            std::cout << "Wait @ HP completed in  " << m_driveTimer.Get().value() << "s" << std::endl;
            NextDriveState();
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
            IO.shooter.SetShooterRPM(0_rpm);
            IO.shooter.SetIndexer(5_V);
            IO.shooter.SetFeeder(-4_V);

            IO.rjVision.SetLED(true);
        }

        PassiveTurretAim();

        break;
    }
    case 1:
    {
        // aim at target
        if (m_newShooterState) {
            // IO.shooter.SetIntakeState(Shooter::Position::Stowed);
            IO.shooter.SetShooterRatio(-1.0);
            IO.shooter.SetShooterRPM(3450_rpm);
        }

        if (FindVisionTarget() || m_shooterTimer.Get() > m_shooterTimeout)
        {
            std::cout << "Aim phase completed in  " << m_shooterTimer.Get().value() << "s" << std::endl;
            NextShooterState();
        }

        break;
    }
    case 2:
    {
        // shoot 3 balls
        if (m_newShooterState)
        {
            IO.shooter.ResetEdgeDetector();
            IO.shooter.SetFeeder(8_V);
            IO.shooter.SetIndexer(8_V);
        }

        FindVisionTarget();

        if (IO.shooter.Shoot_EdgeDetector())
        {
            m_shotCount += 1;
        }

        if (m_shotCount >= 3 || m_shooterTimer.Get() > m_shooterTimeout) {
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
            IO.shooter.SetIntakeState(Shooter::Position::Deployed);
            IO.shooter.SetShooterRPM(3250_rpm);
            IO.shooter.SetFeeder(-4_V);
        }

        PassiveTurretAim();

        break;
    }
    case 4:
    {
        // aim at target
        if (m_newShooterState) {
            IO.shooter.SetIntakeState(Shooter::Position::Stowed);
            IO.shooter.SetShooterRatio(-1.0);
            IO.shooter.SetShooterRPM(3250_rpm);
        }

        if (FindVisionTarget() || m_shooterTimer.Get() > m_shooterTimeout)
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
            IO.shooter.SetFeeder(8_V);
            IO.shooter.SetIndexer(8_V);
        }
    
        if (IO.shooter.Shoot_EdgeDetector())
        {
            m_shotCount += 1;
        }

        if (m_shotCount >= 5 || m_shooterTimer.Get() > m_shooterTimeout) {
            std::cout << "Shot phase completed in  " << m_shooterTimer.Get().value() << "s" << std::endl;
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
        m_newDriveState = false;
    }
}

// Called Automagically by AutoPrograms (RobotPeriodic)
void AutoFiveBallMeta::UpdateSmartDash()
{
    frc::SmartDashboard::PutNumber("Auto/DriveState", m_driveState);
    frc::SmartDashboard::PutNumber("Auto/ShooterState", m_shooterState);
}