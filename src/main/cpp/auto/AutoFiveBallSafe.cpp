#include "auto/AutoFiveBallSafe.hpp"

#include "lib/AutoHelper.h"

#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/trajectory/constraint/CentripetalAccelerationConstraint.h>

#include <iostream>

// Name for Smart Dash Chooser
std::string AutoFiveBallSafe::GetName()
{
    return "06 - Five Ball Safe";
}

// Initialization
// Constructor requires a reference to the robot map
AutoFiveBallSafe::AutoFiveBallSafe(Robotmap &IO) : IO(IO)
{
    m_state = 0;
}

AutoFiveBallSafe::~AutoFiveBallSafe() {}

// process
// 1 - grab second
// 2 - shoot both
// 3 - grab 3, 4
// 4 - shoot both
// 5 - grab 5
// 6 - shoot last

// State Machine
void AutoFiveBallSafe::NextState()
{
    m_state++;

    switch(m_state)
    {
        case 0:
        {
            break;
        }
        case 1:
        {
            IO.shooter.SetIntakeState(Shooter::Position::Deployed);
            IO.shooter.SetIntake(8_V);
            IO.shooter.SetShooterRPM(2750_rpm);
            IO.shooter.SetIndexer(8_V);
            IO.shooter.SetFeeder(-2_V);

            break;
        }
        case 2:
        {
            IO.drivetrain.Arcade(0.0, 0.0);
            hasLimelightData = false;

            break;
        }
        case 3:
        {
            IO.shooter.SetFeeder(-2_V);

            break;
        }
        case 4:
        {
            IO.drivetrain.Arcade(0.0, 0.0);
            hasLimelightData = false;

            break;
        }
        case 5:
        {
            IO.shooter.SetFeeder(-2_V);

            break;
        }
        case 6:
        {
            IO.drivetrain.Arcade(0.0, 0.0);
            hasLimelightData = false;

            break;
        }
        default:
        {
            IO.shooter.SetIntakeState(Shooter::Position::Stowed);
            IO.shooter.SetIntake(0_V);
            IO.shooter.SetShooterRPM(0_rpm);
            IO.shooter.SetIndexer(0_V);
            IO.shooter.SetFeeder(0_V);
            std::cout << "Auton completed in " << m_totalTimer.Get().value() << "s" << std::endl;
            break;
        }
    }

    m_autoTimer.Reset();
    m_autoTimer.Start();
}

void AutoFiveBallSafe::Init()
{
    units::feet_per_second_t maxLinearVel = 8_fps;
    // units::standard_gravity_t maxCentripetalAcc = 0.5_SG;
    units::feet_per_second_squared_t maxLinearAcc = 1.5_mps_sq;

    // frc::TrajectoryConfig config(Drivetrain::kMaxSpeedLinear, Drivetrain::kMaxAccelerationLinear);
    frc::TrajectoryConfig config(maxLinearVel, maxLinearAcc);
    config.AddConstraint(frc::CentripetalAccelerationConstraint{maxLinearAcc});
    config.AddConstraint(frc::DifferentialDriveVoltageConstraint{IO.drivetrain.GetFeedForward(), IO.drivetrain.GetKinematics(), 12_V});
    config.AddConstraint(frc::DifferentialDriveKinematicsConstraint{IO.drivetrain.GetKinematics(), maxLinearVel});
    config.SetReversed(false);

    m_trajectory_first = rj::AutoHelper::LoadTrajectory ("06 - 5 Ball Safe 1", &config);
    m_trajectory_second = rj::AutoHelper::LoadTrajectory("06 - 5 Ball Safe 2 HOME", &config);
    m_trajectory_third = rj::AutoHelper::LoadTrajectory ("06 - 5 Ball Safe 3", &config);

    IO.drivetrain.ResetOdometry(m_trajectory_first.InitialPose());

    m_autoTimer.Reset();
    m_autoTimer.Start();
    m_totalTimer.Reset();
    m_totalTimer.Start();
}

// Execute the program
void AutoFiveBallSafe::Run()
{
    switch (m_state)
    {
        case 0:
        {        
            NextState();

            break;
        }
        case 1:
        {
            auto reference = m_trajectory_first.Sample(m_autoTimer.Get());

            IO.drivetrain.Drive(reference);

            if (m_autoTimer.Get() > m_trajectory_first.TotalTime())
            {
                NextState();
            }

            break;
        }
        case 2:
        {
            vision::RJVisionPipeline::visionData data = IO.rjVision.Run();
            if (data.filled && !hasLimelightData)
            {
                // Set Turret
                units::degree_t tol{ntVisionAngleTol.GetDouble(kVisionAngleTolDefault)};

                turretTarget = IO.shooter.GetTurretAngle() + data.angle;


                bool turretAtAngle = IO.shooter.SetTurretAngle(turretTarget, tol);

                Shooter::State shotStat = IO.shooter.CalculateShot(data.distance);

                IO.shooter.SetShooterRPM(3000_rpm);

                hasLimelightData = true;
            } else if (hasLimelightData) {
                // Set Turret
                units::degree_t tol{ntVisionAngleTol.GetDouble(kVisionAngleTolDefault)};

                bool turretAtAngle = IO.shooter.SetTurretAngle(turretTarget, tol);

                // Shoot Maybe
                if (turretAtAngle)
                {
                    IO.shooter.SetFeeder(8_V);
                    if (IO.shooter.Shoot()) {
                        NextState();
                    }
                }
            }

            break;
        }
        case 3:
        {
            auto reference = m_trajectory_second.Sample(m_autoTimer.Get());

            IO.drivetrain.Drive(reference);

            if (m_autoTimer.Get() > m_trajectory_second.TotalTime())
            {
                NextState();
            }

            break;
        }
        case 4:
        {
            vision::RJVisionPipeline::visionData data = IO.rjVision.Run();
            if (data.filled && !hasLimelightData)
            {
                // Set Turret
                units::degree_t tol{ntVisionAngleTol.GetDouble(kVisionAngleTolDefault)};

                turretTarget = IO.shooter.GetTurretAngle() + data.angle;


                bool turretAtAngle = IO.shooter.SetTurretAngle(turretTarget, tol);

                Shooter::State shotStat = IO.shooter.CalculateShot(data.distance);

                IO.shooter.SetShooterRPM(3000_rpm);

                hasLimelightData = true;
            } else if (hasLimelightData) {
                // Set Turret
                units::degree_t tol{ntVisionAngleTol.GetDouble(kVisionAngleTolDefault)};

                bool turretAtAngle = IO.shooter.SetTurretAngle(turretTarget, tol);

                // Shoot Maybe
                if (turretAtAngle)
                {
                    IO.shooter.SetFeeder(8_V);
                    if (IO.shooter.Shoot()) {
                        NextState();
                    }
                }
            }

            break;
        }
        case 5:
        {
            auto reference = m_trajectory_third.Sample(m_autoTimer.Get());

            IO.drivetrain.Drive(reference);

            if (m_autoTimer.Get() > m_trajectory_third.TotalTime())
            {
                NextState();
            }

            break;
        }
        case 6:
        {
            vision::RJVisionPipeline::visionData data = IO.rjVision.Run();
            if (data.filled && !hasLimelightData)
            {
                // Set Turret
                units::degree_t tol{ntVisionAngleTol.GetDouble(kVisionAngleTolDefault)};

                turretTarget = IO.shooter.GetTurretAngle() + data.angle;


                bool turretAtAngle = IO.shooter.SetTurretAngle(turretTarget, tol);

                Shooter::State shotStat = IO.shooter.CalculateShot(data.distance);

                IO.shooter.SetShooterRPM(3000_rpm);

                hasLimelightData = true;
            } else if (hasLimelightData) {
                // Set Turret
                units::degree_t tol{ntVisionAngleTol.GetDouble(kVisionAngleTolDefault)};

                bool turretAtAngle = IO.shooter.SetTurretAngle(turretTarget, tol);

                // Shoot Maybe
                if (turretAtAngle)
                {
                    IO.shooter.SetFeeder(8_V);
                    if (IO.shooter.Shoot()) {
                        NextState();
                    }
                }
            }

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
void AutoFiveBallSafe::UpdateSmartDash()
{
    frc::SmartDashboard::PutNumber("Auto/State", m_state);
}