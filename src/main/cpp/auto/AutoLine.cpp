#include "auto/AutoLine.hpp"

#include <frc/Filesystem.h>
#include <wpi/fs.h>
#include <wpi/SmallString.h>

#include <pathplanner/lib/PathPlanner.h>

#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrajectoryParameterizer.h>

#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/trajectory/constraint/CentripetalAccelerationConstraint.h>

// Name for Smart Dash Chooser
std::string AutoLine::GetName()
{
    return "1 - Line";
}

// Initialization
// Constructor requires a reference to the robot map
AutoLine::AutoLine(Robotmap &IO) : IO(IO)
{
    m_state = 0;
}

AutoLine::~AutoLine() {}

//State Machine
void AutoLine::NextState()
{
    m_state++;
    m_autoTimer.Reset();
    m_autoTimer.Start();
}

void AutoLine::Init()
{

    units::feet_per_second_t maxLinearVel = 4_fps;
    // units::standard_gravity_t maxCentripetalAcc = 0.5_SG;
    units::feet_per_second_squared_t maxLinearAcc = 4_fps_sq;

    // frc::TrajectoryConfig config(Drivetrain::kMaxSpeedLinear, Drivetrain::kMaxAccelerationLinear);
    frc::TrajectoryConfig config(maxLinearVel, maxLinearAcc);
    config.AddConstraint(frc::CentripetalAccelerationConstraint{5_mps_sq});
    config.AddConstraint(frc::DifferentialDriveVoltageConstraint{IO.drivetrain.GetFeedForward(), IO.drivetrain.GetKinematics(), 5_V});
    config.AddConstraint(frc::DifferentialDriveKinematicsConstraint{IO.drivetrain.GetKinematics(), 4_fps});
    config.SetReversed(false);

    // velocity, accel don't matter
    // but let's use the configured ones anyway
    pathplanner::PathPlannerTrajectory pp_traj = pathplanner::PathPlanner::loadPath("Straight Line path", config.MaxVelocity(), config.MaxAcceleration());

    std::vector<frc::TrajectoryGenerator::PoseWithCurvature> path;

    for (int ind = 0; ind < pp_traj.numStates(); ind++) {
        auto pp_state = pp_traj.getState(ind);


        frc::Rotation2d heading_diff;
        if (ind == 0) {
            // sample this and next heading
            heading_diff = pp_traj.getState(ind+1)->pose.Rotation() - pp_state->pose.Rotation();
        } else if (ind == pp_traj.numStates() - 1) {
            // sample this and last heading
            heading_diff = pp_traj.getState(ind+1)->pose.Rotation() - pp_traj.getState(ind-1)->pose.Rotation();
        } else {
            // sample last and next heading
            heading_diff = pp_traj.getState(ind+1)->pose.Rotation() - pp_state->pose.Rotation();
        }

        int curv_sign = 0;

        if (heading_diff.Radians() > 0_rad) {
            curv_sign = 1;
        } else if (heading_diff.Radians() < 0_rad) {
            curv_sign = -1;
        }

        path.push_back(frc::TrajectoryGenerator::PoseWithCurvature{pp_state->pose, pp_state->curvature * curv_sign});
    }

    m_trajectory = frc::TrajectoryParameterizer::TimeParameterizeTrajectory(path, config.Constraints(), config.StartVelocity(), config.EndVelocity(), config.MaxVelocity(), config.MaxAcceleration(), config.IsReversed());


    /*
    std::vector<frc::Spline<5>::ControlVector> p1;

    {
        std::string filePath = frc::filesystem::GetDeployDirectory();
        filePath = fs::path{filePath}.append("PathWeaver").append("Paths").append("Line.path").c_str();
        

        io::CSVReader<6> csv(filePath);
        csv.read_header(io::ignore_extra_column | io::ignore_missing_column, "X", "Y", "Tangent X", "Tangent Y", "ddx", "ddy");
        double x, y, dx, dy, ddx = 0, ddy = 0;
        while (csv.read_row(x, y, dx, dy, ddx, ddy))
        {
            std::cout << x << ", " << dx << ", " << ddx << ", " << y << ", " << dy << ", " << ddy << ", " << std::endl;
            p1.push_back({{x, dx, ddx}, {y, dy, ddy}});
        }
    }

    m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(p1, config);
    */

    m_autoTimer.Reset();
    m_autoTimer.Start();

    IO.drivetrain.ResetOdometry(m_trajectory.InitialPose());
}

// Execute the program
void AutoLine::Run()
{
    switch (m_state)
    {
    case 0:
    {
        auto reference = m_trajectory.Sample(m_autoTimer.Get());
        auto speeds = IO.m_ramsete.Calculate(IO.drivetrain.GetPose(), reference);

        IO.drivetrain.Drive(speeds.vx, speeds.omega);

        if ((m_autoTimer.Get() > m_trajectory.TotalTime()))
        {
            NextState();
        }
        break;
    }
    default:
    {
        IO.drivetrain.Arcade(0.0, 0.0);
    }
    }

    UpdateSmartDash();
}

void AutoLine::UpdateSmartDash()
{
    frc::SmartDashboard::PutNumber("Auto State", m_state);
}