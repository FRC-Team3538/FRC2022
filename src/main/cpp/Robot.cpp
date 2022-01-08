// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/filter/SlewRateLimiter.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/controller/RamseteController.h>
#include <frc/Preferences.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Timer.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/fs.h>
#include <wpi/SmallString.h>
#include <wpi/json.h>
#include <frc/DigitalInput.h>
#include <frc/Servo.h>

#include "subsystems/Drivetrain.h"

#include <lib/DiffyDriveTrajectoryConstraint.hpp>
#include <DrivetrainModel.hpp>
#include <lib/EllipticAccelerationConstraint.h>
#include <frc/trajectory/constraint/DifferentialDriveKinematicsConstraint.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/trajectory/constraint/RectangularRegionConstraint.h>
#include <lib/csv.h>

#include <memory>
#include <thread>
#include <vector>

using namespace std;

frc::Trajectory LoadWaypointCSV(std::string path, frc::TrajectoryConfig &config)
{
  std::vector<frc::Spline<5>::ControlVector> points;

  io::CSVReader<6> csv(path);
  csv.read_header(io::ignore_extra_column | io::ignore_missing_column, "X", "Y", "Tangent X", "Tangent Y", "ddx", "ddy");
  double x, y, dx, dy, ddx = 0, ddy = 0;
  while (csv.read_row(x, y, dx, dy, ddx, ddy))
  {
    //std::cout << x << ", " << dx << ", " << ddx << ", " << y << ", " << dy << ", " << ddy << ", " << std::endl;
    points.push_back({{x, dx, ddx}, {y, dy, ddy}});
  }

  return frc::TrajectoryGenerator::GenerateTrajectory(
      points,
      config);
}

class Robot : public frc::TimedRobot
{
public:
  void RobotInit() override
  {
    // Flush NetworkTables every loop. This ensures that robot pose and other
    // values are sent during every iteration.
    //SetNetworkTablesFlushEnabled(true);

    // Auto Programs
    m_chooser.SetDefaultOption("None", "None");
    m_chooser.AddOption("TEST", "TEST");
    m_chooser.AddOption("Bounce", "Bounce");
    frc::SmartDashboard::PutData(&m_chooser);

    // Smart Dash
    frc::SmartDashboard::PutNumber("VoltageL", 0.0);
    frc::SmartDashboard::PutNumber("VoltageR", 0.0);
    frc::SmartDashboard::PutNumber("Left Rate", 0.0);
    frc::SmartDashboard::PutNumber("Right Rate", 0.0);
    frc::SmartDashboard::PutString("Reference", "");
    frc::SmartDashboard::PutString("Pose", "");

    // 
    SupplyCurrentLimitConfiguration config{true, 30.0, 40.0, 0.0};
    intake.ConfigSupplyCurrentLimit(config);

    auto maxLinearVel = 17.5_fps;
    auto maxCentripetalAcc = 1_SG;
    auto maxLinearAcc = 20_fps_sq;

    frc::EllipticAccelerationConstraint m_elliptic_constraint{maxCentripetalAcc, maxLinearAcc};

    frc::DifferentialDriveKinematicsConstraint m_kinematic_constraint{m_drive.GetKinematics(), maxLinearVel};

    frc::DifferentialDriveVoltageConstraint m_voltage_constraint{m_drive.GetFeedForward(), m_drive.GetKinematics(), 12.5_V};

    rj::DiffyDriveTrajectoryConstraint m_drivetrain_constraint{grasshopper, 0.065_psi}; //.125

    frc::TrajectoryConfig tjconfig(maxLinearVel, maxLinearAcc); //17.5 15
    tjconfig.AddConstraint(m_elliptic_constraint);
    tjconfig.AddConstraint(m_voltage_constraint);
    tjconfig.AddConstraint(m_kinematic_constraint);
    tjconfig.SetEndVelocity(maxLinearVel);

  }

  void RobotPeriodic() override
  {
    m_drive.Periodic();

    frc::SmartDashboard::PutNumber("m_autoState", m_autoState);
    frc::SmartDashboard::PutNumber("m_autoTimer", m_autoTimer.Get().value());

    frc::SmartDashboard::PutBoolean("SLOS", slewOS);

    wpi::json j;
    frc::to_json(j, m_drive.GetPose());
    pose_json = j.dump(0);

    frc::SmartDashboard::PutString("Pose", pose_json);

    double vel = m_drive.GetVel();
    double acc = (vel - prevVel) / 0.02;
    double smoothAcc = 0.0;

    vels.push_back(acc);
    if (vels.size() > 5)
      vels.erase(vels.begin());

    if (vels.size() == 5)
      smoothAcc = (vels[0] + vels[1] + vels[2] + vels[3] + vels[4]) / 5.0;

    prevVel = vel;
    topVel = (abs(vel) > abs(topVel)) ? vel : topVel;
    topAcc = (smoothAcc > topAcc) ? smoothAcc : topAcc;

    frc::SmartDashboard::PutNumber("topVel", topVel);
    frc::SmartDashboard::PutNumber("topAcc", topAcc);
  }

  void AutonomousInit() override
  {
    // Generate / load paths
    auto path = m_chooser.GetSelected();
    if (m_chooser.GetSelected() == "None")
      return;

    if (path == "Bounce")
    {
      // wpi::SmallString<64> deployDirectory;
      // frc::filesystem::GetDeployDirectory(deployDirectory);
      // wpi::sys::path::append(deployDirectory, "output");
      // wpi::sys::path::append(deployDirectory, "Bounce-#.wpilib.json");
      // auto i = deployDirectory.rfind('#');

      // // Load each path
      // deployDirectory[i] = '1';
      // auto t1 = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
      // auto s1 = t1.States();

      // deployDirectory[i] = '2';
      // auto t2 = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
      // auto s2 = t2.States();

      // deployDirectory[i] = '3';
      // auto t3 = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
      // auto s3 = t3.States();

      // deployDirectory[i] = '4';
      // auto t4 = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory);
      // auto s4 = t4.States();

      auto maxLinearVel = 17.5_fps;
      auto maxCentripetalAcc = 0.7_SG;
      auto maxLinearAcc = 13.5_fps_sq;

      frc::EllipticAccelerationConstraint m_elliptic_constraint{maxCentripetalAcc, maxLinearAcc};

      frc::DifferentialDriveKinematicsConstraint m_kinematic_constraint{m_drive.GetKinematics(), maxLinearVel};

      frc::DifferentialDriveVoltageConstraint m_voltage_constraint{m_drive.GetFeedForward(), m_drive.GetKinematics(), 12.5_V};

      rj::DiffyDriveTrajectoryConstraint m_drivetrain_constraint{grasshopper, 0.065_psi}; //.125

      frc::TrajectoryConfig config(maxLinearVel, maxLinearAcc); //17.5 15
      config.AddConstraint(m_elliptic_constraint);
      config.AddConstraint(m_voltage_constraint);
      config.AddConstraint(m_kinematic_constraint);
      config.SetEndVelocity(maxLinearVel);

      std::vector<frc::Spline<5>::ControlVector> p1;
      {
        io::CSVReader<6> csv(m_deploy_dir + "/PathWeaver/Paths/Bounce-1");
        csv.read_header(io::ignore_extra_column | io::ignore_missing_column, "X", "Y", "Tangent X", "Tangent Y", "ddx", "ddy");
        double x, y, dx, dy, ddx = 0, ddy = 0;
        while (csv.read_row(x, y, dx, dy, ddx, ddy))
        {
          //std::cout << x << ", " << dx << ", " << ddx << ", " << y << ", " << dy << ", " << ddy << ", " << std::endl;
          p1.push_back({{x, dx, ddx}, {y, dy, ddy}});
        }
      }

      std::vector<frc::Spline<5>::ControlVector> p2;
      {
        io::CSVReader<6> csv(m_deploy_dir + "/PathWeaver/Paths/Bounce-2");
        csv.read_header(io::ignore_extra_column | io::ignore_missing_column, "X", "Y", "Tangent X", "Tangent Y", "ddx", "ddy");
        double x, y, dx, dy, ddx = 0, ddy = 0;
        while (csv.read_row(x, y, dx, dy, ddx, ddy))
        {
          //std::cout << x << ", " << dx << ", " << ddx << ", " << y << ", " << dy << ", " << ddy << ", " << std::endl;
          p2.push_back({{x, dx, ddx}, {y, dy, ddy}});
        }
      }

      std::vector<frc::Spline<5>::ControlVector> p3;
      {
        io::CSVReader<6> csv(m_deploy_dir + "/PathWeaver/Paths/Bounce-3");
        csv.read_header(io::ignore_extra_column | io::ignore_missing_column, "X", "Y", "Tangent X", "Tangent Y", "ddx", "ddy");
        double x, y, dx, dy, ddx = 0, ddy = 0;
        while (csv.read_row(x, y, dx, dy, ddx, ddy))
        {
          //std::cout << x << ", " << dx << ", " << ddx << ", " << y << ", " << dy << ", " << ddy << ", " << std::endl;
          p3.push_back({{x, dx, ddx}, {y, dy, ddy}});
        }
      }

      std::vector<frc::Spline<5>::ControlVector> p4;
      {
        io::CSVReader<6> csv(m_deploy_dir + "/PathWeaver/Paths/Bounce-4");
        csv.read_header(io::ignore_extra_column | io::ignore_missing_column, "X", "Y", "Tangent X", "Tangent Y", "ddx", "ddy");
        double x, y, dx, dy, ddx = 0, ddy = 0;
        while (csv.read_row(x, y, dx, dy, ddx, ddy))
        {
          //std::cout << x << ", " << dx << ", " << ddx << ", " << y << ", " << dy << ", " << ddy << ", " << std::endl;
          p4.push_back({{x, dx, ddx}, {y, dy, ddy}});
        }
      }

      auto t1 = frc::TrajectoryGenerator::GenerateTrajectory(p1, config);
      auto s1 = t1.States();

      config.SetReversed(true);

      auto t2 = frc::TrajectoryGenerator::GenerateTrajectory(p2, config);
      auto s2 = t2.States();

      config.SetReversed(false);

      auto t3 = frc::TrajectoryGenerator::GenerateTrajectory(p3, config);
      auto s3 = t3.States();

      config.SetReversed(true);

      auto t4 = frc::TrajectoryGenerator::GenerateTrajectory(p4, config);
      auto s4 = t4.States();

      {
        auto start_time = s1.back().t;
        for (size_t i = 0; i < s2.size(); i++)
        {
          s2[i].t += start_time;
        }

        // Add the second path to the overall path
        s1.insert(s1.end(), s2.begin(), s2.end());
      }

      {
        auto start_time = s1.back().t;
        for (size_t i = 0; i < s3.size(); i++)
        {
          s3[i].t += start_time;
        }

        // Add the second path to the overall path
        s1.insert(s1.end(), s3.begin(), s3.end());
      }

      {
        auto start_time = s1.back().t;
        for (size_t i = 0; i < s4.size(); i++)
        {
          s4[i].t += start_time;
        }

        // Add the second path to the overall path
        s1.insert(s1.end(), s4.begin(), s4.end());
      }

      // Save this Trajectory
      m_trajectory = frc::Trajectory(s1);

      intakeRetention.SetAngle(100.0);
    }
    else if (path == "TEST")
    {
      // Get path from preferences for testing & tuning
      auto x0 = units::inch_t(frc::Preferences::GetDouble("x0", 0));
      auto y0 = units::inch_t(frc::Preferences::GetDouble("y0", 0));
      auto r0 = units::degree_t(frc::Preferences::GetDouble("r0", 0));

      auto x1 = units::inch_t(frc::Preferences::GetDouble("x1", 0));
      auto y1 = units::inch_t(frc::Preferences::GetDouble("y1", 0));

      auto x2 = units::inch_t(frc::Preferences::GetDouble("x2", 0));
      auto y2 = units::inch_t(frc::Preferences::GetDouble("y2", 0));

      auto x3 = units::inch_t(frc::Preferences::GetDouble("x3", 0));
      auto y3 = units::inch_t(frc::Preferences::GetDouble("y3", 0));
      auto r3 = units::degree_t(frc::Preferences::GetDouble("r3", 0));

      auto vel = units::meters_per_second_t(frc::Preferences::GetDouble("v", 2));
      auto accel = units::meters_per_second_squared_t(frc::Preferences::GetDouble("a", 2));

      m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          frc::Pose2d(x0, y0, r0), {frc::Translation2d(x1, y1), frc::Translation2d(x2, y2)},
          frc::Pose2d(x3, y3, r3),
          frc::TrajectoryConfig(vel, accel));
    }

    // Reset State Vars
    m_autoTimer.Reset();
    m_autoTimer.Start();
    m_drive.ResetOdometry(m_trajectory.InitialPose());
    m_autoState = 0;
  }

  void AutonomousPeriodic() override
  {
    auto path = m_chooser.GetSelected();
    if (m_chooser.GetSelected() == "None")
    {
      return;
    }
    else if (path == "PowerPort")
    {
      AutoPowerPort();
    }
    else if (path == "Accuracy")
    {
      AutoAccuracy();
    }
    else
    {
      //
      // Detect when we cross the finish line and stop
      //

      // Distance from center of robot to the outside perimeter of the bumper
      auto BumperDist = 14_in;

      if ((path == "A-Red" || path == "A-Blue" || path == "B-Red" || path == "A-Blue"))
      {
        intake.Set(-1.0);
        if (m_drive.GetPose().X() > (350.0_in - BumperDist))
        {
          m_drive.Drive(0_mps, 0_deg_per_s);
          m_autoTimer.Stop();
          return;
        }
      }
      else if (path == "Barrel" || path == "Slalom")
      {

        // if ((m_autoTimer.Get() > m_trajectory.TotalTime() / 2.0) && m_drive.GetPose().X() < (60.0_in + BumperDist))
        // {
        //   m_drive.Drive(0_mps, 0_deg_per_s);
        //   m_autoTimer.Stop();
        //   return;
        // }
        if (m_autoTimer.Get() > m_trajectory.TotalTime())
        {
          m_autoTimer.Reset();
          m_drive.Drive(0_mps, 0_deg_per_s);
          //m_autoTimer.Start();
        }
      }
      else if (path == "Bounce")
      {
        if (m_drive.GetPose().X() > (330.0_in - BumperDist))
        {
          m_drive.Drive(0_mps, 0_deg_per_s);
          m_autoTimer.Stop();
          return;
        }
      }

      //
      // Just Follow a Path...
      //
      if (m_autoTimer.Get().value() > 7.4 && path == "Barrel")
        m_drive.SetImpel(0.0);
      else
        m_drive.SetImpel(-1.0);

      auto reference = m_trajectory.Sample(m_autoTimer.Get());
      auto speeds = m_ramsete.Calculate(m_drive.GetPose(), reference);
      m_drive.Drive(speeds.vx, speeds.omega);

      wpi::json j;
      frc::to_json(j, reference);
      auto reference_json = j.dump(0);

      frc::SmartDashboard::PutString("Reference", reference_json);

      //std::cout << reference.pose.X().value() << "," << reference.pose.Y().value() << "," << reference.pose.Rotation().Degrees().value() /* << "," << m_drive.GetPose().X().value() << "," << m_drive.GetPose().Y().value() << "," << m_drive.GetPose().Rotation().Degrees().value()*/ << "," << speeds.vx.value() << "," << speeds.omega.value() << std::endl;
    }
  }

  void TeleopInit() override
  {
    SupplyCurrentLimitConfiguration config{true, 40.0, 50.0, 0.0};
    m_drive.impel.ConfigSupplyCurrentLimit(config);
    m_drive.impel2.ConfigSupplyCurrentLimit(config);
    intakeRetention.SetAngle(150.0);
  }

  void TeleopPeriodic() override
  {
    double forward = deadband(pow(m_controller.GetRawAxis(1), 1));

    m_drive.Arcade(forward, deadband(0.5 * pow(m_controller.GetRawAxis(2), 1)));
    double intakeSpd = ((m_controller.GetRawAxis(3) / 2.0) + 0.5) - ((m_controller.GetRawAxis(4) / 2.0) + 0.5);

    if (m_controller.GetRightBumper())
      m_drive.SetImpel(-1.0);
    else
      m_drive.SetImpel(0.0);

    if (m_controller.GetLeftBumper())
      intakeRetention.SetAngle(100.0);

    intake.Set(intakeSpd);
  }

  void DisabledInit() override
  {
    brakeTimer.Reset();
    brakeTimer.Start();
  }

  void DisabledPeriodic() override
  {
    m_drive.Drive(0_mps, 0_deg_per_s);
    slewOS = false;
    impelOS = false;
    m_speedLimiter.Reset(0.0);
    if (brakeTimer.Get() > 2.0_s)
    {
      // TODO: Set Coast Mode
    }
  }

  double deadband(double in)
  {
    if (fabs(in) < 0.05)
      return 0.0;

    return in;
  }

  void SimulationPeriodic() override { m_drive.SimulationPeriodic(); }

  void AutoPowerPort()
  {

    auto delay = units::second_t(frc::Preferences::GetDouble("Load_Delay", 3.0));

    if (m_autoState == 0)
    {
      // Go To reload
      auto elapsed = m_autoTimer.Get();
      auto reference = m_trajectory.Sample(elapsed);

      wpi::json j;
      frc::to_json(j, reference);
      auto reference_json = j.dump(0);

      frc::SmartDashboard::PutString("Reference", reference_json);

      auto speeds = m_ramsete.Calculate(m_drive.GetPose(), reference);
      m_drive.Drive(speeds.vx, speeds.omega);

      // Next State
      if (elapsed > (m_trajectory.TotalTime() + delay))
      {
        m_autoState = 1;
        m_autoTimer.Reset();
        m_autoTimer.Start();
      }
    }
    else if (m_autoState == 1)
    {
      // Go To Shoot
      auto elapsed = m_autoTimer.Get();
      auto reference = m_trajectory_PP.Sample(elapsed);
      auto speeds = m_ramsete.Calculate(m_drive.GetPose(), reference);
      m_drive.Drive(speeds.vx, speeds.omega);

      // Next State
      if (elapsed > m_trajectory_PP.TotalTime() + delay)
      {
        m_autoState = 0;
        m_autoTimer.Reset();
        m_autoTimer.Start();
      }
    }
    else
    {
      m_drive.Drive(0_mps, 0_deg_per_s);
    }
  }

  void AutoAccuracy()
  {
    // Load Delay
    auto delay = units::second_t(frc::Preferences::GetDouble("Load_Delay", 3.0));

    // Pathing
    auto elapsed = m_autoTimer.Get();
    auto reference = m_trajectory_IA[m_autoState].Sample(elapsed);
    auto speeds = m_ramsete.Calculate(m_drive.GetPose(), reference);
    m_drive.Drive(speeds.vx, speeds.omega);

    // Next State
    if (elapsed > (m_trajectory_IA[m_autoState].TotalTime() + delay))
    {
      m_autoState++;
      m_autoState %= 10;
      m_autoTimer.Reset();
      m_autoTimer.Start();
    }
  }

private:
  // TODO(Dereck): Change to PS4 controller
  frc::XboxController m_controller{0};

  frc::Timer Slew;
  bool slewOS = false;
  bool impelOS = false;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  frc::SlewRateLimiter<units::scalar> m_speedLimiter{2 / 1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{2 / 1_s};

  Drivetrain m_drive{IsSimulation()};

  frc::Trajectory m_trajectory;
  frc::Trajectory m_trajectory_PP;
  frc::Trajectory m_trajectory_IA[10];
  frc::RamseteController m_ramsete{units::unit_t<frc::RamseteController::b_unit>{2.0},
                                   units::unit_t<frc::RamseteController::zeta_unit>{0.7}};
  frc::Timer m_autoTimer;

  frc::SendableChooser<std::string> m_chooser;

  // Auto State
  uint m_autoState = 0;

  // Deploy Directory
  std::string m_deploy_dir = frc::filesystem::GetDeployDirectory();

  double prevVel = 0.0;
  double topVel = 0.0;
  double topAcc = 0.0;
  double prevCol = 0.0;
  int colCt = 0;

  vector<double> vels;

  string pose_json;

  WPI_TalonFX intake{8};

  frc::Servo intakeRetention{9};

  frc::Timer brakeTimer;
};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
