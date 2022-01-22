// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

void Robot::RobotInit()
{
  frc::SmartDashboard::PutNumber("Target Feeder", 0.0);
  frc::SmartDashboard::PutNumber("Target Shooter", 0.0);
  frc::SmartDashboard::PutNumber("Target Hood", 0.0);

  frc::SmartDashboard::PutNumber("Target Hood RPM", 0.0);
  frc::SmartDashboard::PutNumber("Target Shooter RPM", 0.0);

  IO.ConfigureMotors();

  dataLogUtils.EnableNTConnectionLogging();
  dataLogUtils.EnableNTEntryLogging();
}

void Robot::RobotPeriodic()
{
  IO.UpdateSmartDash();
  IO.drivetrain.Periodic();
  IO.rjVision.Periodic();
  autoprograms.SmartDash();
}

void Robot::AutonomousInit()
{
  autoprograms.Init();
}

void Robot::AutonomousPeriodic()
{
  autoprograms.Run();
}

void Robot::TeleopInit() {}

double deadband(double min, double max, double val)
{
  if (val < 0)
  {
    if (val > -min)
    {
      return 0;
    }
    return (val + min) / (max - min);
  }
  else
  {
    if (val < min)
    {
      return 0;
    }
    return (val - min) / (max - min);
  }
}

void Robot::TeleopPeriodic()
{
  if (IO.mainController.IsConnected())
  {
    double fwd = -deadband(0.05, 1, IO.mainController.GetLeftY());
    double rot = -deadband(0.05, 1, IO.mainController.GetRightX());

    IO.drivetrain.Arcade(fwd, rot);

    // double intake = deadband(0.05, 1, IO.mainController.GetR2Axis() / 2) - deadband(0.05, 1, IO.mainController.GetL2Axis() / 2);
    // IO.shooter.SetFeeder(units::volt_t(intake) * IO.pdp.GetVoltage());

    if (IO.mainController.GetL1Button())
    {
      IO.shooter.SetIntake(-8_V);
    }
    else if (IO.mainController.GetR1Button())
    {
      IO.shooter.SetIntake(8_V);
    }
    else
    {
      IO.shooter.SetIntake(0_V);
    }

    if (IO.mainController.GetPOV() == 90)
    {
      shooterPresetVal = 2500.0;
      hoodPresetVal = 2500.0;
      feederPresetVal = 8.0;
    }

    if (IO.mainController.GetCircleButtonPressed())
    {
      IO.shooter.SetHoodRPM(units::revolutions_per_minute_t{hoodPresetVal});
      IO.shooter.SetShooterRPM(units::revolutions_per_minute_t{shooterPresetVal});
    }

    if (IO.mainController.GetCrossButtonPressed())
    {
      IO.shooter.SetHoodRPM(units::revolutions_per_minute_t{0});
      IO.shooter.SetShooterRPM(units::revolutions_per_minute_t{0});
    }

    if (IO.mainController.GetTriangleButton())
    {
      IO.shooter.SetFeeder(units::volt_t{8.0});
    }
    else
    {
      IO.shooter.SetFeeder(units::volt_t{0});
    }
  }

  // {
  //   double shooterVoltage;

  //   if (lockShooterVoltage.GetBoolean(false))
  //   {
  //     shooterVoltage = frc::SmartDashboard::GetNumber("Target Shooter", 0.0); // targetShooterVoltage.GetDouble(0);
  //   }
  //   else
  //   {
  //     shooterVoltage = 0.0;
  //   }

  //   IO.shooter.SetShooter(units::volt_t(shooterVoltage));
  // }

  // {
  //   double hoodVoltage;

  //   if (lockHoodVoltage.GetBoolean(false))
  //   {
  //     // hoodVoltage = targetHoodVoltage.GetDouble(0);
  //     hoodVoltage = frc::SmartDashboard::GetNumber("Target Hood", 0.0); // targetShooterVoltage.GetDouble(0);
  //   }
  //   else
  //   {
  //     hoodVoltage = 0.0;
  //   }

  //   IO.shooter.SetHood(units::volt_t(hoodVoltage));
  // }

  // {
  //   double feederVoltage;

  //   if (lockFeederVoltage.GetBoolean(false))
  //   {
  //     feederVoltage = frc::SmartDashboard::GetNumber("Target Feeder", 0.0); // targetFeederVoltage.GetDouble(0);
  //   }
  //   else
  //   {
  //     feederVoltage = 0.0;
  //   }

  //   IO.shooter.SetFeeder(units::volt_t(feederVoltage));
  // }
}

void Robot::DisabledInit()
{
  lockShooterVoltage.SetBoolean(true);
  // targetShooterVoltage.SetDouble(0.0);

  lockFeederVoltage.SetBoolean(true);
  // targetFeederVoltage.SetDouble(0.0);

  lockHoodVoltage.SetBoolean(true);
  targetHoodVoltage.SetDouble(0.0);
  targetIntakePercent.SetDouble(0.6);
}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic()
{
  IO.shooter.SetShooterRPM(units::revolutions_per_minute_t{frc::SmartDashboard::GetNumber("Target Shooter RPM", 0.0)});
  IO.shooter.SetHoodRPM(units::revolutions_per_minute_t{frc::SmartDashboard::GetNumber("Target Hood RPM", 0.0)});
  IO.shooter.SetFeeder(units::volt_t{frc::SmartDashboard::GetNumber("Target Feeder", 0.0)});
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
