// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package io.robojackets;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.y2023.math.geometry.Rotation2d;
import io.robojackets.auto.AutoPrograms;
import io.robojackets.subsystems.Drivetrain;
import io.robojackets.subsystems.RobotMap;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  RobotMap IO = new RobotMap();
  AutoPrograms auton = new AutoPrograms(IO);

  Timer shotTimer = new Timer();
  Timer brakeTimer = new Timer();
  Timer intakeTimer = new Timer();

  Timer seedEncoderTimer = new Timer();

  NetworkTableEntry calibrationEntry = SmartDashboard.getEntry("Recalibrate");

  boolean manualJog;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    LiveWindow.disableAllTelemetry();
    LiveWindow.setEnabled(false);

    IO.ConfigureSystem();

    SmartDashboard.putData("Drivetrain", IO.drivetrain);

    calibrationEntry.setBoolean(false);
    calibrationEntry.addListener(
        notif -> {
          if (notif.value.getBoolean()) {
            IO.drivetrain.ConfigureSystem();
            calibrationEntry.setBoolean(false);
          }
        },
        0x10);

    seedEncoderTimer.start();

    IO.drivetrain.ResetYaw(new Rotation2d());
  }

  @Override
  public void robotPeriodic() {
    IO.drivetrain.UpdateOdometry();

    if (!IO.drivetrain.Active() && seedEncoderTimer.get() > 5 && RobotBase.isReal()) {
      System.out.println("Seeding Encoders");

      SmartDashboard.putNumber(
          "Drivetrain/SeedEncoderLastResult", IO.drivetrain.SeedEncoders().value);
      seedEncoderTimer.reset();
    }

    if (IO.mainController.getOptionsButtonPressed()) {
      IO.drivetrain.ResetYaw(new Rotation2d());
    }

    if (IO.mainController.getShareButtonPressed()) {
      IO.drivetrain.SetFieldCentric(!IO.drivetrain.GetFieldCentric());
    }
  }

  @Override
  public void autonomousInit() {
    auton.Init();
  }

  @Override
  public void autonomousPeriodic() {
    auton.Run();
  }

  @Override
  public void teleopInit() {
    IO.drivetrain.SetFieldCentric(true);
  }

  @Override
  public void teleopPeriodic() {
    double forward =
        deadband(IO.mainController.getLeftY(), 0.1, 1.0)
            * Drivetrain.kMaxSpeedLinearMetersPerSecond;
    double strafe =
        deadband(IO.mainController.getLeftX(), 0.1, 1.0)
            * Drivetrain.kMaxSpeedLinearMetersPerSecond;
    double rotate =
        deadband(IO.mainController.getRightX(), 0.1, 1.0)
            * Drivetrain.kMaxSpeedAngularRadiansPerSecond;

    IO.drivetrain.Drive(forward, strafe, rotate, false);
  }

  @Override
  public void disabledInit() {
    auton.End();
    IO.drivetrain.Stop();
    brakeTimer.reset();
    brakeTimer.start();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {
    IO.SimInit();
  }

  @Override
  public void simulationPeriodic() {
    IO.SimPeriodic(0, null);
  }

  private double deadband(double val, double min, double max) {
    if (val > max) {
      return max;
    } else if (val < -max) {
      return -max;
    } else if (Math.abs(val) < min) {
      return 0;
    } else {
      double sign = Math.signum(val);

      return sign * (Math.abs(val) - min) / (max - min) * max;
    }
  }
}
