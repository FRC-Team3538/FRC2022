package io.robojackets.auto.impl;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.y2023.math.geometry.Rotation2d;
import io.robojackets.auto.IAutoProgram;
import io.robojackets.subsystems.RobotMap;

public class DriveForwardAuto implements IAutoProgram {
  RobotMap IO;

  double speedMetersPerSecond;
  Rotation2d heading = new Rotation2d();
  double durationSeconds;

  Timer timer = new Timer();

  public DriveForwardAuto(RobotMap IO) {
    this.IO = IO;
  }

  public String GetName() {
    return "01 - Drive Forward";
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty(
        "auton/speed", () -> speedMetersPerSecond, value -> speedMetersPerSecond = value);
    builder.addDoubleProperty(
        "auton/heading",
        () -> heading.getDegrees(),
        value -> heading = Rotation2d.fromDegrees(value));
    builder.addDoubleProperty(
        "auton/duration", () -> durationSeconds, value -> durationSeconds = value);
  }

  @Override
  public void Init() {
    timer.reset();
    timer.start();

    IO.drivetrain.SetFieldCentric(false);
    IO.drivetrain.ResetYaw(heading);
  }

  @Override
  public void Run() {
    if (timer.get() < durationSeconds) {
      IO.drivetrain.Drive(speedMetersPerSecond, 0, 0, false);
    } else {
      IO.drivetrain.Stop();
    }
  }

  @Override
  public void End() {
    IO.drivetrain.Stop();
  }
}
