package io.robojackets.subsystems;

import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.Nat;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.y2023.math.Matrix;
import edu.wpi.first.y2023.math.controller.HolonomicDriveController;
import edu.wpi.first.y2023.math.controller.PIDController;
import edu.wpi.first.y2023.math.controller.ProfiledPIDController;
import edu.wpi.first.y2023.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.y2023.math.geometry.Pose2d;
import edu.wpi.first.y2023.math.geometry.Rotation2d;
import edu.wpi.first.y2023.math.geometry.Translation2d;
import edu.wpi.first.y2023.math.kinematics.ChassisSpeeds;
import edu.wpi.first.y2023.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.y2023.math.trajectory.TrapezoidProfile;
import edu.wpi.first.y2023.math.util.Units;
import io.robojackets.config.FeedForwardConfig;
import io.robojackets.config.PIDConfig;
import io.robojackets.config.SwerveModuleConfig;

public class Drivetrain extends Subsystem {
  public static final double kMaxSpeedLinearMetersPerSecond = Units.feetToMeters(16);
  public static final double kMaxSpeedAngularRadiansPerSecond = Units.degreesToRadians(360);
  public static final double kMaxAccelerationLinearMetersPerSecondPerSecond =
      Units.feetToMeters(20);
  public static final double kWheelDistance = Units.inchesToMeters(20.5);

  private boolean fieldRelative = true;
  Translation2d frontLeftLocation = new Translation2d(kWheelDistance / 2, kWheelDistance / 2);
  Translation2d frontRightLocation = new Translation2d(kWheelDistance / 2, -kWheelDistance / 2);
  Translation2d backLeftLocation = new Translation2d(-kWheelDistance / 2, kWheelDistance / 2);
  Translation2d backRightLocation = new Translation2d(-kWheelDistance / 2, -kWheelDistance / 2);

  WPI_Pigeon2 imu = new WPI_Pigeon2(30);
  BasePigeonSimCollection imuSim = imu.getSimCollection();

  Field2d fieldDisplay = new Field2d();
  FieldObject2d estimatedPose = fieldDisplay.getRobotObject();

  ChassisSpeeds command;
  ChassisSpeeds originalCommand;
  ChassisSpeeds measuredVelocity;

  boolean yawLockActive = true;
  PIDController yawLockPID;

  SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          GetYaw(),
          new Pose2d(),
          kinematics,
          Matrix.mat(Nat.N3(), Nat.N1()).fill(0.01, 0.01, 0.1),
          Matrix.mat(Nat.N1(), Nat.N1()).fill(0.01),
          Matrix.mat(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.2));

  SwerveModuleConfig frontLeftConfig =
      SwerveModuleConfig.builder()
          .angleOffsetRadians(Units.degreesToRadians(-128.496))
          .drivePIDConfig(PIDConfig.builder().kP(0.0092534).build())
          .turnPIDConfig(PIDConfig.builder().kP(0.098996).kD(0.00055669).build())
          .driveFeedForwardConfig(
              FeedForwardConfig.builder().kS(0.66323).kV(2.1798).kA(0.15467).build())
          .turnFeedForwardConfig(
              FeedForwardConfig.builder().kS(0.72584).kV(0.21377).kA(0.0027946).build())
          .build();

  SwerveModuleConfig frontRightConfig =
      SwerveModuleConfig.builder()
          .angleOffsetRadians(Units.degreesToRadians(-128.496))
          .drivePIDConfig(PIDConfig.builder().kP(0.0092534).build())
          .turnPIDConfig(PIDConfig.builder().kP(0.098996).kD(0.00055669).build())
          .driveFeedForwardConfig(
              FeedForwardConfig.builder().kS(0.66323).kV(2.1798).kA(0.15467).build())
          .turnFeedForwardConfig(
              FeedForwardConfig.builder().kS(0.72584).kV(0.21377).kA(0.0027946).build())
          .build();

  SwerveModuleConfig backLeftConfig =
      SwerveModuleConfig.builder()
          .angleOffsetRadians(Units.degreesToRadians(-128.496))
          .drivePIDConfig(PIDConfig.builder().kP(0.0092534).build())
          .turnPIDConfig(PIDConfig.builder().kP(0.098996).kD(0.00055669).build())
          .driveFeedForwardConfig(
              FeedForwardConfig.builder().kS(0.66323).kV(2.1798).kA(0.15467).build())
          .turnFeedForwardConfig(
              FeedForwardConfig.builder().kS(0.72584).kV(0.21377).kA(0.0027946).build())
          .build();

  SwerveModuleConfig backRightConfig =
      SwerveModuleConfig.builder()
          .angleOffsetRadians(Units.degreesToRadians(-128.496))
          .drivePIDConfig(PIDConfig.builder().kP(0.0092534).build())
          .turnPIDConfig(PIDConfig.builder().kP(0.098996).kD(0.00055669).build())
          .driveFeedForwardConfig(
              FeedForwardConfig.builder().kS(0.66323).kV(2.1798).kA(0.15467).build())
          .turnFeedForwardConfig(
              FeedForwardConfig.builder().kS(0.72584).kV(0.21377).kA(0.0027946).build())
          .build();

  SwerveModule frontLeft = new SwerveModule("FL", 0, 1, 20, frontLeftConfig);
  SwerveModule frontRight = new SwerveModule("FR", 2, 3, 20, frontRightConfig);
  SwerveModule backLeft = new SwerveModule("BL", 4, 5, 20, backLeftConfig);
  SwerveModule backRight = new SwerveModule("BR", 6, 7, 20, backRightConfig);

  HolonomicDriveController trajectoryController =
      new HolonomicDriveController(
          new PIDController(2.0, 0.0, 0.0),
          new PIDController(2.0, 0.0, 0.0),
          new ProfiledPIDController(
              2.0,
              0.0,
              0.0,
              new TrapezoidProfile.Constraints(
                  Units.degreesToRadians(360), Units.degreesToRadians(720))));

  public Rotation2d GetYaw() {
    return Rotation2d.fromDegrees(imu.getYaw());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub

  }

  @Override
  public void ConfigureSystem() {
    // TODO Auto-generated method stub

  }

  @Override
  public void RegisterDataEntries(DataLog log) {
    // TODO Auto-generated method stub

  }

  @Override
  public void LogDataEntries(DataLog log) {
    // TODO Auto-generated method stub

  }

  @Override
  public void SimInit() {
    // TODO Auto-generated method stub

  }

  @Override
  public void SimPeriodic(double battery, double[] currentDraw) {
    // TODO Auto-generated method stub
  }
}
