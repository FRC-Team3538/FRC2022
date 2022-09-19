package io.robojackets.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.Nat;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import edu.wpi.first.y2023.math.kinematics.SwerveModuleState;
import edu.wpi.first.y2023.math.trajectory.Trajectory.State;
import edu.wpi.first.y2023.math.trajectory.TrapezoidProfile;
import io.robojackets.config.FeedForwardConfig;
import io.robojackets.config.PIDConfig;
import io.robojackets.config.SwerveModuleConfig;
import io.robojackets.lib.UnitsConstants;

public class Drivetrain extends Subsystem {
  public static final double kMaxSpeedLinearMetersPerSecond = 16 * UnitsConstants.METERS_PER_FOOT;
  public static final double kMaxSpeedAngularRadiansPerSecond =
      360 / UnitsConstants.DEGREES_PER_RADIAN;
  public static final double kMaxAccelerationLinearMetersPerSecondPerSecond =
      20 * UnitsConstants.METERS_PER_FOOT;
  public static final double kWheelDistance = 20.5 * UnitsConstants.METERS_PER_INCH;

  private boolean fieldRelative = true;
  Translation2d frontLeftLocation = new Translation2d(kWheelDistance / 2, kWheelDistance / 2);
  Translation2d frontRightLocation = new Translation2d(kWheelDistance / 2, -kWheelDistance / 2);
  Translation2d backLeftLocation = new Translation2d(-kWheelDistance / 2, kWheelDistance / 2);
  Translation2d backRightLocation = new Translation2d(-kWheelDistance / 2, -kWheelDistance / 2);

  WPI_Pigeon2 imu = new WPI_Pigeon2(30);
  BasePigeonSimCollection imuSim = imu.getSimCollection();

  Field2d fieldDisplay = new Field2d();
  FieldObject2d estimatedPose = fieldDisplay.getRobotObject();

  ChassisSpeeds command = new ChassisSpeeds();
  ChassisSpeeds originalCommand = new ChassisSpeeds();
  ChassisSpeeds measuredVelocity = new ChassisSpeeds();

  boolean yawLockActive = true;
  PIDController yawLockPID = new PIDController(5, 0, 0);

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
          .angleOffsetRadians(-128.496 / UnitsConstants.DEGREES_PER_RADIAN)
          .drivePIDConfig(PIDConfig.builder().kP(0.0092534).build())
          .turnPIDConfig(PIDConfig.builder().kP(0.098996).kD(0.00055669).build())
          .driveFeedForwardConfig(
              FeedForwardConfig.builder().kS(0.66323).kV(2.1798).kA(0.15467).build())
          .turnFeedForwardConfig(
              FeedForwardConfig.builder().kS(0.72584).kV(0.21377).kA(0.0027946).build())
          .maxTurnVelocityRadiansPerSecond(36.0)
          .maxTurnAccelRadiansPerSecondPerSecond(1000.0)
          .build();

  SwerveModuleConfig frontRightConfig =
      SwerveModuleConfig.builder()
          .angleOffsetRadians(78.838 / UnitsConstants.DEGREES_PER_RADIAN)
          .drivePIDConfig(PIDConfig.builder().kP(0.0092534).build())
          .turnPIDConfig(PIDConfig.builder().kP(0.098996).kD(0.00055669).build())
          .driveFeedForwardConfig(
              FeedForwardConfig.builder().kS(0.66323).kV(2.1798).kA(0.15467).build())
          .turnFeedForwardConfig(
              FeedForwardConfig.builder().kS(0.72584).kV(0.21377).kA(0.0027946).build())
          .maxTurnVelocityRadiansPerSecond(36.0)
          .maxTurnAccelRadiansPerSecondPerSecond(1000.0)
          .build();

  SwerveModuleConfig backLeftConfig =
      SwerveModuleConfig.builder()
          .angleOffsetRadians(-4.219 / UnitsConstants.DEGREES_PER_RADIAN)
          .drivePIDConfig(PIDConfig.builder().kP(0.0092534).build())
          .turnPIDConfig(PIDConfig.builder().kP(0.098996).kD(0.00055669).build())
          .driveFeedForwardConfig(
              FeedForwardConfig.builder().kS(0.66323).kV(2.1798).kA(0.15467).build())
          .turnFeedForwardConfig(
              FeedForwardConfig.builder().kS(0.72584).kV(0.21377).kA(0.0027946).build())
          .maxTurnVelocityRadiansPerSecond(36.0)
          .maxTurnAccelRadiansPerSecondPerSecond(1000.0)
          .build();

  SwerveModuleConfig backRightConfig =
      SwerveModuleConfig.builder()
          .angleOffsetRadians(-60.645 / UnitsConstants.DEGREES_PER_RADIAN)
          .drivePIDConfig(PIDConfig.builder().kP(0.0092534).build())
          .turnPIDConfig(PIDConfig.builder().kP(0.098996).kD(0.00055669).build())
          .driveFeedForwardConfig(
              FeedForwardConfig.builder().kS(0.66323).kV(2.1798).kA(0.15467).build())
          .turnFeedForwardConfig(
              FeedForwardConfig.builder().kS(0.72584).kV(0.21377).kA(0.0027946).build())
          .maxTurnVelocityRadiansPerSecond(36.0)
          .maxTurnAccelRadiansPerSecondPerSecond(1000.0)
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
                  360 / UnitsConstants.DEGREES_PER_RADIAN,
                  720 / UnitsConstants.DEGREES_PER_RADIAN)));

  public Rotation2d GetYaw() {
    return Rotation2d.fromDegrees(imu.getYaw());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("DriveBase");
    builder.setActuator(true);

    // Modules
    frontLeft.initSendable(builder);
    frontRight.initSendable(builder);
    backLeft.initSendable(builder);
    backRight.initSendable(builder);

    builder.addDoubleProperty("gyro", () -> GetYaw().getRadians(), null);

    // Pose
    builder.addDoubleProperty(
        "poseEstimator/x", () -> poseEstimator.getEstimatedPosition().getX(), null);
    builder.addDoubleProperty(
        "poseEstimator/y", () -> poseEstimator.getEstimatedPosition().getY(), null);
    builder.addDoubleProperty(
        "poseEstimator/yaw",
        () -> poseEstimator.getEstimatedPosition().getRotation().getRadians(),
        null);

    // Command
    builder.addDoubleProperty("cmd/x", () -> command.vxMetersPerSecond, null);
    builder.addDoubleProperty("cmd/y", () -> command.vyMetersPerSecond, null);
    builder.addDoubleProperty("cmd/yaw", () -> command.omegaRadiansPerSecond, null);
    builder.addDoubleProperty("demand/x", () -> originalCommand.vxMetersPerSecond, null);
    builder.addDoubleProperty("demand/y", () -> originalCommand.vyMetersPerSecond, null);
    builder.addDoubleProperty("demand/yaw", () -> originalCommand.omegaRadiansPerSecond, null);

    // State
    builder.addDoubleProperty("state/x", () -> measuredVelocity.vxMetersPerSecond, null);
    builder.addDoubleProperty("state/y", () -> measuredVelocity.vyMetersPerSecond, null);
    builder.addDoubleProperty("state/yaw", () -> measuredVelocity.omegaRadiansPerSecond, null);

    // Operating Mode
    builder.addBooleanProperty("cmd/fieldRelative", () -> fieldRelative, null);
    builder.addDoubleProperty("yawpid/setpoint", () -> yawLockPID.getSetpoint(), null);
    builder.addDoubleProperty("yawpid/error", () -> yawLockPID.getPositionError(), null);
    builder.addBooleanProperty("yawpid/atSetpoint", () -> yawLockPID.atSetpoint(), null);
  }

  @Override
  public void ConfigureSystem() {
    imu.zeroGyroBiasNow(50);

    ResetYaw(new Rotation2d());

    yawLockPID.enableContinuousInput(-Math.PI, Math.PI);
    yawLockPID.setTolerance(3.0 / 180.0 * Math.PI);

    SmartDashboard.putData("Field", fieldDisplay);

    frontLeft.ConfigureSystem();
    frontRight.ConfigureSystem();
    backLeft.ConfigureSystem();
    backRight.ConfigureSystem();
  }

  public void Drive(State state, Rotation2d yaw) {
    ChassisSpeeds command =
        trajectoryController.calculate(poseEstimator.getEstimatedPosition(), state, yaw);

    Drive(
        command.vxMetersPerSecond, command.vyMetersPerSecond, command.omegaRadiansPerSecond, false);
  }

  public void Drive(
      double xSpeedMetersPerSecond,
      double ySpeedMetersPerSecond,
      double rotationRadiansPerSecond,
      boolean openLoop) {
    final double no_rotation_threshold_RadiansPerSecond = 0.1 / UnitsConstants.DEGREES_PER_RADIAN;
    yawLockActive = Math.abs(rotationRadiansPerSecond) < no_rotation_threshold_RadiansPerSecond;

    double translation_magnitude_MetersPerSecond =
        Math.hypot(xSpeedMetersPerSecond, ySpeedMetersPerSecond);

    if (yawLockActive && translation_magnitude_MetersPerSecond > 0.1) {
      double rot_demand = yawLockPID.calculate(GetYaw().getRadians());
      if (!yawLockPID.atSetpoint()) {
        rotationRadiansPerSecond = rot_demand;
      } else {
        yawLockPID.reset();
      }
    } else {
      yawLockPID.reset();
      yawLockPID.setSetpoint(GetYaw().getRadians());
    }

    if (fieldRelative) {
      command =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              xSpeedMetersPerSecond, ySpeedMetersPerSecond, rotationRadiansPerSecond, GetYaw());
    } else {
      command =
          new ChassisSpeeds(xSpeedMetersPerSecond, ySpeedMetersPerSecond, rotationRadiansPerSecond);
    }

    originalCommand = command;

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(command);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxSpeedLinearMetersPerSecond);

    command = kinematics.toChassisSpeeds(states);

    frontLeft.SetModule(states[0], openLoop);
    frontRight.SetModule(states[1], openLoop);
    backLeft.SetModule(states[2], openLoop);
    backRight.SetModule(states[3], openLoop);
  }

  public void Stop() {
    frontLeft.Stop();
    frontRight.Stop();
    backLeft.Stop();
    backRight.Stop();
  }

  public void UpdateOdometry() {
    Pose2d estPose =
        poseEstimator.update(
            GetYaw(),
            frontLeft.GetState(),
            frontRight.GetState(),
            backLeft.GetState(),
            backRight.GetState());

    measuredVelocity =
        kinematics.toChassisSpeeds(
            frontLeft.GetState(), frontRight.GetState(), backLeft.GetState(), backRight.GetState());

    edu.wpi.first.math.geometry.Pose2d actualPose =
        new edu.wpi.first.math.geometry.Pose2d(
            estPose.getX(),
            estPose.getY(),
            edu.wpi.first.math.geometry.Rotation2d.fromDegrees(estPose.getRotation().getDegrees()));

    estimatedPose.setPose(actualPose);
  }

  public void ResetYaw(Rotation2d heading) {
    imu.setYaw(heading.getDegrees(), 50);
    Pose2d pose = new Pose2d(GetPose().getTranslation(), heading);

    poseEstimator.resetPosition(pose, heading);

    yawLockPID.reset();
    yawLockPID.setSetpoint(heading.getRadians());
  }

  public void ResetOdometry(Pose2d pose) {
    poseEstimator.resetPosition(pose, GetYaw());
    yawLockPID.reset();
    yawLockPID.setSetpoint(GetYaw().getRadians());
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
    frontLeft.SimInit();
    frontRight.SimInit();
    backLeft.SimInit();
    backRight.SimInit();
  }

  @Override
  public void SimPeriodic(double battery, double[] currentDraw) {
    imuSim.addHeading(
        GetChassisSpeeds().omegaRadiansPerSecond
            * UnitsConstants.DEGREES_PER_RADIAN
            * 20
            * UnitsConstants.MILLISECONDS);

    frontLeft.SimPeriodic(battery, currentDraw);
    frontRight.SimPeriodic(battery, currentDraw);
    backLeft.SimPeriodic(battery, currentDraw);
    backRight.SimPeriodic(battery, currentDraw);
  }

  public ErrorCode SeedEncoders() {
    ErrorCode error = frontLeft.SeedTurnMotor();
    if (ErrorCode.OK != error) {
      return error;
    }

    error = frontRight.SeedTurnMotor();
    if (ErrorCode.OK != error) {
      return error;
    }

    error = backLeft.SeedTurnMotor();
    if (ErrorCode.OK != error) {
      return error;
    }

    return backRight.SeedTurnMotor();
  }

  public boolean Active() {
    return Math.abs(frontLeft.GetAngularVelocityRotationsPerSecond()) > 0.01
        || Math.abs(frontRight.GetAngularVelocityRotationsPerSecond()) > 0.01
        || Math.abs(backLeft.GetAngularVelocityRotationsPerSecond()) > 0.01
        || Math.abs(backRight.GetAngularVelocityRotationsPerSecond()) > 0.01;
  }

  public Pose2d GetPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public boolean GetFieldCentric() {
    return fieldRelative;
  }

  public void SetFieldCentric(boolean fieldCentric) {
    fieldRelative = fieldCentric;
  }

  public ChassisSpeeds GetChassisSpeeds() {
    return measuredVelocity;
  }
}
