package io.robojackets.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderSimCollection;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.y2023.math.Matrix;
import edu.wpi.first.y2023.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.y2023.math.geometry.Rotation2d;
import edu.wpi.first.y2023.math.kinematics.SwerveModuleState;
import edu.wpi.first.y2023.math.system.LinearSystem;
import edu.wpi.first.y2023.math.system.plant.DCMotor;
import edu.wpi.first.y2023.math.system.plant.LinearSystemId;
import io.robojackets.config.SwerveModuleConfig;
import io.robojackets.lib.UnitConversion;
import io.robojackets.sim.SwerveWheelSim;
import io.robojackets.sim.TurretSim;

public class SwerveModule extends Subsystem {
  private static final double WHEEL_RADIUS_METERS = 2 * UnitConversion.METERS_PER_INCH;
  private static final double DRIVE_GEARBOX_RATIO = 6.75;
  private static final double TURN_GEARBOX_RATIO = 12.8;
  private static final double DRIVE_SCALE_FACTOR = WHEEL_RADIUS_METERS / DRIVE_GEARBOX_RATIO;

  private static final double DRIVE_CURRENT_LIMIT_AMPS = 40;
  private static final double TURN_CURRENT_LIMIT_AMPS = 30;

  private final SimpleMotorFeedforward driveFeedForward;
  private final SimpleMotorFeedforward turnFeedForward;

  private final double angleOffset;

  private SwerveModuleState currentState = new SwerveModuleState();
  private SwerveModuleState targetState = new SwerveModuleState();
  private SwerveModuleState originalTargetState = new SwerveModuleState();

  private final String moduleID;

  private final SwerveModuleConfig config;

  private final WPI_TalonFX driveMotor;
  private final WPI_TalonFX turnMotor;
  private final CANCoder turnEncoder;

  private double driveVolts;
  private double turnVolts;

  private final TalonFXSimCollection driveMotorSim;
  private final TalonFXSimCollection turnMotorSim;
  private final CANCoderSimCollection turnEncoderSim;

  private final LinearSystem<N1, N1, N1> drivePlant;

  private final SwerveWheelSim driveSim;

  private final LinearSystem<N2, N1, N1> turnPlant;

  private final TurretSim turnSim;

  public SwerveModule(
      String moduleID, int driveID, int turnID, int encID, SwerveModuleConfig config) {
    this.moduleID = moduleID;
    driveMotor = new WPI_TalonFX(driveID);
    turnMotor = new WPI_TalonFX(turnID);
    turnEncoder = new CANCoder(encID);

    this.config = config;

    angleOffset = config.getAngleOffsetRadians();

    driveFeedForward =
        new SimpleMotorFeedforward(
            RobotBase.isSimulation() ? 0 : config.getDriveFeedForwardConfig().getKS(),
            config.getDriveFeedForwardConfig().getKV(),
            config.getDriveFeedForwardConfig().getKA());
    turnFeedForward =
        new SimpleMotorFeedforward(
            RobotBase.isSimulation() ? 0 : config.getTurnFeedForwardConfig().getKS(),
            config.getTurnFeedForwardConfig().getKV(),
            config.getTurnFeedForwardConfig().getKA());

    driveMotorSim = driveMotor.getSimCollection();
    turnMotorSim = turnMotor.getSimCollection();
    turnEncoderSim = turnEncoder.getSimCollection();

    drivePlant =
        LinearSystemId.identifyVelocitySystem(
            config.getDriveFeedForwardConfig().getKV() * WHEEL_RADIUS_METERS,
            config.getDriveFeedForwardConfig().getKA() * WHEEL_RADIUS_METERS);
    driveSim =
        new SwerveWheelSim(
            drivePlant,
            DCMotor.getFalcon500(1),
            DRIVE_GEARBOX_RATIO,
            WHEEL_RADIUS_METERS,
            Matrix.mat(Nat.N1(), Nat.N1()).fill(0.0));

    turnPlant =
        LinearSystemId.identifyPositionSystem(
            config.getTurnFeedForwardConfig().getKV(), config.getTurnFeedForwardConfig().getKA());

    turnSim =
        new TurretSim(
            turnPlant,
            DCMotor.getFalcon500(1),
            TURN_GEARBOX_RATIO,
            Matrix.mat(Nat.N1(), Nat.N1()).fill(0.0));
  }

  public SwerveModuleState GetState() {
    return new SwerveModuleState(GetVelocity(), GetAngle());
  }

  public double GetVelocity() {
    return driveMotor.getSelectedSensorVelocity()
        / 2048
        * DRIVE_SCALE_FACTOR
        / (100 * UnitConversion.MILLISECONDS)
        * UnitConversion.RADIANS_PER_ROTATION;
  }

  public Rotation2d GetAngle() {
    return Rotation2d.fromDegrees(turnEncoder.getAbsolutePosition());
  }

  public Rotation2d GetMotorAngle() {
    return Rotation2d.fromRotations(
        turnMotor.getSelectedSensorPosition() / 2048 / TURN_GEARBOX_RATIO);
  }

  void Drive(double targetMetersPerSecond, boolean openLoop) {
    driveVolts = driveFeedForward.calculate(targetMetersPerSecond);

    double target_in_native_units =
        targetMetersPerSecond / DRIVE_SCALE_FACTOR * 100 * UnitConversion.MILLISECONDS * 2048;

    if (openLoop) {
      driveMotor.setVoltage(driveVolts);
    } else {
      driveMotor.set(
          TalonFXControlMode.Velocity,
          target_in_native_units,
          DemandType.ArbitraryFeedForward,
          driveVolts / 12);
    }
  }

  void Turn(Rotation2d target) {
    Rotation2d normalized_target_angle = new Rotation2d(target.getCos(), target.getSin());
    Rotation2d normalized_current_angle = GetAngle();
    Rotation2d actual_current_angle = GetMotorAngle();
    Rotation2d normalized_actual_current_angle =
        new Rotation2d(actual_current_angle.getCos(), actual_current_angle.getSin());

    Rotation2d target_diff = normalized_target_angle.minus(normalized_actual_current_angle);
    SmartDashboard.putNumber(moduleID + "wtf/diff", target_diff.getRadians());
    SmartDashboard.putNumber(moduleID + "wtf/target", normalized_target_angle.getRadians());
    SmartDashboard.putNumber(moduleID + "wtf/current", normalized_current_angle.getRadians());
    SmartDashboard.putNumber(moduleID + "wtf/actual", normalized_actual_current_angle.getRadians());
    SmartDashboard.putNumber(
        moduleID + "wtf/encoder_ticks", normalized_current_angle.getRotations() * 4096);

    double actual_target_angle = actual_current_angle.getRadians() + target_diff.getRadians();

    double target_in_native_units =
        actual_target_angle / UnitConversion.RADIANS_PER_ROTATION * TURN_GEARBOX_RATIO * 2048;

    turnMotor.set(TalonFXControlMode.MotionMagic, target_in_native_units);
  }

  void SetModule(final SwerveModuleState state, boolean openLoop) {
    originalTargetState = state;
    targetState = SwerveModuleState.optimize(state, GetMotorAngle());

    Drive(targetState.speedMetersPerSecond, openLoop);
    Turn(targetState.angle);
  }

  void Stop() {
    driveMotor.stopMotor();
    turnMotor.stopMotor();

    driveSim.setInputVoltage(0);
    turnSim.setInputVoltage(0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {

    builder.setSmartDashboardType("SwerveModule");
    builder.setActuator(true);

    // Turning Encoders
    builder.addDoubleProperty(moduleID + "/abs_encoder", () -> GetAngle().getRadians(), null);
    builder.addDoubleProperty(
        moduleID + "/turn_motor_encoder", () -> GetMotorAngle().getRadians(), null);

    builder.addDoubleProperty(moduleID + "/raw_drive_volts", () -> driveVolts, null);
    builder.addDoubleProperty(
        moduleID + "/sim_drive_volts", () -> driveMotorSim.getMotorOutputLeadVoltage(), null);
    builder.addDoubleProperty(moduleID + "/raw_turn_volts", () -> turnVolts, null);
    builder.addDoubleProperty(
        moduleID + "/sim_turn_volts", () -> turnMotorSim.getMotorOutputLeadVoltage(), null);

    // Thermal
    builder.addDoubleProperty(
        moduleID + "/Drive Temp [C]", () -> driveMotor.getTemperature(), null);
    builder.addDoubleProperty(moduleID + "/Angle Temp [C]", () -> turnMotor.getTemperature(), null);

    builder.addDoubleProperty(
        moduleID + "/state/speed", () -> GetState().speedMetersPerSecond, null);
    builder.addDoubleProperty(moduleID + "/state/angle", () -> GetState().angle.getRadians(), null);

    builder.addDoubleProperty(
        moduleID + "/goal/speed", () -> targetState.speedMetersPerSecond, null);
    builder.addDoubleProperty(moduleID + "/goal/angle", () -> targetState.angle.getRadians(), null);
    builder.addDoubleProperty(
        moduleID + "/og_goal/speed", () -> originalTargetState.speedMetersPerSecond, null);
    builder.addDoubleProperty(
        moduleID + "/og_goal/angle", () -> originalTargetState.angle.getRadians(), null);

    builder.addDoubleProperty(moduleID + "/sim/speed", () -> driveSim.GetLinearVelocity(), null);
    builder.addDoubleProperty(
        moduleID + "/sim/turret_speed", () -> turnSim.getAngularVelocityRadiansPerSecond(), null);
    builder.addDoubleProperty(moduleID + "/sim/angle", () -> turnSim.getAngleRadians(), null);

    builder.addDoubleProperty(
        moduleID + "/turn/SensorVel", () -> turnMotor.getSelectedSensorVelocity(0), null);
    builder.addDoubleProperty(
        moduleID + "/turn/SensorPos", () -> turnMotor.getSelectedSensorPosition(0), null);
    builder.addDoubleProperty(
        moduleID + "/turn/MotorOutputPercent", () -> turnMotor.getMotorOutputPercent(), null);
    builder.addDoubleProperty(
        moduleID + "/turn/MotorOutputSimVoltage",
        () -> turnMotorSim.getMotorOutputLeadVoltage(),
        null);
    builder.addDoubleProperty(
        moduleID + "/turn/ClosedLoopError", () -> turnMotor.getClosedLoopError(0), null);
    builder.addDoubleProperty(
        moduleID + "/turn/ClosedLoopTarget", () -> turnMotor.getClosedLoopTarget(0), null);

    builder.addDoubleProperty(
        moduleID + "/drive/SensorVel", () -> driveMotor.getSelectedSensorVelocity(0), null);
    builder.addDoubleProperty(
        moduleID + "/drive/SensorPos", () -> driveMotor.getSelectedSensorPosition(0), null);
    builder.addDoubleProperty(
        moduleID + "/drive/MotorOutputPercent", () -> driveMotor.getMotorOutputPercent(), null);
    builder.addDoubleProperty(
        moduleID + "/drive/MotorOutputSimVoltage",
        () -> driveMotorSim.getMotorOutputLeadVoltage(),
        null);
    builder.addDoubleProperty(
        moduleID + "/drive/ClosedLoopError", () -> driveMotor.getClosedLoopError(0), null);
    builder.addDoubleProperty(
        moduleID + "/drive/ClosedLoopTarget", () -> driveMotor.getClosedLoopTarget(0), null);
  }

  @Override
  public void ConfigureSystem() {

    driveMotor.setSensorPhase(false);
    driveMotor.setInverted(TalonFXInvertType.Clockwise);
    driveMotor.setNeutralMode(NeutralMode.Brake);
    driveMotor.configFactoryDefault();

    TalonFXConfiguration driveSettings = new TalonFXConfiguration();
    driveMotor.getAllConfigs(driveSettings);

    driveSettings.supplyCurrLimit =
        new SupplyCurrentLimitConfiguration(
            true, DRIVE_CURRENT_LIMIT_AMPS, DRIVE_CURRENT_LIMIT_AMPS, 0);
    driveSettings.slot0.kP = config.getDrivePIDConfig().getKP();
    driveSettings.slot0.kD = config.getDrivePIDConfig().getKD();

    driveMotor.configAllSettings(driveSettings);

    turnMotor.setSensorPhase(false);
    turnMotor.setInverted(TalonFXInvertType.CounterClockwise);
    turnMotor.setNeutralMode(NeutralMode.Brake);
    turnMotor.configFactoryDefault();

    TalonFXConfiguration turnSettings = new TalonFXConfiguration();
    turnMotor.getAllConfigs(turnSettings);

    turnSettings.supplyCurrLimit =
        new SupplyCurrentLimitConfiguration(
            true, TURN_CURRENT_LIMIT_AMPS, TURN_CURRENT_LIMIT_AMPS, 0);
    turnSettings.slot0.kP = config.getTurnPIDConfig().getKP();
    turnSettings.slot0.kD = config.getTurnPIDConfig().getKD();

    double vel_in_turn_motor_frame =
        config.getMaxTurnVelocityRadiansPerSecond() * TURN_GEARBOX_RATIO;
    double vel_in_turn_motor_units =
        vel_in_turn_motor_frame
            / UnitConversion.RADIANS_PER_ROTATION
            * 2048
            * 100
            * UnitConversion.MILLISECONDS;

    double accel_in_turn_motor_frame =
        config.getMaxTurnAccelRadiansPerSecondPerSecond() * TURN_GEARBOX_RATIO;
    double accel_in_turn_motor_units =
        accel_in_turn_motor_frame
            / UnitConversion.RADIANS_PER_ROTATION
            * 2048
            * 100
            * UnitConversion.MILLISECONDS;

    turnSettings.motionCruiseVelocity = vel_in_turn_motor_units;
    turnSettings.motionAcceleration = accel_in_turn_motor_units;

    turnMotor.configAllSettings(turnSettings);

    CANCoderConfiguration encoderSettings = new CANCoderConfiguration();
    turnEncoder.getAllConfigs(encoderSettings);

    encoderSettings.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    encoderSettings.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    if (RobotBase.isReal()) {
      encoderSettings.magnetOffsetDegrees = angleOffset * UnitConversion.DEGREES_PER_RADIAN;
    }

    encoderSettings.sensorDirection = false;
    turnEncoder.configAllSettings(encoderSettings);
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
    double driveAmps = SimDrive(battery);
    double turnAmps = SimTurn(battery);
  }

  private double SimDrive(double battery) {
    driveMotorSim.setBusVoltage(12);
    driveSim.setInputVoltage(-driveMotorSim.getMotorOutputLeadVoltage());

    driveSim.update(20 * UnitConversion.MILLISECONDS);

    driveMotorSim.setStatorCurrent(driveSim.getCurrentDrawAmps());

    double speed_at_wheel_RadiansPerSecond = driveSim.getAngularVelocityRadPerSec();
    double speed_at_motor_RadiansPerSecond = speed_at_wheel_RadiansPerSecond * DRIVE_GEARBOX_RATIO;
    double speed_in_native_units =
        speed_at_motor_RadiansPerSecond
            / UnitConversion.RADIANS_PER_ROTATION
            * 2048
            * 100
            * UnitConversion.MILLISECONDS;

    driveMotorSim.setIntegratedSensorVelocity(-(int) speed_in_native_units);

    return driveSim.getCurrentDrawAmps();
  }

  private double SimTurn(double battery) {
    turnMotorSim.setBusVoltage(12);
    turnEncoderSim.setBusVoltage(12);

    turnSim.setInputVoltage(turnMotorSim.getMotorOutputLeadVoltage());

    turnSim.update(20 * UnitConversion.MILLISECONDS);

    turnMotorSim.setStatorCurrent(turnSim.getCurrentDrawAmps());

    double angular_pos = turnSim.getAngleRadians();
    double angular_vel = turnSim.getAngularVelocityRadiansPerSecond();

    double angular_pos_at_encoder = angular_pos / UnitConversion.RADIANS_PER_ROTATION * 4096;
    double angular_vel_at_encoder =
        angular_vel
            / UnitConversion.RADIANS_PER_ROTATION
            * 4096
            * 100
            * UnitConversion.MILLISECONDS;

    SmartDashboard.putNumber(moduleID + "wtf/sim_pos", angular_pos);
    SmartDashboard.putNumber(moduleID + "wtf/sim_pos_encoder", angular_pos_at_encoder);

    turnEncoderSim.setRawPosition((int) angular_pos_at_encoder);
    turnEncoderSim.setVelocity((int) angular_vel_at_encoder);

    double angular_pos_at_motor =
        angular_pos / UnitConversion.RADIANS_PER_ROTATION * TURN_GEARBOX_RATIO * 2048;
    double angular_vel_at_motor =
        angular_vel
            / UnitConversion.RADIANS_PER_ROTATION
            * TURN_GEARBOX_RATIO
            * 2048
            * 100
            * UnitConversion.MILLISECONDS;

    turnMotorSim.setIntegratedSensorRawPosition((int) angular_pos_at_motor);
    turnMotorSim.setIntegratedSensorVelocity((int) angular_vel_at_motor);

    return turnSim.getCurrentDrawAmps();
  }

  public ErrorCode SeedTurnMotor() {
    double pos_at_encoder = GetAngle().getRotations();
    double pos_at_motor = pos_at_encoder * TURN_GEARBOX_RATIO * 2048;

    turnMotor.stopMotor();

    return turnMotor.setSelectedSensorPosition(pos_at_motor);
  }

  public double GetAngularVelocityRotationsPerSecond() {
    double motor_rps =
        turnMotor.getSelectedSensorVelocity() / 2048 / (100 * UnitConversion.MILLISECONDS);

    return motor_rps / TURN_GEARBOX_RATIO;
  }
}
