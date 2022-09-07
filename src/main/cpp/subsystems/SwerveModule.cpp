#include "subsystems/SwerveModule.h"
#include <units/angle.h>
#include <frc/smartdashboard/SmartDashboard.h>

/**
 * Constructor for the SwerveModule class
 * 
 * @param moduleID String for module identification
 * @param driveMotorChannel CAN ID of the drive Falcon
 * @param turningMotorChannel CAN ID of the turning Falcon
 * @param turningEncoderChannel CAN ID of turning ABS encoder
 * @param config The configuration data for this specific module
 */
SwerveModule::SwerveModule(std::string moduleID, int driveMotorChannel, int turningMotorChannel, int turningEncoderChannel, SwerveModuleConfig config)
    : moduleID(moduleID),
      m_driveMotor(driveMotorChannel),
      m_turningMotor(turningMotorChannel),
      turningEncAbs(turningEncoderChannel),
      m_drivePIDController{config.GetDrivePIDController()},
      m_turningPIDController{config.GetTurnPIDController()},
      m_driveFeedforward{config.GetDriveFeedForward()},
      m_turnFeedforward{config.GetTurnFeedForward()},
      angle_offset(config.GetModuleOffset())
{}

/**
 * Returns the current state of the module
 * 
 * @return frc::SwerveModuleState The current state of the swerve module
 */
frc::SwerveModuleState SwerveModule::GetState()
{
  currentState.angle = GetAngle();
  currentState.speed = GetVelocity();
  return currentState;
}

/**
 * Returns the current velocity of the module
 * 
 * @return units::meters_per_second_t The current velocity of the swerve module
 */
units::meters_per_second_t SwerveModule::GetVelocity()
{
  // Real Hardware
  return m_driveMotor.GetSelectedSensorVelocity(0) * kDriveScaleFactor / 100_ms;
}

/**
 * Returns the current angle of the module
 * 
 * @return frc::Rotation2d The current angle of the swerve module
 */
frc::Rotation2d SwerveModule::GetAngle()
{
  // Real Hardware
  auto un_normalized = frc::Rotation2d(units::degree_t(turningEncAbs.GetAbsolutePosition()));
  return frc::Rotation2d(un_normalized.Cos(), un_normalized.Sin());
}

/**
 * Sets the module's target state
 * 
 * @param state The module's target state
 */
void SwerveModule::SetModule(const frc::SwerveModuleState &state)
{
  currentState = GetState();

  targetState = state;

  const auto opt_state = frc::SwerveModuleState::Optimize(targetState, currentState.angle);

  // Drive
  const auto driveOutput = m_drivePIDController.Calculate(
      currentState.speed,
      opt_state.speed);

  const auto driveFeedforward = m_driveFeedforward.Calculate(m_drivePIDController.GetSetpoint().position, m_drivePIDController.GetSetpoint().velocity);

  m_driveVolts = units::volt_t{driveOutput} + driveFeedforward;

  // Angle
  const auto turnOutput = m_turningPIDController.Calculate(
      currentState.angle.Radians(),
      opt_state.angle.Radians());

  const auto turnFeedforward = m_turnFeedforward.Calculate(
      m_turningPIDController.GetSetpoint().velocity);

  m_turnVolts = units::volt_t{turnOutput} + turnFeedforward;

  frc::SmartDashboard::PutNumber(moduleID + "/raw_drive", m_driveVolts.value());
  frc::SmartDashboard::PutNumber(moduleID + "/raw_turn", m_turnVolts.value());

  // Output
  //std::cout << m_driveVolts.value() << std::endl;
  m_driveMotor.SetVoltage(m_driveVolts);
  m_turningMotor.SetVoltage(m_turnVolts);
}

/**
 * Just Stop....
 *
 */
void SwerveModule::Stop()
{
  m_drivePIDController.Reset(0_mps);
  m_turningPIDController.Reset(0_deg);

  m_driveMotor.SetVoltage(0_V);
  m_turningMotor.SetVoltage(0_V);

  // Simulation
  m_driveSim.SetInputVoltage(0_V);
  m_turnSim.SetInputVoltage(0_V);
}

void SwerveModule::ConfigureSystem()
{
  // Drive Motor Configuration
  m_driveMotor.ConfigFactoryDefault();
  m_driveMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 18);
  m_driveMotor.SetInverted(true); // Remember: forward-positive!
  m_driveMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  m_driveMotor.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, kDriveMotorCurrentLimit.value(), kDriveMotorCurrentLimit.value(), 0.0));
  m_driveMotor.SetSensorPhase(false);

  m_driveMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 250, 50);
  // m_driveMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 250, 50);
  m_driveMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat, 250, 50);
  m_driveMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_6_Misc, 250, 50);
  m_driveMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_7_CommStatus, 250, 50);
  m_driveMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_9_MotProfBuffer, 250, 50);
  m_driveMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_Targets, 250, 50);
  m_driveMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, 250, 50);
  m_driveMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 250, 50);
  m_driveMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, 250, 50);
  m_driveMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_15_FirmareApiStatus, 250, 50);
  m_driveMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_17_Targets1, 250, 50);
  m_driveMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature, 250, 50);
  m_driveMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth, 250, 50);
  m_driveMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_11_UartGadgeteer, 250, 50);
  m_driveMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_Brushless_Current, 250, 50);

  // Turning Motor Configuration
  m_turningMotor.ConfigFactoryDefault();
  m_turningMotor.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 18);
  m_turningMotor.SetInverted(false); // Remember: forward-positive!
  m_turningMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  m_turningMotor.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, kTurningMotorCurrentLimit.value(), kTurningMotorCurrentLimit.value(), 0.0));

  m_turningMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 250, 50);
  m_turningMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 250, 50);
  m_turningMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat, 250, 50);
  m_turningMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_6_Misc, 250, 50);
  m_turningMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_7_CommStatus, 250, 50);
  m_turningMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_9_MotProfBuffer, 250, 50);
  m_turningMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_Targets, 250, 50);
  m_turningMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, 250, 50);
  m_turningMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 250, 50);
  m_turningMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, 250, 50);
  m_turningMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_15_FirmareApiStatus, 250, 50);
  m_turningMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_17_Targets1, 250, 50);
  m_turningMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature, 250, 50);
  m_turningMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth, 250, 50);
  m_turningMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_11_UartGadgeteer, 250, 50);
  m_turningMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_Brushless_Current, 250, 50);


  // Turning Encoder Config
  ctre::phoenix::sensors::CANCoderConfiguration encoderConfig;
  turningEncAbs.GetAllConfigs(encoderConfig);
  encoderConfig.enableOptimizations = true;
  encoderConfig.initializationStrategy = ctre::phoenix::sensors::SensorInitializationStrategy::BootToAbsolutePosition;
  encoderConfig.absoluteSensorRange = ctre::phoenix::sensors::AbsoluteSensorRange::Signed_PlusMinus180;
  encoderConfig.magnetOffsetDegrees = angle_offset.value();
  encoderConfig.sensorDirection = false;
  turningEncAbs.ConfigAllSettings(encoderConfig);

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.EnableContinuousInput(-units::radian_t(units::constants::detail::PI_VAL),
                                               units::radian_t(units::constants::detail::PI_VAL));
  m_turningPIDController.Reset(units::degree_t(turningEncAbs.GetAbsolutePosition()));
}

void SwerveModule::RegisterDataEntries(wpi::log::DataLog &log)
{
}

void SwerveModule::LogDataEntries(wpi::log::DataLog &log)
{

}

void SwerveModule::UpdateTelemetry()
{
  
}

void SwerveModule::InitSendable(wpi::SendableBuilder &builder)
{
  builder.SetSmartDashboardType("SwerveModule");
  builder.SetActuator(true);

  // Drive Control
  builder.AddDoubleProperty(
      moduleID + "/Drive kP", [this] { return m_drivePIDController.GetP(); }, [this](double value) { m_drivePIDController.SetP(value); });
  builder.AddDoubleProperty(
      moduleID + "/Drive kI", [this] { return m_drivePIDController.GetI(); }, [this](double value) { m_drivePIDController.SetI(value); });
  builder.AddDoubleProperty(
      moduleID + "/Drive kD", [this] { return m_drivePIDController.GetD(); }, [this](double value) { m_drivePIDController.SetD(value); });
  builder.AddDoubleProperty(
      moduleID + "/Drive Goal",
      [this] { return units::meters_per_second_t(m_drivePIDController.GetGoal().position).value(); },
      [this](double value) { m_drivePIDController.SetGoal(units::meters_per_second_t(value)); });
  builder.AddDoubleProperty(
      moduleID + "/Drive SP",
      [this] { return units::meters_per_second_t(m_drivePIDController.GetSetpoint().position).value(); }, nullptr);
  builder.AddDoubleProperty(
      moduleID + "/Velocity", [this] { return units::meters_per_second_t(GetVelocity()).value(); }, nullptr);
  builder.AddDoubleProperty(
      moduleID + "/m_driveVolts", [this] { return m_driveVolts.value(); }, nullptr);

  // Angle Control
  builder.AddDoubleProperty(
      moduleID + "/Angle kP", [this] { return m_turningPIDController.GetP(); }, [this](double value) { m_turningPIDController.SetP(value); });
  builder.AddDoubleProperty(
      moduleID + "/Angle kI", [this] { return m_turningPIDController.GetI(); }, [this](double value) { m_turningPIDController.SetI(value); });
  builder.AddDoubleProperty(
      moduleID + "/Angle kD", [this] { return m_turningPIDController.GetD(); }, [this](double value) { m_turningPIDController.SetD(value); });
  builder.AddDoubleProperty(
      moduleID + "/Angle Goal",
      [this] { return units::degree_t(m_turningPIDController.GetGoal().position).value(); },
      [this](double value) { m_turningPIDController.SetGoal(units::degree_t(value)); });
  builder.AddDoubleProperty(
      moduleID + "/Angle SP",
      [this] { return units::degree_t(m_turningPIDController.GetSetpoint().position).value(); }, nullptr);
  builder.AddDoubleProperty(
      moduleID + "/Angle", [this] { return GetAngle().Degrees().value(); }, nullptr);
  builder.AddDoubleProperty(
      moduleID + "/m_turnVolts", [this] { return m_turnVolts.value(); }, nullptr);

  // builder.AddDoubleProperty(
  //     "Angle Offset",
  //     [this] { return prefs->GetDouble(m_angleOffsetPref); },
  //     [this](double value) {
  //         prefs->PutDouble(m_angleOffsetPref, value);
  //         m_turningEncoder.ConfigMagnetOffset(value);
  //     });

  // Turning Encoders
  // builder.AddDoubleProperty(
  //     "Encoder CTRE", [this] { return m_turningEncoder.GetAbsolutePosition(); }, nullptr);

  // Thermal
  builder.AddDoubleProperty(
      moduleID + "/Drive Temp [C]", [this] { return m_driveMotor.GetTemperature(); }, nullptr);
  builder.AddDoubleProperty(
      moduleID + "/Angle Temp [C]", [this] { return m_turningMotor.GetTemperature(); }, nullptr);
}

units::ampere_t SwerveModule::SimPeriodic(units::volt_t battery)
{
  driveMotorSim.SetBusVoltage(battery / 1_V);
  turningMotorSim.SetBusVoltage(battery / 1_V);
  encoderSim.SetBusVoltage(battery / 1_V);
  frc::SmartDashboard::PutNumber(moduleID + "/sim_drive_voltage", driveMotorSim.GetMotorOutputLeadVoltage());
  frc::SmartDashboard::PutNumber(moduleID + "/sim_turn_voltage", turningMotorSim.GetMotorOutputLeadVoltage());
  m_driveSim.SetInputVoltage(1_V * driveMotorSim.GetMotorOutputLeadVoltage());
  m_turnSim.SetInputVoltage(1_V * turningMotorSim.GetMotorOutputLeadVoltage());

  m_driveSim.Update(20_ms);
  m_turnSim.Update(20_ms);

  driveMotorSim.SetStatorCurrent(m_driveSim.GetCurrentDraw() / 1_A);
  turningMotorSim.SetStatorCurrent(m_turnSim.GetCurrentDraw() / 1_A);

  auto angular_pos = m_turnSim.GetAngularPosition();
  auto angular_vel = m_turnSim.GetAngularVelocity();

  // native cancoder units are 4096 ticks per rotation, vel measured per 100ms
  auto angular_pos_at_cancoder = angular_pos / 1_tr * 4096;
  auto angular_vel_at_cancoder = angular_vel / 1_tr * 4096 * 100_ms;

  encoderSim.SetRawPosition(angular_pos_at_cancoder);
  encoderSim.SetVelocity(angular_vel_at_cancoder);

  // native falcon units are 2048 ticks per rotation, vel measured per 100ms
  auto angular_pos_at_motor = angular_pos / 1_tr * kTurnGearboxRatio * 2048;
  auto angular_vel_at_motor = angular_vel / 1_tr * kDriveGearboxRatio * 2048 * 100_ms;

  turningMotorSim.SetIntegratedSensorRawPosition(angular_pos_at_motor);
  turningMotorSim.SetIntegratedSensorVelocity(angular_vel_at_motor);


  auto linear_vel = m_driveSim.GetAngularVelocity();

  auto linear_vel_at_motor = linear_vel / 1_tr * kDriveGearboxRatio * 2048 * 100_ms;

  driveMotorSim.SetIntegratedSensorVelocity(linear_vel_at_motor);


  return m_driveSim.GetCurrentDraw() + m_turnSim.GetCurrentDraw();
}