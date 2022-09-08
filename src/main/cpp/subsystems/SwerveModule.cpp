#include "subsystems/SwerveModule.h"
#include <units/angle.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/simulation/SimHooks.h>
#include <lib/FalconHelper.h>
#include <thread>

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
{
    // Drive Motor Configuration
  FalconHelper::CheckError(m_driveMotor.ConfigFactoryDefault());

  // Turning Motor Configuration
  FalconHelper::CheckError(m_turningMotor.ConfigFactoryDefault());

  FalconHelper::CheckError(turningEncAbs.ConfigFactoryDefault());

  // Limit the PID Controller's input range between -pi and pi and set the input
  // to be continuous.
  m_turningPIDController.EnableContinuousInput(-units::radian_t(units::constants::detail::PI_VAL),
                                               units::radian_t(units::constants::detail::PI_VAL));
  m_turningPIDController.Reset(units::degree_t(turningEncAbs.GetAbsolutePosition()));
}

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

  frc::SmartDashboard::PutNumber(moduleID + "/raw_drive", m_driveVolts / 1_V);
  frc::SmartDashboard::PutNumber(moduleID + "/raw_turn", m_turnVolts / 1_V);

  // Output
  //std::cout << m_driveVolts.value() << std::endl;
  m_driveMotor.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::PercentOutput, m_driveVolts / 12_V);
  m_turningMotor.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::PercentOutput, m_turnVolts / 12_V);
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
}

units::ampere_t SwerveModule::SimDrive(units::volt_t battery)
{
    m_driveMotorSim->SetBusVoltage(12); //battery / 1_V);

    FalconHelper::CheckError(m_driveMotorSim->GetLastError());

    auto lead_voltage = m_driveMotorSim->GetMotorOutputLeadVoltage() * 1_V;

    FalconHelper::CheckError(m_driveMotorSim->GetLastError());

    m_driveSim.SetInputVoltage(lead_voltage);

    m_driveSim.Update(20_ms);

    auto current_draw = m_driveSim.GetCurrentDraw();

    m_driveMotorSim->SetStatorCurrent(current_draw / 1_A);

    FalconHelper::CheckError(m_driveMotorSim->GetLastError());

    auto linear_vel = m_driveSim.GetAngularVelocity();
    auto linear_vel_at_motor = linear_vel / 1_tr * kDriveGearboxRatio * 2048 * 100_ms;

    m_driveMotorSim->AddIntegratedSensorPosition(linear_vel_at_motor * 20_ms / 100_ms);

    FalconHelper::CheckError(m_driveMotorSim->GetLastError());

    m_driveMotorSim->SetIntegratedSensorVelocity(linear_vel_at_motor);

    FalconHelper::CheckError(m_driveMotorSim->GetLastError());

    auto floor_vel = m_driveSim.GetLinearVelocity();

    frc::SmartDashboard::PutNumber(moduleID + "/sim_drive_voltage", lead_voltage / 1_V);
    frc::SmartDashboard::PutNumber(moduleID + "/sim_drive_current_draw", current_draw / 1_A);
    frc::SmartDashboard::PutNumber(moduleID + "/sim_drive_linear_vel", floor_vel / 1_mps);

    return current_draw;
}

units::ampere_t SwerveModule::SimTurn(units::volt_t battery)
{
    frc::sim::StepTiming(20_ms);

    m_turnMotorSim->SetBusVoltage(12); //battery / 1_V);
    m_encoderSim->SetBusVoltage(12); //battery / 1_V);

    frc::SmartDashboard::PutNumber(moduleID + "/sim_turn_voltage", m_turnMotorSim->GetMotorOutputLeadVoltage());
    m_turnSim.SetInputVoltage(m_turnMotorSim->GetMotorOutputLeadVoltage() * 1_V);

    m_turnSim.Update(20_ms);

    m_turnMotorSim->SetStatorCurrent(m_turnSim.GetCurrentDraw() / 1_A);

    auto angular_pos = m_turnSim.GetAngularPosition();
    auto angular_vel = m_turnSim.GetAngularVelocity();

    // native cancoder units are 4096 ticks per rotation, vel measured per 100ms
    auto angular_pos_at_cancoder = angular_pos / 1_tr * 4096;
    auto angular_vel_at_cancoder = angular_vel / 1_tr * 4096 * 100_ms;

    m_encoderSim->SetRawPosition(angular_pos_at_cancoder);
    m_encoderSim->SetVelocity(angular_vel_at_cancoder);

    // native falcon units are 2048 ticks per rotation, vel measured per 100ms
    auto angular_pos_at_motor = angular_pos / 1_tr * kTurnGearboxRatio * 2048;
    auto angular_vel_at_motor = angular_vel / 1_tr * kDriveGearboxRatio * 2048 * 100_ms;

    m_turnMotorSim->SetIntegratedSensorRawPosition(angular_pos_at_motor);
    m_turnMotorSim->SetIntegratedSensorVelocity(angular_vel_at_motor);

    return m_turnSim.GetCurrentDraw();
}

units::ampere_t SwerveModule::SimPeriodic(units::volt_t battery)
{
    return SimDrive(battery); // + SimTurn(battery);
}
