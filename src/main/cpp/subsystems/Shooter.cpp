#include "subsystems/Shooter.hpp"

#include <cmath>
#include <wpi/DataLog.h>

#include "frc/DigitalInput.h"
#include "frc/Solenoid.h"
#include "frc/filter/LinearFilter.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/math.h"
#include "wpi/sendable/SendableBuilder.h"
#include "ctre/phoenix/motorcontrol/ControlMode.h"
#include "ctre/phoenix/motorcontrol/NeutralMode.h"
#include "ctre/phoenix/motorcontrol/StatusFrame.h"
#include "ctre/phoenix/motorcontrol/SupplyCurrentLimitConfiguration.h"
#include <wpi/timestamp.h>

using namespace ctre::phoenix::motorcontrol;

Shooter::Shooter()
{
    // Factory Defaults
    intake.ConfigFactoryDefault();
    indexerA.ConfigFactoryDefault();
    // indexerB.ConfigFactoryDefault();
    feeder.ConfigFactoryDefault();
    shooterA.ConfigFactoryDefault();
    shooterB.ConfigFactoryDefault();
    turret.ConfigFactoryDefault();
    // hood.ConfigFactoryDefault();

    // Invert motors
    intake.SetInverted(false);
    indexerA.SetInverted(true);
    // indexerB.SetInverted(false);
    feeder.SetInverted(false);
    shooterA.SetInverted(false);
    shooterB.SetInverted(true);
    // hood.SetInverted(false);

    // Break / Coast mode (Not affected by ConfigFactoryDefault)
    intake.SetNeutralMode(NeutralMode::Coast);
    indexerA.SetNeutralMode(NeutralMode::Coast);
    // indexerB.SetNeutralMode(NeutralMode::Coast);
    shooterA.SetNeutralMode(NeutralMode::Coast);
    shooterB.SetNeutralMode(NeutralMode::Coast);
    // hood.SetNeutralMode(NeutralMode::Coast);

    // Closed Loop Configuration
    shooterA.GetSlotConfigs(shooterSlotConfig, 0);
    shooterSlotConfig.kF = 0.056494409;
    shooterSlotConfig.kP = 0.225;
    shooterSlotConfig.kI = 0.0001;
    shooterSlotConfig.kD = 6.000;
    shooterSlotConfig.integralZone = 400.0;

    FalconSlotConfig(shooterA, 0, shooterSlotConfig);
    FalconSlotConfig(shooterB, 0, shooterSlotConfig);

    SetStatusFrames(shooterA, 250);
    SetStatusFrames(shooterB, 250);
    SetStatusFrames(intake, 250);
    SetStatusFrames(indexerA, 250);
    SetStatusFrames(feeder, 250);
    SetStatusFrames(turret, 250);

    shooterA.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 18);
    shooterB.SetStatusFramePeriod(ctre::phoenix::motorcontrol::StatusFrameEnhanced::Status_3_Quadrature, 18);

    // turret.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 10, 50);
    turret.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 20, 50);
    turret.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 20, 50);
    turret.SetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature, 20, 50);

    turret.SetSelectedSensorPosition(0.0);
    turret.SetInverted(true);
    turret.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    turret.ConfigPeakOutputForward(0.625); // Crank it!!!
    turret.ConfigPeakOutputReverse(-0.625);
    turret.ConfigForwardSoftLimitThreshold(kTurretMax / kScaleFactorTurret);
    turret.ConfigReverseSoftLimitThreshold(kTurretMin / kScaleFactorTurret);
    turret.Config_IntegralZone(0, 2_deg / kScaleFactorTurret);
    turret.ConfigAllowableClosedloopError(0, 0.5_deg / kScaleFactorTurret);
    turret.ConfigForwardSoftLimitEnable(true);
    turret.ConfigReverseSoftLimitEnable(true);

    turret.ConfigNeutralDeadband(0.05);

    indexerA.ConfigSupplyCurrentLimit(ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration(true, 20, 20, 0));
    intake.ConfigSupplyCurrentLimit(ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration(true, 30, 30, 0));


    turret.Config_kP(0, 0.4);

    turret.Config_kP(0, 0.041334);
    turret.Config_kD(0, 0.010188);
    // 90 deg/s * 198 / 11040 * 2048 / 10 = 330.573913 tp100ms / 2 = 165.2869565
    turret.ConfigMotionCruiseVelocity(5289);
    // 360 deg/s/s * 198 / 11040 * 2048 / 10 = 1322.295652 tp100ms / s
    turret.ConfigMotionAcceleration(10578);
}

void Shooter::ConfigureSystem() {}

void Shooter::UpdateTelemetry()
{
    frc::SmartDashboard::PutNumber("Turret Angle Kekw", GetTurretAngle().value());

    // turret.Config_kP(0, frc::SmartDashboard::GetNumber("TURRET P", 0.4));
    // turret.Config_kI(0, frc::SmartDashboard::GetNumber("TURRET I", 0.0001));
    // turret.Config_kD(0, frc::SmartDashboard::GetNumber("TURRET D", 0.0));

    frc::SmartDashboard::PutBoolean("ZEROED", zeroed);
    frc::SmartDashboard::PutBoolean("ZEROED SWITCH", GetTurretSwitch());
    frc::SmartDashboard::PutBoolean("NOT ZEROED!!!", blinkyZeroLight);
}

void Shooter::Periodic()
{
    m_latestFilterResult = filter.Calculate(GetShooterRPM());
}

void Shooter::SetIntakeState(Position pos)
{
    deployPiston.Set((bool)pos);
}

void Shooter::SetIntake(units::volt_t voltage)
{
    intake.SetVoltage(voltage);
}

void Shooter::SetIndexer(units::volt_t voltage)
{
    // std::cout << "Indexer set to " << voltage.value() << "v" << std::endl;
    indexerA.SetVoltage(voltage);
    // indexerB.SetVoltage(voltage);
}

void Shooter::SetFeeder(units::volt_t voltage)
{
    // std::cout << "Feeder set to " << voltage.value() << "v" << std::endl;
    feeder.SetVoltage(voltage);
}

void Shooter::SetShooter(units::volt_t voltage)
{
    // std::cout << "Shooter set to " << voltage.value() << "v" << std::endl;
    shooterA.SetVoltage(voltage);
    shooterB.SetVoltage(voltage);
}

void Shooter::SetShooterRPM(units::revolutions_per_minute_t targetRPM)
{
    // std::cout << "Shooter set to " << targetRPM.value() << "rpm" << std::endl;
    cmd_shooterRPM = targetRPM;

    if (targetRPM < 1.0_rpm)
    {
        shooterA.Set(0.0);
        shooterB.Set(0.0);
        return;
    }

    shooterA.Set(ControlMode::Velocity, targetRPM.value() / kTicks2RPM);
    shooterB.Set(ControlMode::Velocity, targetRPM.value() / kTicks2RPM * shooter_ratio);
}

void Shooter::SetShooterRPM()
{
    SetShooterRPM(cmd_shooterRPM);
}

void Shooter::SetShooterRatio(double ratio)
{
    shooter_ratio = ratio;
}

void Shooter::SetTurretCoastMode()
{
    turret.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
}

void Shooter::SetTurretBrakeMode()
{
    turret.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
}

void Shooter::SetTurret(units::volt_t voltage)
{
    turret.SetVoltage(voltage);
}

bool Shooter::GetTurretSwitch()
{
    return !turretZeroSwitch.Get();
}

void Shooter::ZeroTurret(bool negative)
{
    if (negative)
        turret.SetSelectedSensorPosition(-3_deg / kScaleFactorTurret);
    else
        turret.SetSelectedSensorPosition(0.4_deg / kScaleFactorTurret);
    zeroed = true;
}

void Shooter::ZeroTurret()
{
    turret.SetSelectedSensorPosition(0.0);
    zeroed = true;
    blinkyZeroLight = true;
}

void Shooter::SetBlinkyZeroThing()
{
    epilepsyTimer.Start();
    if (epilepsyTimer.Get() > 0.2_s)
    {
        blinkyZeroLight = !blinkyZeroLight;
        epilepsyTimer.Reset();
        epilepsyTimer.Start();
    }
}

bool Shooter::SetTurretAngle(units::degree_t targetAngle, units::degree_t tol)
{
    if (!zeroed)
        return false;

    goalAngle = targetAngle;

    turret.Config_kP(0, 0.4);


    turret.Set(ctre::phoenix::motorcontrol::ControlMode::Position, targetAngle / kScaleFactorTurret);

    return (units::math::abs((GetTurretAngle() - targetAngle)) < tol);
}


bool Shooter::SetTurretAngleSmooth(units::degree_t targetAngle, units::degree_t tol)
{
    if (!zeroed)
        return false;

    goalAngle = targetAngle;
    turret.Config_kP(0, 0.041334);
    turret.Config_kD(0, 0.010188);

    turret.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic, targetAngle / kScaleFactorTurret);

    return (units::math::abs((GetTurretAngle() - targetAngle)) < tol);
}

void Shooter::SetHoodAngle(Shooter::HoodPosition pos)
{
    return;
    cmd_hoodPosition = pos;
    hoodPosOS = false;

    if (pos == HoodPosition::Top)
    {
        hoodStop.Set(false);
        hood.Set(true);
    }
}

void Shooter::SetHoodAngle()
{
    return;
    // std::cout << (int)cmd_hoodPosition << std::endl;

    if (cmd_hoodPosition == HoodPosition::Top)
        return;

    switch (cmd_hoodPosition)
    {
    case HoodPosition::Top:
    {
        hoodStop.Set(false);
        hood.Set(true);
    }
    break;

    case HoodPosition::Middle:
    {
        hoodStop.Set(true);
        if (!hoodPosOS)
        {
            hoodPosTimer.Reset();
            hoodPosTimer.Start();
            hood.Set(true);
            hoodPosOS = true;
        }

        if ((hoodPosTimer.Get() > 0.5_s) && (hoodPosOS))
            hood.Set(false);
    }
    break;

    case HoodPosition::Bottom:
    {
        if (!hoodPosOS)
        {
            hoodStop.Set(false);
            hoodPosTimer.Reset();
            hoodPosTimer.Start();
            hood.Set(true);
            hoodPosOS = true;
        }

        if ((hoodPosTimer.Get() > 0.2_s) && (hoodPosOS))
            hood.Set(false);
    }
    break;
    }
}

units::revolutions_per_minute_t Shooter::GetShooterRPM()
{
    return units::revolutions_per_minute_t{shooterA.GetSelectedSensorVelocity() * kTicks2RPM};
}

units::degree_t Shooter::GetTurretAngle()
{
    return turret.GetSelectedSensorPosition() * kScaleFactorTurret;
}

void Shooter::ResetEdgeDetector()
{
    m_upToSpeed = false;
}

bool Shooter::Shoot_EdgeDetector()
{
    // Resume Last Command (in case shooter was stopped)
    SetShooterRPM(cmd_shooterRPM);

#ifdef __FRC_ROBORIO__

    const double tol = 0.1;
    if (m_upToSpeed) {
        if (m_latestFilterResult < (cmd_shooterRPM * tol) - cmd_shooterRPM) {
            m_upToSpeed = false;
            return true;
        }
        return false;
    } else {
        m_upToSpeed = units::math::abs(GetShooterRPM() - cmd_shooterRPM) > (cmd_shooterRPM * tol);
        return false;
    }
#else
    // TODO: how to handle in sim
    // for now just assume it's legendary
    return true;
#endif
}

bool Shooter::Shoot(units::second_t settleTime)
{

    static frc::Timer settleTimer;
    settleTimer.Start();

    // Resume Last Command (in case shooter was stopped)
    SetShooterRPM(cmd_shooterRPM);
// SetTurretAngle(cmd_turretAngle);
// SetHoodAngle(cmd_hoodAngle);

// Percent of setpoint to accept and begin shooting
// TODO: Sensor Override Mode
#ifdef __FRC_ROBORIO__
    // Real Robot
    const double tol = 0.1;
    if (units::math::abs(GetShooterRPM() - cmd_shooterRPM) > (cmd_shooterRPM * tol))
        settleTimer.Reset();
#else
    // Simulator
    // Just wait and proceed.
#endif
    // if(units::math::abs(GetTurretAngle() - cmd_TurretAngle) > (1.5_deg)) settleTimer.Reset();
    // if(units::math::abs(GetHoodAngle() - cmd_HoodAngle) > (0.5_deg)) settleTimer.Reset();

    // Prevent the intake from holding on to a ball while stowed
    // TODO: Should intake be automatic?
    // SetIntakeState(Position::Deployed);

    // Wait for shooter to settle before feeding the ball
    // if(settleTimer.Get() > 0.5_s) // TODO: Move to NT Parameter / Class Constant?
    // {
    //     //SetIntake(10_V);
    //     //SetIndexer(10_V);
    //     SetFeeder(10_V); // TODO: Move to NT Parameter / Class Constant?
    // } else {
    //     //SetIntake(0_V);
    //     //SetIndexer(0_V);
    //     SetFeeder(-2_V);
    // }

    // Wait for balls to exit robot in auto
    if (settleTimer.Get() > settleTime)
    {
        settleTimer.Reset();
        settleTimer.Stop();

        return true;
    }
    return false; // TODO: Move to NT Parameter / Class Constant?
}

bool Shooter::AtRPM(units::revolutions_per_minute_t threshold)
{
    return units::math::abs(GetShooterRPM() - cmd_shooterRPM) < threshold;
}

Shooter::State Shooter::CalculateShot(units::inch_t distance)
{
    double mainWheel = 2031.0 + (0.0015 * std::pow(distance.value(), 3)) + (-0.34445 * std::pow(distance.value(), 2)) + (28.5196 * std::pow(distance.value(), 1));

    State shotStates;
    shotStates.hoodAngle = HoodPosition::Middle;
    shotStates.shooterRPM = units::revolutions_per_minute_t{mainWheel};

    return shotStates;
}

void Shooter::FalconSlotConfig(WPI_TalonFX &motor, int slot, SlotConfiguration &config)
{
    motor.Config_kF(slot, config.kF);
    motor.Config_kP(slot, config.kP);
    motor.Config_kI(slot, config.kI);
    motor.Config_kD(slot, config.kD);
    motor.Config_IntegralZone(slot, config.integralZone);
    motor.ConfigAllowableClosedloopError(slot, config.allowableClosedloopError);
    motor.ConfigMaxIntegralAccumulator(slot, config.maxIntegralAccumulator);
    motor.ConfigClosedLoopPeakOutput(slot, config.closedLoopPeakOutput);
    motor.ConfigClosedLoopPeriod(slot, config.closedLoopPeriod);

    // Note: CTRE is adding this function for us in the next release
    // https://github.com/CrossTheRoadElec/Phoenix-Releases/issues/27
    // motor.ConfigureSlot(config, 0);
}

void Shooter::FalconSendableHelper(wpi::SendableBuilder &builder, WPI_TalonFX &motor, std::string name)
{
    builder.AddDoubleProperty(
        name + "/percent",
        [&motor]
        { return motor.Get(); },
        nullptr);
    //[&motor] (double value) { motor.Set(value); });
    builder.AddDoubleProperty(
        name + "/voltage",
        [&motor]
        { return motor.GetMotorOutputVoltage(); },
        nullptr);
    //[&motor](double value) { motor.SetVoltage(units::volt_t(value)); });
    builder.AddDoubleProperty(
        name + "/rpm",
        [&motor]
        { return motor.GetSelectedSensorVelocity() * kTicks2RPM; },
        nullptr);
    // [this, &motor](double value) {
    //     motor.Set(TalonFXControlMode::Velocity, value / kTicks2RPM);
    // });
    builder.AddDoubleProperty(
        name + "/temperature",
        [&motor]
        { return motor.GetTemperature(); },
        nullptr);

    builder.AddDoubleProperty(
        name + "/current",
        [&motor]
        { return motor.GetSupplyCurrent(); },
        nullptr);
}

void Shooter::InitSendable(wpi::SendableBuilder &builder)
{
    builder.SetSmartDashboardType("Shooter");
    builder.SetActuator(true);

    // Commands
    builder.AddDoubleProperty(
        "cmd/shooterRPM",
        [this]
        { return cmd_shooterRPM.value(); },
        nullptr);
    //[this] (double value) {  cmd_shooterRPM = units::revolutions_per_minute_t{value}; });

    // Basic Motors
    FalconSendableHelper(builder, intake, "intake");
    FalconSendableHelper(builder, indexerA, "indexerA");
    FalconSendableHelper(builder, feeder, "feeder");
    FalconSendableHelper(builder, shooterA, "shooterA");
    FalconSendableHelper(builder, shooterB, "shooterB");
    // FalconSendableHelper(builder, turret, "turret");
    // FalconSendableHelper(builder, hood, "hood");

    // Shooter PID
    // TODO: Disable this for Competition?
    /*
    builder.AddDoubleProperty(
        "shooter/kP",
        [this]
        { return shooterSlotConfig.kP; },
        [this](double value)
        {
            shooterSlotConfig.kP = value;
            shooterA.Config_kP(0, value);
            shooterB.Config_kP(0, value);
        });
    builder.AddDoubleProperty(
        "shooter/kI",
        [this]
        { return shooterSlotConfig.kI; },
        [this](double value)
        {
            shooterSlotConfig.kI = value;
            shooterA.Config_kI(0, value);
            shooterB.Config_kI(0, value);
        });
    builder.AddDoubleProperty(
        "shooter/kD",
        [this]
        { return shooterSlotConfig.kD; },
        [this](double value)
        {
            shooterSlotConfig.kD = value;
            shooterA.Config_kD(0, value);
            shooterB.Config_kD(0, value);
        });
    builder.AddDoubleProperty(
        "shooter/kF",
        [this]
        { return shooterSlotConfig.kF; },
        [this](double value)
        {
            shooterSlotConfig.kF = value;
            shooterA.Config_kF(0, value);
            shooterB.Config_kF(0, value);
        });
    builder.AddDoubleProperty(
        "shooter/kIZ",
        [this]
        { return shooterSlotConfig.integralZone; },
        [this](double value)
        {
            shooterSlotConfig.integralZone = value;
            shooterA.Config_IntegralZone(0, value);
            shooterB.Config_IntegralZone(0, value);
        });
    builder.AddDoubleProperty(
        "shooter/kMaxError",
        [this]
        { return shooterSlotConfig.allowableClosedloopError; },
        [this](double value)
        {
            shooterSlotConfig.allowableClosedloopError = value;
            shooterA.ConfigAllowableClosedloopError(0, value);
            shooterB.ConfigAllowableClosedloopError(0, value);
        });
    builder.AddDoubleProperty(
        "shooter/kMaxIntergral",
        [this]
        { return shooterSlotConfig.maxIntegralAccumulator; },
        [this](double value)
        {
            shooterSlotConfig.maxIntegralAccumulator = value;
            shooterA.ConfigMaxIntegralAccumulator(0, value);
            shooterB.ConfigMaxIntegralAccumulator(0, value);
        });

    // Pneumatics
    // TODO: Log PSI & compressor state
    builder.AddStringProperty(
        "intake/sol", [this]
        { return deployPiston.Get() ? "Deployed" : "Stowed"; },
        nullptr);
    */

    builder.AddDoubleProperty("shooter/hp_filter_result", [this] { return m_latestFilterResult.value(); }, nullptr);
}

void Shooter::LogDataEntries(wpi::log::DataLog &log)
{
    log.AppendDouble(GetDataEntry("Shooter/cmd/shooterRPM"), cmd_shooterRPM.value(), 0);

    FalconEntryHelper(log, intake, "Shooter/intake");
    FalconEntryHelper(log, indexerA, "Shooter/indexerA");
    FalconEntryHelper(log, feeder, "Shooter/feeder");
    FalconEntryHelper(log, shooterA, "Shooter/shooterA");
    FalconEntryHelper(log, shooterB, "Shooter/shooterB");
    FalconEntryHelper(log, turret, "Shooter/turret");
    // FalconEntryHelper(log, hood, "hood");

    uint64_t time = wpi::Now();

    log.AppendDouble(GetDataEntry("Shooter/turret/goal"), goalAngle.value(), time);
    log.AppendDouble(GetDataEntry("Shooter/turret/position"), GetTurretAngle().value(), time);

    // Shooter PID
    log.AppendDouble(GetDataEntry("Shooter/shooter/kP"), shooterSlotConfig.kP, 0);
    log.AppendDouble(GetDataEntry("Shooter/shooter/kI"), shooterSlotConfig.kI, 0);
    log.AppendDouble(GetDataEntry("Shooter/shooter/kD"), shooterSlotConfig.kD, 0);
    log.AppendDouble(GetDataEntry("Shooter/shooter/kF"), shooterSlotConfig.kF, 0);
    log.AppendDouble(GetDataEntry("Shooter/shooter/kIZ"), shooterSlotConfig.integralZone, 0);
    log.AppendDouble(GetDataEntry("Shooter/shooter/kMaxError"), shooterSlotConfig.allowableClosedloopError, 0);
    log.AppendDouble(GetDataEntry("Shooter/shooter/kMaxIntegral"), shooterSlotConfig.maxIntegralAccumulator, 0);

    // Pneumatics
    log.AppendBoolean(GetDataEntry("Shooter/intake/sol"), deployPiston.Get(), 0);

    log.AppendDouble(GetDataEntry("Shooter/shooter/hp_filter_result"), m_latestFilterResult.value(), 0);
}

void Shooter::RegisterDataEntries(wpi::log::DataLog &log)
{
    RegisterDataEntry(log, "Shooter/cmd/shooterRPM", "double");

    FalconEntryStartHelper(log, "Shooter/intake");
    FalconEntryStartHelper(log, "Shooter/indexerA");
    FalconEntryStartHelper(log, "Shooter/feeder");
    FalconEntryStartHelper(log, "Shooter/shooterA");
    FalconEntryStartHelper(log, "Shooter/shooterB");
    FalconEntryStartHelper(log, "Shooter/turret");
    // FalconEntryStartHelper(log, "Shooter/hood");
    
    
    // Shooter PID
    RegisterDataEntry(log, "Shooter/turret/goal", "double");
    RegisterDataEntry(log, "Shooter/turret/position", "double");

    // Shooter PID
    RegisterDataEntry(log, "Shooter/shooter/kP", "double");
    RegisterDataEntry(log, "Shooter/shooter/kI", "double");
    RegisterDataEntry(log, "Shooter/shooter/kD", "double");
    RegisterDataEntry(log, "Shooter/shooter/kF", "double");
    RegisterDataEntry(log, "Shooter/shooter/kIZ", "double");
    RegisterDataEntry(log, "Shooter/shooter/kMaxError", "double");
    RegisterDataEntry(log, "Shooter/shooter/kMaxIntegral", "double");

    // Pneumatics
    RegisterDataEntry(log, "Shooter/intake/sol", "boolean");

    RegisterDataEntry(log, "Shooter/shooter/hp_filter_result", "double");
}
