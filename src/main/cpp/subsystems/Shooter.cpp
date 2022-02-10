#include "subsystems/Shooter.hpp"

Shooter::Shooter()
{
    // Factory Defaults
    intake.ConfigFactoryDefault();
    indexerA.ConfigFactoryDefault();
    //indexerB.ConfigFactoryDefault();
    feeder.ConfigFactoryDefault();
    shooterA.ConfigFactoryDefault();
    shooterB.ConfigFactoryDefault();
    shooterTop.ConfigFactoryDefault();
    //turret.ConfigFactoryDefault();
    //hood.ConfigFactoryDefault();

    // Invert motors
    intake.SetInverted(false);
    indexerA.SetInverted(true);
    //indexerB.SetInverted(false);
    feeder.SetInverted(false);
    shooterA.SetInverted(true);
    shooterB.SetInverted(false);
    shooterTop.SetInverted(false);
    //turret.SetInverted(false);
    //hood.SetInverted(false);

    // Break / Coast mode (Not affected by ConfigFactoryDefault)
    intake.SetNeutralMode(NeutralMode::Coast);
    indexerA.SetNeutralMode(NeutralMode::Coast);
    //indexerB.SetNeutralMode(NeutralMode::Coast);
    shooterA.SetNeutralMode(NeutralMode::Coast);
    shooterB.SetNeutralMode(NeutralMode::Coast);
    shooterTop.SetNeutralMode(NeutralMode::Coast);
    //turret.SetNeutralMode(NeutralMode::Coast);
    //hood.SetNeutralMode(NeutralMode::Coast);


    // Closed Loop Configuration
    shooterA.GetSlotConfigs(shooterSlotConfig, 0);
    shooterSlotConfig.kF = 0.056494409;
    shooterSlotConfig.kP = 0.225;
    shooterSlotConfig.kI = 0.0001;
    shooterSlotConfig.kD = 6.000;
    shooterSlotConfig.integralZone = 200.0;

    FalconSlotConfig(shooterA, 0, shooterSlotConfig);
    FalconSlotConfig(shooterB, 0, shooterSlotConfig);
    FalconSlotConfig(shooterTop, 0, shooterSlotConfig);
}

void Shooter::ConfigureSystem() { }

void Shooter::UpdateTelemetry() { }

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
    indexerA.SetVoltage(voltage);
    //indexerB.SetVoltage(voltage);
}

void Shooter::SetFeeder(units::volt_t voltage)
{
    feeder.SetVoltage(voltage);
}

void Shooter::SetShooter(units::volt_t voltage)
{
    shooterA.SetVoltage(voltage);
}

void Shooter::SetShooterRPM(units::revolutions_per_minute_t targetRPM)
{
    if (targetRPM == 0.0_rpm) // Riski
    {
        shooterA.Set(0.0);
        shooterB.Set(0.0);
        return;
    }

    shooterA.Set(ControlMode::Velocity, ((targetRPM.value() / kScaleFactorFly) / 600.0));
    shooterB.Set(ControlMode::Velocity, ((targetRPM.value() / kScaleFactorFly) / 600.0));
}


void Shooter::SetShooterTop(units::volt_t voltage)
{
    shooterTop.SetVoltage(voltage);
}

void Shooter::SetShooterTopRPM(units::revolutions_per_minute_t targetRPM)
{    
    if(targetRPM == 0.0_rpm)
    {
        shooterTop.Set(0.0);
        return;
    }

    shooterTop.Set(ControlMode::Velocity, ((targetRPM.value() / kScaleFactorFly) / 600.0));
}

// void Shooter::SetTurret(units::volt_t voltage)
// {
//     // turret.SetVoltage(voltage);
// }

// void Shooter::SetTurretAngle(units::degree_t targetAngle)
// {
//     units::degree_t currentAng = GetTurretAngle();

//     if (targetAngle != turretPID.GetGoal().position)
//     {
//         turretPID.SetGoal(targetAngle);
//     }

//     SetTurret(units::volt_t{turretPID.Calculate(currentAng)});
// }

// void Shooter::SetHood(units::volt_t voltage)
// {
//     // hood.SetVoltage(voltage);
// }

// void Shooter::SetHoodAngle(units::degree_t targetAngle)
// {
//     units::degree_t currentAng = GetTurretAngle();

//     if (targetAngle != turretPID.GetGoal().position)
//     {
//         turretPID.SetGoal(targetAngle);
//     }

//     SetHood(units::volt_t{turretPID.Calculate(currentAng)});
// }

units::revolutions_per_minute_t Shooter::GetShooterRPM()
{
    return units::revolutions_per_minute_t{shooterA.GetSelectedSensorVelocity() * kScaleFactorFly * 600.0};
}

// units::degree_t Shooter::GetTurretAngle()
// {
//     // double ang = turret.GetSelectedSensorPosition() * kScaleFactorTurret;
//     // return units::degree_t(ang);
//     return 0_deg;
// }

Shooter::State Shooter::CalculateShot(units::inch_t distance)
{
    double ratio = 7.5 / 7;

    double mainWheel = 8073 + (-0.00325 * std::pow(distance.value(), 3)) + (1.2191 * std::pow(distance.value(), 2)) + (-139.806 * std::pow(distance.value(), 1));

    State shotStats = {units::revolutions_per_minute_t{mainWheel}, units::revolutions_per_minute_t{mainWheel * ratio}, 0_deg};
    return shotStats;
}

void Shooter::SetShooterState(State shotStats)
{
    State zero = {0.0_rpm, 0.0_rpm, 0.0_deg};

    if(shotStats == zero)
    {
        shooterA.Set(0.0);
        shooterB.Set(0.0);
        shooterTop.Set(0.0);
        return;
    }
    SetShooterRPM(shotStats.shooterVelocity);
    SetShooterTopRPM(shotStats.hoodVelocity);
}

bool Shooter::TempUpToSpeed()
{
    return (std::abs(shooterTop.GetSelectedSensorVelocity() - shooterTop.GetClosedLoopTarget()) < 100) 
        && (std::abs(shooterA.GetSelectedSensorVelocity() - shooterA.GetClosedLoopTarget()) < 100);
}

void Shooter::FalconSlotConfig(WPI_TalonFX& motor, int slot, SlotConfiguration& config)
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
    //motor.ConfigureSlot(config, 0);
}

void Shooter::FalconSendableHelper(wpi::SendableBuilder &builder, WPI_TalonFX& motor, std::string name)
{
    builder.AddDoubleProperty(
        name + "/percent", 
        [&motor] { return motor.Get(); }, 
        [&motor] (double value) { motor.Set(value); });
    builder.AddDoubleProperty(
        name + "/voltage", 
        [&motor] { return motor.GetMotorOutputVoltage(); }, 
        [&motor](double value) { motor.SetVoltage(units::volt_t(value)); });
    builder.AddDoubleProperty(
        name + "/rpm", 
        [&motor] { return motor.GetSelectedSensorVelocity() / 2048.0 * 10.0 * 60.0; }, 
        [&motor](double value) { motor.Set(TalonFXControlMode::Velocity, value * 2048.0 / 10.0 / 60.0); });
    builder.AddDoubleProperty(
        name + "/temperature", 
        [&motor] { return motor.GetTemperature(); }, 
        nullptr);
}

void Shooter::InitSendable(wpi::SendableBuilder &builder)
{
    builder.SetSmartDashboardType("Shooter");
    builder.SetActuator(true);

    // Motors
    FalconSendableHelper(builder, intake, "intake");
    FalconSendableHelper(builder, indexerA, "indexerA");
    //FalconSendableHelper(builder, indexerB, "indexerB");
    FalconSendableHelper(builder, shooterA, "shooterA");
    //FalconSendableHelper(builder, shooterA, "shooterB"); (Slaved)
    FalconSendableHelper(builder, shooterTop, "shooterTop");
    //FalconSendableHelper(builder, turret, "turret");
    //FalconSendableHelper(builder, hood, "hood");

    // Shooter PID 
    // TODO: Disable this for Competition?
    builder.AddDoubleProperty(
        "shooter/kP", 
        [this] { return shooterSlotConfig.kP; }, 
        [this] (double value) { 
            shooterSlotConfig.kP = value; 
            shooterA.Config_kP(0, value); 
            shooterB.Config_kP(0, value); 
            shooterTop.Config_kP(0, value); 
            });
    builder.AddDoubleProperty(
        "shooter/kI", 
        [this] { return shooterSlotConfig.kI; }, 
        [this] (double value) { 
            shooterSlotConfig.kI = value; 
            shooterA.Config_kI(0, value); 
            shooterB.Config_kI(0, value); 
            shooterTop.Config_kI(0, value); 
            });
    builder.AddDoubleProperty(
        "shooter/kD", 
        [this] { return shooterSlotConfig.kD; }, 
        [this] (double value) { 
            shooterSlotConfig.kD = value; 
            shooterA.Config_kD(0, value); 
            shooterB.Config_kD(0, value); 
            shooterTop.Config_kD(0, value); 
            });
    builder.AddDoubleProperty(
        "shooter/kF", 
        [this] { return shooterSlotConfig.kF; }, 
        [this] (double value) { 
            shooterSlotConfig.kF = value; 
            shooterA.Config_kF(0, value); 
            shooterB.Config_kF(0, value); 
            shooterTop.Config_kF(0, value); 
            });
    builder.AddDoubleProperty(
        "shooter/kIZ", 
        [this] { return shooterSlotConfig.integralZone; }, 
        [this] (double value) { 
            shooterSlotConfig.kF = value; 
            shooterA.Config_IntegralZone(0, value); 
            shooterB.Config_IntegralZone(0, value); 
            shooterTop.Config_IntegralZone(0, value); 
            });
    builder.AddDoubleProperty(
        "shooter/kMaxError", 
        [this] { return shooterSlotConfig.allowableClosedloopError; }, 
        [this] (double value) { 
            shooterSlotConfig.allowableClosedloopError = value; 
            shooterA.ConfigAllowableClosedloopError(0, value); 
            shooterB.ConfigAllowableClosedloopError(0, value); 
            shooterTop.ConfigAllowableClosedloopError(0, value); 
            });
    builder.AddDoubleProperty(
        "shooter/kMaxIntergral", 
        [this] { return shooterSlotConfig.maxIntegralAccumulator; }, 
        [this] (double value) { 
            shooterSlotConfig.maxIntegralAccumulator = value; 
            shooterA.ConfigMaxIntegralAccumulator(0, value); 
            shooterB.ConfigMaxIntegralAccumulator(0, value); 
            shooterTop.ConfigMaxIntegralAccumulator(0, value); 
            });

    // Pneumatics
    // TODO: Log PSI & compressor state
    builder.AddStringProperty(
        "intake/sol", [this] { return deployPiston.Get() ? "Deployed":"Stowed"; }, nullptr);
}