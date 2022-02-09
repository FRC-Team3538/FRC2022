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
    intake.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
    indexerA.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
    //indexerB.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
    shooterA.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
    shooterB.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
    shooterTop.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
    //turret.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
    //hood.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);


    // Closed Loop Configuration
    shooterA.Config_kF(0, 0.056494409);
    shooterA.Config_kP(0, 0.225);
    shooterA.Config_kI(0, 0.0001);
    shooterA.Config_kD(0, 6.000);
    shooterA.Config_IntegralZone(0, 200.0);

    shooterB.Config_kF(0, 0.056494409);
    shooterB.Config_kP(0, 0.225);
    shooterB.Config_kI(0, 0.0001);
    shooterB.Config_kD(0, 6.000);
    shooterB.Config_IntegralZone(0, 200.0);

    shooterTop.SetInverted(false);
    shooterTop.Config_kF(0, 0.056494409);
    shooterTop.Config_kP(0, 0.225);
    shooterTop.Config_kI(0, 0.0001);
    shooterTop.Config_kD(0, 6.000);
    shooterTop.Config_IntegralZone(0, 200.0);
}

void Shooter::ConfigureSystem() { }

void Shooter::UpdateTelemetry() { }

void Shooter::SetIntakeState(Position pos)
{
    deployPiston.Set((bool)pos);
}

void Shooter::SetIntake(units::volt_t voltage)
{
    cmd_intake = voltage.value();
    intake.SetVoltage(voltage);
}

void Shooter::SetIndexer(units::volt_t voltage)
{
    cmd_indexer = voltage.value();
    indexerA.SetVoltage(voltage);
    //indexerB.SetVoltage(voltage);
}

void Shooter::SetFeeder(units::volt_t voltage)
{
    cmd_feeder = voltage.value();
    feeder.SetVoltage(voltage);
}

void Shooter::SetShooter(units::volt_t voltage)
{
    cmd_shooter = voltage.value();
    shooterA.SetVoltage(voltage);
}

void Shooter::SetShooterRPM(units::revolutions_per_minute_t targetRPM)
{
    cmd_shooter = targetRPM.value();

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
    cmd_shooterTop = voltage.value();

    shooterTop.SetVoltage(voltage);
}

void Shooter::SetShooterTopRPM(units::revolutions_per_minute_t targetRPM)
{
    cmd_shooter = targetRPM.value();
    
    if(targetRPM == 0.0_rpm)
    {
        shooterTop.Set(0.0);
        return;
    }

    shooterTop.Set(ControlMode::Velocity, ((targetRPM.value() / kScaleFactorFly) / 600.0));
}

// void Shooter::SetTurret(units::volt_t voltage)
// {
//     cmd_turret = voltage.value();
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
//     cmd_hood = voltage.value();
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

void Shooter::InitSendable(wpi::SendableBuilder &builder)
{
    builder.SetSmartDashboardType("Shooter");
    builder.SetActuator(true);

    // Commands
    builder.AddDoubleProperty(
        "cmd/intake", [this] { return cmd_intake; }, nullptr);

    // Intake Motor
    builder.AddDoubleProperty(
        "intake/percent", [this] { return intake.Get(); }, nullptr);
    builder.AddDoubleProperty(
        "intake/voltage", [this] { return intake.GetMotorOutputVoltage(); }, nullptr);
    builder.AddDoubleProperty(
        "intake/rpm", [this] { return intake.GetSelectedSensorVelocity() / 2048.0 * 10.0 * 60.0; }, nullptr);
    builder.AddStringProperty(
        "intake/sol", [this] { return deployPiston.Get() ? "Deployed":"Stowed"; }, nullptr);

    // indexerA Motor
    builder.AddDoubleProperty(
        "indexerA/percent", [this] { return indexerA.Get(); }, nullptr);
    builder.AddDoubleProperty(
        "indexerA/voltage", [this] { return indexerA.GetMotorOutputVoltage(); }, nullptr);
    builder.AddDoubleProperty(
        "indexerA/rpm", [this] { return indexerA.GetSelectedSensorVelocity() / 2048.0 * 10.0 * 60.0; }, nullptr);

    // indexerB Motor
    // builder.AddDoubleProperty(
    //     "indexerB/percent", [this] { return indexerB.Get(); }, nullptr);
    // builder.AddDoubleProperty(
    //     "indexerB/voltage", [this] { return indexerB.GetMotorOutputVoltage(); }, nullptr);
    // builder.AddDoubleProperty(
    //     "indexerB/rpm", [this] { return indexerB.GetSelectedSensorVelocity() / 2048.0 * 10.0 * 60.0; }, nullptr);

    // shooterA Motor
    builder.AddDoubleProperty(
        "shooterA/percent", [this] { return shooterA.Get(); }, nullptr);
    builder.AddDoubleProperty(
        "shooterA/voltage", [this] { return shooterA.GetMotorOutputVoltage(); }, nullptr);
    builder.AddDoubleProperty(
        "shooterA/rpm", [this] { return shooterA.GetSelectedSensorVelocity() / 2048.0 * 10.0 * 60.0; }, nullptr);

    // shooterB Motor (Slaved)
    // builder.AddDoubleProperty(
    //     "shooterB/percent", [this] { return shooterB.Get(); }, nullptr);
    // builder.AddDoubleProperty(
    //     "shooterB/voltage", [this] { return shooterB.GetMotorOutputVoltage(); }, nullptr);
    // builder.AddDoubleProperty(
    //     "shooterB/rpm", [this] { return shooterB.GetSelectedSensorVelocity() / 2048.0 * 10.0 * 60.0; }, nullptr);

    // shooterTop Motor
    builder.AddDoubleProperty(
        "shooterTop/percent", [this] { return shooterTop.Get(); }, nullptr);
    builder.AddDoubleProperty(
        "shooterTop/voltage", [this] { return shooterTop.GetMotorOutputVoltage(); }, nullptr);
    builder.AddDoubleProperty(
        "shooterTop/rpm", [this] { return shooterTop.GetSelectedSensorVelocity() / 2048.0 * 10.0 * 60.0; }, nullptr);

    // shooterTop Motor
    builder.AddDoubleProperty(
        "shooterTop/percent", [this] { return shooterTop.Get(); }, nullptr);
    builder.AddDoubleProperty(
        "shooterTop/voltage", [this] { return shooterTop.GetMotorOutputVoltage(); }, nullptr);
    builder.AddDoubleProperty(
        "shooterTop/rpm", [this] { return shooterTop.GetSelectedSensorVelocity() / 2048.0 * 10.0 * 60.0; }, nullptr);

    // turret Motor
    // builder.AddDoubleProperty(
    //     "turret/percent", [this] { return turret.Get(); }, nullptr);
    // builder.AddDoubleProperty(
    //     "turret/voltage", [this] { return turret.GetMotorOutputVoltage(); }, nullptr);
    // builder.AddDoubleProperty(
    //     "turret/rpm", [this] { return turret.GetSelectedSensorVelocity() / 2048.0 * 10.0 * 60.0; }, nullptr);

    // hood Motor
    // builder.AddDoubleProperty(
    //     "hood/percent", [this] { return hood.Get(); }, nullptr);
    // builder.AddDoubleProperty(
    //     "hood/voltage", [this] { return hood.GetMotorOutputVoltage(); }, nullptr);
    // builder.AddDoubleProperty(
    //     "hood/rpm", [this] { return hood.GetSelectedSensorVelocity() / 2048.0 * 10.0 * 60.0; }, nullptr); 
}