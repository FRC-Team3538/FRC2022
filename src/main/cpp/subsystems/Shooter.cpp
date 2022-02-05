#include "subsystems/Shooter.hpp"

Shooter::Shooter()
{
}

void Shooter::ConfigureSystem()
{
    intake.ConfigFactoryDefault();
    feeder.ConfigFactoryDefault();
    hood.ConfigFactoryDefault();

    hood.SetInverted(true);
    intake.SetInverted(true);

    shooterA.ConfigFactoryDefault();
    shooterB.ConfigFactoryDefault();

    shooterB.Follow(shooterA);

    shooterA.SetInverted(false);

    shooterB.SetInverted(true);

    shooterA.Config_kF(0, 0.056494409);
    shooterA.Config_kP(0, 0.225);
    shooterA.Config_kI(0, 0.0001);
    shooterA.Config_kD(0, 6.000);

    shooterA.Config_IntegralZone(0, 200.0);

    hood.SetInverted(false);
    hood.Config_kF(0, 0.056494409);
    hood.Config_kP(0, 0.225);
    hood.Config_kI(0, 0.0001);
    hood.Config_kD(0, 6.000);

    hood.Config_IntegralZone(0, 200.0);
}

void Shooter::UpdateTelemetry()
{
    // ticks/100ms to rpm
    units::revolutions_per_minute_t shooterRPM = units::revolutions_per_minute_t(shooterA.GetSelectedSensorVelocity() * 10 * 60 / 2048);
    // 3.9" dia colsons
    units::feet_per_second_t shooterSurfaceSpeed = shooterRPM * units::inch_t(3.9) * wpi::numbers::pi / units::turn_t(1);

    // ticks/100ms to rpm
    units::revolutions_per_minute_t hoodRPM = units::revolutions_per_minute_t(hood.GetSelectedSensorVelocity() * 10 * 60 / 2048);
    // 2.00" dia compliant
    units::feet_per_second_t hoodSurfaceSpeed = hoodRPM * wpi::numbers::pi * units::inch_t(2.00) / units::turn_t(1);

    // shooter RPM SHOULD be positive / 0
    // hood RPM SHOULD be negative / 0
    units::feet_per_second_t shotEffort = (shooterSurfaceSpeed + hoodSurfaceSpeed) / 2;
    units::feet_per_second_t backspinEffort = (shooterSurfaceSpeed - hoodSurfaceSpeed) / 2;

    // 9.5" nominal ball
    units::revolutions_per_minute_t impartedBackspin = backspinEffort / wpi::numbers::pi / units::inch_t(9.5) * units::turn_t(1);

    // frc::SmartDashboard::PutNumber("shooter/Shooter_Voltage", shooterA.GetMotorOutputVoltage());
    // frc::SmartDashboard::PutNumber("shooter/Shooter_RPM", shooterRPM.value());
    // frc::SmartDashboard::PutNumber("shooter/Shooter_Surface_Speed_FPS", shooterSurfaceSpeed.value());

    // frc::SmartDashboard::PutNumber("shooter/Feeder_Voltage", feeder.GetMotorOutputVoltage());
    // frc::SmartDashboard::PutNumber("shooter/Feeder_RPM", feeder.GetSelectedSensorVelocity() * 10 * 60 / 2048);

    // frc::SmartDashboard::PutNumber("shooter/Hood_Voltage", 0); // hood.GetMotorOutputVoltage());
    // frc::SmartDashboard::PutNumber("shooter/Hood_RPM", hoodRPM.value());
    // frc::SmartDashboard::PutNumber("shooter/Hood_Surface_Speed_FPS", hoodSurfaceSpeed.value());

    // frc::SmartDashboard::PutNumber("shooter/Shot_Effort_FPS", shotEffort.value());
    // frc::SmartDashboard::PutNumber("shooter/Backspin_Effort_FPS", backspinEffort.value());
    // frc::SmartDashboard::PutNumber("shooter/Imparted_Backspin_RPM", impartedBackspin.value());

    shooterVoltageEntry.SetDouble(shooterA.GetMotorOutputVoltage());
    shooterRPMEntry.SetDouble(shooterRPM.value());
    shooterSurfaceSpeedEntry.SetDouble(shooterSurfaceSpeed.value());

    feederVoltageEntry.SetDouble(feeder.GetMotorOutputVoltage());
    feederRPMEntry.SetDouble(feeder.GetSelectedSensorVelocity() * 10 * 60 / 2048); // ticks/100ms to rpm

    hoodVoltageEntry.SetDouble(hood.GetMotorOutputVoltage());
    hoodRPMEntry.SetDouble(hoodRPM.value());
    hoodSurfaceSpeedEntry.SetDouble(hoodSurfaceSpeed.value());

    shotEffortEntry.SetDouble(shotEffort.value());
    backspinEffortEntry.SetDouble(backspinEffort.value());
    impartedBackspinEntry.SetDouble(impartedBackspin.value());
}

void Shooter::SetTurretAngle(units::degree_t targetAngle)
{
    units::degree_t currentAng = GetTurretAngle();

    if (targetAngle != turretPID.GetGoal().position)
    {
        turretPID.SetGoal(targetAngle);
    }

    SetTurret(units::volt_t{turretPID.Calculate(currentAng)});
}

void Shooter::SetShooterRPM(units::revolutions_per_minute_t targetRPM)
{
    if (targetRPM == 0.0_rpm)
    {
        shooterA.Set(0.0);
        return;
    }

    shooterA.Set(ControlMode::Velocity, ((targetRPM.value() / kScaleFactorFly) / 600.0));
}

void Shooter::SetShooterState(State shotStats)
{
    SetShooterRPM(shotStats.shooterVelocity);
    SetHoodRPM(shotStats.hoodVelocity);
}

void Shooter::SetTurret(units::volt_t targetVolts)
{
    // turret.SetVoltage(targetVolts);
}

void Shooter::SetShooter(units::volt_t targetVolts)
{
    shooterA.SetVoltage(targetVolts);
}

void Shooter::SetFeeder(units::volt_t targetVoltage)
{
    indexerA.SetVoltage(targetVoltage);
    feeder.SetVoltage(-targetVoltage);
}

void Shooter::SetHood(units::volt_t targetVoltage)
{
    hood.SetVoltage(targetVoltage);
}

void Shooter::SetHoodRPM(units::revolutions_per_minute_t targetRPM)
{
    hood.Set(ControlMode::Velocity, ((targetRPM.value() / kScaleFactorFly) / 600.0));
}

bool Shooter::TempUpToSpeed()
{
    if ((std::abs(hood.GetSelectedSensorVelocity() - hood.GetClosedLoopTarget()) < 100) && (std::abs(shooterA.GetSelectedSensorVelocity() - shooterA.GetClosedLoopTarget()) < 100))
    {
        return true;
    }
    else
    {
        return false;
    }
}

void Shooter::SetIndexer(double setValue)
{
    // indexerA.SetVoltage(units::volt_t{setValue * 13.0});
    // indexerB.SetVoltage(units::volt_t{setValue * 13.0});
}

void Shooter::SetIntakeState(Position pos)
{
    switch (pos)
    {
    case Position::Stowed:
    {
        // deployPiston.Set(false);
        break;
    }

    case Position::Deployed:
    {
        // deployPiston.Set(true);
        break;
    }
    default:
    {
        std::cout << "Confusion" << std::endl;
    }
    }
}

void Shooter::SetIntake(units::volt_t voltage)
{
    intake.SetVoltage(voltage);
}

units::degree_t Shooter::GetTurretAngle()
{
    // double ang = turret.GetSelectedSensorPosition() * kScaleFactorTurret;
    // return units::degree_t(ang);
    return 0_deg;
}

units::revolutions_per_minute_t Shooter::GetShooterRPM()
{
    return units::revolutions_per_minute_t{shooterA.GetSelectedSensorVelocity() * kScaleFactorFly * 600.0};
}

Shooter::State Shooter::CalculateShot(units::inch_t distance)
{
    double ratio = 7.5 / 7;

    double mainWheel = 8073 + (-0.00325 * std::pow(distance.value(), 3)) + (1.2191 * std::pow(distance.value(), 2)) + (-139.806 * std::pow(distance.value(), 1));

    State shotStats = {units::revolutions_per_minute_t{mainWheel}, units::revolutions_per_minute_t{mainWheel * ratio}, 0_deg};
    return shotStats;
}