#include "subsystems/Shooter.hpp"

Shooter::Shooter()
{
}

void Shooter::ConfigureSystem()
{
    turret.ConfigFactoryDefault();
    turret.SetSelectedSensorPosition(0.0); // 90 / kScaleFactorTurret);
    turret.SetInverted(true);
    turret.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    turret.ConfigPeakOutputForward(0.2);
    turret.ConfigPeakOutputReverse(-0.2);
    turret.ConfigForwardSoftLimitThreshold(-180.0 / kScaleFactorTurret);
    turret.ConfigReverseSoftLimitThreshold(0.0);
    turret.ConfigForwardSoftLimitEnable(true);
    turret.ConfigReverseSoftLimitEnable(true);

    shooterA.ConfigFactoryDefault();
    shooterB.ConfigFactoryDefault();

    shooterB.Follow(shooterA);

    shooterA.SetInverted(true);

    shooterB.SetInverted(false);

    shooterA.Config_kF(0, 0.056494409);
    shooterA.Config_kP(0, 0.225);
    shooterA.Config_kI(0, 0.0001);
    shooterA.Config_kD(0, 6.000);

    shooterA.Config_IntegralZone(0, 200.0);

    turret.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);

    feeder.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 250, 50);
    feeder.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 250, 50);
    feeder.SetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat, 250, 50);
    feeder.SetStatusFramePeriod(StatusFrameEnhanced::Status_6_Misc, 250, 50);
    feeder.SetStatusFramePeriod(StatusFrameEnhanced::Status_7_CommStatus, 250, 50);
    feeder.SetStatusFramePeriod(StatusFrameEnhanced::Status_9_MotProfBuffer, 250, 50);
    feeder.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_Targets, 250, 50);
    feeder.SetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, 250, 50);
    feeder.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 250, 50);
    feeder.SetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, 250, 50);
    feeder.SetStatusFramePeriod(StatusFrameEnhanced::Status_15_FirmareApiStatus, 250, 50);
    feeder.SetStatusFramePeriod(StatusFrameEnhanced::Status_17_Targets1, 250, 50);
    feeder.SetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature, 250, 50);
    feeder.SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth, 250, 50);
    feeder.SetStatusFramePeriod(StatusFrameEnhanced::Status_11_UartGadgeteer, 250, 50);
    feeder.SetStatusFramePeriod(StatusFrameEnhanced::Status_Brushless_Current, 250, 50);

    turret.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 250, 50);
    // turret.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 250, 50);
    turret.SetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat, 250, 50);
    turret.SetStatusFramePeriod(StatusFrameEnhanced::Status_6_Misc, 250, 50);
    turret.SetStatusFramePeriod(StatusFrameEnhanced::Status_7_CommStatus, 250, 50);
    turret.SetStatusFramePeriod(StatusFrameEnhanced::Status_9_MotProfBuffer, 250, 50);
    turret.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_Targets, 250, 50);
    turret.SetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, 250, 50);
    turret.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 250, 50);
    turret.SetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, 250, 50);
    turret.SetStatusFramePeriod(StatusFrameEnhanced::Status_15_FirmareApiStatus, 250, 50);
    turret.SetStatusFramePeriod(StatusFrameEnhanced::Status_17_Targets1, 250, 50);
    turret.SetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature, 250, 50);
    turret.SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth, 250, 50);
    turret.SetStatusFramePeriod(StatusFrameEnhanced::Status_11_UartGadgeteer, 250, 50);
    turret.SetStatusFramePeriod(StatusFrameEnhanced::Status_Brushless_Current, 250, 50);

    shooterA.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 250, 50);
    // shooterA.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 250, 50);
    shooterA.SetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat, 250, 50);
    shooterA.SetStatusFramePeriod(StatusFrameEnhanced::Status_6_Misc, 250, 50);
    shooterA.SetStatusFramePeriod(StatusFrameEnhanced::Status_7_CommStatus, 250, 50);
    shooterA.SetStatusFramePeriod(StatusFrameEnhanced::Status_9_MotProfBuffer, 250, 50);
    shooterA.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_Targets, 250, 50);
    shooterA.SetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, 250, 50);
    shooterA.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 250, 50);
    shooterA.SetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, 250, 50);
    shooterA.SetStatusFramePeriod(StatusFrameEnhanced::Status_15_FirmareApiStatus, 250, 50);
    shooterA.SetStatusFramePeriod(StatusFrameEnhanced::Status_17_Targets1, 250, 50);
    shooterA.SetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature, 250, 50);
    shooterA.SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth, 250, 50);
    shooterA.SetStatusFramePeriod(StatusFrameEnhanced::Status_11_UartGadgeteer, 250, 50);
    shooterA.SetStatusFramePeriod(StatusFrameEnhanced::Status_Brushless_Current, 250, 50);

    shooterB.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 250, 50);
    shooterB.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 250, 50);
    shooterB.SetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat, 250, 50);
    shooterB.SetStatusFramePeriod(StatusFrameEnhanced::Status_6_Misc, 250, 50);
    shooterB.SetStatusFramePeriod(StatusFrameEnhanced::Status_7_CommStatus, 250, 50);
    shooterB.SetStatusFramePeriod(StatusFrameEnhanced::Status_9_MotProfBuffer, 250, 50);
    shooterB.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_Targets, 250, 50);
    shooterB.SetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, 250, 50);
    shooterB.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 250, 50);
    shooterB.SetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, 250, 50);
    shooterB.SetStatusFramePeriod(StatusFrameEnhanced::Status_15_FirmareApiStatus, 250, 50);
    shooterB.SetStatusFramePeriod(StatusFrameEnhanced::Status_17_Targets1, 250, 50);
    shooterB.SetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature, 250, 50);
    shooterB.SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth, 250, 50);
    shooterB.SetStatusFramePeriod(StatusFrameEnhanced::Status_11_UartGadgeteer, 250, 50);
    shooterB.SetStatusFramePeriod(StatusFrameEnhanced::Status_Brushless_Current, 250, 50);
}

void Shooter::UpdateTelemetry()
{
}

void Shooter::SetTurretAngle(units::degree_t targetAngle)
{
}

void Shooter::SetShooterRPM(units::revolutions_per_minute_t targetRPM)
{
}

void Shooter::SetTurret(units::volt_t targetVolts)
{
    turret.SetVoltage(targetVolts);
}

void Shooter::SetShooter(units::volt_t targetVolts)
{
    shooterA.SetVoltage(targetVolts);
}

void Shooter::SetFeeder(double setValue)
{
    feeder.SetVoltage(units::volt_t{setValue * 13.0});
}

void Shooter::SetIndexer(double setValue)
{
    indexerA.SetVoltage(units::volt_t{setValue * 13.0});
    indexerB.SetVoltage(units::volt_t{setValue * 13.0});
}

void Shooter::SetIntakeState(Position pos)
{
    switch (pos)
    {
    case Position::Stowed:
    {
        deployPiston.Set(false);
        break;
    }

    case Position::Deployed:
    {
        deployPiston.Set(true);
        break;
    }
    default:
    {
        std::cout << "Confusion" << std::endl;
    }
    }
}

void Shooter::SetIntake(double setValue)
{
    intake.SetVoltage(units::volt_t{setValue * 13.0});
}

units::degree_t Shooter::GetTurretAngle()
{
    double ang = turret.GetSelectedSensorPosition() * kScaleFactorTurret;
    return units::degree_t(ang);
}

units::revolutions_per_minute_t Shooter::GetShooterRPM()
{
    return units::revolutions_per_minute_t{shooterA.GetSelectedSensorVelocity() * kScaleFactorFly * 600.0};
}

Shooter::State Shooter::CalculateShot(units::inch_t distance)
{
    State shotStats = {0_rpm, 0_deg};
    return shotStats;
}