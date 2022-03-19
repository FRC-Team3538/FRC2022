#include "subsystems/Climber.hpp"

Climber::Climber()
{
}

void Climber::ConfigureSystem()
{
    climberA.ConfigFactoryDefault();
    climberB.ConfigFactoryDefault();

    climberA.SetInverted(true);
    climberB.SetInverted(true);

    climberA.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    climberB.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

    SetStatusFrames(climberA, 250);
    SetStatusFrames(climberB, 250);

    climberA.ConfigForwardSoftLimitThreshold(1 / kScaleFactor);
    climberA.ConfigReverseSoftLimitThreshold(1500.0);
    climberA.ConfigForwardSoftLimitEnable(true);
    climberA.ConfigReverseSoftLimitEnable(true);

    climberB.ConfigForwardSoftLimitThreshold(1 / kScaleFactor);
    climberB.ConfigReverseSoftLimitThreshold(1500.0);
    climberB.ConfigForwardSoftLimitEnable(true);
    climberB.ConfigReverseSoftLimitEnable(true);

    climberA.SetSelectedSensorPosition(0.0);
    climberB.SetSelectedSensorPosition(0.0);
}

void Climber::UpdateTelemetry()
{
    frc::SmartDashboard::PutBoolean("Climber Sensor Enabled", !sensorOverrode);
    // frc::SmartDashboard::PutNumber("CLIMBER TICKS", climberA.GetSelectedSensorPosition());
    // frc::SmartDashboard::PutNumber("CLIMBER TICKS AGAIN", climberB.GetSelectedSensorPosition());
}

void Climber::SetSensorOverride(bool override)
{
    if (sensorOverrode != override)
    {
        climberA.ConfigForwardSoftLimitEnable(!override);
        climberA.ConfigReverseSoftLimitEnable(!override);
        climberB.ConfigForwardSoftLimitEnable(!override);
        climberB.ConfigReverseSoftLimitEnable(!override);
    }

    sensorOverrode = override;
}

void Climber::SetClimber(units::volt_t targetVoltage)
{
    // if (!sensorOverrode)
    // {
    //     if (bottomMagSwitch.Get() && targetVoltage > 0.0_V)
    //     {
    //         climberA.SetVoltage(targetVoltage);
    //         climberB.SetVoltage(targetVoltage);
    //     }
    //     else if (bottomMagSwitch.Get() && targetVoltage < 0.0_V)
    //     {
    //         climberA.SetVoltage(0.0_V);
    //         climberB.SetVoltage(0.0_V);
    //     }
    //     else
    //     {
    //         climberA.SetVoltage(targetVoltage);
    //         climberB.SetVoltage(targetVoltage);
    //     }
    // }
    // else
    // {
    climberA.SetVoltage(targetVoltage);
    climberB.SetVoltage(targetVoltage);
    //}
}

bool Climber::GetSensorOverride()
{
    return sensorOverrode;
}

void Climber::SetClimberState(ClimbState climbPosition)
{
    switch (climbPosition)
    {
    case ClimbState::Up:
    {
        tiltPiston.Set(true);
        break;
    }

    case ClimbState::Down:
    {
        tiltPiston.Set(false);
        break;
    }
    }
}

Climber::ClimbState Climber::GetClimberState()
{
    return tiltPiston.Get() ? ClimbState::Up : ClimbState::Down;
}

void Climber::InitSendable(wpi::SendableBuilder &builder)
{
    builder.SetSmartDashboardType("Climber");
    builder.SetActuator(true);

    builder.AddDoubleProperty(
        "elevator/percent", [this]
        { return climberA.Get(); },
        nullptr);
    builder.AddDoubleProperty(
        "elevator/voltage", [this]
        { return climberA.GetMotorOutputVoltage(); },
        nullptr);
    builder.AddDoubleProperty(
        "elevator/position", [this]
        { return climberA.GetSelectedSensorPosition(); },
        nullptr);
    builder.AddDoubleProperty(
        "elevator/velocity", [this]
        { return climberA.GetSelectedSensorVelocity(); },
        nullptr);
    builder.AddBooleanProperty(
        "sol", [this]
        { return GetClimberState() == Climber::ClimbState::Up; },
        nullptr);
    
}