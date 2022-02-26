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
}

void Climber::UpdateTelemetry()
{
    frc::SmartDashboard::PutBoolean("Climber Sensor Enabled", !sensorOverrode);
}

void Climber::SetSensorOverride(bool override)
{
    sensorOverrode = override;
}

void Climber::SetClimber(units::volt_t targetVoltage)
{
    if (!sensorOverrode)
    {
        if (bottomMagSwitch.Get() && targetVoltage > 0.0_V)
        {
            climberA.SetVoltage(targetVoltage);
            climberB.SetVoltage(targetVoltage);
        }
        else if (bottomMagSwitch.Get() && targetVoltage < 0.0_V)
        {
            climberA.SetVoltage(0.0_V);
            climberB.SetVoltage(0.0_V);
        }
        else
        {
            climberA.SetVoltage(targetVoltage);
            climberB.SetVoltage(targetVoltage);
        }
    }
    else
    {
        climberA.SetVoltage(targetVoltage);
        climberB.SetVoltage(targetVoltage);
    }
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