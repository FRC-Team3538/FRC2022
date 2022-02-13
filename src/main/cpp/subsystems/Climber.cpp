#include "subsystems/Climber.hpp"

Climber::Climber()
{
}

void Climber::ConfigureSystem()
{
    climberA.ConfigFactoryDefault();
    climberB.ConfigFactoryDefault();

    climberA.SetInverted(false);
    climberB.SetInverted(false);

    climberA.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    climberB.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

    climberB.Follow(climberA);
}

void Climber::UpdateTelemetry()
{
}

void Climber::SetClimber(units::volt_t targetVoltage)
{ 
    climberA.SetVoltage(targetVoltage);
}

void Climber::SetClimberState(ClimbState climbPosition)
{
    switch(climbPosition)
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
       "elevator/percent", [this] { return climberA.Get(); }, nullptr);
   builder.AddDoubleProperty(
       "elevator/voltage", [this] { return climberA.GetMotorOutputVoltage(); }, nullptr);
   builder.AddDoubleProperty(
       "elevator/position", [this] { return climberA.GetSelectedSensorPosition(); }, nullptr);
   builder.AddDoubleProperty(
       "elevator/velocity", [this] { return climberA.GetSelectedSensorVelocity(); }, nullptr);
   builder.AddBooleanProperty(
       "pitchedUp", [this] { return GetClimberState() == Climber::ClimbState::Up; }, nullptr);

    builder.AddStringProperty(
        "sol", [this] { return  (GetClimberState() == Climber::ClimbState::Up) ? "UP":"DOWN"; }, nullptr);
   
   
   
}