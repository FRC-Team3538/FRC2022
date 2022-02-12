#include "subsystems/Climber.hpp"

Climber::Climber()
{
}

void Climber::ConfigureSystem()
{
    climberB.Follow(climberA);
}

void Climber::UpdateTelemetry()
{
}

void Climber::SetClimber(double setValue)
{
    climberA.SetVoltage(units::volt_t{setValue * 13.0});
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