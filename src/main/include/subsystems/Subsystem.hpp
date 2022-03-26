#pragma once

#include <stdint.h>  // for uint8_t
#include <ctre/Phoenix.h>

#include "ctre/phoenix/motorcontrol/IMotorController.h"

namespace ctre {
namespace phoenix {
namespace motorcontrol {
namespace can {
class WPI_TalonFX;
}  // namespace can
}  // namespace motorcontrol
}  // namespace phoenix
}  // namespace ctre

using namespace ctre::phoenix::motorcontrol::can;

/**
 * Subsystem Interface
 * 
 * Has UpdateTelemetry() and ConfigureMotors() functions
 * Just trying this out because it seems convenient
 */
class Subsystem
{
public:
    virtual void UpdateTelemetry() = 0; 
    virtual void ConfigureSystem() = 0;

    void SetStatusFrames(WPI_TalonFX &talon, uint8_t framePeriod);
};