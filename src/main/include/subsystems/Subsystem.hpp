#pragma once

#include <frc/smartdashboard/SmartDashboard.h>

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
};