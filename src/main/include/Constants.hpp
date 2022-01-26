#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/numbers>
#include <math.h>

namespace Constants
{
    // *** DRIVE CONSTANTS***

    // double kMaxVelocity = 1.0;
    // double kMaxAcceleration = 1.0;
    // double kMaxAngularVelocity = 1.0;
    // double kMaxAngularAcceleration = 1.0;

    // double kTrackWidth = 1.0;
    // double kWheelBase = 1.0;

    // double kWheelRadius = 1.0;

    // *** VISION CONSTANTS ***

    const units::degree_t cameraAngle = 0.0_deg;   // Angle of elevation of camera
    const units::inch_t deltaH = 0.0_in;   // Distance between camera lens and vision target midpoint

    // *** SHOOTER CONSTANTS ***
}