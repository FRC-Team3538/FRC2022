#pragma once

#include <lib/DiffyDriveTrajectoryConstraint.hpp>

// Source: motors.vex.com
constexpr rj::MotorModel falcon{
    6380_rpm,
    1.5_A,
    783_W,
    4.69_Nm,
    257_A,
    12_V
};

// SET THESE BEFORE USE
constexpr rj::DiffyDriveModel AR_drivebase{
    41.95_lb, // Mass TBD
    3.75_in,  // Wheel Diameter
    8.48,    // Gear Ratio
    // This will be different for every run - TODO: set this in AutonomousInit()
    12.8_V,
    // This will be different for every battery, but there's no good way to get this on-bot.
    // Maybe beak it and shuffleboard?
    0.03_Ohm,
    4,
    // Per Motor
    55_A,
    0.9,
    .579_m, //.579
    falcon};