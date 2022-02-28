#pragma once

#include <units/angle.h>
#include <units/length.h>
#include <cmath>
#include <frc/geometry/Pose2d.h>

class VectorMath
{
public:
    VectorMath();

    VectorMath(units::inch_t x, units::inch_t y);

    VectorMath(units::inch_t magnitude, units::degree_t theta);

    VectorMath(frc::Translation2d translation);

    void SetX(units::inch_t value);
    void SetY(units::inch_t value);

    void SetTheta(units::degree_t value);
    void SetMagnitude(units::inch_t value);

    units::inch_t GetX();
    units::inch_t GetY();

    units::inch_t GetMagnitude();
    units::degree_t GetTheta();

    VectorMath operator+(const VectorMath &b);
    VectorMath operator-(const VectorMath &b);

    // MAYBE ADD DOT AND CROSS PRODUCT LATER?

private:
    units::inch_t x;
    units::inch_t y;

    units::inch_t magnitude;
    units::degree_t theta;

    void CalculatePolar();
    void CalculateCartesian();
};