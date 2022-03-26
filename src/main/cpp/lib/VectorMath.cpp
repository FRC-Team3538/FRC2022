#include <lib/VectorMath.hpp>
#include "frc/geometry/Translation2d.h"  // for Translation2d
#include "units/angle.h"                 // for degree_t, operator""_deg
#include "units/base.h"                  // for operator*, operator+, operator-
#include "units/length.h"                // for inch_t, operator""_in

VectorMath::VectorMath()
{
    x = 0_in;
    y = 0_in;
    theta = 0_deg;
    magnitude = 0_in;
}

VectorMath::VectorMath(units::inch_t x, units::inch_t y) : x(x), y(y)
{
    CalculatePolar();
}

VectorMath::VectorMath(units::inch_t magnitude, units::degree_t theta) : magnitude(magnitude), theta(theta)
{
    CalculateCartesian();
}

VectorMath::VectorMath(frc::Translation2d translation) : x(translation.X()), y(translation.Y())
{ 
    CalculatePolar();
}

void VectorMath::SetX(units::inch_t value)
{
    x = value;
    CalculatePolar();
}

void VectorMath::SetY(units::inch_t value)
{
    y = value;
    CalculatePolar();
}

void VectorMath::SetTheta(units::degree_t value)
{
    theta = value;
    CalculateCartesian();
}

void VectorMath::SetMagnitude(units::inch_t value)
{
    magnitude = value;
    CalculateCartesian();
}

units::inch_t VectorMath::GetX()
{
    return x;
}

units::inch_t VectorMath::GetY()
{
    return y;
}

units::inch_t VectorMath::GetMagnitude()
{
    return magnitude;
}

units::degree_t VectorMath::GetTheta()
{
    return theta;
}

void VectorMath::CalculatePolar()
{
    magnitude = units::inch_t{sqrt(pow(x.value(), 2) + pow(y.value(), 2))};
    theta = units::radian_t{atan2(y.value(), x.value())};
}

void VectorMath::CalculateCartesian()
{
    x = magnitude * cos((theta.value() * (M_PI / 180.0)));
    y = magnitude * sin((theta.value() * (M_PI / 180.0)));
}

VectorMath VectorMath::operator+(const VectorMath &b)
{
    return VectorMath{this->x + b.x, this->y + b.y};
}

VectorMath VectorMath::operator-(const VectorMath &b)
{
    return VectorMath{this->x - b.x, this->y - b.y};
}