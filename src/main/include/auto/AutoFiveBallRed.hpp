#pragma once

#include <string>                // for string
#include "AutoFiveBallSafe.hpp"  // for AutoFiveBallSafe
class Robotmap;

class AutoFiveBallRed : public AutoFiveBallSafe
{
public:
    // Name of this program, used by SmartDash
    static std::string GetName();

    // Constructor requires a reference to the RobotMap
    AutoFiveBallRed() = delete;
    AutoFiveBallRed(Robotmap &);
    ~AutoFiveBallRed();

    // Auto Program Logic
    void Init() override;
};