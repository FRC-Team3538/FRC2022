#pragma once

#include "AutoFiveBallSafe.hpp"

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