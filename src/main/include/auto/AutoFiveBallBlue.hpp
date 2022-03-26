#pragma once

#include <string>                // for string
#include "AutoFiveBallSafe.hpp"  // for AutoFiveBallSafe
class Robotmap;

class AutoFiveBallBlue : public AutoFiveBallSafe
{
public:
    // Name of this program, used by SmartDash
    static std::string GetName();

    // Constructor requires a reference to the RobotMap
    AutoFiveBallBlue() = delete;
    AutoFiveBallBlue(Robotmap &);
    ~AutoFiveBallBlue();

    // Auto Program Logic
    void Init() override;
};