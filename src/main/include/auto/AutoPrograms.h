#pragma once

#include "networktables/NetworkTableEntry.inc"
#include "wpi/SmallVector.h"
#include "wpi/StringMap.h"
#include <frc/smartdashboard/SendableChooser.h>
#include <string>

class AutoInterface;
class Robotmap;

class AutoPrograms
{

private:
    // Get a referance to the robotmap
    Robotmap &IO;

    // Selected Auto Program
    AutoInterface *m_autoProgram;

    // SmartDash Chooser

public:
    // Constructor requires a reference to the RobotMap
    AutoPrograms() = delete;
    AutoPrograms(Robotmap &);

    // Choose a program to Initialize
    void Init();

    // Run the selected program
    void Run();
    void SmartDash();
    frc::SendableChooser<std::string> m_chooser;

};