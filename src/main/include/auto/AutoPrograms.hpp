#pragma once

#include <frc/smartdashboard/SendableChooser.h>  // for SendableChooser
#include <string>                                // for string
#include "networktables/NetworkTableEntry.inc"   // for NetworkTableEntry::S...
#include "wpi/SmallVector.h"                     // for SmallVector
#include "wpi/StringMap.h"                       // for StringMap

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