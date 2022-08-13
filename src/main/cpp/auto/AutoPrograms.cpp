#include "auto/AutoPrograms.h"
#include <stddef.h>                    // for NULL
#include "auto/AutoInterface.h"      // for AutoInterface
#include "frc/smartdashboard/SendableChooser.h"   // for SendableChooser
#include "frc/smartdashboard/SendableChooser.inc" // for SendableChooser::A...
#include "frc/smartdashboard/SmartDashboard.h"    // for SmartDashboard
class Robotmap;

// Constructor requires a reference to the robot map
AutoPrograms::AutoPrograms(Robotmap &IO) : IO(IO)
{
    // SmartDash Chooser [List 2 of 3]
    m_chooser.SetDefaultOption("00 - None", "00 - None");
}

// Initialize the selected auto program
void AutoPrograms::Init()
{
    // Get Selected Program from SmartDash Chooser
    std::string name = m_chooser.GetSelected();

    // Delete previously selected auto program
    delete m_autoProgram;
    m_autoProgram = NULL;

    // Create the Selected auto program [List 3 of 3]

    if (m_autoProgram != NULL)
        m_autoProgram->Init();
}

// Run the selected Auto Program
void AutoPrograms::Run()
{
    if (m_autoProgram != NULL)
    {
        m_autoProgram->Run();
    }
}

void AutoPrograms::SmartDash()
{
    frc::SmartDashboard::PutData("Choose Auto", &m_chooser);
    std::string name = m_chooser.GetSelected();
    frc::SmartDashboard::PutString("Selected Auto", name);

    // Update Smartdash
    if (m_autoProgram != NULL)
    {
        m_autoProgram->UpdateSmartDash();
    }
}