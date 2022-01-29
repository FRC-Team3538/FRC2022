#include "auto/AutoPrograms.hpp"

// Include all auto programs [List 1 of 3]
#include "auto/AutoLine.hpp"
#include "auto/AutoLine_Backward.hpp"
#include "auto/AutoTurn.hpp"

// Constructor requires a reference to the robot map
AutoPrograms::AutoPrograms(Robotmap &IO) : IO(IO)
{
    // SmartDash Chooser [List 2 of 3]
    m_chooser.SetDefaultOption("0 - None", "0 - None");
    m_chooser.AddOption(AutoLine::GetName(), AutoLine::GetName());
    m_chooser.AddOption(AutoLine_Backward::GetName(), AutoLine_Backward::GetName());
    m_chooser.AddOption(AutoTurn::GetName(), AutoTurn::GetName());
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
    if (name == AutoLine::GetName())
    {
        m_autoProgram = new AutoLine(IO);
    } 
    else if (name == AutoLine_Backward::GetName()) 
    {
        m_autoProgram = new AutoLine_Backward(IO);
    } 
    else if (name == AutoTurn::GetName()) 
    {
        m_autoProgram = new AutoTurn(IO);
    }
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
}