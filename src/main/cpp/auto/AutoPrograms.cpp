#include "auto/AutoPrograms.hpp"

// Include all auto programs [List 1 of 3]
#include "auto/AutoLine.hpp"
#include "auto/AutoTurn.hpp"
#include "auto/AutoBackForward.hpp"
#include "auto/AutoTwoBall.hpp"
#include "auto/AutoFourBall.hpp"
#include "auto/AutoFiveBallRed.hpp"
#include "auto/AutoFiveBallBlue.hpp"
#include "auto/AutoFiveBallSafe.hpp"
#include "auto/AutoFiveBallSneaky.hpp"


// Constructor requires a reference to the robot map
AutoPrograms::AutoPrograms(Robotmap &IO) : IO(IO)
{
    // SmartDash Chooser [List 2 of 3]
    m_chooser.SetDefaultOption("00 - None", "00 - None");
    m_chooser.AddOption(AutoLine::GetName(), AutoLine::GetName());
    m_chooser.AddOption(AutoTwoBall::GetName(), AutoTwoBall::GetName());
    m_chooser.AddOption(AutoFourBall::GetName(), AutoFourBall::GetName());
    m_chooser.AddOption(AutoFiveBallRed::GetName(), AutoFiveBallRed::GetName());
    m_chooser.AddOption(AutoFiveBallBlue::GetName(), AutoFiveBallBlue::GetName());
    m_chooser.AddOption(AutoFiveBallSafe::GetName(), AutoFiveBallSafe::GetName());
    m_chooser.AddOption(AutoFiveBallSneaky::GetName(), AutoFiveBallSneaky::GetName());

    // Test programs
    m_chooser.AddOption(AutoBackForward::GetName(), AutoBackForward::GetName());
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

     
    //Create the Selected auto program [List 3 of 3]
    if (name == AutoLine::GetName())
    {
        m_autoProgram = new AutoLine(IO);
    }
    else if (name == AutoTwoBall::GetName())
    {
        m_autoProgram = new AutoTwoBall(IO);
    }
    else if (name == AutoFourBall::GetName())
    {
        m_autoProgram = new AutoFourBall(IO);
    }
    else if (name == AutoFiveBallRed::GetName())
    {
        m_autoProgram = new AutoFiveBallRed(IO);
    }
     else if (name == AutoFiveBallBlue::GetName())
    {
        m_autoProgram = new AutoFiveBallBlue(IO);
    }
    else if (name == AutoFiveBallSafe::GetName())
    {
        m_autoProgram = new AutoFiveBallSafe(IO);
    }
    else if (name == AutoFiveBallSneaky::GetName())
    {
        m_autoProgram = new AutoFiveBallSneaky(IO);
    }
    else if (name == AutoBackForward::GetName())
    {
        m_autoProgram = new AutoBackForward(IO);
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

    // Update Smartdash
    if (m_autoProgram != NULL)
    {
        m_autoProgram->UpdateSmartDash();
    }
}