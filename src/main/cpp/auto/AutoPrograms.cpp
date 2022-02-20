#include "auto/AutoPrograms.hpp"

// Include all auto programs [List 1 of 3]
#include "auto/AutoLine.hpp"
#include "auto/AutoLine_Backward.hpp"
#include "auto/AutoTurn.hpp"
#include "auto/AutoBackForward.hpp"
#include "auto/Auto4ft.hpp"
#include "auto/AutoTwoBall.hpp"
#include "auto/NinetyDegreeAngle.hpp"
#include "auto/AutoFourBall.hpp"
#include "auto/AutoFiveBall.hpp"


// Constructor requires a reference to the robot map
AutoPrograms::AutoPrograms(Robotmap &IO) : IO(IO)
{
    // SmartDash Chooser [List 2 of 3]
    m_chooser.SetDefaultOption("00 - None", "00 - None");
    m_chooser.AddOption(AutoLine::GetName(), AutoLine::GetName());
    m_chooser.AddOption(AutoLine_Backward::GetName(), AutoLine_Backward::GetName());
    m_chooser.AddOption(AutoTurn::GetName(), AutoTurn::GetName());
    m_chooser.AddOption(AutoBackForward::GetName(), AutoBackForward::GetName());
    m_chooser.AddOption(Auto4ft::GetName(), Auto4ft::GetName());
    m_chooser.AddOption(AutoTwoBall::GetName(), AutoTwoBall::GetName());
    m_chooser.AddOption(NinetyDegreeAngle::GetName(), NinetyDegreeAngle::GetName());
    m_chooser.AddOption(AutoFourBall::GetName(), AutoFourBall::GetName());
    m_chooser.AddOption(AutoFiveBall::GetName(), AutoFiveBall::GetName());
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
    else if (name == AutoLine_Backward::GetName()) 
    {
        m_autoProgram = new AutoLine_Backward(IO);
    } 
    else if (name == AutoTurn::GetName()) 
    {
        m_autoProgram = new AutoTurn(IO);
    }
    else if (name == AutoBackForward::GetName())
    {
        m_autoProgram = new AutoBackForward(IO);
    }
    else if (name == Auto4ft::GetName())
    {
        m_autoProgram = new Auto4ft(IO);
    }
    else if (name == AutoTwoBall::GetName())
    {
        m_autoProgram = new AutoTwoBall(IO);
    }
    else if (name == AutoFourBall::GetName())
    {
        m_autoProgram = new AutoFourBall(IO);
    }
    else if (name == AutoFiveBall::GetName())
    {
        m_autoProgram = new AutoFiveBall(IO);
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