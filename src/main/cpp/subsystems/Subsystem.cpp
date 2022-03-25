#include "subsystems/Subsystem.hpp"

using namespace ctre::phoenix::motorcontrol::can;
using namespace ctre::phoenix::motorcontrol;

/*
Trust me when I say this is a good thing - 
use coprimes for status frames to minimize collision
if you would like to generate more, see find_coprime_set_fast.py
[255, 254, 253, 251, 247, 241, 239, 233, 229, 227, 223, 217, 211, 199, 197]
*/
void Subsystem::SetStatusFrames(WPI_TalonFX &talon, uint8_t framePeriod)
{
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 20, 50);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature, 20, 50);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat, 40, 50);

    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 255, 50);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_6_Misc, 254, 50);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_7_CommStatus, 253, 50);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_9_MotProfBuffer, 251, 50);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_Targets, 247, 50);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, 241, 50);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 239, 50);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, 233, 50);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_15_FirmareApiStatus, 229, 50);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_17_Targets1, 227, 50);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth, 223, 50);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_11_UartGadgeteer, 217, 50);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_Brushless_Current, 211, 50);
}