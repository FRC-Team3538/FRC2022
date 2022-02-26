#include "subsystems/Subsystem.hpp"

void Subsystem::SetStatusFrames(WPI_TalonFX &talon, uint8_t framePeriod)
{
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, framePeriod, 50);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, framePeriod, 50);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat, framePeriod, 50);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_6_Misc, framePeriod, 50);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_7_CommStatus, framePeriod, 50);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_9_MotProfBuffer, framePeriod, 50);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_Targets, framePeriod, 50);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_12_Feedback1, framePeriod, 50);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, framePeriod, 50);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_14_Turn_PIDF1, framePeriod, 50);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_15_FirmareApiStatus, framePeriod, 50);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_17_Targets1, framePeriod, 50);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature, framePeriod, 50);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth, framePeriod, 50);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_11_UartGadgeteer, framePeriod, 50);
    talon.SetStatusFramePeriod(StatusFrameEnhanced::Status_Brushless_Current, framePeriod, 50);
}