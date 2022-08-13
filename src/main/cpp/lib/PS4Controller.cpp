
#include <lib/PS4Controller.h>
#include "frc/PS4Controller.h"             // for PS4Controller
#include "wpi/sendable/SendableBuilder.h"  // for SendableBuilder

// stuff here eventually

RJ::PS4Controller::PS4Controller(int port) : frc::PS4Controller(port) {}


void RJ::PS4Controller::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("PS4Controller");
    builder.SetActuator(false);

    // Commands
    builder.AddDoubleProperty(
        "axis/LeftX", [this] { return GetLeftX(); }, nullptr);
    builder.AddDoubleProperty(
        "axis/LeftY", [this] { return GetLeftY(); }, nullptr);
    builder.AddDoubleProperty(
        "axis/RightX", [this] { return GetRightX(); }, nullptr);
    builder.AddDoubleProperty(
        "axis/RightY", [this] { return GetRightY(); }, nullptr);
    builder.AddDoubleProperty(
        "axis/LeftTrigger", [this] { return GetL2Axis(); }, nullptr);
    builder.AddDoubleProperty(
        "axis/RightTrigger", [this] { return GetR2Axis(); }, nullptr);
    builder.AddBooleanProperty(
        "button/Options", [this] { return GetOptionsButton(); }, nullptr
    );
    builder.AddBooleanProperty(
        "button/Sharebutton", [this] { return GetShareButton(); }, nullptr
    );
    builder.AddBooleanProperty(
        "button/Touchpad", [this] { return GetTouchpad(); }, nullptr
    );
    builder.AddBooleanProperty(
        "button/PSButton", [this] { return GetPSButton(); }, nullptr
    );
    builder.AddDoubleProperty(
        "button/POV", [this] { return GetPOV(); }, nullptr
    );
    builder.AddBooleanProperty(
        "button/Squarebutton", [this] { return GetSquareButton(); }, nullptr
    );
    builder.AddBooleanProperty(
        "button/Trianglebutton", [this] { return GetTriangleButton(); }, nullptr
    );
    builder.AddBooleanProperty(
        "button/Crossbutton", [this] { return GetCrossButton(); }, nullptr
    );
    builder.AddBooleanProperty(
        "button/Circlebutton", [this] { return GetCircleButton(); }, nullptr
    );
    builder.AddBooleanProperty(
        "button/L1", [this] { return GetL1Button(); }, nullptr
    );
    builder.AddBooleanProperty(
        "button/L2", [this] { return GetL2Button(); }, nullptr
    );
    builder.AddBooleanProperty(
        "button/R1", [this] { return GetR1Button(); }, nullptr
    );
    builder.AddBooleanProperty(
        "button/R2", [this] { return GetR2Button(); }, nullptr
    );
    builder.AddBooleanProperty(
        "button/L3", [this] { return GetL3Button(); }, nullptr
    );
     builder.AddBooleanProperty(
        "button/R3", [this] { return GetR3Button(); }, nullptr
    );
    builder.AddBooleanProperty(
        "button/Isconected", [this] { return IsConnected(); }, nullptr
    );

}