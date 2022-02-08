
#include <lib/PS4Controller.hpp>

// stuff here eventually

RJ::PS4Controller::PS4Controller(int port) : frc::PS4Controller(port) {}


void RJ::PS4Controller::InitSendable(wpi::SendableBuilder &builder)
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
}