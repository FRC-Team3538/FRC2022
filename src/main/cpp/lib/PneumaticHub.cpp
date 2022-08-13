#include <lib/PneumaticHub.h>
#include "frc/PneumaticHub.h"              // for PneumaticHub
#include "units/current.h"                 // for ampere_t
#include "units/pressure.h"                // for pounds_per_square_inch_t
#include "wpi/sendable/SendableBuilder.h"  // for SendableBuilder

// stuff here eventually

RJ::PneumaticHub::PneumaticHub(int port) : frc::PneumaticHub (port) {}


void RJ::PneumaticHub::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("PneumaticHub");
    builder.SetActuator(false);

    // Commands
    builder.AddDoubleProperty(
        "Solenoids", [this] { return GetSolenoids(); }, nullptr);

    builder.AddDoubleProperty(
        "PressureHigh", [this] { return GetPressure(0).value(); }, nullptr);

    builder.AddDoubleProperty(
        "PressureLow", [this] { return GetPressure(1).value(); }, nullptr);

    builder.AddDoubleProperty(
        "PressureSwitch", [this] { return GetPressureSwitch(); }, nullptr);

    builder.AddDoubleProperty(
        "Compressor", [this] { return GetCompressor(); }, nullptr);

    builder.AddDoubleProperty(
        "CompressorCurrent", [this] { return GetCompressorCurrent().value(); }, nullptr);

    builder.AddDoubleProperty(
        "Faults", [this] {
            u_faults u{GetFaults()};
            return u.uint32_faults;
            }, nullptr);

    builder.AddDoubleProperty(
        "StickyFaults", [this] {
            u_stickyfaults u{GetStickyFaults()};
            return u.uint32_stickyfaults;
            }, nullptr);
}
