#pragma once

#include <stdint.h>  // for uint8_t
#include <ctre/Phoenix.h>
#include <unordered_map>
#include <wpi/DataLog.h>
#include <units/current.h>
#include <units/voltage.h>

#include "ctre/phoenix/motorcontrol/IMotorController.h"

namespace ctre {
namespace phoenix {
namespace motorcontrol {
namespace can {
class WPI_TalonFX;
}  // namespace can
}  // namespace motorcontrol
}  // namespace phoenix
}  // namespace ctre

using namespace ctre::phoenix::motorcontrol::can;


/**
 * Subsystem Interface
 *
 * Has UpdateTelemetry() and ConfigureMotors() functions
 * Just trying this out because it seems convenient
 */
class Subsystem
{
public:
    virtual void UpdateTelemetry() = 0;
    virtual void ConfigureSystem() = 0;
    virtual void RegisterDataEntries(wpi::log::DataLog &log) = 0;
    virtual void LogDataEntries(wpi::log::DataLog &log) = 0;
    virtual void SimInit() = 0;
    virtual units::ampere_t SimPeriodic(units::volt_t battery) = 0;

    void SetStatusFrames(WPI_TalonFX &talon, uint8_t framePeriod);

    void RegisterDataEntry(wpi::log::DataLog &log, std::string entry_name, std::string_view type, std::string_view metadata = {}, int64_t timestamp = 0);
    int GetDataEntry(std::string key);

    void FalconEntryStartHelper(wpi::log::DataLog &log, std::string name, bool primary = false);
    void FalconEntryHelper(wpi::log::DataLog &log, WPI_TalonFX &motor, std::string name, uint64_t timestamp = 0, bool primary = false);
private:
    std::unordered_map<std::string, int> data_entries{};

    // TODO this belongs elsewhere but ok
    static constexpr double kTicks2RPM = (1.0 / (2048.0)) * 10.0 * 60.0;
};