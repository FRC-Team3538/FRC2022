#pragma once

#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>

#include <unordered_map>

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

    void SetStatusFrames(WPI_TalonFX &talon, uint8_t framePeriod);

    void RegisterDataEntry(wpi::log::DataLog &log, std::string_view entry_name, std::string_view type, std::string_view metadata = {}, int64_t timestamp = 0);
    int GetDataEntry(std::string_view key);

    void FalconEntryStartHelper(wpi::log::DataLog &log, std::string name);
    void FalconEntryHelper(wpi::log::DataLog &log, WPI_TalonFX &motor, std::string name, uint64_t timestamp = 0);
private:
    std::unordered_map<std::string_view, int> data_entries;

    // TODO this belongs elsewhere but ok
    static constexpr double kTicks2RPM = (1.0 / (2048.0)) * 10.0 * 60.0;
};