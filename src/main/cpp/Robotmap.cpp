#include "Robotmap.h"
#include <iostream>                         // for operator<<, endl, basic_o...
#include "frc/PowerDistribution.h"          // for PowerDistribution
#include "subsystems/Subsystem.h"         // for Subsystem

#include <wpi/DataLog.h>

// Constructor
// *** ALSO PUT SUBSYSTEMS HERE ***
Robotmap::Robotmap()
{

}

/**
 * Ran periodically in Robot.cpp
 * Cycles through the systems (one system per loop)
 * and runs UpdateTelemetry()
 *
 */
void Robotmap::UpdateSmartDash()
{
    if (telemetryCt < subsystems.size())
    {
        subsystems[telemetryCt]->UpdateTelemetry();
    }
    else if (telemetryCt == subsystems.size())
    {
        // Restart the loop
        telemetryCt = 0;
    }

    ++telemetryCt;
}

/**
 * Loops through subsystems and
 * runs ConfigureSystem()
 *
 */
void Robotmap::ConfigureSystem()
{
    pdp.SetSwitchableChannel(true);
    for (auto system : subsystems)
        system->ConfigureSystem();
}

// TODO - any commented out fields are arrays I haven't bothered to setup
void Robotmap::RegisterDataEntries(wpi::log::DataLog &log)
{
    data_entries["PDH/Voltage"] = log.Start("PDH/Voltage", "double");
    data_entries["PDH/Temperature"] = log.Start("PDH/Temperature", "double");
    // data_entries["PDH/Current[channel]"] = log.Start("PDH/Current[channel]", "double[]");
    data_entries["PDH/TotalCurrent"] = log.Start("PDH/TotalCurrent", "double");
    data_entries["PDH/TotalPower"] = log.Start("PDH/TotalPower", "double");
    data_entries["PDH/TotalEnergy"] = log.Start("PDH/TotalEnergy", "double");
    data_entries["PDH/Module"] = log.Start("PDH/Module", "int64");
    data_entries["PDH/Type"] = log.Start("PDH/Type", "int64");
    data_entries["PDH/SwitchableChannel"] = log.Start("PDH/SwitchableChannel", "boolean");
    // data_entries["PDH/Version"] = log.Start("PDH/Version", "integer[]");
    // data_entries["PDH/Faults"] = log.Start("PDH/Faults", "boolean[]");
    // data_entries["PDH/StickyFaults"] = log.Start("PDH/StickyFaults", "boolean[]");

    data_entries["PH/Compressor"] = log.Start("PH/Compressor", "boolean");
    data_entries["PH/CompressorConfigType"] = log.Start("PH/CompressorConfigType", "int64");
    data_entries["PH/PressureSwitch"] = log.Start("PH/PressureSwitch", "boolean");
    data_entries["PH/CompressorCurrent"] = log.Start("PH/CompressorCurrent", "double");
    // data_entries["PH/Solenoids"] = log.Start("PH/Solenoids", "boolean[]");
    data_entries["PH/ModuleNumber"] = log.Start("PH/ModuleNumber", "int64");
    // data_entries["PH/SolenoidDisabledList"] = log.Start("PH/SolenoidDisabledList", "boolean[]");
    // data_entries["PH/Version"] = log.Start("PH/Version", "integer[]");
    // data_entries["PH/Faults"] = log.Start("PH/Faults", "boolean[]");
    // data_entries["PH/StickyFaults"] = log.Start("PH/StickyFaults", "boolean[]");
    data_entries["PH/InputVoltage"] = log.Start("PH/InputVoltage", "double");
    data_entries["PH/RegulatedVoltage_5V"] = log.Start("PH/RegulatedVoltage_5V", "double");
    data_entries["PH/SolenoidsTotalCurrent"] = log.Start("PH/SolenoidsTotalCurrent", "double");
    data_entries["PH/SolenoidsVoltage"] = log.Start("PH/SolenoidsVoltage", "double");
    // data_entries["PH/AnalogVoltage"] = log.Start("PH/AnalogVoltage", "double[]");
    // data_entries["PH/Pressure"] = log.Start("PH/Pressure", "double[]");


    for (auto system : subsystems)
        system->RegisterDataEntries(log);
}

void Robotmap::LogDataEntries(wpi::log::DataLog &log)
{
    // log.AppendDouble(data_entries["PDH/Voltage"], pdp.GetVoltage(), 0);
    // log.AppendDouble(data_entries["PDH/Temperature"], pdp.GetTemperature(), 0);
    // // log.AppendDoubleArray(data_entries["PDH/Current"], 0.0, 0);
    // log.AppendDouble(data_entries["PDH/TotalCurrent"], pdp.GetTotalCurrent(), 0);
    // log.AppendDouble(data_entries["PDH/TotalPower"], pdp.GetTotalPower(), 0);
    // log.AppendDouble(data_entries["PDH/TotalEnergy"], pdp.GetTotalEnergy(), 0);
    // log.AppendInteger(data_entries["PDH/Module"], pdp.GetModule(), 0);
    // log.AppendInteger(data_entries["PDH/Type"], (int64_t) pdp.GetType(), 0);
    // log.AppendBoolean(data_entries["PDH/SwitchableChannel"], pdp.GetSwitchableChannel(), 0);
    // // log.AppendIntegerArray(data_entries["PDH/Version"], 0.0, 0);
    // // log.AppendBooleanArray(data_entries["PDH/Faults"], 0.0, 0);
    // // log.AppendBooleanArray(data_entries["PDH/StickyFaults"], 0.0, 0);

    // log.AppendBoolean(data_entries["PH/Compressor"], ph.GetCompressor(), 0);
    // log.AppendInteger(data_entries["PH/CompressorConfigType"], (int64_t) ph.GetCompressorConfigType(), 0);
    // log.AppendBoolean(data_entries["PH/PressureSwitch"], ph.GetPressureSwitch(), 0);
    // log.AppendDouble(data_entries["PH/CompressorCurrent"], ph.GetCompressorCurrent().value(), 0);
    // // log.AppendBooleanArray(data_entries["PH/Solenoids"], 0.0, 0);
    // log.AppendInteger(data_entries["PH/ModuleNumber"], ph.GetModuleNumber(), 0);
    // // log.AppendBooleanArray(data_entries["PH/SolenoidDisabledList"], 0.0, 0);
    // // log.AppendIntegerArray(data_entries["PH/Version"], 0.0, 0);
    // // log.AppendBooleanArray(data_entries["PH/Faults"], 0.0, 0);
    // // log.AppendBooleanArray(data_entries["PH/StickyFaults"], 0.0, 0);
    // log.AppendDouble(data_entries["PH/InputVoltage"], ph.GetInputVoltage().value(), 0);
    // log.AppendDouble(data_entries["PH/RegulatedVoltage_5V"], ph.Get5VRegulatedVoltage().value(), 0);
    // log.AppendDouble(data_entries["PH/SolenoidsTotalCurrent"], ph.GetSolenoidsTotalCurrent().value(), 0);
    // log.AppendDouble(data_entries["PH/SolenoidsVoltage"], ph.GetSolenoidsVoltage().value(), 0);
    // // log.AppendDoubleArray(data_entries["PH/AnalogVoltage"], 0.0, 0);
    // // log.AppendDoubleArray(data_entries["PH/Pressure"], 0.0, 0);

    for (auto system : subsystems)
        system->LogDataEntries(log);
}

void Robotmap::watchDog()
{
    std::cout << "SAD WATCHDOG" << std::endl;
}