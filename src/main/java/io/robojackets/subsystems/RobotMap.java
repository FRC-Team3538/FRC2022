package io.robojackets.subsystems;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.y2023.wpilibj.simulation.BatterySim;
import edu.wpi.first.y2023.wpilibj.simulation.PDPSim;
import edu.wpi.first.y2023.wpilibj.simulation.REVPHSim;
import java.util.List;

public class RobotMap extends Subsystem {
  public Drivetrain drivetrain = new Drivetrain();

  public PS4Controller mainController = new PS4Controller(0);
  public PS4Controller secondaryController = new PS4Controller(1);

  private PowerDistribution pdp = new PowerDistribution();
  private PneumaticHub ph = new PneumaticHub();

  // Simulation
  private REVPHSim phSim = new REVPHSim(ph);
  private PDPSim pdpSim = new PDPSim(pdp);

  private double battery_voltage;
  private double[] currentDraw = new double[20];

  private List<Subsystem> subsystems = List.of(drivetrain);

  public void ConfigureSystem() {
    subsystems.forEach(Subsystem::ConfigureSystem);

    pdp.setSwitchableChannel(true);
  }

  public void RegisterDataEntries(DataLog log) {
    subsystems.forEach(sys -> sys.RegisterDataEntries(log));

    RegisterDataEntry(log, "PDH/Voltage", "double");
    RegisterDataEntry(log, "PDH/Temperature", "double");
    RegisterDataEntry(log, "PDH/Current[channel]", "double[]");
    RegisterDataEntry(log, "PDH/TotalCurrent", "double");
    RegisterDataEntry(log, "PDH/TotalPower", "double");
    RegisterDataEntry(log, "PDH/TotalEnergy", "double");
    RegisterDataEntry(log, "PDH/Module", "int64");
    RegisterDataEntry(log, "PDH/Type", "int64");
    RegisterDataEntry(log, "PDH/SwitchableChannel", "boolean");
    RegisterDataEntry(log, "PDH/Version", "integer[]");
    RegisterDataEntry(log, "PDH/Faults", "boolean[]");
    RegisterDataEntry(log, "PDH/StickyFaults", "boolean[]");

    RegisterDataEntry(log, "PH/Compressor", "boolean");
    RegisterDataEntry(log, "PH/CompressorConfigType", "int64");
    RegisterDataEntry(log, "PH/PressureSwitch", "boolean");
    RegisterDataEntry(log, "PH/CompressorCurrent", "double");
    RegisterDataEntry(log, "PH/Solenoids", "boolean[]");
    RegisterDataEntry(log, "PH/ModuleNumber", "int64");
    RegisterDataEntry(log, "PH/SolenoidDisabledList", "boolean[]");
    RegisterDataEntry(log, "PH/Version", "integer[]");
    RegisterDataEntry(log, "PH/Faults", "boolean[]");
    RegisterDataEntry(log, "PH/StickyFaults", "boolean[]");
    RegisterDataEntry(log, "PH/InputVoltage", "double");
    RegisterDataEntry(log, "PH/RegulatedVoltage_5V", "double");
    RegisterDataEntry(log, "PH/SolenoidsTotalCurrent", "double");
    RegisterDataEntry(log, "PH/SolenoidsVoltage", "double");
    RegisterDataEntry(log, "PH/AnalogVoltage", "double[]");
    RegisterDataEntry(log, "PH/Pressure", "double[]");
  }

  public void LogDataEntries(DataLog log) {
    subsystems.forEach(sys -> sys.LogDataEntries(log));
  }

  public void SimInit() {
    subsystems.forEach(Subsystem::SimInit);
  }

  public void SimPeriodic(double _battery, double[] _currentDraw) {
    subsystems.forEach(sys -> sys.SimPeriodic(battery_voltage, currentDraw));

    double totalCurrent = 0;
    for (int i = 0; i < currentDraw.length; i++) {
      totalCurrent += currentDraw[i];
      pdpSim.setCurrent(i, currentDraw[i]);
    }

    battery_voltage = BatterySim.calculateDefaultBatteryLoadedVoltage(totalCurrent);
    pdpSim.setVoltage(battery_voltage);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // TODO Auto-generated method stub
  }
}
