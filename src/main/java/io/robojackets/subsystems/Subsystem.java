package io.robojackets.subsystems;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.sendable.Sendable;
import io.robojackets.lib.FalconUtils;
import java.util.HashMap;

public abstract class Subsystem implements Sendable {
  HashMap<String, Integer> dataEntries = new HashMap<>();

  public abstract void ConfigureSystem();

  public abstract void RegisterDataEntries(DataLog log);

  public abstract void LogDataEntries(DataLog log);

  public abstract void SimInit();

  public abstract void SimPeriodic(double battery, double[] currentDraw);

  protected static void SetStatusFrames(final TalonFX falcon) {
    falcon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20, 50);
    falcon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 20, 50);
    falcon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 40, 50);

    falcon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255, 50);
    falcon.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 255, 50);
    falcon.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 255, 50);
    falcon.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255, 50);
    falcon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 255, 50);
    falcon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255, 50);
    falcon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255, 50);
    falcon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255, 50);
    falcon.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 255, 50);
    falcon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255, 50);
    falcon.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255, 50);
    falcon.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 255, 50);
  }

  protected void RegisterDataEntry(final DataLog log, final String entry_name, final String type) {
    RegisterDataEntry(log, entry_name, type, "", 0);
  }

  protected void RegisterDataEntry(
      final DataLog log, final String entry_name, final String type, final String metadata) {
    RegisterDataEntry(log, entry_name, type, metadata, 0);
  }

  protected void RegisterDataEntry(
      final DataLog log, final String entry_name, final String type, final int timestamp) {
    RegisterDataEntry(log, entry_name, type, "", timestamp);
  }

  protected void RegisterDataEntry(
      final DataLog log,
      final String entry_name,
      final String type,
      final String metadata,
      final int timestamp) {
    dataEntries.put(entry_name, log.start(entry_name, type, metadata, timestamp));
  }

  protected Integer GetDataEntry(final String entry_name) {
    return dataEntries.get(entry_name);
  }

  protected void FalconEntryStartHelper(
      final DataLog log, final String name, final boolean primary) {
    RegisterDataEntry(log, name + "/percent", "double");
    RegisterDataEntry(log, name + "/voltage", "double");
    RegisterDataEntry(log, name + "/temperature", "double");
    RegisterDataEntry(log, name + "/current", "double");

    if (primary) {
      RegisterDataEntry(log, name + "/rpm", "double");
    }
  }

  protected void FalconEntryHelper(
      final DataLog log,
      final TalonFX motor,
      final String name,
      final int timestamp,
      boolean primary) {
    log.appendDouble(GetDataEntry(name + "/percent"), motor.getMotorOutputPercent(), timestamp);
    log.appendDouble(GetDataEntry(name + "/voltage"), motor.getMotorOutputVoltage(), timestamp);
    log.appendDouble(GetDataEntry(name + "/temperature"), motor.getTemperature(), timestamp);
    log.appendDouble(GetDataEntry(name + "/current"), motor.getStatorCurrent(), timestamp);

    if (primary) {
      log.appendDouble(
          GetDataEntry(name + "/rpm"),
          FalconUtils.NativeVelocityToRPM(motor.getSelectedSensorVelocity()),
          timestamp);
    }
  }
}
