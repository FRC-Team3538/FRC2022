package io.robojackets.auto.impl;

import edu.wpi.first.util.sendable.SendableBuilder;
import io.robojackets.auto.IAutoProgram;

public class DefaultAuton implements IAutoProgram {

  @Override
  public void initSendable(SendableBuilder builder) {}

  @Override
  public void Init() {}

  @Override
  public void Run() {}

  @Override
  public void End() {}

  @Override
  public String GetName() {
    return "00 - None";
  }
}
