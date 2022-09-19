package io.robojackets.auto;

import edu.wpi.first.util.sendable.Sendable;

public interface IAutoProgram extends Sendable {
  void Init();

  void Run();

  void End();

  String GetName();
}
