package io.robojackets.lib;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class LockableSendableChooser<T> extends SendableChooser<T> {
  boolean frozen = false;
  T selection;

  public LockableSendableChooser() {
    super();
  }

  public void Freeze() {
    frozen = true;
  }

  public void UnFreeze() {
    frozen = false;
  }

  public boolean IsFrozen() {
    return frozen;
  }

  @Override
  public T getSelected() {
    if (!frozen) {
      selection = super.getSelected();
    }

    return selection;
  }
}
