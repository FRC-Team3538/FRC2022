package io.robojackets.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.robojackets.auto.impl.DefaultAuton;
import io.robojackets.auto.impl.DriveForwardAuto;
import io.robojackets.lib.LockableSendableChooser;
import io.robojackets.subsystems.RobotMap;
import java.util.List;

public class AutoPrograms {
  RobotMap IO;
  IAutoProgram selected_auton;
  List<IAutoProgram> auton_list;
  LockableSendableChooser<IAutoProgram> autoChooser = new LockableSendableChooser<>();

  public AutoPrograms(RobotMap IO) {
    this.IO = IO;

    auton_list = List.of(new DriveForwardAuto(IO));

    DefaultAuton defaultAuton = new DefaultAuton();

    autoChooser.setDefaultOption(defaultAuton.GetName(), defaultAuton);

    for (IAutoProgram auton : auton_list) {
      SmartDashboard.putData("Auton/" + auton.GetName(), auton);
      autoChooser.addOption(auton.GetName(), auton);
    }

    SmartDashboard.putData("Choose Auto", autoChooser);
  }

  public void Init() {
    autoChooser.Freeze();
    autoChooser.getSelected().Init();
  }

  public void Run() {
    autoChooser.getSelected().Run();
  }

  public void End() {
    autoChooser.UnFreeze();
  }
}
