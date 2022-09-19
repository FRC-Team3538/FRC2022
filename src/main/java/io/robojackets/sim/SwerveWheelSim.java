package io.robojackets.sim;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.y2023.math.Matrix;
import edu.wpi.first.y2023.math.system.LinearSystem;
import edu.wpi.first.y2023.math.system.plant.DCMotor;
import edu.wpi.first.y2023.wpilibj.simulation.FlywheelSim;

public class SwerveWheelSim extends FlywheelSim {
  double wheel_radius;

  public SwerveWheelSim(
      LinearSystem<N1, N1, N1> plant,
      DCMotor gearbox,
      double gearing,
      double wheel_radius,
      Matrix<N1, N1> measurementStdDevs) {
    super(plant, gearbox, gearing, measurementStdDevs);
    this.wheel_radius = wheel_radius;
  }

  public double GetLinearVelocity() {
    return getAngularVelocityRadPerSec() * this.wheel_radius;
  }
}
