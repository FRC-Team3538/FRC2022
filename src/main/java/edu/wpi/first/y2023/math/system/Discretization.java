// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.y2023.math.system;

import edu.wpi.first.math.Num;
import edu.wpi.first.y2023.math.Matrix;
import edu.wpi.first.y2023.math.Pair;
import org.ejml.simple.SimpleMatrix;

@SuppressWarnings({"ParameterName", "MethodTypeParameterName"})
public final class Discretization {
  private Discretization() {
    // Utility class
  }

  /**
   * Discretizes the given continuous A matrix.
   *
   * @param <States> Num representing the number of states.
   * @param contA Continuous system matrix.
   * @param dtSeconds Discretization timestep.
   * @return the discrete matrix system.
   */
  public static <States extends Num> Matrix<States, States> discretizeA(
      Matrix<States, States> contA, double dtSeconds) {
    // A_d = eᴬ<sup>T</sup>
    return contA.times(dtSeconds).exp();
  }

  /**
   * Discretizes the given continuous A and B matrices.
   *
   * @param <States> Nat representing the states of the system.
   * @param <Inputs> Nat representing the inputs to the system.
   * @param contA Continuous system matrix.
   * @param contB Continuous input matrix.
   * @param dtSeconds Discretization timestep.
   * @return a Pair representing discA and diskB.
   */
  @SuppressWarnings("LocalVariableName")
  public static <States extends Num, Inputs extends Num>
      Pair<Matrix<States, States>, Matrix<States, Inputs>> discretizeAB(
          Matrix<States, States> contA, Matrix<States, Inputs> contB, double dtSeconds) {
    int states = contA.getNumRows();
    int inputs = contB.getNumCols();

    // M = [A  B]
    //     [0  0]
    var M = new Matrix<>(new SimpleMatrix(states + inputs, states + inputs));
    M.assignBlock(0, 0, contA);
    M.assignBlock(0, contA.getNumCols(), contB);

    //  ϕ = eᴹ<sup>T</sup> = [A_d  B_d]
    //            [ 0    I ]
    var phi = M.times(dtSeconds).exp();

    var discA = new Matrix<States, States>(new SimpleMatrix(states, states));
    discA.extractFrom(0, 0, phi);

    var discB = new Matrix<States, Inputs>(new SimpleMatrix(states, inputs));
    discB.extractFrom(0, contB.getNumRows(), phi);

    return new Pair<>(discA, discB);
  }

  /**
   * Discretizes the given continuous A and Q matrices.
   *
   * @param <States> Nat representing the number of states.
   * @param contA Continuous system matrix.
   * @param contQ Continuous process noise covariance matrix.
   * @param dtSeconds Discretization timestep.
   * @return a pair representing the discrete system matrix and process noise covariance matrix.
   */
  @SuppressWarnings("LocalVariableName")
  public static <States extends Num>
      Pair<Matrix<States, States>, Matrix<States, States>> discretizeAQ(
          Matrix<States, States> contA, Matrix<States, States> contQ, double dtSeconds) {
    int states = contA.getNumRows();

    // Make continuous Q symmetric if it isn't already
    Matrix<States, States> Q = contQ.plus(contQ.transpose()).div(2.0);

    // M = [−A  Q ]
    //     [ 0  A<sup>T</sup>]
    final var M = new Matrix<>(new SimpleMatrix(2 * states, 2 * states));
    M.assignBlock(0, 0, contA.times(-1.0));
    M.assignBlock(0, states, Q);
    M.assignBlock(states, 0, new Matrix<>(new SimpleMatrix(states, states)));
    M.assignBlock(states, states, contA.transpose());

    // ϕ = eᴹ<sup>T</sup> = [−A_d  A_d<sup>-1</sup>Q_d]
    //           [ 0      A_d<sup>T</sup>  ]
    final var phi = M.times(dtSeconds).exp();

    // ϕ<sub>1</sub><sub>2</sub> = A_d<sup>-1</sup>Q_d
    final Matrix<States, States> phi12 = phi.block(states, states, 0, states);

    // ϕ<sub>2</sub><sub>2</sub> = A_d<sup>T</sup>
    final Matrix<States, States> phi22 = phi.block(states, states, states, states);

    final var discA = phi22.transpose();

    Q = discA.times(phi12);

    // Make discrete Q symmetric if it isn't already
    final var discQ = Q.plus(Q.transpose()).div(2.0);

    return new Pair<>(discA, discQ);
  }

  /**
   * Discretizes the given continuous A and Q matrices.
   *
   * <p>Rather than solving a 2N x 2N matrix exponential like in DiscretizeQ() (which is expensive),
   * we take advantage of the structure of the block matrix of A and Q.
   *
   * <ul>
   *   <li>eᴬ<sup>T</sup>, which is only N x N, is relatively cheap.
   *   <li>The upper-right quarter of the 2N x 2N matrix, which we can approximate using a taylor
   *       series to several terms and still be substantially cheaper than taking the big
   *       exponential.
   * </ul>
   *
   * @param <States> Nat representing the number of states.
   * @param contA Continuous system matrix.
   * @param contQ Continuous process noise covariance matrix.
   * @param dtSeconds Discretization timestep.
   * @return a pair representing the discrete system matrix and process noise covariance matrix.
   */
  @SuppressWarnings("LocalVariableName")
  public static <States extends Num>
      Pair<Matrix<States, States>, Matrix<States, States>> discretizeAQTaylor(
          Matrix<States, States> contA, Matrix<States, States> contQ, double dtSeconds) {
    //       T
    // Q_d = ∫ e^(Aτ) Q e^(A<sup>T</sup>τ) dτ
    //       0
    //
    // M = [−A  Q ]
    //     [ 0  A<sup>T</sup>]
    // ϕ = eᴹ<sup>T</sup>
    // ϕ<sub>1</sub><sub>2</sub> = A_d<sup>-1</sup>Q_d
    //
    // Taylor series of ϕ:
    //
    //   ϕ = eᴹ<sup>T</sup> = I + MT + 1/2 M²T² + 1/6 M³T³ + …
    //   ϕ = eᴹ<sup>T</sup> = I + MT + 1/2 T²M² + 1/6 T³M³ + …
    //
    // Taylor series of ϕ expanded for ϕ<sub>1</sub><sub>2</sub>:
    //
    //   ϕ<sub>1</sub><sub>2</sub> = 0 + QT + 1/2 T² (−AQ + QA<sup>T</sup>) + 1/6 T³ (−A lastTerm +
    // Q
    // A<sup>T</sup>²) + …
    //
    // ```
    // lastTerm = Q
    // lastCoeff = T
    // ATn = A<sup>T</sup>
    // ϕ<sub>1</sub><sub>2</sub> = lastTerm lastCoeff = QT
    //
    // for i in range(2, 6):
    //   // i = 2
    //   lastTerm = −A lastTerm + Q ATn = −AQ + QA<sup>T</sup>
    //   lastCoeff *= T/i → lastCoeff *= T/2 = 1/2 T²
    //   ATn *= A<sup>T</sup> = A<sup>T</sup>²
    //
    //   // i = 3
    //   lastTerm = −A lastTerm + Q ATn = −A (−AQ + QA<sup>T</sup>) + QA<sup>T</sup>² = …
    //   …
    // ```

    // Make continuous Q symmetric if it isn't already
    Matrix<States, States> Q = contQ.plus(contQ.transpose()).div(2.0);

    Matrix<States, States> lastTerm = Q.copy();
    double lastCoeff = dtSeconds;

    // A<sup>Tn</sup>
    Matrix<States, States> ATn = contA.transpose();

    Matrix<States, States> phi12 = lastTerm.times(lastCoeff);

    // i = 6 i.e. 5th order should be enough precision
    for (int i = 2; i < 6; ++i) {
      lastTerm = contA.times(-1).times(lastTerm).plus(Q.times(ATn));
      lastCoeff *= dtSeconds / ((double) i);

      phi12 = phi12.plus(lastTerm.times(lastCoeff));

      ATn = ATn.times(contA.transpose());
    }

    var discA = discretizeA(contA, dtSeconds);
    Q = discA.times(phi12);

    // Make Q symmetric if it isn't already
    var discQ = Q.plus(Q.transpose()).div(2.0);

    return new Pair<>(discA, discQ);
  }

  /**
   * Returns a discretized version of the provided continuous measurement noise covariance matrix.
   * Note that dt=0.0 divides R by zero.
   *
   * @param <O> Nat representing the number of outputs.
   * @param R Continuous measurement noise covariance matrix.
   * @param dtSeconds Discretization timestep.
   * @return Discretized version of the provided continuous measurement noise covariance matrix.
   */
  public static <O extends Num> Matrix<O, O> discretizeR(Matrix<O, O> R, double dtSeconds) {
    // R_d = 1/T R
    return R.div(dtSeconds);
  }
}