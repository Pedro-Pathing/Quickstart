package org.firstinspires.ftc.teamcode.robot;

/**
 * The CRServo turret physics: what {@link TurretSim} runs and what the convergence proof ({@code
 * verification/java/.../TurretConvergence.java}) assumes, so the tests and the proof share one
 * model. Commanded power reaches the servo through a first-order lag; the servo output moves the
 * turret at {@code k} deg/s per unit of output beyond a direction-dependent static friction; the
 * mechanical stops hold it.
 */
final class TurretPlant {
  private TurretPlant() {}

  /** Servo output after one loop of {@code dt} s: a first-order lag of {@code lagSec} s. */
  /*@ normal_behavior
  @   requires Double.isFinite(output) && Double.isFinite(power);
  @   requires 0 < dt && dt <= 1 && 0 <= lagSec && lagSec <= 1;
  @   ensures lagSec <= dt ==> \result == power;
  @   ensures dt < lagSec ==> \result == output + (power - output) * (dt / lagSec);
  @ pure
  @*/
  static double servoOutput(double output, double power, double dt, double lagSec) {
    // min(1, dt / lagSec) for a positive lag, 1 for none; this way round the proof has no min.
    double alpha = lagSec <= dt ? 1.0 : dt / lagSec;
    return output + (power - output) * alpha;
  }

  /**
   * The part of the servo output that moves the turret: none within the static friction for the
   * direction ({@code stictionPositive} or {@code stictionNegative}, in power units), else the
   * excess, signed.
   */
  /*@ normal_behavior
  @   requires 0 <= stictionPositive && 0 <= stictionNegative;
  @   ensures output >= 0 ==> 0 <= \result && \result <= output;
  @   ensures output < 0 ==> output <= \result && \result <= 0;
  @   ensures output > stictionPositive ==> \result == output - stictionPositive;
  @   ensures output < -stictionNegative ==> \result == output + stictionNegative;
  @   ensures 0 <= output && output <= stictionPositive ==> \result == 0.0;
  @   ensures output < 0 && -output <= stictionNegative ==> \result == 0.0;
  @ pure
  @*/
  static double effective(double output, double stictionPositive, double stictionNegative) {
    if (output >= 0) {
      return output <= stictionPositive ? 0.0 : output - stictionPositive;
    }
    return -output <= stictionNegative ? 0.0 : output + stictionNegative;
  }

  /** Turret angle after {@code dt} s at {@code effective} output, held by the stops. */
  /*@ normal_behavior
  @   requires stopMin < stopMax;
  @   requires Double.isFinite(angle + k * effective * dt);
  @   ensures stopMin <= \result && \result <= stopMax;
  @   ensures stopMin <= angle + k * effective * dt && angle + k * effective * dt <= stopMax
  @           ==> \result == angle + k * effective * dt;
  @   ensures angle + k * effective * dt < stopMin ==> \result == stopMin;
  @   ensures stopMax < angle + k * effective * dt ==> \result == stopMax;
  @ pure
  @*/
  static double angle(
      double angle, double effective, double k, double dt, double stopMin, double stopMax) {
    // Math.clamp's own order of operations, as min/max: it gives the same values for ordered
    // stops, and its NaN and signed-zero argument checks are no part of the physics.
    return Math.min(stopMax, Math.max(angle + k * effective * dt, stopMin));
  }
}
