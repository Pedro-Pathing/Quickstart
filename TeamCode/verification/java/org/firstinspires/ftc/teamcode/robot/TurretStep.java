package org.firstinspires.ftc.teamcode.robot;

/**
 * One turret control loop on a still target, as exact formulas: each method runs the real {@link
 * TurretLaw} decision or {@link TurretPlant} physics, and its contract {@code ensures \result ==
 * <formula>} is proven by OpenJML for in-range inputs. {@code verification/TurretProof} generates its
 * Lean definitions from exactly these formulas (gen_step.py) and proves convergence over them, so the
 * Lean model and this code cannot drift apart without a proof failing.
 *
 * <p>The loop: the autotuner's {@code i = d = f = 0} (Pedro's PID term is {@code p * error}) and no
 * feedforward (a still target). Three encoder reads, each with its own noise: {@code n1} for the
 * error, {@code n2} for the travel-limit cut, {@code n3} for the gate.
 *
 * <p>Formula language (what gen_step.py translates): + - * /, comparisons, && || !, ?:, Math.min,
 * Math.max, Math.abs, literals, parameters and calls to other methods of this class.
 */
final class TurretStep {
  private TurretStep() {}

  /** The measured error: the target minus one encoder read. */
  /*@ normal_behavior
  @   requires Math.abs(theta) <= 100 && Math.abs(target) <= 75 && Math.abs(n1) <= 1;
  @   ensures \result == target - (theta + n1);
  @ pure
  @*/
  static double measuredError(double target, double theta, double n1) {
    return TurretLaw.error(target, theta + n1);
  }

  /** Whether the wider exit band applies: settled on a target within a tolerance of this one. */
  /*@ normal_behavior
  @   requires Math.abs(target) <= 75 && Math.abs(settledTarget) <= 1000 && 0 < tol && tol <= 10;
  @   ensures \result == (settled && Math.abs(target - settledTarget) <= tol);
  @ pure
  @*/
  static boolean holdsBand(boolean settled, double settledTarget, double target, double tol) {
    return TurretLaw.holdsBand(settled, settledTarget, target, tol);
  }

  /** Whether the turret is settled after this loop. */
  /*@ normal_behavior
  @   requires Math.abs(theta) <= 100 && Math.abs(target) <= 75 && Math.abs(n1) <= 1;
  @   requires Math.abs(settledTarget) <= 1000 && 0 < tol && tol <= 10;
  @   requires 1 <= exitFactor && exitFactor <= 10;
  @   ensures \result == (holdsBand(settled, settledTarget, target, tol)
  @       ? Math.abs(measuredError(target, theta, n1)) <= exitFactor * tol
  @       : Math.abs(measuredError(target, theta, n1)) <= tol);
  @ pure
  @*/
  static boolean settledNext(
      boolean settled,
      double settledTarget,
      double target,
      double theta,
      double n1,
      double tol,
      double exitFactor) {
    double absError = Math.abs(measuredError(target, theta, n1));
    return TurretLaw.settledAfter(settled, settledTarget, target, absError, tol, exitFactor);
  }

  /** The target the turret is settled on after this loop. */
  /*@ normal_behavior
  @   requires Math.abs(theta) <= 100 && Math.abs(target) <= 75 && Math.abs(n1) <= 1;
  @   requires Math.abs(settledTarget) <= 1000 && 0 < tol && tol <= 10;
  @   ensures \result == (holdsBand(settled, settledTarget, target, tol)
  @       ? settledTarget
  @       : (Math.abs(measuredError(target, theta, n1)) <= tol ? target : settledTarget));
  @ pure
  @*/
  static double settledTargetNext(
      boolean settled, double settledTarget, double target, double theta, double n1, double tol) {
    double absError = Math.abs(measuredError(target, theta, n1));
    return TurretLaw.settledTargetAfter(settled, settledTarget, target, absError, tol);
  }

  /** The command while driving, limited: P plus the direction's static friction. */
  /*@ normal_behavior
  @   requires Math.abs(error) <= 200 && 0 <= p && p <= 1 && 0 <= ksPos && ksPos <= 1;
  @   requires 0 <= ksNeg && ksNeg <= 1 && 0 < maxPower && maxPower <= 1;
  @   ensures \result == (error > 0 ? Math.min(maxPower, p * error + ksPos)
  @       : (error < 0 ? Math.max(-maxPower, p * error - ksNeg) : 0.0));
  @ pure
  @*/
  static double driveCommand(
      double error, double p, double ksPos, double ksNeg, double maxPower) {
    double command = TurretLaw.drivingCommand(error, p * error, 0.0, 0.0, 0.0, 0.0, ksPos, ksNeg);
    return TurretLaw.limit(command, maxPower);
  }

  /** Whether a power would push past a travel limit at an angle ({@link TurretLaw#pushesPastTravel}). */
  /*@ normal_behavior
  @   requires Math.abs(power) <= 1 && Math.abs(angle) <= 200 && Math.abs(min) <= 75;
  @   requires Math.abs(max) <= 75;
  @   ensures \result == ((power > 0 && angle >= max) || (power < 0 && angle <= min));
  @ pure
  @*/
  static boolean pushesPast(double power, double angle, double min, double max) {
    return TurretLaw.pushesPastTravel(power, angle, min, max);
  }

  /**
   * The servo power for a limited command: cut where it pushes past a travel limit, then gated (a
   * reading more than 10 deg outside the travel is not trusted).
   */
  /*@ normal_behavior
  @   requires Math.abs(command) <= 1 && Math.abs(theta) <= 100 && Math.abs(n2) <= 1;
  @   requires Math.abs(n3) <= 1 && -75 <= min && min <= max && max <= 75;
  @   ensures \result == (theta + n3 >= min - 10 && theta + n3 <= max + 10
  @           && !pushesPast(pushesPast(command, theta + n2, min, max) ? 0.0 : command,
  @               theta + n3, min, max)
  @       ? (pushesPast(command, theta + n2, min, max) ? 0.0 : command)
  @       : 0.0);
  @ pure
  @*/
  static double output(double command, double theta, double n2, double n3, double min, double max) {
    double c = TurretLaw.pushesPastTravel(command, theta + n2, min, max) ? 0.0 : command;
    return TurretLaw.gate(c, 1.0, true, false, theta + n3, min, max, 10.0);
  }

  /** The servo power this loop: zero while settled on a still target, else the driving command. */
  /*@ normal_behavior
  @   requires Math.abs(theta) <= 100 && Math.abs(target) <= 75 && Math.abs(n1) <= 1;
  @   requires Math.abs(n2) <= 1 && Math.abs(n3) <= 1;
  @   requires Math.abs(settledTarget) <= 1000 && 0 < tol && tol <= 10;
  @   requires 1 <= exitFactor && exitFactor <= 10 && 0 < ffMin && Double.isFinite(kv);
  @   requires 0 <= p && p <= 1 && 0 <= ksPos && ksPos <= 1 && 0 <= ksNeg && ksNeg <= 1;
  @   requires 0 < maxPower && maxPower <= 1 && -75 <= min && min <= max && max <= 75;
  @   ensures \result == output(
  @       settledNext(settled, settledTarget, target, theta, n1, tol, exitFactor)
  @           ? 0.0
  @           : driveCommand(measuredError(target, theta, n1), p, ksPos, ksNeg, maxPower),
  @       theta, n2, n3, min, max);
  @ pure
  @*/
  static double power(
      boolean settled,
      double settledTarget,
      double target,
      double theta,
      double n1,
      double n2,
      double n3,
      double tol,
      double exitFactor,
      double ffMin,
      double kv,
      double p,
      double ksPos,
      double ksNeg,
      double maxPower,
      double min,
      double max) {
    // As Turret.updateTurret: decide settled, take that branch's command, limit it, cut, gate.
    double error = measuredError(target, theta, n1);
    boolean settledNow = settledNext(settled, settledTarget, target, theta, n1, tol, exitFactor);
    double command =
        settledNow
            ? TurretLaw.limit(TurretLaw.settledCommand(error, 0.0, kv, ffMin, p, ksPos, ksNeg), maxPower)
            : driveCommand(error, p, ksPos, ksNeg, maxPower);
    return output(command, theta, n2, n3, min, max);
  }

  /** The servo output after one loop ({@link TurretPlant#servoOutput}). */
  /*@ normal_behavior
  @   requires Math.abs(y) <= 1 && Math.abs(power) <= 1 && 0 < dt && dt <= 1;
  @   requires 0 <= lag && lag <= 1;
  @   ensures \result == (lag <= dt ? power : y + (power - y) * (dt / lag));
  @ pure
  @*/
  static double servo(double y, double power, double dt, double lag) {
    return TurretPlant.servoOutput(y, power, dt, lag);
  }

  /** The part of a servo output that moves the turret ({@link TurretPlant#effective}). */
  /*@ normal_behavior
  @   requires Math.abs(y) <= 1 && 0 <= sPos && sPos <= 1 && 0 <= sNeg && sNeg <= 1;
  @   ensures \result == (y >= 0 ? (y <= sPos ? 0.0 : y - sPos) : (-y <= sNeg ? 0.0 : y + sNeg));
  @ pure
  @*/
  static double effective(double y, double sPos, double sNeg) {
    return TurretPlant.effective(y, sPos, sNeg);
  }

  /** The turret angle after one loop ({@link TurretPlant#angle}), held by the stops. */
  /*@ normal_behavior
  @   requires Math.abs(theta) <= 100 && Math.abs(eff) <= 1 && 0 < k && k <= 1000;
  @   requires 0 < dt && dt <= 1 && stopMin < stopMax && Math.abs(stopMin) <= 1000;
  @   requires Math.abs(stopMax) <= 1000;
  @   ensures \result == Math.min(stopMax, Math.max(theta + k * eff * dt, stopMin));
  @ pure
  @*/
  static double angle(
      double theta, double eff, double k, double dt, double stopMin, double stopMax) {
    return TurretPlant.angle(theta, eff, k, dt, stopMin, stopMax);
  }
}
