package org.firstinspires.ftc.teamcode.robot;

/**
 * Every decision the shot feed gate makes, as pure functions of numbers and flags: no config,
 * hardware, clock or state. {@link ShotController} reads those, calls these, and drives the intake.
 *
 * <p>The JML contracts are proven against this exact code by OpenJML, like {@link TurretLaw}. Every
 * method has a contract with no precondition, so none can throw whatever the sensors or callers
 * give it. The safety theorem is {@link #gate}: feeding implies the flywheel is ready and the
 * turret aligned, and, unless the caller explicitly waived it for a manual fixed-power shot ({@code
 * requireValidSolution == false}), that the solution is valid and current and the HIVE settled. A
 * manual shot therefore gives up exactly the solution and HIVE-settled checks, nothing else.
 */
final class FeedLaw {
  private FeedLaw() {}

  /** True if a solution was solved for the HIVE epoch and ball type in force now. */
  /*@ normal_behavior
  @   ensures \result <==> solutionEpoch == epoch && solutionBallType == ballType;
  @ pure
  @*/
  static boolean isCurrent(int solutionEpoch, int epoch, int solutionBallType, int ballType) {
    return solutionEpoch == epoch && solutionBallType == ballType;
  }

  /** Flywheel velocity as a fraction of its target; 0 when there is no positive target. */
  /*@ normal_behavior
  @   ensures !(target > 0) ==> \result == 0.0;
  @   ensures target > 0 && Double.isFinite(target) && Double.isFinite(velocity)
  @           ==> \result == velocity / target;
  @   ensures target > 0 && Double.isNaN(velocity) ==> Double.isNaN(\result);
  @ pure
  @*/
  static double ratio(double velocity, double target) {
    return target > 0 ? velocity / target : 0.0;
  }

  /** True if the flywheel velocity is inside [target * minThreshold, target * maxThreshold]. */
  /*@ normal_behavior
  @   ensures \result <==> velocity >= target * minThreshold && velocity <= target * maxThreshold;
  @   ensures Double.isNaN(velocity) ==> !\result;
  @ pure
  @*/
  static boolean inBand(double velocity, double target, double minThreshold, double maxThreshold) {
    return velocity >= target * minThreshold && velocity <= target * maxThreshold;
  }

  /**
   * The feed latch: arms inside [armLow, armHigh], then holds until the ratio drops below
   * releaseLow. A NaN ratio never arms and always releases.
   */
  /*@ normal_behavior
  @   ensures !currentlyFeeding ==> (\result <==> armLow <= ratio && ratio <= armHigh);
  @   ensures currentlyFeeding ==> (\result <==> ratio >= releaseLow);
  @   ensures Double.isNaN(ratio) ==> !\result;
  @ pure
  @*/
  static boolean shouldFeed(
      double ratio, boolean currentlyFeeding, double armLow, double armHigh, double releaseLow) {
    return currentlyFeeding ? ratio >= releaseLow : ratio >= armLow && ratio <= armHigh;
  }

  /** Solution check, waived (true) when the shot does not require a valid solution. */
  /*@ normal_behavior
  @   ensures \result <==> (!requireValidSolution || (solutionCurrent && solutionValid));
  @ pure
  @*/
  static boolean solutionOk(
      boolean requireValidSolution, boolean solutionCurrent, boolean solutionValid) {
    return !requireValidSolution || (solutionCurrent && solutionValid);
  }

  /** HIVE-settled check, waived (true) when the shot does not require a valid solution. */
  /*@ normal_behavior
  @   ensures \result <==> (!requireValidSolution || hiveSettled);
  @ pure
  @*/
  static boolean hiveOk(boolean requireValidSolution, boolean hiveSettled) {
    return !requireValidSolution || hiveSettled;
  }

  /**
   * Whether to feed. The flywheel latch and turret alignment are always required; the solution and
   * HIVE checks are required unless {@code requireValidSolution} is false.
   */
  /*@ normal_behavior
  @   ensures \result ==> flywheelReady && turretAligned;
  @   ensures \result ==> (requireValidSolution ==> (solutionCurrent && solutionValid && hiveSettled));
  @   ensures requireValidSolution
  @           ==> (\result <==> flywheelReady && turretAligned && solutionCurrent
  @                              && solutionValid && hiveSettled);
  @   ensures !requireValidSolution ==> (\result <==> flywheelReady && turretAligned);
  @ pure
  @*/
  static boolean gate(
      boolean flywheelReady,
      boolean turretAligned,
      boolean requireValidSolution,
      boolean solutionCurrent,
      boolean solutionValid,
      boolean hiveSettled) {
    return flywheelReady
        && turretAligned
        && solutionOk(requireValidSolution, solutionCurrent, solutionValid)
        && hiveOk(requireValidSolution, hiveSettled);
  }
}
