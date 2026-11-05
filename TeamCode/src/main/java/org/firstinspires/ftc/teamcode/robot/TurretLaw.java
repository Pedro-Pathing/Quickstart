package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Every decision {@link Turret} makes, as pure functions of numbers: no config, hardware, clock or
 * state. {@link Turret} reads those, calls these, and writes the servo.
 *
 * <p>The JML contracts ({@code //@} and {@code /*@} comments) are proven against this exact code by
 * OpenJML: {@code ./gradlew verifyTurret}. They are ordinary comments to javac, so the robot runs
 * the same bytecode either way. Every method has a contract with no precondition, so none can throw
 * or hang whatever the config, sensors or callers give it; the safety facts about the servo output
 * ({@link #gate}) and the aim claim ({@link #aimed}) hold for every input too. Functional facts
 * that need a sane config are stated under {@code requires}.
 */
final class TurretLaw {
  private TurretLaw() {}

  /** Largest turret angle magnitude (deg) the functional contracts are stated for. */
  static final double MAX_ANGLE_DEG = 180.0;

  /*@ normal_behavior
  @   ensures \result <==> Double.isFinite(x) && Math.abs(x) <= MAX_ANGLE_DEG;
  @ pure
  @*/
  static boolean isAngle(double x) {
    return Double.isFinite(x) && Math.abs(x) <= MAX_ANGLE_DEG;
  }

  // ---------------------------------------------------------------------------------------------
  // Angles

  /**
   * An angle wrapped to [-180, 180). The exact IEEE remainder comes first because the SDK's {@code
   * AngleUnit.normalizeDegrees} never returns for some huge finite inputs (e.g. -7.66e86): its
   * reduction leaves a value too large for its 360-degree steps to change. Infinite or NaN gives
   * NaN.
   */
  /*@ normal_behavior
  @   ensures Double.isFinite(deg) ==> -180 <= \result && \result < 180;
  @   ensures !Double.isFinite(deg) ==> Double.isNaN(\result);
  @   ensures -180 <= deg && deg < 180 ==> \result == deg;
  @ pure
  @*/
  static double wrap(double deg) {
    return AngleUnit.normalizeDegrees(Math.IEEEremainder(deg, 360.0));
  }

  /** Signed angle from {@code currentDeg} to {@code targetDeg}, the short way round. */
  /*@ normal_behavior
  @   ensures Double.isFinite(targetDeg - currentDeg) ==> -180 <= \result && \result < 180;
  @   ensures !Double.isFinite(targetDeg - currentDeg) ==> Double.isNaN(\result);
  @   ensures -180 <= targetDeg - currentDeg && targetDeg - currentDeg < 180
  @           ==> \result == targetDeg - currentDeg;
  @ pure
  @*/
  static double error(double targetDeg, double currentDeg) {
    return wrap(targetDeg - currentDeg);
  }

  /**
   * An angle limited to the travel, in whichever order the limits are configured. min/max rather
   * than {@code Math.clamp}, which throws for a NaN or misordered limit. NaN stays NaN.
   */
  /*@ normal_behavior
  @   ensures !Double.isNaN(deg) && !Double.isNaN(a) && !Double.isNaN(b)
  @           ==> Math.min(a, b) <= \result && \result <= Math.max(a, b);
  @   ensures Double.isNaN(deg) ==> Double.isNaN(\result);
  @   ensures a <= b && a <= deg && deg <= b ==> \result == deg;
  @ pure
  @*/
  static double toTravel(double deg, double a, double b) {
    return Math.max(Math.min(a, b), Math.min(Math.max(a, b), deg));
  }

  // ---------------------------------------------------------------------------------------------
  // Encoder

  /**
   * An encoder offset (V) wrapped to the half of the encoder's turn either side of zero: [-{@code
   * fullScale}/2, {@code fullScale}/2). Left alone when {@code fullScale} is not positive.
   */
  /*@ normal_behavior
  @   requires Double.isFinite(delta) && 0 < fullScale && fullScale <= 10;
  @   requires Math.abs(delta) <= 2 * fullScale;
  @   ensures -fullScale / 2.0 <= \result && \result < fullScale / 2.0;
  @ also normal_behavior
  @   requires Double.isNaN(delta);
  @   ensures Double.isNaN(\result);
  @ also normal_behavior
  @   ensures true;
  @ pure
  @*/
  static double wrapVolts(double delta, double fullScale) {
    if (fullScale > 0) {
      double half = fullScale / 2.0;
      double turns = (delta + half) / fullScale;
      double whole = Math.floor(turns);
      // Proof steps (the floor scaled back to volts), checked only where the first contract
      // applies. Block comments: google-java-format turns a //@ line into a plain // @ comment.
      /*@ ghost boolean sane =
      @     Double.isFinite(delta) && fullScale <= 10 && Math.abs(delta) <= 2 * fullScale;
      @ assert sane ==> turns * fullScale == delta + half;
      @ assert sane ==> 0 <= (turns - whole) * fullScale
      @     && (turns - whole) * fullScale < fullScale;
      @*/
      delta -= whole * fullScale;
    }
    return delta;
  }

  /**
   * Turret angle for an encoder voltage: the offset from the zero voltage, wrapped to the half of
   * the encoder's turn either side of zero, signed and scaled. A NaN reading stays NaN.
   */
  /*@ normal_behavior
  @   requires Double.isFinite(zeroVoltage) && Double.isFinite(voltage);
  @   requires 0 < fullScale && fullScale <= 10;
  @   requires Math.abs(voltage - zeroVoltage) <= 2 * fullScale;
  @   requires Double.isFinite(degreesPerVolt) && fullScale * Math.abs(degreesPerVolt) <= 350;
  @   ensures isAngle(\result);
  @ also normal_behavior
  @   requires Double.isNaN(voltage);
  @   ensures Double.isNaN(\result);
  @ also normal_behavior
  @   ensures true;
  @ pure
  @*/
  static double angleForVoltage(
      double voltage,
      double zeroVoltage,
      double fullScale,
      boolean inverted,
      double degreesPerVolt) {
    double delta = wrapVolts(voltage - zeroVoltage, fullScale);
    if (inverted) {
      delta = -delta;
    }
    return delta * degreesPerVolt;
  }

  /** Whether an encoder angle is close enough to the travel to be believed. NaN is not. */
  /*@ normal_behavior
  @   ensures \result <==> (angle >= min - margin && angle <= max + margin);
  @   ensures \result ==> !Double.isNaN(angle);
  @ pure
  @*/
  static boolean readingTrusted(double angle, double min, double max, double margin) {
    return angle >= min - margin && angle <= max + margin;
  }

  // ---------------------------------------------------------------------------------------------
  // Aim tolerance and settling

  /**
   * Aim tolerance (deg) at a distance: the angle that keeps the lateral miss within {@code
   * maxDrift} at the target, bounded by {@code minDeg} and {@code maxDeg} in whichever order they
   * are configured; {@code nearVal} closer than {@code nearCutoff}. min/max rather than {@code
   * Math.clamp}, which throws for a NaN or misordered bound.
   */
  /*@ normal_behavior
  @   requires Double.isFinite(minDeg) && Double.isFinite(maxDeg) && minDeg <= maxDeg;
  @   requires Double.isFinite(maxDrift) && 0 < nearCutoff;
  @   ensures !(distance < nearCutoff) && !Double.isNaN(distance)
  @           ==> minDeg <= \result && \result <= maxDeg;
  @ also normal_behavior
  @   ensures distance < nearCutoff ==> \result == nearVal || Double.isNaN(nearVal);
  @ pure
  @*/
  static double tolerance(
      double distance,
      double nearCutoff,
      double nearVal,
      double maxDrift,
      double minDeg,
      double maxDeg) {
    if (distance < nearCutoff) {
      return nearVal;
    }
    double ratio = Math.max(-1.0, Math.min(1.0, (maxDrift / 2.0) / distance));
    double degrees = Math.toDegrees(Math.asin(ratio));
    return Math.max(Math.min(minDeg, maxDeg), Math.min(Math.max(minDeg, maxDeg), degrees));
  }

  /** The settle hysteresis factor: never below 1. */
  /*@ normal_behavior
  @   ensures \result >= 1.0 || Double.isNaN(\result);
  @   ensures Double.isFinite(configured) ==> Double.isFinite(\result);
  @ pure
  @*/
  static double exitFactor(double configured) {
    return Math.max(1.0, configured);
  }

  /** Whether the wider exit band applies: settled, and still on (nearly) the same target. */
  /*@ normal_behavior
  @   ensures \result <==> settled && Math.abs(target - settledTarget) <= tolerance;
  @ pure
  @*/
  static boolean holdsBand(boolean settled, double settledTarget, double target, double tolerance) {
    return settled && Math.abs(target - settledTarget) <= tolerance;
  }

  /**
   * Whether the turret is settled after this loop. Once settled on a target it stays settled until
   * the error passes {@code tolerance * exitFactor}; a target that moved more than a tolerance from
   * the one it settled on is judged afresh against {@code tolerance}.
   */
  /*@ normal_behavior
  @   ensures holdsBand(settled, settledTarget, target, tolerance)
  @           ==> (\result <==> absError <= tolerance * exitFactor);
  @   ensures !holdsBand(settled, settledTarget, target, tolerance)
  @           ==> (\result <==> absError <= tolerance);
  @   ensures \result ==> !Double.isNaN(absError);
  @ also normal_behavior
  @   requires Double.isFinite(tolerance) && tolerance >= 0;
  @   requires Double.isFinite(exitFactor) && exitFactor >= 1;
  @   ensures \result ==> absError <= tolerance * exitFactor;
  @   ensures absError <= tolerance ==> \result;
  @ pure
  @*/
  static boolean settledAfter(
      boolean settled,
      double settledTarget,
      double target,
      double absError,
      double tolerance,
      double exitFactor) {
    if (holdsBand(settled, settledTarget, target, tolerance)) {
      return absError <= tolerance * exitFactor;
    }
    return absError <= tolerance;
  }

  /** The target the turret is settled on after this loop (see {@link #settledAfter}). */
  /*@ normal_behavior
  @   ensures holdsBand(settled, settledTarget, target, tolerance)
  @           ==> \result == settledTarget;
  @   ensures !holdsBand(settled, settledTarget, target, tolerance) && absError <= tolerance
  @           && !Double.isNaN(target) ==> \result == target;
  @   ensures !holdsBand(settled, settledTarget, target, tolerance) && !(absError <= tolerance)
  @           && !Double.isNaN(settledTarget) ==> \result == settledTarget;
  @ pure
  @*/
  static double settledTargetAfter(
      boolean settled, double settledTarget, double target, double absError, double tolerance) {
    if (holdsBand(settled, settledTarget, target, tolerance)) {
      return settledTarget;
    }
    return absError <= tolerance ? target : settledTarget;
  }

  /**
   * Whether the shooter is pointed at {@code desired}: it is inside the travel and the turret is
   * within the band the controller holds in. Holds for any input: a NaN anywhere is never aimed.
   */
  /*@ normal_behavior
  @   ensures \result ==> min <= desired && desired <= max;
  @   ensures \result ==> !Double.isNaN(current) && Double.isFinite(desired);
  @   ensures \result && holdsBand(settled, settledTarget, desired, tolerance)
  @           ==> Math.abs(error(desired, current)) <= tolerance * exitFactor;
  @   ensures \result && !holdsBand(settled, settledTarget, desired, tolerance)
  @           ==> Math.abs(error(desired, current)) <= tolerance;
  @ pure
  @*/
  static boolean aimed(
      double desired,
      double current,
      double min,
      double max,
      double tolerance,
      boolean settled,
      double settledTarget,
      double exitFactor) {
    // Written as "inside", not "not outside": a NaN travel limit must mean never aimed.
    if (!(Double.isFinite(desired) && desired >= min && desired <= max)) {
      return false;
    }
    double error = error(desired, current);
    boolean holdBand = holdsBand(settled, settledTarget, desired, tolerance);
    return Math.abs(error) <= tolerance * (holdBand ? exitFactor : 1.0);
  }

  // ---------------------------------------------------------------------------------------------
  // Commands

  /** Static-friction step for motion in the direction of {@code sign} (asymmetric kS). */
  /*@ normal_behavior
  @   ensures sign > 0 && !Double.isNaN(ksPositive) ==> \result == Math.abs(ksPositive);
  @   ensures sign < 0 && !Double.isNaN(ksNegative) ==> \result == -Math.abs(ksNegative);
  @   ensures !(sign > 0) && !(sign < 0) ==> \result == 0.0;
  @   ensures sign > 0 ==> !(\result < 0);
  @   ensures sign < 0 ==> !(\result > 0);
  @ pure
  @*/
  static double staticFriction(double sign, double ksPositive, double ksNegative) {
    if (sign > 0) {
      return Math.abs(ksPositive);
    }
    if (sign < 0) {
      return -Math.abs(ksNegative);
    }
    return 0.0;
  }

  /**
   * Command while settled. While the target moves (chassis-driven rate at least {@code
   * ffMinRateDps}) follow it with kS sign(v) + kV v plus a proportional correction, so a kS or kV
   * mismatch cannot creep the aim out of the band. A still target is left alone: below {@code
   * ffMinRateDps} the rate is odometry noise, and a static-friction step there would chatter.
   */
  /*@ normal_behavior
  @   ensures !(Math.abs(feedforwardRateDps) >= ffMinRateDps) ==> \result == 0.0;
  @ pure
  @*/
  static double settledCommand(
      double error,
      double feedforwardRateDps,
      double kv,
      double ffMinRateDps,
      double p,
      double ksPositive,
      double ksNegative) {
    boolean targetMoving = Math.abs(feedforwardRateDps) >= ffMinRateDps;
    double frictionSign = kv > 0.0 ? feedforwardRateDps : error;
    return targetMoving
        ? kv * feedforwardRateDps + staticFriction(frictionSign, ksPositive, ksNegative) + p * error
        : 0.0;
  }

  /**
   * Command while driving to the target, before limiting: P (and D) from the PID controller, the
   * {@code f} step, the static-friction step, the clamped I term and the rate feedforward.
   */
  /*@ normal_behavior
  @   ensures Double.isFinite(error) && Double.isFinite(pidTerm) && Double.isFinite(integral)
  @       && Double.isFinite(feedforwardRateDps) && Double.isFinite(kv) && Double.isFinite(f)
  @       && Double.isFinite(ksPositive) && Double.isFinite(ksNegative)
  @       ==> \result == pidTerm + f * Math.signum(error)
  @           + staticFriction(error, ksPositive, ksNegative) + integral + kv * feedforwardRateDps;
  @ pure
  @*/
  static double drivingCommand(
      double error,
      double pidTerm,
      double integral,
      double feedforwardRateDps,
      double kv,
      double f,
      double ksPositive,
      double ksNegative) {
    return pidTerm
        + f * Math.signum(error)
        + staticFriction(error, ksPositive, ksNegative)
        + integral
        + kv * feedforwardRateDps;
  }

  /** +1 if a raw command is railed high, -1 if railed low, 0 if in range (for the I term). */
  /*@ normal_behavior
  @   ensures \result == 1 <==> raw > maxPower;
  @   ensures \result == -1 <==> !(raw > maxPower) && raw < -maxPower;
  @   ensures -1 <= \result && \result <= 1;
  @ pure
  @*/
  static int saturationSign(double raw, double maxPower) {
    return raw > maxPower ? 1 : (raw < -maxPower ? -1 : 0);
  }

  /**
   * A command limited to +/-|{@code maxPower}|. min/max rather than {@code Math.clamp}, which
   * throws for a NaN limit. NaN in gives NaN out.
   */
  /*@ normal_behavior
  @   ensures !Double.isNaN(\result) ==> Math.abs(\result) <= Math.abs(maxPower);
  @   ensures !Double.isNaN(\result) ==> (\result > 0 ==> command > 0) && (\result < 0 ==> command < 0);
  @   ensures Math.abs(command) <= Math.abs(maxPower) ==> \result == command;
  @   ensures !Double.isNaN(maxPower) && command > Math.abs(maxPower)
  @           ==> \result == Math.abs(maxPower);
  @   ensures !Double.isNaN(maxPower) && command < -Math.abs(maxPower)
  @           ==> \result == -Math.abs(maxPower);
  @ pure
  @*/
  static double limit(double command, double maxPower) {
    double m = Math.abs(maxPower);
    return Math.max(-m, Math.min(m, command));
  }

  // ---------------------------------------------------------------------------------------------
  // Output gate

  /** Whether power would drive the turret further past a travel limit it is already at. */
  /*@ normal_behavior
  @   ensures \result <==> (power > 0 && angle >= max) || (power < 0 && angle <= min);
  @ pure
  @*/
  static boolean pushesPastTravel(double power, double angle, double min, double max) {
    if (power == 0) {
      return false;
    }
    return (power > 0 && angle >= max) || (power < 0 && angle <= min);
  }

  /**
   * The power the servo is given for a requested power: zero unless the request is finite, the
   * turret is enabled and not faulted, its encoder reading is within {@code margin} of the travel,
   * and the power would not push past a travel limit; otherwise the request limited to +/-|{@code
   * maxPower}|. With no encoder, pass angle 0.
   *
   * <p>Every guarantee here holds for every input, NaN and infinities included.
   */
  /*@ normal_behavior
  @   ensures Double.isFinite(\result);
  @   ensures Math.abs(\result) <= Math.abs(maxPower) || \result == 0.0;
  @   ensures !Double.isFinite(requested) || !enabled || faulted
  @           || !readingTrusted(angle, min, max, margin) ==> \result == 0.0;
  @   ensures \result > 0 ==> requested > 0 && angle < max;
  @   ensures \result < 0 ==> requested < 0 && angle > min;
  @   ensures \result != 0.0 ==> readingTrusted(angle, min, max, margin);
  @   ensures Double.isFinite(requested) && enabled && !faulted
  @           && readingTrusted(angle, min, max, margin)
  @           && !pushesPastTravel(requested, angle, min, max)
  @           && Double.isFinite(maxPower) ==> \result == limit(requested, maxPower);
  @ pure
  @*/
  static double gate(
      double requested,
      double maxPower,
      boolean enabled,
      boolean faulted,
      double angle,
      double min,
      double max,
      double margin) {
    if (!Double.isFinite(requested)
        || !enabled
        || faulted
        || !readingTrusted(angle, min, max, margin)
        || pushesPastTravel(requested, angle, min, max)) {
      return 0.0;
    }
    double limited = limit(requested, maxPower);
    return Double.isFinite(limited) ? limited : 0.0;
  }
}
