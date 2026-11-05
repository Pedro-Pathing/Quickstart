package org.firstinspires.ftc.teamcode.utilities;

public final class AntiWindupIntegrator {

  /** Largest plausible loop period (seconds); longer gaps are treated as a scheduling stall. */
  static final double MAX_DT_SECONDS = 0.25;

  private double accumulator = 0.0;

  /** Clears the accumulated integral. Call when the mechanism stops or the setpoint jumps. */
  public void reset() {
    accumulator = 0.0;
  }

  /** Current accumulated error-seconds, before multiplication by {@code ki}. */
  public double getAccumulator() {
    return accumulator;
  }

  /**
   * Advances the integrator and returns its contribution to the motor command.
   *
   * @param error setpoint minus measurement, in ticks/s
   * @param dtSeconds loop period; non-positive or implausibly long values are skipped
   * @param ki integral gain; zero or negative disables the term entirely
   * @param band only integrate when {@code |error| <= band}; non-positive means always integrate
   * @param maxContribution bound on the absolute returned contribution, in power units
   * @param saturationSign +1 if the command is railed high, -1 if railed low, 0 if in range
   * @return {@code ki * accumulator}, clamped to {@code +/-maxContribution}
   */
  public double update(
      double error,
      double dtSeconds,
      double ki,
      double band,
      double maxContribution,
      int saturationSign) {

    if (!(ki > 0.0)) {
      accumulator = 0.0;
      return 0.0;
    }
    accumulator =
        nextAccumulator(accumulator, error, dtSeconds, ki, band, maxContribution, saturationSign);
    return contribution(accumulator, ki, maxContribution);
  }

  /** Bound on the accumulator (error-seconds): {@code maxContribution / ki}, or unbounded. */
  /*@ normal_behavior
  @   ensures !Double.isNaN(\result) && \result >= 0.0;
  @   ensures !(maxContribution > 0.0) ==> \result == Double.MAX_VALUE;
  @ pure
  @*/
  static double accumulatorLimit(double ki, double maxContribution) {
    double limit = maxContribution > 0.0 ? Math.abs(maxContribution / ki) : Double.MAX_VALUE;
    // inf / inf, or 0 / 0 from a zero gain, is NaN: treat as unbounded rather than propagate it.
    return Double.isNaN(limit) ? Double.MAX_VALUE : limit;
  }

  /** {@code v} limited to {@code [-limit, +limit]} without {@code Math.clamp}, which can throw. */
  /*@ normal_behavior
  @   requires !Double.isNaN(limit) && limit >= 0.0;
  @   ensures Double.isNaN(v) ==> Double.isNaN(\result);
  @   ensures !Double.isNaN(v) ==> -limit <= \result && \result <= limit;
  @   ensures -limit <= v && v <= limit ==> \result == v;
  @ pure
  @*/
  static double limitTo(double v, double limit) {
    return Math.max(-limit, Math.min(limit, v));
  }

  /** True when this loop's error may be added to the accumulator. */
  /*@ normal_behavior
  @   ensures \result ==> Double.isFinite(error);
  @   ensures \result ==> dtSeconds > 0.0 && dtSeconds <= MAX_DT_SECONDS;
  @   ensures \result ==> band <= 0.0 || Math.abs(error) <= band;
  @   ensures \result ==> !(saturationSign > 0 && error > 0.0);
  @   ensures \result ==> !(saturationSign < 0 && error < 0.0);
  @   ensures Double.isFinite(error) && dtSeconds > 0.0 && dtSeconds <= MAX_DT_SECONDS
  @           && (band <= 0.0 || Math.abs(error) <= band)
  @           && !(saturationSign > 0 && error > 0.0) && !(saturationSign < 0 && error < 0.0)
  @           ==> \result;
  @ pure
  @*/
  static boolean mayIntegrate(double error, double dtSeconds, double band, int saturationSign) {
    // A non-finite error would poison the accumulator for good; skip it like a stalled loop.
    boolean errorUsable = Double.isFinite(error);
    boolean dtUsable = dtSeconds > 0.0 && dtSeconds <= MAX_DT_SECONDS;
    boolean withinBand = band <= 0.0 || Math.abs(error) <= band;
    // Charging further into a rail cannot move the output, so freeze that direction.
    boolean pushingIntoRail =
        (saturationSign > 0 && error > 0.0) || (saturationSign < 0 && error < 0.0);
    return errorUsable && dtUsable && withinBand && !pushingIntoRail;
  }

  /**
   * The accumulator after one loop. A gain that is zero, negative or NaN clears it. A NaN
   * accumulator (never produced here) is treated as zero.
   */
  /*@ normal_behavior
  @   ensures !(ki > 0.0) ==> \result == 0.0;
  @   ensures ki > 0.0 ==> !Double.isNaN(\result)
  @           && -accumulatorLimit(ki, maxContribution) <= \result
  @           && \result <= accumulatorLimit(ki, maxContribution);
  @   ensures ki > 0.0 && !mayIntegrate(error, dtSeconds, band, saturationSign)
  @           ==> \result == limitTo(Double.isNaN(accumulator) ? 0.0 : accumulator,
  @                                  accumulatorLimit(ki, maxContribution));
  @   ensures ki > 0.0 && mayIntegrate(error, dtSeconds, band, saturationSign)
  @           ==> \result == limitTo((Double.isNaN(accumulator) ? 0.0 : accumulator)
  @                                  + error * dtSeconds, accumulatorLimit(ki, maxContribution));
  @   ensures ki > 0.0 && saturationSign > 0 && error > 0.0
  @           ==> \result == limitTo(Double.isNaN(accumulator) ? 0.0 : accumulator,
  @                                  accumulatorLimit(ki, maxContribution));
  @   ensures ki > 0.0 && saturationSign < 0 && error < 0.0
  @           ==> \result == limitTo(Double.isNaN(accumulator) ? 0.0 : accumulator,
  @                                  accumulatorLimit(ki, maxContribution));
  @ pure
  @*/
  static double nextAccumulator(
      double accumulator,
      double error,
      double dtSeconds,
      double ki,
      double band,
      double maxContribution,
      int saturationSign) {
    if (!(ki > 0.0)) {
      return 0.0;
    }
    double acc = Double.isNaN(accumulator) ? 0.0 : accumulator;
    if (mayIntegrate(error, dtSeconds, band, saturationSign)) {
      acc += error * dtSeconds;
    }
    // Infinity + finite stays infinite, but inf - inf cannot happen: error * dt is finite.
    return limitTo(acc, accumulatorLimit(ki, maxContribution));
  }

  /**
   * {@code ki * accumulator}, never beyond {@code +/-maxContribution} when that is positive, even
   * by the rounding of {@code ki * (maxContribution / ki)}. A non-positive or NaN gain, or a NaN
   * product, gives 0.
   */
  /*@ normal_behavior
  @   ensures !(ki > 0.0) ==> \result == 0.0;
  @   ensures !Double.isNaN(\result);
  @   ensures ki > 0.0 && maxContribution > 0.0
  @           ==> -maxContribution <= \result && \result <= maxContribution;
  @ pure
  @*/
  static double contribution(double accumulator, double ki, double maxContribution) {
    if (!(ki > 0.0)) {
      return 0.0;
    }
    double out = ki * accumulator;
    if (Double.isNaN(out)) { // NaN accumulator, or 0 * infinite gain
      return 0.0;
    }
    return maxContribution > 0.0 ? limitTo(out, maxContribution) : out;
  }
}
