package org.firstinspires.ftc.teamcode.utilities;

/**
 * The flywheel's recovery kick: after a ball hit (or on spin-up), the most power the wheel's own
 * model allows without passing the feed band's floor in the next loop, until that is no more than
 * the normal loop would ask for; then the normal loop again.
 *
 * <p>The model is the one the feedforward already uses (a wheel at nominal-voltage power u heads
 * for (u - kS) / kV ticks/s), corrected by what the loop's integrator has learned about the
 * feedforward's error, plus the wheel's time constant tau. Without that correction a wheel 30%
 * easier to spin than calibrated was predicted slow, kicked a loop too long and overshot to 1.13x.
 * The velocity reading lags the wheel, so the kick runs the model forward over the powers sent
 * during that lag (a Smith predictor) and decides on the wheel's speed now.
 *
 * <p>Each loop it asks for the power that would bring a wheel {@code speedMargin} times faster than
 * the model exactly to target by the next loop, capped at full power: far below target that is full
 * power, and near it just enough. It stops once the predicted speed reaches {@code stopRatio} of
 * target or the normal loop asks for as much. Ending on a whole loop of full power overshot to
 * 1.10x on a fast wheel, where one such loop is a fifth of the target speed; aiming at the stop
 * itself never got there. While kicking the model runs over the sensor lag plus {@code
 * lagMarginSec}: overestimating the lag of an accelerating wheel predicts it faster, so the kick
 * errs toward stopping early. A mis-measured tau or lag then costs a little speed, not an
 * overshoot. Nothing here is a gain: kS, kV, tau, the lag and their measurement errors are
 * measured.
 *
 * <p>The decisions are the static methods; this object only remembers the powers sent and whether a
 * kick is under way.
 */
public final class FlywheelKick {
  /** Powers remembered: covers sensor_lag_sec's 0.3 s maximum at a 10 ms loop. */
  static final int HISTORY = 32;

  private final double[] powers = new double[HISTORY];
  private final double[] durations = new double[HISTORY];
  private int count = 0;
  private int newest = -1;
  private boolean kicking = false;

  /** Forgets the powers sent and ends any kick (the flywheel stopped or the loop restarted). */
  public void reset() {
    count = 0;
    newest = -1;
    kicking = false;
  }

  public boolean isKicking() {
    return kicking;
  }

  /**
   * Records that nominal-voltage power {@code power} was applied for {@code durationSec}. Ignored
   * unless both are finite and the duration positive.
   */
  public void record(double power, double durationSec) {
    if (!Double.isFinite(power) || !(durationSec > 0.0) || !Double.isFinite(durationSec)) {
      return;
    }
    newest = (newest + 1) % HISTORY;
    powers[newest] = power;
    durations[newest] = durationSec;
    count = Math.min(count + 1, HISTORY);
  }

  /**
   * Decides this loop and remembers the outcome.
   *
   * @param measured velocity reading (ticks/s), {@code lagSec} old
   * @param target target velocity (ticks/s)
   * @param fullPower the nominal-voltage power full output delivers on this battery
   * @param normalPower the nominal-voltage power the normal loop would deliver this loop
   * @param dtSec expected length of this loop (the last loop's)
   * @param lagMarginSec how far {@code lagSec} may be short; added to it while kicking
   * @param ks the wheel's power offset: configured kS plus the integrator's learned correction
   * @return the nominal-voltage power to deliver, or NaN to leave this loop to the normal loop
   */
  public double update(
      double measured,
      double target,
      double fullPower,
      double normalPower,
      double dtSec,
      double tauSec,
      double lagSec,
      double lagMarginSec,
      double ks,
      double kv,
      double triggerFraction,
      double stopRatio,
      double speedMargin) {
    double margin = kicking && lagMarginSec > 0.0 ? lagMarginSec : 0.0;
    double predicted = predictNow(measured, lagSec + margin, tauSec, ks, kv);
    double power =
        kickPower(
            kicking,
            measured,
            predicted,
            target,
            fullPower,
            normalPower,
            dtSec,
            tauSec,
            ks,
            kv,
            triggerFraction,
            stopRatio,
            speedMargin);
    kicking = !Double.isNaN(power);
    return power;
  }

  /**
   * The wheel's speed now: {@code measured} run forward through the powers sent in the last {@code
   * lagSec} (a Smith predictor). The reading itself for no lag, no history or a non-positive tau.
   */
  public double predictNow(double measured, double lagSec, double tauSec, double ks, double kv) {
    if (!(lagSec > 0.0) || !(tauSec > 0.0) || count == 0) {
      return measured;
    }
    // Walk back from the newest power to find where the lag window starts, then replay forward.
    double remaining = lagSec;
    int used = 0;
    double firstPart = 0.0;
    while (used < count && remaining > 0.0) {
      int idx = Math.floorMod(newest - used, HISTORY);
      firstPart = Math.min(durations[idx], remaining);
      remaining -= firstPart;
      used++;
    }
    double speed = measured;
    for (int k = used - 1; k >= 0; k--) {
      int idx = Math.floorMod(newest - k, HISTORY);
      double dt = k == used - 1 ? firstPart : durations[idx];
      speed = step(speed, powers[idx], dt, tauSec, ks, kv);
    }
    return speed;
  }

  /**
   * Steady speed (ticks/s) the feedforward model gives for nominal-voltage power {@code u}: (|u| -
   * kS) / kV with u's sign, and zero inside a positive kS (static friction). kS here is the power
   * offset the wheel actually needs, the configured kS plus what the integrator learned, so it can
   * come out negative (no dead zone). Zero for a non-positive or non-finite kV or a non-finite u or
   * kS.
   */
  public static double steadySpeed(double u, double ks, double kv) {
    if (!(kv > 0.0) || !Double.isFinite(kv) || !Double.isFinite(u) || !Double.isFinite(ks)) {
      return 0.0;
    }
    double drive = Math.abs(u) - ks;
    return drive <= 0.0 ? 0.0 : Math.copySign(drive, u) / kv;
  }

  /**
   * One first-order step: {@code speed} moves toward the steady speed of {@code u} by the fraction
   * dt / tau of the gap (all of it once dt reaches tau, as {@code FlywheelSim} integrates). Returns
   * {@code speed} unchanged for a non-positive tau or dt.
   */
  public static double step(
      double speed, double u, double dtSec, double tauSec, double ks, double kv) {
    if (!(tauSec > 0.0) || !(dtSec > 0.0)) {
      return speed;
    }
    double gain = Math.min(1.0, dtSec / tauSec);
    return speed + (steadySpeed(u, ks, kv) - speed) * gain;
  }

  /**
   * The kick's power this loop, or NaN for none. A kick starts once the reading is {@code
   * triggerFraction} below a positive target. While armed and the predicted speed is under {@code
   * stopRatio} of target, it is the power that takes a wheel {@code speedMargin} (at least 1) times
   * faster than the model exactly to target in one loop, capped at {@code fullPower}; the kick ends
   * (NaN) once that is no more than {@code normalPower}. Never kicks without a positive finite
   * target, tau, dt and kV, or on a non-finite input.
   */
  public static double kickPower(
      boolean kicking,
      double measured,
      double predicted,
      double target,
      double fullPower,
      double normalPower,
      double dtSec,
      double tauSec,
      double ks,
      double kv,
      double triggerFraction,
      double stopRatio,
      double speedMargin) {
    if (!(target > 0.0) || !(tauSec > 0.0) || !(dtSec > 0.0) || !(kv > 0.0)) {
      return Double.NaN;
    }
    if (!Double.isFinite(measured)
        || !Double.isFinite(predicted)
        || !Double.isFinite(target)
        || !Double.isFinite(fullPower)
        || !Double.isFinite(normalPower)
        || !Double.isFinite(ks)
        || !Double.isFinite(kv)) {
      return Double.NaN;
    }
    boolean armed = kicking || measured < (1.0 - triggerFraction) * target;
    double stop = stopRatio * target;
    if (!armed || !(predicted < stop)) {
      return Double.NaN;
    }
    // Speed whose pull, over one loop on a wheel margin x faster, lands exactly on target.
    double gain = Math.min(1.0, dtSec / tauSec) * Math.max(1.0, speedMargin);
    double aim = predicted + (target - predicted) / gain;
    double power = Math.min(fullPower, ks + kv * aim);
    return power > normalPower ? power : Double.NaN;
  }
}
