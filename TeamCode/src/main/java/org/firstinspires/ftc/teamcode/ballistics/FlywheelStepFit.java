package org.firstinspires.ftc.teamcode.ballistics;

import androidx.annotation.Nullable;
import org.apache.commons.math3.exception.MathIllegalStateException;
import org.apache.commons.math3.fitting.leastsquares.LeastSquaresBuilder;
import org.apache.commons.math3.fitting.leastsquares.LeastSquaresOptimizer;
import org.apache.commons.math3.fitting.leastsquares.LevenbergMarquardtOptimizer;
import org.apache.commons.math3.fitting.leastsquares.MultivariateJacobianFunction;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.stat.descriptive.rank.Median;
import org.apache.commons.math3.stat.regression.SimpleRegression;
import org.apache.commons.math3.util.Pair;

/**
 * The flywheel's time constant and sensor lag from one open-loop power step. A first-order plant
 * with dead time L has normalized response {@code y = (v - v0) / (vFinal - v0) = 1 - exp(-(t - L) /
 * tau)} after L, so {@code ln(1 - y) = L / tau - t / tau} is a line in t: a least-squares fit over
 * every sample between {@link #FIT_LOW} and {@link #FIT_HIGH} of the step gives tau from the slope
 * and L from the intercept. That starts a Levenberg-Marquardt fit of the response itself over the
 * first {@link #REFINE_TAUS} time constants, which the log's noise amplification does not bias: on
 * simulated steps with 3% encoder noise the line alone missed the lag by up to 15 ms, the refined
 * fit by 6 ms. A two-point (28% / 63% crossing) fit missed tau by 10% or more.
 *
 * <p>The dead time is everything between the power write and the reading moving: the hub write,
 * motor current rise and the hub's velocity window. That is exactly what the recovery kick ({@code
 * shooter.kick.sensor_lag_sec}) runs its model forward over. Pure math, unit tested on the JVM.
 */
public final class FlywheelStepFit {
  private FlywheelStepFit() {}

  /** Smallest step (ticks/s) worth fitting: below this, encoder noise is the response. */
  public static final double MIN_STEP_TICKS = 50.0;

  /**
   * Fit window, as fractions of the step. Below it the dead time and noise dominate; above it ln(1
   * - y) amplifies noise without bound.
   */
  static final double FIT_LOW = 0.20;

  static final double FIT_HIGH = 0.70;

  /** A crossing must hold for this many further samples to count. */
  static final int CONFIRM_SAMPLES = 2;

  /** Fewest samples in the window for a fit. */
  static final int MIN_SAMPLES = 5;

  /** The refined fit covers the step from the power write to this many first-guess taus. */
  static final double REFINE_TAUS = 3.0;

  /** Readings this many robust standard deviations off the first guess are left out. */
  static final double OUTLIER_SIGMAS = 5.0;

  /** The first-order-plus-dead-time step response at time t. */
  static double response(double t, double v0, double span, double tau, double lag) {
    return t <= lag ? v0 : v0 + span * (1.0 - Math.exp(-(t - lag) / tau));
  }

  /**
   * @param tauSec time constant, NaN when the fit failed
   * @param lagSec dead time, at least 0, NaN when the fit failed
   * @param problem why the fit failed, null when it did not
   */
  public record Result(double tauSec, double lagSec, @Nullable String problem) {
    public boolean valid() {
      return problem == null;
    }
  }

  /**
   * Fits a step recorded from time 0 (the power write).
   *
   * @param timesSec sample times, increasing, from the power write
   * @param velocities readings (ticks/s) at those times
   * @param v0 settled speed before the step
   * @param vFinal settled speed after it
   */
  public static Result fit(double[] timesSec, double[] velocities, double v0, double vFinal) {
    if (timesSec.length != velocities.length) {
      return failed("times and velocities differ in length");
    }
    double span = vFinal - v0;
    if (!Double.isFinite(span) || Math.abs(span) < MIN_STEP_TICKS) {
      return failed("step too small to fit: " + span + " ticks/s");
    }
    // Pick the window by time, from confirmed crossings, then fit every sample inside it. Choosing
    // samples by their own noisy value would keep the high-noise ones near FIT_LOW and drop the
    // low ones, so the step would seem to start early and the lag come out short.
    int first = confirmedCrossing(velocities, v0, span, FIT_LOW);
    int last = confirmedCrossing(velocities, v0, span, FIT_HIGH);
    SimpleRegression line = new SimpleRegression();
    if (first >= 0) {
      int end = last >= 0 ? last : velocities.length;
      for (int i = first; i < end; i++) {
        double y = (velocities[i] - v0) / span;
        if (Double.isFinite(timesSec[i]) && y < 1.0) {
          line.addData(timesSec[i], Math.log(1.0 - y));
        }
      }
    }
    if (line.getN() < MIN_SAMPLES) {
      return failed(
          "only "
              + line.getN()
              + " samples between 20% and 70% of the step: hold it longer or"
              + " sample faster");
    }
    double slope = line.getSlope();
    if (!(slope < 0.0)) {
      return failed("the reading did not rise toward the step's final speed");
    }
    double tau = -1.0 / slope;
    double lag = Math.max(0.0, line.getIntercept() * tau);
    return refine(timesSec, velocities, v0, span, tau, lag);
  }

  /**
   * Least-squares fit of {@code v = v0 + span (1 - exp(-(t - lag) / tau))} (v0 before lag) from the
   * line's estimate, over samples up to lag + REFINE_TAUS tau. Falls back to the estimate if the
   * optimizer does not converge or leaves the physical range.
   */
  private static Result refine(
      double[] t, double[] v, double v0, double span, double tau0, double lag0) {
    double end = lag0 + REFINE_TAUS * tau0;
    int n = 0;
    while (n < t.length && t[n] <= end) {
      n++;
    }
    if (n < MIN_SAMPLES) {
      return new Result(tau0, lag0, null);
    }
    // Drop outliers (a glitched reading) against the line's model: more than OUTLIER_SIGMAS robust
    // standard deviations (1.4826 x the median absolute residual) off. Squared error lets one
    // full-step spike move tau by 7%.
    double[] residuals = new double[n];
    for (int i = 0; i < n; i++) {
      residuals[i] = Math.abs(v[i] - response(t[i], v0, span, tau0, lag0));
    }
    double sigma = 1.4826 * new Median().evaluate(residuals);
    double cut = OUTLIER_SIGMAS * sigma + 1.0;
    int kept = 0;
    double[] times = new double[n];
    double[] observed = new double[n];
    for (int i = 0; i < n; i++) {
      if (residuals[i] <= cut) {
        times[kept] = t[i];
        observed[kept] = v[i];
        kept++;
      }
    }
    if (kept < MIN_SAMPLES) {
      return new Result(tau0, lag0, null);
    }
    times = java.util.Arrays.copyOf(times, kept);
    observed = java.util.Arrays.copyOf(observed, kept);
    double[] fitTimes = times;
    MultivariateJacobianFunction model =
        point -> {
          double tau = point.getEntry(0);
          double lag = point.getEntry(1);
          RealVector value = new ArrayRealVector(fitTimes.length);
          Array2DRowRealMatrix jacobian = new Array2DRowRealMatrix(fitTimes.length, 2);
          for (int i = 0; i < fitTimes.length; i++) {
            double x = Math.max(0.0, fitTimes[i] - lag);
            double e = Math.exp(-x / tau);
            value.setEntry(i, v0 + span * (1.0 - e));
            if (fitTimes[i] > lag) {
              jacobian.setEntry(i, 0, -span * e * x / (tau * tau));
              jacobian.setEntry(i, 1, -span * e / tau);
            }
          }
          return new Pair<>(value, jacobian);
        };
    try {
      LeastSquaresOptimizer.Optimum optimum =
          new LevenbergMarquardtOptimizer()
              .optimize(
                  new LeastSquaresBuilder()
                      .start(new double[] {tau0, lag0})
                      .model(model)
                      .target(observed)
                      .maxEvaluations(200)
                      .maxIterations(200)
                      .build());
      double tau = optimum.getPoint().getEntry(0);
      double lag = optimum.getPoint().getEntry(1);
      if (!(tau > 0.0) || !Double.isFinite(lag) || !(lag < end)) {
        return new Result(tau0, lag0, null);
      }
      return new Result(tau, Math.max(0.0, lag), null);
    } catch (MathIllegalStateException e) {
      return new Result(tau0, lag0, null);
    }
  }

  /**
   * First index where the normalized response is at least {@code level} and stays so for {@link
   * #CONFIRM_SAMPLES} more samples (so one noise spike cannot fake it); -1 if never.
   */
  static int confirmedCrossing(double[] v, double v0, double span, double level) {
    outer:
    for (int i = 0; i + CONFIRM_SAMPLES < v.length; i++) {
      for (int k = i; k <= i + CONFIRM_SAMPLES; k++) {
        if (!((v[k] - v0) / span >= level)) {
          continue outer;
        }
      }
      return i;
    }
    return -1;
  }

  private static Result failed(String why) {
    return new Result(Double.NaN, Double.NaN, why);
  }
}
