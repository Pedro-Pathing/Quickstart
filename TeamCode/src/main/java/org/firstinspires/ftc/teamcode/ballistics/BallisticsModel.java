package org.firstinspires.ftc.teamcode.ballistics;

import androidx.annotation.Nullable;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.solvers.BrentSolver;
import org.apache.commons.math3.fitting.leastsquares.LeastSquaresFactory;
import org.apache.commons.math3.fitting.leastsquares.LeastSquaresOptimizer;
import org.apache.commons.math3.fitting.leastsquares.LeastSquaresProblem;
import org.apache.commons.math3.fitting.leastsquares.LevenbergMarquardtOptimizer;
import org.apache.commons.math3.fitting.leastsquares.MultivariateJacobianFunction;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.apache.commons.math3.ode.FirstOrderDifferentialEquations;
import org.apache.commons.math3.ode.events.EventHandler;
import org.apache.commons.math3.ode.nonstiff.ClassicalRungeKuttaIntegrator;
import org.apache.commons.math3.util.Pair;
import org.firstinspires.ftc.teamcode.records.BallisticsParameters;

/**
 * 2D point-mass trajectory integrator in the vertical plane using Apache Commons Math 3. Includes
 * quadratic air drag and signed backspin Magnus lift.
 */
public class BallisticsModel {

  public record TrajectoryResult(
      double flightTimeSec,
      double heightAtTargetInches,
      double entryAngleRad,
      boolean reachedDistance) {}

  public record SolvedElevation(
      double elevationAngleDeg,
      double hoodServoPosition,
      double flightTimeSec,
      double entryAngleRad) {}

  public record FittedBallistics(double v0, double k, double magnusL, double residualRmsInches) {}

  /** A full shot: the exit speed to launch at and the hood elevation to launch it at. */
  public record SolvedShot(
      double exitVelocityIps,
      double elevationAngleDeg,
      double hoodServoPosition,
      double flightTimeSec,
      double entryAngleRad,
      boolean feasible,
      String infeasibleReason) {}

  /** Angle step (deg) for the reachability scan inside the exit-speed search. */
  private static final double FEASIBILITY_SCAN_STEP_DEG = 5.0;

  /** Bisection steps for the minimum-exit-speed search. 12 resolves a 250 in/s span to ~0.06. */
  private static final int V0_SEARCH_ITERATIONS = 12;

  private static final double MAX_FLIGHT_TIME_SEC = 3.0;
  private static final double GROUND_HEIGHT_INCHES = -20.0;

  public static TrajectoryResult integrateTrajectory(
      double elevationAngleDeg, double targetDistanceInches, BallisticsParameters params) {
    double thetaRad = Math.toRadians(elevationAngleDeg);
    double vx0 = params.v0() * Math.cos(thetaRad);
    double vz0 = params.v0() * Math.sin(thetaRad);

    double[] y0 = new double[] {0.0, params.launchHeightInches(), vx0, vz0};
    double[] yFinal = new double[4];

    WiffleTrajectoryODE ode =
        new WiffleTrajectoryODE(params.k(), params.L(), params.gInchesPerSec2());
    ClassicalRungeKuttaIntegrator integrator = new ClassicalRungeKuttaIntegrator(0.002);

    CrossingEventHandler distanceHandler = new CrossingEventHandler(targetDistanceInches, 0);
    CrossingEventHandler groundHandler = new CrossingEventHandler(GROUND_HEIGHT_INCHES, 1);
    integrator.addEventHandler(distanceHandler, Double.POSITIVE_INFINITY, 1e-6, 100);
    integrator.addEventHandler(groundHandler, Double.POSITIVE_INFINITY, 1e-6, 100);

    integrator.integrate(ode, 0.0, y0, MAX_FLIGHT_TIME_SEC, yFinal);

    double entryAngle = Math.atan2(yFinal[3], yFinal[2]);
    if (distanceHandler.fired) {
      return new TrajectoryResult(distanceHandler.eventTime, yFinal[1], entryAngle, true);
    }

    double finalTime = groundHandler.fired ? groundHandler.eventTime : MAX_FLIGHT_TIME_SEC;
    return new TrajectoryResult(finalTime, yFinal[1], entryAngle, false);
  }

  private static class CrossingEventHandler implements EventHandler {
    private final double level;
    private final int component;
    boolean fired = false;
    double eventTime = Double.NaN;

    CrossingEventHandler(double level, int component) {
      this.level = level;
      this.component = component;
    }

    @Override
    public void init(double t0, double[] y0, double t) {
      fired = false;
      eventTime = Double.NaN;
    }

    @Override
    public double g(double t, double[] y) {
      return y[component] - level;
    }

    @Override
    public Action eventOccurred(double t, double[] y, boolean increasing) {
      fired = true;
      eventTime = t;
      return Action.STOP;
    }

    @Override
    public void resetState(double t, double[] y) {}
  }

  public static SolvedElevation solveLoftedElevation(
      double targetDistanceInches, BallisticsParameters params) {
    double minAngle = params.minHoodAngleDeg();
    double maxAngle = params.maxHoodAngleDeg();

    UnivariateFunction heightErrorFunc =
        angleDeg -> {
          TrajectoryResult res = integrateTrajectory(angleDeg, targetDistanceInches, params);
          if (!res.reachedDistance()) {
            return -1000.0 - (targetDistanceInches - res.heightAtTargetInches());
          }
          return res.heightAtTargetInches() - params.targetHeightInches();
        };

    double step = 1.0;
    double solvedAngle = Double.NaN;
    BrentSolver solver = new BrentSolver(1e-4);

    double bestAngle = maxAngle;
    double minAbsErr = Double.MAX_VALUE;

    double prevAngle = maxAngle;
    double f1 = heightErrorFunc.value(prevAngle);
    if (Math.abs(f1) < minAbsErr) {
      minAbsErr = Math.abs(f1);
      bestAngle = prevAngle;
    }

    for (double a = maxAngle - step; a >= minAngle; a -= step) {
      double f2 = heightErrorFunc.value(a);
      if (Math.abs(f2) < minAbsErr) {
        minAbsErr = Math.abs(f2);
        bestAngle = a;
      }
      if (f1 * f2 <= 0) {
        try {
          solvedAngle = solver.solve(100, heightErrorFunc, a, prevAngle);
          break;
        } catch (Exception ignored) {
        }
      }
      prevAngle = a;
      f1 = f2;
    }

    if (Double.isNaN(solvedAngle)) {
      solvedAngle = bestAngle;
    }

    TrajectoryResult result = integrateTrajectory(solvedAngle, targetDistanceInches, params);
    double servoPos = angleToServoPos(solvedAngle, params);

    double validFlightTime = result.reachedDistance() ? result.flightTimeSec() : -1.0;

    return new SolvedElevation(solvedAngle, servoPos, validFlightTime, result.entryAngleRad());
  }

  private static boolean canReachTarget(double targetDistanceInches, BallisticsParameters params) {
    for (double a = params.maxHoodAngleDeg();
        a >= params.minHoodAngleDeg();
        a -= FEASIBILITY_SCAN_STEP_DEG) {
      TrajectoryResult res = integrateTrajectory(a, targetDistanceInches, params);
      if (res.reachedDistance() && res.heightAtTargetInches() >= params.targetHeightInches()) {
        return true;
      }
    }
    return false;
  }

  public static SolvedShot solveShot(double targetDistanceInches, BallisticsParameters params) {
    double preferredV0 = params.rpmToV0(params.preferredShotRpm());
    double maxV0 = params.rpmToV0(params.maxShotRpm());
    double margin = 1.0 + params.v0MarginFraction();

    if (canReachTarget(targetDistanceInches, params.withV0(preferredV0 / margin))) {
      return shotAt(preferredV0, targetDistanceInches, params, true, "Valid");
    }

    if (!canReachTarget(targetDistanceInches, params.withV0(maxV0))) {
      return shotAt(
          maxV0,
          targetDistanceInches,
          params,
          false,
          String.format(
              Locale.ROOT,
              "Distance %.1f in unreachable at max %.0f RPM",
              targetDistanceInches,
              params.maxShotRpm()));
    }

    double lo = preferredV0 / margin;
    double hi = maxV0;
    for (int i = 0; i < V0_SEARCH_ITERATIONS; i++) {
      double mid = 0.5 * (lo + hi);
      if (canReachTarget(targetDistanceInches, params.withV0(mid))) {
        hi = mid;
      } else {
        lo = mid;
      }
    }

    double targetV0 = Math.clamp(hi * margin, preferredV0, maxV0);
    return shotAt(targetV0, targetDistanceInches, params, true, "Valid");
  }

  /** Solves the lofted hood elevation at a fixed exit speed and packages the result. */
  private static SolvedShot shotAt(
      double exitVelocityIps,
      double targetDistanceInches,
      BallisticsParameters params,
      boolean feasible,
      String reason) {
    SolvedElevation elevation =
        solveLoftedElevation(targetDistanceInches, params.withV0(exitVelocityIps));
    return new SolvedShot(
        exitVelocityIps,
        elevation.elevationAngleDeg(),
        elevation.hoodServoPosition(),
        elevation.flightTimeSec(),
        elevation.entryAngleRad(),
        feasible,
        reason);
  }

  /** Converts elevation angle (degrees) to continuous hood servo position (0.0 to 1.0). */
  public static double angleToServoPos(double elevationAngleDeg, BallisticsParameters params) {
    double minA = params.minHoodAngleDeg();
    double maxA = params.maxHoodAngleDeg();
    double minP = params.minHoodServoPos();
    double maxP = params.maxHoodServoPos();

    double norm = (elevationAngleDeg - minA) / Math.max(1e-6, maxA - minA);
    double clampedNorm = Math.clamp(norm, 0.0, 1.0);
    return minP + clampedNorm * (maxP - minP);
  }

  /**
   * Converts continuous hood servo position (0.0 to 1.0) to elevation angle (degrees).
   *
   * <p>Handles a reversed linkage, where the servo position for the minimum angle is numerically
   * greater than the one for the maximum angle. The span is used signed for exactly that reason: an
   * unsigned {@code Math.max(1e-6, maxP - minP)} guard silently turned a negative span into a
   * divide-by-1e-6 and reported every steep hood angle as its shallow mirror image.
   */
  public static double servoPosToAngle(double servoPos, BallisticsParameters params) {
    double minA = params.minHoodAngleDeg();
    double maxA = params.maxHoodAngleDeg();
    double span = params.maxHoodServoPos() - params.minHoodServoPos();

    double norm = Math.abs(span) < 1e-9 ? 0.0 : (servoPos - params.minHoodServoPos()) / span;
    double clampedNorm = Math.clamp(norm, 0.0, 1.0);
    return minA + clampedNorm * (maxA - minA);
  }

  /**
   * Fits (v0, k, L) parameters to a set of accepted distance and elevation angle trial points using
   * Apache Commons Math 3 LevenbergMarquardtOptimizer.
   *
   * <p>Assumes every point was shot at {@code initialParams.v0ReferenceRpm()}. Use {@link
   * #fitParametersWithRpm} for calibration runs where the RPM solver had to raise the flywheel
   * above the reference speed to reach some of the distances.
   */
  public static FittedBallistics fitParameters(
      @Nullable List<double[]> distanceAndElevationPoints, BallisticsParameters initialParams) {
    if (distanceAndElevationPoints == null) {
      return fitParametersWithRpm(null, initialParams);
    }
    List<double[]> atReferenceRpm = new ArrayList<>(distanceAndElevationPoints.size());
    for (double[] p : distanceAndElevationPoints) {
      atReferenceRpm.add(new double[] {p[0], p[1], initialParams.v0ReferenceRpm()});
    }
    return fitParametersWithRpm(atReferenceRpm, initialParams);
  }

  /**
   * As {@link #fitParameters}, but each point also carries the flywheel RPM it was actually shot
   * at: {@code {distanceInches, elevationDeg, rpm}}. A calibration point taken with the flywheel
   * raised above {@code v0ReferenceRpm} (the RPM solver's response to a distance the reference
   * speed alone could not loft to the CELL opening height) has a correspondingly higher true exit
   * speed; fixing every point to one exit speed regardless of what RPM it was actually shot at
   * would corrupt the fit for exactly the long-range points that needed the RPM solver in the first
   * place. The fitted free parameter {@code v0} remains the exit speed at {@code v0ReferenceRpm} —
   * each point's effective exit speed is {@code v0 * (pointRpm / v0ReferenceRpm)}.
   */
  public static FittedBallistics fitParametersWithRpm(
      @Nullable List<double[]> distanceAngleRpmPoints, BallisticsParameters initialParams) {
    if (distanceAngleRpmPoints == null || distanceAngleRpmPoints.size() < 3) {
      return new FittedBallistics(initialParams.v0(), initialParams.k(), initialParams.L(), 0.0);
    }

    int n = distanceAngleRpmPoints.size();
    double[] targetHeights = new double[n];
    Arrays.fill(targetHeights, initialParams.targetHeightInches());
    double refRpm = initialParams.v0ReferenceRpm();

    MultivariateJacobianFunction modelJacobian =
        point -> {
          double v0 = Math.max(10.0, point.getEntry(0));
          double k = Math.max(1e-6, point.getEntry(1));
          double L = Math.max(0.0, point.getEntry(2));

          BallisticsParameters testParams = initialParams.withV0(v0).withDragAndLift(k, L);

          RealVector value = new ArrayRealVector(n);
          RealMatrix jacobian = new Array2DRowRealMatrix(n, 3);

          double h = 1e-4;
          for (int i = 0; i < n; i++) {
            double dist = distanceAngleRpmPoints.get(i)[0];
            double angle = distanceAngleRpmPoints.get(i)[1];
            double rpmScale = distanceAngleRpmPoints.get(i)[2] / refRpm;

            TrajectoryResult res =
                integrateTrajectory(angle, dist, testParams.withV0(v0 * rpmScale));
            value.setEntry(i, res.heightAtTargetInches());

            BallisticsParameters pV = testParams.withV0(Math.max(10.0, v0 + h) * rpmScale);
            BallisticsParameters pK =
                testParams.withDragAndLift(Math.max(1e-6, k + h), L).withV0(v0 * rpmScale);
            BallisticsParameters pL =
                testParams.withDragAndLift(k, Math.max(0.0, L + h)).withV0(v0 * rpmScale);

            double resV = integrateTrajectory(angle, dist, pV).heightAtTargetInches();
            double resK = integrateTrajectory(angle, dist, pK).heightAtTargetInches();
            double resL = integrateTrajectory(angle, dist, pL).heightAtTargetInches();

            jacobian.setEntry(i, 0, (resV - res.heightAtTargetInches()) / h);
            jacobian.setEntry(i, 1, (resK - res.heightAtTargetInches()) / h);
            jacobian.setEntry(i, 2, (resL - res.heightAtTargetInches()) / h);
          }
          return new Pair<>(value, jacobian);
        };

    LeastSquaresProblem problem =
        LeastSquaresFactory.create(
            modelJacobian,
            new ArrayRealVector(targetHeights),
            new ArrayRealVector(
                new double[] {initialParams.v0(), initialParams.k(), initialParams.L()}),
            null,
            100,
            100);

    LevenbergMarquardtOptimizer optimizer = new LevenbergMarquardtOptimizer();
    LeastSquaresOptimizer.Optimum optimum = optimizer.optimize(problem);

    RealVector fit = optimum.getPoint();
    double rms = optimum.getRMS();

    return new FittedBallistics(
        Math.max(100.0, fit.getEntry(0)),
        Math.max(0.0001, fit.getEntry(1)),
        Math.max(0.001, fit.getEntry(2)),
        rms);
  }

  /**
   * ODE implementation for 2D wiffle ball trajectory with quadratic drag and backspin Magnus lift.
   */
  private static class WiffleTrajectoryODE implements FirstOrderDifferentialEquations {
    private final double k;
    private final double L;
    private final double g;

    public WiffleTrajectoryODE(double k, double L, double g) {
      this.k = k;
      this.L = L;
      this.g = g;
    }

    @Override
    public int getDimension() {
      return 4;
    }

    @Override
    public void computeDerivatives(double t, double[] y, double[] yDot) {
      double vx = y[2];
      double vz = y[3];
      double speed = Math.hypot(vx, vz);

      yDot[0] = vx;
      yDot[1] = vz;
      yDot[2] = -k * speed * vx - L * vz;
      yDot[3] = -g - k * speed * vz + L * vx;
    }
  }
}
