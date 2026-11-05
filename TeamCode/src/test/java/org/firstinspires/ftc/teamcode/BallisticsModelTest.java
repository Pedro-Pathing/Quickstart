package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.firstinspires.ftc.teamcode.ballistics.BallisticsModel;
import org.firstinspires.ftc.teamcode.records.BallisticsParameters;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class BallisticsModelTest {

  private BallisticsParameters params;

  @BeforeEach
  public void setUp() {
    params =
        new BallisticsParameters(
            360.0, // v0 = 360 in/s (30 ft/s)
            0.0012, // k drag
            0.015, // L Magnus lift
            15.0, // launch height inches
            59.56, // target height inches (upward CELL opening)
            386.088, // g in/s2
            15.0, // min hood deg
            65.0, // max hood deg
            0.15, // min servo pos
            0.85, // max servo pos
            60.0, // max speed ips
            30.0, // min dist
            160.0, // max dist
            1.0, // lead bias gain
            1000.0, // v0 reference rpm
            1000.0, // preferred shot rpm
            1450.0, // max shot rpm
            0.12, // v0 margin fraction
            0.5, // flight time base sec
            0.004, // flight time sec per inch
            10.0, // lead min speed ips
            60.0 // max off-axis deg
            );
  }

  /**
   * The real hood linkage is reversed: servo 0.0 is the steepest arc, so the servo position for the
   * minimum angle is numerically the LARGER of the pair. An earlier unsigned span guard collapsed
   * that negative span to a divide-by-1e-6 and reported every steep angle as its shallow mirror,
   * which silently corrupted a whole calibration run's recorded hood angles.
   */
  @Test
  public void testReversedHoodLinkageMapping() {
    BallisticsParameters reversed =
        new BallisticsParameters(
            360.0, 0.0012, 0.015, 15.0, 45.0, 386.088, 15.0, // min hood deg -> flattest
            65.0, // max hood deg -> steepest
            0.85, // servo AT MIN ANGLE (numerically larger: reversed linkage)
            0.0, // servo AT MAX ANGLE
            60.0, 30.0, 160.0, 1.0, 1000.0, 1000.0, 1450.0, 0.12, 0.5, 0.004, 10.0, 60.0);

    assertEquals(
        15.0,
        BallisticsModel.servoPosToAngle(0.85, reversed),
        1e-6,
        "servo at the min-angle end must map back to the min angle");
    assertEquals(
        65.0,
        BallisticsModel.servoPosToAngle(0.0, reversed),
        1e-6,
        "servo 0.0 is the steepest arc, so it must map to the max angle");
    assertEquals(
        40.0,
        BallisticsModel.servoPosToAngle(0.425, reversed),
        1e-6,
        "midpoint of travel maps to the mid angle");

    // Steeper must mean a NUMERICALLY SMALLER servo position on this linkage.
    assertTrue(
        BallisticsModel.angleToServoPos(60.0, reversed)
            < BallisticsModel.angleToServoPos(20.0, reversed),
        "a steeper angle must command a smaller servo position when the linkage is reversed");

    // Round-trip through both directions.
    for (double deg : new double[] {15.0, 25.0, 40.0, 55.0, 65.0}) {
      double servo = BallisticsModel.angleToServoPos(deg, reversed);
      assertEquals(
          deg,
          BallisticsModel.servoPosToAngle(servo, reversed),
          1e-6,
          "angle->servo->angle must round-trip on a reversed linkage");
    }

    assertEquals(0.0, reversed.servoTravelLow(), 1e-9, "ordered travel low");
    assertEquals(0.85, reversed.servoTravelHigh(), 1e-9, "ordered travel high");
  }

  @Test
  public void testTrajectoryIntegrationStability() {
    BallisticsModel.TrajectoryResult result =
        BallisticsModel.integrateTrajectory(35.0, 72.0, params);

    assertTrue(result.reachedDistance(), "trajectory should reach target distance");
    assertTrue(result.flightTimeSec() > 0.0, "flight time should be positive");
    assertTrue(result.flightTimeSec() < 2.0, "flight time should be realistic (< 2.0s)");
  }

  @Test
  public void testLoftedElevationSolver() {
    BallisticsModel.SolvedElevation solved = BallisticsModel.solveLoftedElevation(84.0, params);

    assertNotNull(solved, "solved elevation should not be null");
    assertTrue(
        solved.elevationAngleDeg() >= params.minHoodAngleDeg()
            && solved.elevationAngleDeg() <= params.maxHoodAngleDeg(),
        "elevation angle should be within hood limits");

    assertTrue(
        solved.hoodServoPosition() >= params.servoTravelLow()
            && solved.hoodServoPosition() <= params.servoTravelHigh(),
        "hood servo position should be within servo bounds");

    assertTrue(solved.flightTimeSec() > 0.0, "flight time should be positive");
  }

  @Test
  public void testContinuousServoPositionMapping() {
    double minPos = BallisticsModel.angleToServoPos(params.minHoodAngleDeg(), params);
    double maxPos = BallisticsModel.angleToServoPos(params.maxHoodAngleDeg(), params);

    assertEquals(params.minHoodServoPos(), minPos, 1e-6);
    assertEquals(params.maxHoodServoPos(), maxPos, 1e-6);
  }

  @Test
  public void testSolveShotStaysWithinFlywheelRpmBounds() {
    for (double dist : new double[] {36.0, 60.0, 84.0, 120.0, 150.0}) {
      BallisticsModel.SolvedShot shot = BallisticsModel.solveShot(dist, params);

      double rpm = params.v0ToRpm(shot.exitVelocityIps());
      assertTrue(
          rpm >= params.preferredShotRpm() - 1e-6,
          "solved RPM at " + dist + " in must not fall below preferred_shot_rpm, was " + rpm);
      assertTrue(
          rpm <= params.maxShotRpm() + 1e-6,
          "solved RPM at " + dist + " in must not exceed max_shot_rpm, was " + rpm);
      assertTrue(
          shot.hoodServoPosition() >= params.servoTravelLow()
              && shot.hoodServoPosition() <= params.servoTravelHigh(),
          "hood servo position must stay within bounds at " + dist + " in");
    }
  }

  /**
   * Robot-representative parameters: v0 250 in/s at 1000 RPM, matching config.yaml. Weak enough
   * that the hood alone cannot cover the whole field, so both halves of the policy are exercised.
   */
  private static BallisticsParameters realisticParams() {
    return new BallisticsParameters(
        250.0, 0.0012, 0.015, 15.0, 45.0, 386.088, 15.0, 65.0, 0.0, 0.85, 60.0, 30.0, 150.0, 1.0,
        1000.0, 1000.0, 1450.0, 0.12, 0.5, 0.004, 10.0, 60.0);
  }

  @Test
  public void testHoodAloneCoversCloseRangeAtPreferredRpm() {
    // Regression guard: an earlier minimum-energy policy dropped a 48in shot to ~845 RPM, which
    // tested far too weak on the robot. Close and mid range must stay parked at the preferred
    // speed and let the hood do the aiming.
    BallisticsParameters p = realisticParams();

    for (double dist : new double[] {30.0, 48.0, 60.0, 72.0}) {
      double rpm = p.v0ToRpm(BallisticsModel.solveShot(dist, p).exitVelocityIps());
      assertEquals(
          p.preferredShotRpm(),
          rpm,
          1e-6,
          "hood should cover " + dist + " in at the preferred RPM, not slow the flywheel down");
    }
  }

  @Test
  public void testFlywheelRisesOnlyBeyondHoodAuthority() {
    BallisticsParameters p = realisticParams();

    double previous = 0.0;
    for (double dist : new double[] {30.0, 48.0, 72.0, 96.0, 120.0, 144.0}) {
      double rpm = p.v0ToRpm(BallisticsModel.solveShot(dist, p).exitVelocityIps());
      assertTrue(
          rpm >= p.preferredShotRpm() - 1e-6,
          "solved RPM must never drop below the preferred speed (was " + rpm + " at " + dist + ")");
      assertTrue(rpm >= previous - 1e-6, "solved RPM must not decrease with distance at " + dist);
      previous = rpm;
    }

    double farRpm = p.v0ToRpm(BallisticsModel.solveShot(144.0, p).exitVelocityIps());
    assertTrue(
        farRpm > p.preferredShotRpm() + 1.0,
        "a 144in shot is past the hood's authority at 1000 RPM and must raise the flywheel");
  }

  @Test
  public void testMarginKeepsLongShotsOffTheDegenerateLob() {
    // Right at the edge of a speed's range the only solution left is a near-vertical lob. Without
    // the margin, a 96in shot solved to a 63deg hood and 0.94s of hang time.
    BallisticsParameters p = realisticParams();

    BallisticsModel.SolvedShot shot = BallisticsModel.solveShot(96.0, p);

    assertTrue(
        shot.elevationAngleDeg() < 50.0,
        "96in solution should not degenerate into a lob, got " + shot.elevationAngleDeg() + " deg");
    assertTrue(
        shot.flightTimeSec() < 0.7,
        "96in flight time should stay well under a second, got " + shot.flightTimeSec());
  }

  @Test
  public void testSolveShotReachesTargetHeightAtSolvedSpeed() {
    for (double dist : new double[] {48.0, 96.0, 144.0}) {
      BallisticsModel.SolvedShot shot = BallisticsModel.solveShot(dist, params);
      assertTrue(shot.feasible(), "shot at " + dist + " in should be feasible");

      BallisticsModel.TrajectoryResult flown =
          BallisticsModel.integrateTrajectory(
              shot.elevationAngleDeg(), dist, params.withV0(shot.exitVelocityIps()));

      assertTrue(flown.reachedDistance(), "solved trajectory must reach the target distance");
      assertEquals(
          params.targetHeightInches(),
          flown.heightAtTargetInches(),
          1.0,
          "solved trajectory must arrive at the CELL opening height at " + dist + " in");
    }
  }

  @Test
  public void testSolveShotReportsInfeasibleBeyondFlywheelRange() {
    // Cap the flywheel far below what any real shot needs, so nothing is reachable.
    BallisticsParameters crippled =
        new BallisticsParameters(
            360.0, 0.0012, 0.015, 15.0, 45.0, 386.088, 15.0, 65.0, 0.15, 0.85, 60.0, 30.0, 160.0,
            1.0, 1000.0, 50.0, 60.0, 0.12, 0.5, 0.004, 10.0, 60.0);

    BallisticsModel.SolvedShot shot = BallisticsModel.solveShot(140.0, crippled);

    assertFalse(shot.feasible(), "unreachable target must report infeasible");
    assertTrue(
        shot.infeasibleReason().contains("RPM"), "infeasible reason should name the RPM ceiling");
  }

  @Test
  public void testLevenbergMarquardtParameterFitting() {
    java.util.List<double[]> points = new java.util.ArrayList<>();
    points.add(new double[] {48.0, 52.0});
    points.add(new double[] {72.0, 44.0});
    points.add(new double[] {96.0, 36.0});

    BallisticsModel.FittedBallistics fitted = BallisticsModel.fitParameters(points, params);

    assertNotNull(fitted, "fitted parameters should not be null");
    assertTrue(fitted.v0() > 0.0, "fitted v0 should be positive");
    assertTrue(fitted.k() > 0.0, "fitted k drag should be positive");
    assertTrue(fitted.magnusL() > 0.0, "fitted magnus L lift should be positive");
  }

  @Test
  public void testFitParametersWithVariableRpm() {
    BallisticsParameters trueParams =
        new BallisticsParameters(
            420.0, 0.0012, 0.045, 10.0, 30.0, 386.088, 10.0, 55.0, 0.0, 1.0, 60.0, 30.0, 160.0, 1.0,
            1400.0, 1400.0, 2200.0, 0.15, 0.5, 0.004, 10.0, 60.0);

    double refRpm = trueParams.v0ReferenceRpm();
    double[] distances = new double[] {48.0, 72.0, 96.0, 120.0, 144.0, 150.0};
    double[] rpms = new double[] {1400.0, 1400.0, 1400.0, 1400.0, 1600.0, 1700.0};

    java.util.List<double[]> variableRpmPoints = new java.util.ArrayList<>();
    for (int i = 0; i < distances.length; i++) {
      double dist = distances[i];
      double rpm = rpms[i];
      double exitV0 = trueParams.v0() * (rpm / refRpm);
      BallisticsModel.SolvedElevation solved =
          BallisticsModel.solveLoftedElevation(dist, trueParams.withV0(exitV0));
      variableRpmPoints.add(new double[] {dist, solved.elevationAngleDeg(), rpm});
    }

    BallisticsParameters initialGuess = trueParams.withV0(380.0).withDragAndLift(0.0020, 0.020);

    BallisticsModel.FittedBallistics fitted =
        BallisticsModel.fitParametersWithRpm(variableRpmPoints, initialGuess);

    assertNotNull(fitted, "fitted ballistics should not be null");
    assertTrue(
        fitted.residualRmsInches() < 0.1,
        "RMS residual error should be under 0.1 inches, was " + fitted.residualRmsInches());
    assertEquals(420.0, fitted.v0(), 4.2, "fitted v0 should recover true v0 within 1%");
  }
}
