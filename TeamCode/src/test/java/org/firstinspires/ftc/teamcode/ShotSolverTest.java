package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ballistics.ShotSolver;
import org.firstinspires.ftc.teamcode.ballistics.ShotTable;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.BallType;
import org.firstinspires.ftc.teamcode.records.BallisticsParameters;
import org.firstinspires.ftc.teamcode.records.CellTarget;
import org.firstinspires.ftc.teamcode.records.HiveCell;
import org.firstinspires.ftc.teamcode.records.ShotInputs;
import org.firstinspires.ftc.teamcode.records.ShotSolution;
import org.firstinspires.ftc.teamcode.robot.ShotController;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ShotSolverTest {

  private static final int EPOCH = 7;
  private static final Velocity STILL = Velocity.zero();

  private BallisticsParameters params;
  private ShotTable table;
  private ShotTable wideTable;
  private FieldModel red;
  private CellTarget redA;

  @BeforeEach
  public void setUp() {
    params =
        new BallisticsParameters(
            360.0, // v0 = 360 in/s
            0.0012, // k drag
            0.015, // L Magnus lift
            15.0, // launch height
            59.56, // upward CELL opening height
            386.088, // g
            15.0, // min hood deg
            65.0, // max hood deg
            0.15, // min servo pos
            0.85, // max servo pos
            60.0, // max speed ips
            18.0, // min dist
            72.0, // max dist
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
    table =
        ShotTable.fromPoints(
            new double[] {
              24.0, 0.050, 800.0,
              36.0, 0.100, 850.0,
              48.0, 0.200, 900.0,
              60.0, 0.300, 950.0,
              72.0, 0.400, 1000.0
            });
    // Covers the whole distance gate so the gate itself, not the table, decides validity.
    wideTable =
        ShotTable.fromPoints(
            new double[] {
              10.0, 0.0, 700.0,
              100.0, 0.5, 1100.0
            });
    red = new FieldModel(Alliance.RED);
    redA = red.target(HiveCell.A, EPOCH);
  }

  /**
   * A robot pose {@code distance} inches from the opening, {@code offAxisDeg} off its outward
   * direction, heading straight at it.
   */
  private static Pose standingAt(CellTarget target, double distance, double offAxisDeg) {
    double bearing = target.facingRad() + Math.toRadians(offAxisDeg);
    double x = target.x() + distance * Math.cos(bearing);
    double y = target.y() + distance * Math.sin(bearing);
    return new Pose(x, y, Math.atan2(target.y() - y, target.x() - x));
  }

  private static ShotInputs inputs(Pose pose, Velocity velocity, CellTarget target, BallType type) {
    return new ShotInputs(pose, velocity, target, type);
  }

  private ShotSolution solve(Pose pose, Velocity velocity, CellTarget target, ShotTable t) {
    return ShotSolver.solve(inputs(pose, velocity, target, BallType.POLLEN), t, params, 0.4);
  }

  @Test
  public void testNonFinitePoseVelocityAndTargetNeverYieldAValidShot() {
    Pose good = standingAt(redA, 30, 0);
    for (double bad :
        new double[] {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
      for (Pose pose :
          new Pose[] {
            new Pose(bad, good.y(), good.heading()),
            new Pose(good.x(), bad, good.heading()),
            new Pose(good.x(), good.y(), bad)
          }) {
        assertFalse(solve(pose, STILL, redA, wideTable).isValid(), "bad robot pose " + pose);
      }
      for (Velocity velocity :
          new Velocity[] {
            new Velocity(bad, 0, 0), new Velocity(0, bad, 0), new Velocity(0, 0, bad)
          }) {
        assertFalse(
            solve(good, velocity, redA, wideTable).isValid(), "bad robot velocity " + velocity);
      }
      for (CellTarget target :
          new CellTarget[] {
            new CellTarget(HiveCell.A, bad, redA.y(), redA.heightInches(), redA.facingRad(), EPOCH),
            new CellTarget(HiveCell.A, redA.x(), bad, redA.heightInches(), redA.facingRad(), EPOCH),
            new CellTarget(HiveCell.A, redA.x(), redA.y(), bad, redA.facingRad(), EPOCH),
            new CellTarget(HiveCell.A, redA.x(), redA.y(), redA.heightInches(), bad, EPOCH)
          }) {
        assertFalse(solve(good, STILL, target, wideTable).isValid(), "bad CELL target " + target);
      }
    }
  }

  @Test
  public void testStaticRobotInFrontOfCellIsValid() {
    Pose pose = standingAt(redA, 30.0, 0.0);

    ShotSolution s = solve(pose, STILL, redA, wideTable);

    assertTrue(s.isValid(), "a static shot straight in front of the opening must be valid");
    assertEquals("Valid", s.validityReason());
    assertEquals(30.0, s.distanceInches(), 1e-9);
    // RED's CELL A opens toward the audience (-y), so the robot is below it and aims up (+y).
    assertEquals(Math.PI / 2, s.targetAzimuthRad(), 1e-9);
    assertEquals(
        params.flightTimeAt(30.0),
        s.predictedFlightTimeSec(),
        1e-9,
        "flight time comes from the base + per-inch model");
    assertEquals(0.0, s.predictedEntryAngleRad(), 0.0, "table lookups carry no entry-angle model");
  }

  @Test
  public void testSolutionEchoesTargetEpochAndBallType() {
    Pose pose = standingAt(redA, 30.0, 0.0);

    for (BallType type : BallType.values()) {
      ShotSolution s = ShotSolver.solve(inputs(pose, STILL, redA, type), wideTable, params, 0.4);
      assertEquals(EPOCH, s.targetEpoch());
      assertEquals(type, s.ballType());
    }

    // A solution refused for speed or approach side still says which epoch it was refused for, so
    // the controller can tell a stale refusal from a fresh one.
    ShotSolution fast =
        ShotSolver.solve(
            inputs(pose, new Velocity(0, 100, 0), redA, BallType.NECTAR), wideTable, params, 0.4);
    assertFalse(fast.isValid());
    assertEquals(EPOCH, fast.targetEpoch());
    assertEquals(BallType.NECTAR, fast.ballType());

    ShotSolution behind =
        ShotSolver.solve(
            inputs(standingAt(redA, 30.0, 180.0), STILL, redA, BallType.NECTAR),
            wideTable,
            params,
            0.4);
    assertFalse(behind.isValid());
    assertEquals(EPOCH, behind.targetEpoch());
    assertEquals(BallType.NECTAR, behind.ballType());
  }

  @Test
  public void testIsCurrentMatchesOnlyTheSameEpochAndBallType() {
    ShotSolution s =
        ShotSolver.solve(
            inputs(standingAt(redA, 30.0, 0.0), STILL, redA, BallType.NECTAR),
            wideTable,
            params,
            0.4);

    assertTrue(ShotController.isCurrent(s, EPOCH, BallType.NECTAR));
    assertFalse(ShotController.isCurrent(s, EPOCH + 1, BallType.NECTAR), "a TIP bumps the epoch");
    assertFalse(
        ShotController.isCurrent(s, EPOCH, BallType.POLLEN),
        "a different ball needs its own table");
    assertFalse(
        ShotController.isCurrent(ShotSolution.invalid("x"), EPOCH, BallType.POLLEN),
        "the sentinel invalid solution is never current");
  }

  @Test
  public void testSolutionComesStraightFromTheMeasuredTable() {
    // 48 in is exactly a table row.
    ShotSolution s = solve(standingAt(redA, 48.0, 0.0), STILL, redA, table);

    assertTrue(s.isValid());
    assertEquals(0.200, s.targetHoodPosition(), 1e-9, "hood is the table row, not a solved angle");
    assertEquals(900.0, s.targetRpm(), 1e-9, "rpm is the table row");
    assertEquals(
        900.0 * 360.0 / 1000.0,
        s.targetExitVelocityIps(),
        1e-9,
        "exit speed follows the linear flywheel model");
  }

  @Test
  public void testInterpolatesBetweenCalibratedDistances() {
    ShotSolution s = solve(standingAt(redA, 54.0, 0.0), STILL, redA, table);

    assertTrue(s.isValid());
    assertEquals(0.250, s.targetHoodPosition(), 1e-9, "halfway between the 48 and 60 rows");
    assertEquals(925.0, s.targetRpm(), 1e-9);
  }

  @Test
  public void testFartherShotsGetMoreFlywheelAndFlatterHoodFromTheTable() {
    ShotSolution near = solve(standingAt(redA, 30.0, 0.0), STILL, redA, table);
    ShotSolution far = solve(standingAt(redA, 66.0, 0.0), STILL, redA, table);

    assertTrue(near.isValid() && far.isValid());
    assertTrue(far.targetRpm() > near.targetRpm(), "farther shot needs more flywheel");
    assertTrue(
        far.targetHoodPosition() > near.targetHoodPosition(),
        "farther shot needs a flatter arc, which on this table is a larger servo position");
  }

  /** Every CELL of both alliances solves the same way once the robot stands in front of it. */
  @Test
  public void testEveryCellOfEveryAllianceIsShootableFromInFront() {
    for (Alliance alliance : Alliance.values()) {
      FieldModel field = new FieldModel(alliance);
      for (HiveCell cell : HiveCell.values()) {
        CellTarget target = field.target(cell, 3);
        ShotSolution s = solve(standingAt(target, 40.0, 0.0), STILL, target, table);
        String who = alliance + " " + cell;
        assertTrue(s.isValid(), who + ": " + s.validityReason());
        assertEquals(40.0, s.distanceInches(), 1e-9, who);
        // Aiming straight back at the opening means the bearing is the opening's facing + pi.
        assertEquals(
            0.0,
            AngleUnit.normalizeRadians(s.targetAzimuthRad() - (target.facingRad() + Math.PI)),
            1e-9,
            who);
      }
    }
  }

  // -------------------------------------------------------------------------------------------
  // Approach side

  @Test
  public void testOffAxisRadIsZeroInFrontAndPiBehind() {
    assertEquals(0.0, ShotSolver.offAxisRad(58.0, 25.0, redA), 1e-12);
    assertEquals(Math.PI, ShotSolver.offAxisRad(58.0, redA.y() + 25.0, redA), 1e-12);
    assertEquals(Math.PI / 2, ShotSolver.offAxisRad(redA.x() + 25.0, redA.y(), redA), 1e-12);
    assertEquals(Math.PI / 2, ShotSolver.offAxisRad(redA.x() - 25.0, redA.y(), redA), 1e-12);
    // Standing on the opening itself has no direction; refuse it as "behind".
    assertEquals(Math.PI, ShotSolver.offAxisRad(redA.x(), redA.y(), redA), 0.0);
    // Unsigned, and consistent with how the pose helper builds angles.
    for (double deg : new double[] {-170, -90, -45, -5, 0, 5, 45, 90, 170}) {
      Pose p = standingAt(redA, 30.0, deg);
      assertEquals(Math.abs(deg), Math.toDegrees(ShotSolver.offAxisRad(p.x(), p.y(), redA)), 1e-9);
    }
  }

  @Test
  public void testApproachGateAcceptsInsideTheLimitAndRefusesOutsideItOnBothSides() {
    for (Alliance alliance : Alliance.values()) {
      FieldModel field = new FieldModel(alliance);
      for (HiveCell cell : HiveCell.values()) {
        CellTarget target = field.target(cell, 1);
        for (double sign : new double[] {-1.0, 1.0}) {
          String who = alliance + " " + cell + " side " + sign;
          for (double deg : new double[] {0.0, 30.0, 59.0}) {
            ShotSolution s = solve(standingAt(target, 30.0, sign * deg), STILL, target, wideTable);
            assertTrue(s.isValid(), who + " at " + deg + " deg: " + s.validityReason());
          }
          for (double deg : new double[] {61.0, 90.0, 135.0, 180.0}) {
            ShotSolution s = solve(standingAt(target, 30.0, sign * deg), STILL, target, wideTable);
            assertFalse(s.isValid(), who + " at " + deg + " deg must be refused");
            assertTrue(
                s.validityReason().contains("off CELL " + cell + "'s opening axis"),
                s.validityReason());
            // The refusal is still aimed and sized, so the driver can see where to stand.
            assertEquals(30.0, s.distanceInches(), 1e-9);
          }
        }
      }
    }
  }

  @Test
  public void testApproachGateFollowsTheConfiguredLimit() {
    Pose pose = standingAt(redA, 30.0, 45.0);
    assertTrue(solve(pose, STILL, redA, wideTable).isValid());

    BallisticsParameters strict =
        new BallisticsParameters(
            params.v0(),
            params.k(),
            params.L(),
            params.launchHeightInches(),
            params.targetHeightInches(),
            params.gInchesPerSec2(),
            params.minHoodAngleDeg(),
            params.maxHoodAngleDeg(),
            params.minHoodServoPos(),
            params.maxHoodServoPos(),
            params.maxMovingSpeedIps(),
            params.minValidDistanceInches(),
            params.maxValidDistanceInches(),
            params.leadBiasGain(),
            params.v0ReferenceRpm(),
            params.preferredShotRpm(),
            params.maxShotRpm(),
            params.v0MarginFraction(),
            params.flightTimeBaseSec(),
            params.flightTimeSecPerInch(),
            params.leadMinSpeedIps(),
            30.0);
    ShotSolution s =
        ShotSolver.solve(inputs(pose, STILL, redA, BallType.POLLEN), wideTable, strict, 0.4);
    assertFalse(s.isValid(), "45 degrees off axis exceeds a 30 degree limit");
    assertTrue(s.validityReason().contains("max 30"));
  }

  // -------------------------------------------------------------------------------------------
  // Distance and calibration gates

  @Test
  public void testDistanceGateIsInclusiveAtBothEnds() {
    assertFalse(solve(standingAt(redA, 17.9, 0.0), STILL, redA, wideTable).isValid());
    assertTrue(solve(standingAt(redA, 18.0, 0.0), STILL, redA, wideTable).isValid());
    assertTrue(solve(standingAt(redA, 72.0, 0.0), STILL, redA, wideTable).isValid());

    ShotSolution far = solve(standingAt(redA, 72.1, 0.0), STILL, redA, wideTable);
    assertFalse(far.isValid());
    assertTrue(far.validityReason().contains("outside calibrated range"), far.validityReason());
  }

  @Test
  public void testTableRangeNarrowerThanTheDistanceGateStillRefuses() {
    // The table starts at 24 in but the distance gate allows 18 in: 20 in has no measured row.
    ShotSolution s = solve(standingAt(redA, 20.0, 0.0), STILL, redA, table);

    assertFalse(s.isValid());
    assertTrue(s.validityReason().contains("below the calibrated range"), s.validityReason());
    assertEquals(
        0.050,
        s.targetHoodPosition(),
        1e-9,
        "the hood clamps to the nearest row rather than extrapolating");
  }

  @Test
  public void testUncalibratedTableNeverYieldsAValidShot() {
    ShotTable placeholder =
        ShotTable.fromPoints(
            new double[] {
              24.0, 0.050, 800.0,
              72.0, 0.400, 1000.0
            },
            false);
    assertFalse(placeholder.isCalibrated());

    ShotSolution s = solve(standingAt(redA, 48.0, 0.0), STILL, redA, placeholder);

    assertFalse(s.isValid(), "placeholder rows must never authorize a feed");
    assertTrue(s.validityReason().startsWith("UNCALIBRATED"), s.validityReason());
    // The numbers are still produced, so a driver in manual mode and the calibration OpModes can
    // see what the table would command.
    assertEquals(0.225, s.targetHoodPosition(), 1e-9);
    assertEquals(900.0, s.targetRpm(), 1e-9);

    // Other refusals keep their own, more useful, reason ahead of the calibration one.
    ShotSolution behind = solve(standingAt(redA, 48.0, 180.0), STILL, redA, placeholder);
    assertFalse(behind.isValid());
    assertTrue(behind.validityReason().contains("off CELL A"), behind.validityReason());
    ShotSolution tooFar = solve(standingAt(redA, 80.0, 0.0), STILL, redA, placeholder);
    assertTrue(
        tooFar.validityReason().contains("outside calibrated range"), tooFar.validityReason());
  }

  // -------------------------------------------------------------------------------------------
  // Moving shots

  @Test
  public void testMovingRobotLeadsTheShotAgainstItsVelocity() {
    // Robot below CELL A facing it, drifting +x: the aim point shifts toward -x.
    Pose pose = standingAt(redA, 30.0, 0.0);
    ShotSolution right = solve(pose, new Velocity(30, 0, 0), redA, wideTable);
    ShotSolution left = solve(pose, new Velocity(-30, 0, 0), redA, wideTable);
    ShotSolution still = solve(pose, STILL, redA, wideTable);

    assertTrue(right.isValid() && left.isValid());
    assertEquals(Math.PI / 2, still.targetAzimuthRad(), 1e-9);
    assertTrue(right.targetAzimuthRad() > Math.PI / 2 + 0.05, "moving +x leads toward -x");
    assertTrue(left.targetAzimuthRad() < Math.PI / 2 - 0.05, "moving -x leads toward +x");
    // The geometry is symmetric about the robot's line to the opening.
    assertEquals(
        right.targetAzimuthRad() - Math.PI / 2, -(left.targetAzimuthRad() - Math.PI / 2), 1e-9);
    assertEquals(right.distanceInches(), left.distanceInches(), 1e-9);
    assertTrue(right.distanceInches() > 30.0, "a lead target is farther than the real one");
  }

  @Test
  public void testLeadConvergesToAConsistentFlightTimeAndAimPoint() {
    Pose pose = standingAt(redA, 30.0, 0.0);
    double vx = 30.0;

    ShotSolution s = solve(pose, new Velocity(vx, 0, 0), redA, wideTable);

    // The reported flight time is the one that yields the reported distance...
    assertEquals(params.flightTimeAt(s.distanceInches()), s.predictedFlightTimeSec(), 2e-3);
    // ...and the aim point is the opening shifted back by velocity * flight time.
    double virtX = redA.x() - vx * s.predictedFlightTimeSec() * params.leadBiasGain();
    assertEquals(Math.atan2(redA.y() - pose.y(), virtX - pose.x()), s.targetAzimuthRad(), 5e-3);
    assertEquals(Math.hypot(virtX - pose.x(), redA.y() - pose.y()), s.distanceInches(), 0.1);
  }

  /**
   * A robot holding position still reports a small, noisy velocity. Leading on that noise swung the
   * commanded aim azimuth by more than the turret's 1-2 degree settle tolerance, so the turret
   * never reached its target and hunted continuously.
   */
  @Test
  public void testLowSpeedVelocityNoiseDoesNotMoveTheAimAzimuth() {
    Pose pose = standingAt(redA, 30.0, 0.0);
    double still = solve(pose, STILL, redA, wideTable).targetAzimuthRad();

    // Sweep plausible estimator noise below the lead threshold (10 in/s); aim must not budge.
    for (double noise : new double[] {-9.0, -5.0, -2.0, 2.0, 5.0, 9.0}) {
      double jittered = solve(pose, new Velocity(noise, 0, 0), redA, wideTable).targetAzimuthRad();
      assertEquals(
          still, jittered, 1e-9, "velocity noise of " + noise + " in/s must not steer the turret");
    }
  }

  /** Above the threshold the robot is genuinely moving and lead must still be applied. */
  @Test
  public void testLeadStillAppliesOnceActuallyMoving() {
    Pose pose = standingAt(redA, 30.0, 0.0);
    double still = solve(pose, STILL, redA, wideTable).targetAzimuthRad();
    double moving = solve(pose, new Velocity(11.0, 0, 0), redA, wideTable).targetAzimuthRad();

    assertNotEquals(still, moving, 1e-4);
    assertTrue(moving > still);
  }

  /**
   * Over the speed gate the shot is refused, but the aim keeps its lead: dropping to the raw
   * bearing there made the turret jump by the whole lead angle every time the robot crossed the
   * gate.
   */
  @Test
  public void testMaxSpeedGateRefusesButKeepsLeadingTheAim() {
    Pose pose = standingAt(redA, 30.0, 0.0);
    Velocity crossing = new Velocity(80.0, 0, 0);

    ShotSolution fast = solve(pose, crossing, redA, table);

    assertFalse(fast.isValid(), "a solution beyond the max speed gate must be invalid");
    assertTrue(fast.validityReason().contains("speed"), fast.validityReason());
    assertEquals(EPOCH, fast.targetEpoch());
    double rawBearing = Math.atan2(redA.y() - pose.y(), redA.x() - pose.x());
    assertTrue(
        Math.abs(AngleUnit.normalizeRadians(fast.targetAzimuthRad() - rawBearing)) > 0.2,
        "aim must still lead the target while too fast to shoot");
    // Just under the gate the lead (and so the aim) is continuous with just over it.
    ShotSolution under = solve(pose, new Velocity(59.9, 0, 0), redA, wideTable);
    ShotSolution over = solve(pose, new Velocity(60.1, 0, 0), redA, wideTable);
    assertTrue(under.isValid());
    assertFalse(over.isValid());
    assertEquals(under.targetAzimuthRad(), over.targetAzimuthRad(), 0.01);

    // Exactly at the gate is still allowed; the gate is strictly greater-than.
    assertTrue(solve(pose, new Velocity(60.0, 0, 0), redA, wideTable).isValid());
    // Speed is the magnitude of the vector, not one component.
    assertFalse(solve(pose, new Velocity(40.0, 45.0, 0), redA, wideTable).isValid());
  }

  /** Lead fades in between leadMin and twice leadMin instead of switching on in one step. */
  @Test
  public void testLeadFadesInWithoutAStep() {
    double min = params.leadMinSpeedIps();
    assertEquals(0.0, ShotSolver.leadWeight(0.0, min), 0.0);
    assertEquals(0.0, ShotSolver.leadWeight(min, min), 0.0);
    assertEquals(0.5, ShotSolver.leadWeight(1.5 * min, min), 1e-12);
    assertEquals(1.0, ShotSolver.leadWeight(2.0 * min, min), 0.0);
    assertEquals(1.0, ShotSolver.leadWeight(5.0 * min, min), 0.0);
    assertEquals(1.0, ShotSolver.leadWeight(3.0, 0.0), 0.0);

    Pose pose = standingAt(redA, 30.0, 0.0);
    double previous = Double.NaN;
    for (double speed = 0.0; speed <= 3 * min; speed += 0.25) {
      double az = solve(pose, new Velocity(speed, 0, 0), redA, wideTable).targetAzimuthRad();
      if (!Double.isNaN(previous)) {
        assertTrue(
            Math.abs(az - previous) < Math.toRadians(1.5),
            "aim stepped "
                + Math.toDegrees(Math.abs(az - previous))
                + " deg at "
                + speed
                + " in/s");
      }
      previous = az;
    }
  }

  @Test
  public void testRangeGateAppliesToTheLeadCompensatedDistance() {
    // 70 in out and backing away at 50 in/s: the ball must fly the farther, lead distance.
    Pose pose = standingAt(redA, 70.0, 0.0);
    Velocity awayVelocity = new Velocity(0, -50.0, 0);

    ShotSolution s = solve(pose, awayVelocity, redA, wideTable);

    assertTrue(70.0 <= params.maxValidDistanceInches(), "the real distance alone would pass");
    assertTrue(s.distanceInches() > params.maxValidDistanceInches());
    assertFalse(s.isValid());
    assertTrue(s.validityReason().contains("outside calibrated range"), s.validityReason());
  }

  // -------------------------------------------------------------------------------------------
  // Degenerate inputs

  @Test
  public void testMissingInputsYieldAnInvalidSolutionRatherThanAnException() {
    Pose pose = standingAt(redA, 30.0, 0.0);
    ShotInputs ok = inputs(pose, STILL, redA, BallType.POLLEN);

    assertInvalid(ShotSolver.solve(null, wideTable, params, 0.4), "Null");
    assertInvalid(ShotSolver.solve(ok, null, params, 0.4), "Null");
    assertInvalid(ShotSolver.solve(ok, wideTable, null, 0.4), "Null");
    assertInvalid(
        ShotSolver.solve(inputs(null, STILL, redA, BallType.POLLEN), wideTable, params, 0.4),
        "Null");
    assertInvalid(
        ShotSolver.solve(inputs(pose, STILL, null, BallType.POLLEN), wideTable, params, 0.4),
        "No upward CELL");

    // A missing velocity is just a stationary robot.
    assertTrue(
        ShotSolver.solve(inputs(pose, null, redA, BallType.POLLEN), wideTable, params, 0.4)
            .isValid());
  }

  private static void assertInvalid(ShotSolution s, String reasonContains) {
    assertFalse(s.isValid());
    assertTrue(s.validityReason().contains(reasonContains), s.validityReason());
    assertEquals(-1, s.targetEpoch(), "no epoch to be current for");
    assertEquals(0.0, s.targetRpm(), 0.0);
  }

  @Test
  public void testInvalidFactoryNeverAuthorizesAnything() {
    ShotSolution s = ShotSolution.invalid("because");

    assertFalse(s.isValid());
    assertEquals("because", s.validityReason());
    assertEquals(-1, s.targetEpoch());
    assertEquals(BallType.POLLEN, s.ballType());
    assertEquals(0.0, s.targetRpm(), 0.0);
    assertEquals(0.0, s.targetHoodPosition(), 0.0);
    assertEquals(0.0, s.distanceInches(), 0.0);
    // A real epoch of -1 can never exist: the tracker starts at 0 and only counts up.
    assertFalse(ShotController.isCurrent(s, 0, BallType.POLLEN));
  }

  // -------------------------------------------------------------------------------------------
  // Shipped configuration

  /**
   * With the shipped parameters and tables, every volley pose in the codebase must be shootable (in
   * front of its CELL, inside the distance gate) — and the only thing that may stop it is the
   * calibration flag. When the team sets {@code shooter.shot_table.calibrated: true} after running
   * the calibration OpMode, this flips to requiring a fully valid shot.
   */
  @Test
  public void testShippedConfigSolvesEveryVolleyPoseExceptForCalibration() {
    BallisticsParameters shipped = BallisticsParameters.fromConfig();
    boolean calibrated = config.shooter.shot_table.calibrated;

    for (Alliance alliance : Alliance.values()) {
      FieldModel field = new FieldModel(alliance);
      for (AutoFixtures.Volley v : AutoFixtures.allVolleys(alliance)) {
        CellTarget target = field.target(v.cell(), 2);
        for (BallType type : BallType.values()) {
          ShotTable shippedTable = ShotTable.fromConfig(type);
          assertEquals(calibrated, shippedTable.isCalibrated());

          ShotSolution s =
              ShotSolver.solve(inputs(v.pose(), STILL, target, type), shippedTable, shipped, 0.0);

          String who = alliance + " " + v.name() + " " + type;
          if (calibrated) {
            assertTrue(s.isValid(), who + ": " + s.validityReason());
          } else {
            assertFalse(s.isValid(), who);
            assertTrue(
                s.validityReason().startsWith("UNCALIBRATED"), who + ": " + s.validityReason());
          }
          // Whatever the flag says, nothing else is wrong with the geometry.
          assertTrue(
              Math.toDegrees(ShotSolver.offAxisRad(v.pose().x(), v.pose().y(), target))
                  <= shipped.maxOffAxisDeg(),
              who + " is not in front of its CELL");
          assertTrue(
              s.distanceInches() >= shipped.minValidDistanceInches()
                  && s.distanceInches() <= shipped.maxValidDistanceInches(),
              who + " is outside the distance gate");
          assertTrue(
              s.distanceInches() >= shippedTable.minDistance()
                  && s.distanceInches() <= shippedTable.maxDistance(),
              who + " is outside the table's measured rows");
        }
      }
    }
  }
}
