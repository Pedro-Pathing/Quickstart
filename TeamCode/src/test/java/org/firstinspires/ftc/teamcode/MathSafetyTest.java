package org.firstinspires.ftc.teamcode;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.game.HiveTracker;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.BallType;
import org.firstinspires.ftc.teamcode.records.ShotSolution;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.Casablanca;
import org.firstinspires.ftc.teamcode.utilities.FlywheelDipDetector;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.locationtech.jts.geom.Coordinate;
import org.mockito.Mockito;

public class MathSafetyTest {

  private Turret turret;
  private CRServo mockServo;
  private double[] turretVoltage;

  @BeforeEach
  public void setUp() {
    // Mock hardware dependencies for Turret initialization
    HardwareMap hardwareMap = Mockito.mock(HardwareMap.class);
    mockServo = Mockito.mock(CRServo.class);
    DcMotorEx mockMotor = Mockito.mock(DcMotorEx.class);
    Mockito.when(hardwareMap.get(CRServo.class, "turn")).thenReturn(mockServo);
    AnalogInput mockAnalog = Mockito.mock(AnalogInput.class);
    Mockito.when(hardwareMap.get(AnalogInput.class, "turnanalog")).thenReturn(mockAnalog);
    turretVoltage = new double[] {config.turret.analog_encoder.zero_voltage};
    Mockito.when(mockAnalog.getVoltage()).thenAnswer(inv -> turretVoltage[0]);

    Mockito.when(hardwareMap.get(DcMotorEx.class, "leftFront")).thenReturn(mockMotor);
    Mockito.when(hardwareMap.get(DcMotorEx.class, "leftBack")).thenReturn(mockMotor);
    Mockito.when(hardwareMap.get(DcMotorEx.class, "rightFront")).thenReturn(mockMotor);
    Mockito.when(hardwareMap.get(DcMotorEx.class, "rightBack")).thenReturn(mockMotor);

    // Tests in this class exercise turret motion/aiming behavior, not the enabled flag itself
    // (see testTurretDisabledBehavior for that) -- config.turret.enabled defaults to false on a
    // freshly-built robot, so force it on here regardless of test execution order.
    config.turret.enabled = true;

    Telemetry mockTelemetry = Mockito.mock(Telemetry.class);
    turret = new Turret(hardwareMap, mockTelemetry);
  }

  /** Drives the mocked analog voltage so turret.getCurrentTurnAngle() reports the given angle. */
  private void setPhysicalTurretAngle(double chassisRelativeDegrees) {
    var enc = config.turret.analog_encoder;
    double delta =
        enc.inverted
            ? -chassisRelativeDegrees / enc.degrees_per_volt
            : chassisRelativeDegrees / enc.degrees_per_volt;
    turretVoltage[0] = enc.zero_voltage + delta;
  }

  @Test
  public void testAlignPoseAngle() {
    // Test alignPose calculates target angles correctly
    // Angle to (10, 0) from (0, 0) should be 0 radians
    Pose pose1 = Turret.alignPose(0, 0, 10, 0);
    assertEquals(0.0, pose1.heading(), 1e-6);

    // Angle to (0, 10) from (0, 0) should be PI/2 radians (90 degrees)
    Pose pose2 = Turret.alignPose(0, 0, 0, 10);
    assertEquals(Math.PI / 2, pose2.heading(), 1e-6);

    // Angle to (-10, 0) from (0, 0) should be PI radians (180 degrees)
    Pose pose3 = Turret.alignPose(0, 0, -10, 0);
    assertEquals(Math.PI, pose3.heading(), 1e-6);
  }

  @Test
  public void testTurretCalculateDynamicTolerance() {
    config.turret.tolerance.near_cutoff = 24.0; // inches
    config.turret.tolerance.near_val = 2.0; // degrees
    config.turret.tolerance.max_drift = 4.0; // inches
    config.turret.tolerance.min_deg = 1.0; // degrees
    config.turret.tolerance.max_deg = 5.0; // degrees

    // If distance is near cutoff (e.g. 15 inches < 24 inches), target NEAR_VAL tolerance
    assertEquals(2.0, turret.calculateDynamicTolerance(15.0), 1e-6);

    // Dynamic target calculations
    // distance = 50 inches: tolerance = asin((MAX_DRIFT / 2) / dist) = asin(2/50) = asin(0.04) ~
    // 2.29 degrees
    double expectedAngle = Math.toDegrees(Math.asin(2.0 / 50.0));
    assertEquals(expectedAngle, turret.calculateDynamicTolerance(50.0), 1e-6);
  }

  @Test
  public void testCasablancaApplyFriction() {
    // Casablanca friction comp tests
    double kS = 0.1;

    // Zero or micro-input is zeroed out
    assertEquals(0.0, Casablanca.applyFriction(0.005, kS), 1e-6);
    assertEquals(0.0, Casablanca.applyFriction(0.0, kS), 1e-6);

    // Positive input scale math: signum * (kS + input * (1 - kS))
    // input = 0.5 -> 0.1 + 0.5 * 0.9 = 0.55
    assertEquals(0.55, Casablanca.applyFriction(0.5, kS), 1e-6);

    // Negative input scale math
    assertEquals(-0.55, Casablanca.applyFriction(-0.5, kS), 1e-6);
  }

  @Test
  public void testCasablancaApplySlewLimit() {
    // Current = 0.0, target = 0.5, maxChange = 0.2 -> result should be 0.2
    assertEquals(0.2, Casablanca.applySlewLimit(0.0, 0.5, 0.2), 1e-6);

    // Current = 0.0, target = 0.1, maxChange = 0.2 -> diff < maxChange, result should be 0.1
    assertEquals(0.1, Casablanca.applySlewLimit(0.0, 0.1, 0.2), 1e-6);

    // Current = 0.0, target = -0.5, maxChange = 0.2 -> result should be -0.2
    assertEquals(-0.2, Casablanca.applySlewLimit(0.0, -0.5, 0.2), 1e-6);
  }

  @Test
  public void testCasablancaCalculateLaneFade() {
    // zoneMin = 20, zoneMax = 40, buffer = 10
    // botMax = 15, botMin = 10 -> distToStrict = 20 - 15 = 5 (within buffer)
    // fade = 1 - (5 / 10) = 0.5
    assertEquals(0.5, Casablanca.calculateLaneFade(10, 15, 20, 40, 10), 1e-6);

    // botMax = 5, botMin = 0 -> distToStrict = 20 - 5 = 15 (outside buffer) -> fade = 0.0
    assertEquals(0.0, Casablanca.calculateLaneFade(0, 5, 20, 40, 10), 1e-6);

    // botMax = 25, botMin = 20 -> distToStrict = 0 (intersecting/violating) -> fade = 1.0
    assertEquals(1.0, Casablanca.calculateLaneFade(20, 25, 20, 40, 10), 1e-6);
  }

  @Test
  public void testTurretDirectionalKsFeedforward() {
    turret.setKs(0.08, 0.12);
    turret.setPIDF(0, 0, 0, 0);
    assertEquals(0.08, turret.getKsPositive(), 1e-6);
    assertEquals(0.12, turret.getKsNegative(), 1e-6);

    // Current angle at 0 deg, target at 10 deg (error > 0)
    setPhysicalTurretAngle(0.0);
    turret.setTargetTurnAngle(10.0);
    turret.updateTurret(new Pose(0, 0, 0));
    Mockito.verify(mockServo).setPower(Mockito.eq(0.08));

    // Current angle at 0 deg, target at -10 deg (error < 0)
    setPhysicalTurretAngle(0.0);
    turret.setTargetTurnAngle(-10.0);
    turret.updateTurret(new Pose(0, 0, 0));
    Mockito.verify(mockServo).setPower(Mockito.eq(-0.12));
  }

  @Test
  public void testSentinelFootprintRotationAndIntersections() {
    Sentinel sentinel = new Sentinel(Alliance.RED);
    double halfWidth = config.sentinel.robot_width / 2.0;

    // 1. Footprint corners at heading 0: an axis-aligned square around the pose.
    Coordinate[] footprintZero = sentinel.calculateRobotFootprint(new Pose(10, 10, 0));
    assertEquals(4, footprintZero.length);
    assertEquals(10.0 - halfWidth, footprintZero[0].x, 1e-4);
    assertEquals(10.0 - halfWidth, footprintZero[0].y, 1e-4);
    assertEquals(10.0 + halfWidth, footprintZero[1].x, 1e-4);
    assertEquals(10.0 - halfWidth, footprintZero[1].y, 1e-4);

    // 2. Intersection with the HIVE frame. The frame is centred on the field, so the field centre
    // is inside it and a corner of the field is not.
    double centre = config.field.size_in / 2.0;
    assertTrue(
        sentinel.violatesProtectedZone(
            sentinel.calculateRobotFootprint(new Pose(centre, centre, 0))));
    assertFalse(
        sentinel.violatesProtectedZone(sentinel.calculateRobotFootprint(new Pose(10, 10, 0))));

    // 3. Rotation safety. A robot parked just clear of the frame at heading 0 would sweep into it
    // if it turned by half a radian (its corners reach out to halfWidth * (cos + sin)).
    double protectedMaxX = sentinel.getProtectedZones().get(0).getMaxX();
    Pose beside = new Pose(protectedMaxX + halfWidth + 0.5, centre, 0);
    assertFalse(sentinel.isRotationSafe(beside, 1.0, 0.5));
    // A small turn stays clear, and no turn input is always safe.
    assertTrue(sentinel.isRotationSafe(beside, 1.0, 0.01));
    assertTrue(sentinel.isRotationSafe(beside, 0.0, 0.5));
    // A robot already overlapping the frame may turn, so it can always rotate its way back out.
    assertTrue(sentinel.isRotationSafe(new Pose(centre, centre, 0), 1.0, 0.5));
  }

  /**
   * The ball counter behind the Shot Timing Tuner.
   *
   * <p>Everything that makes this detector correct is a threshold, and every one of them fails in a
   * way that still looks like a plausible answer: no refractory turns one ball into four, a missing
   * latch re-fires the same dip the instant the refractory expires, and a baseline that averages
   * during a dip stops counting halfway through a feed. None of that is visible from a driver
   * station reading "6 balls" — you would simply tune auto's window against a number that was never
   * real.
   */
  @Test
  public void testFlywheelDipDetectorCountsOneEventPerBall() {
    // 5% below reference triggers, 5% rebound off the trough re-arms, refractory 120 ms.
    FlywheelDipDetector detector = new FlywheelDipDetector(0.05, 0.05, 0.03, 120);

    // Settle a baseline at 1000 ticks/s.
    for (int i = 0; i < 100; i++) {
      assertFalse(detector.update(1000.0, i * 10L));
    }
    assertEquals(1000.0, detector.getBaselineVelocity(), 1e-6);

    // One ball: a dip held over several loops fires exactly once, on the leading edge.
    assertTrue(detector.update(920.0, 1000L));
    assertFalse(detector.update(900.0, 1010L));
    assertFalse(detector.update(910.0, 1020L));
    assertEquals(1, detector.getEventCount());
    assertEquals(0.10, detector.getDeepestDipFraction(), 1e-9);

    // Recovery above the re-arm line, then a second ball past the refractory window.
    assertFalse(detector.update(1000.0, 1030L));
    assertTrue(detector.update(930.0, 1400L));
    assertEquals(2, detector.getEventCount());

    // A dip that re-crosses inside the refractory window is the same ball's ragged floor.
    assertFalse(detector.update(1000.0, 1410L));
    assertFalse(detector.update(930.0, 1450L));
    assertEquals(2, detector.getEventCount());

    // Noise that never reaches the trigger is not a ball.
    assertFalse(detector.update(1000.0, 2000L));
    assertFalse(detector.update(970.0, 2010L));
    assertEquals(2, detector.getEventCount());

    // Sign is ignored, as everywhere else that reads flywheel velocity.
    assertFalse(detector.update(-1000.0, 2100L));
    assertEquals(2, detector.getEventCount());
  }

  /**
   * Replays the shape of a real two-ball trace off this robot, where the second ball arrives long
   * before the wheel has recovered from the first.
   *
   * <p>The numbers are from {@code shot_timing_log.csv}: idling at 1020 ticks/s, ball one drops it
   * to a 740 trough, the controller claws back to 980 over about 1.4 s, and ball two lands on top
   * of that partial recovery. An earlier version ended a dip only when the wheel returned within 2%
   * of its pre-ball reference — which took 2.2 s, so it stayed latched through ball two and
   * reported one ball for the magazine. Ending the dip on a rebound off the trough is what makes
   * the second one visible.
   */
  @Test
  public void testFlywheelDipDetectorCountsBallLandingBeforeFullRecovery() {
    FlywheelDipDetector detector = new FlywheelDipDetector(0.05, 0.05, 0.03, 120);

    long t = 0;
    for (int i = 0; i < 20; i++, t += 10) {
      assertFalse(detector.update(1020.0, t));
    }

    // Ball one: 1020 -> 740 trough.
    assertTrue(detector.update(920.0, t), "ball one missed");
    t += 10;
    for (double v : new double[] {860, 800, 760, 740}) {
      detector.update(v, t);
      t += 10;
    }
    assertEquals(1, detector.getEventCount());

    // Partial recovery — nowhere near the 1020 it started from.
    for (double v : new double[] {760, 800, 840, 860, 880, 920, 940, 980}) {
      detector.update(v, t);
      t += 10;
    }
    assertEquals(1, detector.getEventCount(), "recovery must not be counted as a ball");

    // Ball two lands on that partial recovery.
    assertTrue(detector.update(800.0, t), "ball two missed — detector latched in ball one's dip");
    assertEquals(2, detector.getEventCount());
  }

  /**
   * A feed that never lets the wheel back up to its setpoint still has to count every ball.
   *
   * <p>This is the case that rules out measuring dips against the commanded target: during a
   * sustained feed the flywheel sits well below setpoint the whole time — that sag is exactly why
   * {@code feed_release_threshold} is 0.80 while the arm window is 0.97 — so a target-relative
   * threshold sees one enormous dip and reports a single ball for the entire burst.
   */
  @Test
  public void testFlywheelDipDetectorCountsBallsWhileWheelStaysSagged() {
    FlywheelDipDetector detector = new FlywheelDipDetector(0.05, 0.05, 0.2, 100);

    // Wheel has settled at 850 ticks/s under a continuous feed — 15% under a 1000 setpoint.
    for (int i = 0; i < 100; i++) {
      detector.update(850.0, i * 10L);
    }
    assertEquals(850.0, detector.getBaselineVelocity(), 1e-3);

    long t = 2000L;
    for (int ball = 0; ball < 4; ball++) {
      assertTrue(detector.update(780.0, t), "ball " + ball + " missed");
      assertEquals(ball + 1, detector.getEventCount());
      // Recover to the local baseline, not to the setpoint.
      for (int i = 0; i < 12; i++) {
        detector.update(850.0, t + 10L + i * 10L);
      }
      t += 300L;
    }
    assertEquals(4, detector.getEventCount());
  }

  /**
   * A wheel pinned at the bottom of a dip is one ball, however long it stays there.
   *
   * <p>The reference must not average downward toward it: if it tracks the ball down, the wheel
   * eventually looks "recovered" without having moved, and the same stuck ball counts again.
   */
  /**
   * The flywheel coasting down must not read as a ball.
   *
   * <p>This is the failure that corrupted a whole calibration run. After the last ball the wheel
   * sits above its setpoint and bleeds back down over more than a second, one encoder quantum per
   * loop. Because the reference follows the wheel up instantly and down slowly, it hangs above the
   * decay — and with the trigger set at 5% it eventually crossed, reporting a third ball in all 11
   * recorded shots and cutting each measurement short before the real one landed. Real balls
   * measured 12.2%-26.4% deep against a 5.4% worst case for the coast, so the trigger belongs
   * between them.
   */
  @Test
  public void testFlywheelDipDetectorIgnoresCoastDown() {
    FlywheelDipDetector detector = new FlywheelDipDetector(0.09, 0.05, 0.03, 120);

    // Overshoot peak, then the real measured bleed-down: 1280 -> 1180 in 20-tick steps.
    long t = 0;
    for (int i = 0; i < 20; i++, t += 20) {
      detector.update(1280.0, t);
    }
    for (double v :
        new double[] {
          1280, 1260, 1280, 1260, 1240, 1260, 1240, 1220, 1220, 1200, 1200, 1200, 1180, 1180
        }) {
      assertFalse(detector.update(v, t), "coast-down at " + v + " must not count as a ball");
      t += 20;
    }
    assertEquals(0, detector.getEventCount());

    // A real ball off that same decayed wheel is still caught: 1180 -> 980 is 17%.
    assertTrue(detector.update(980.0, t), "a real ball after the coast must still register");
    assertEquals(1, detector.getEventCount());
  }

  @Test
  public void testFlywheelDipDetectorHoldsOneEventWhilePinnedInDip() {
    FlywheelDipDetector detector = new FlywheelDipDetector(0.05, 0.05, 0.2, 50);
    for (int i = 0; i < 100; i++) {
      detector.update(1000.0, i * 10L);
    }

    double baselineBefore = detector.getBaselineVelocity();
    for (int i = 0; i < 40; i++) {
      detector.update(900.0, 2000L + i * 10L);
    }
    assertEquals(baselineBefore, detector.getBaselineVelocity(), 1e-9);
    assertEquals(1, detector.getEventCount());
  }

  @Test
  public void testCasablancaSymmetryAndCollisionMath() {
    Sentinel sentinel = new Sentinel(Alliance.RED);
    Casablanca casablanca = new Casablanca(sentinel);

    // Disable heading lock, input smoothing, and friction compensation for predictable math testing
    Casablanca.enableHeadingLock = false;
    Casablanca.enableInputSmoothing = false;
    Casablanca.enableFrictionComp = false;
    Casablanca.fieldCentric = false;

    double halfWidth = config.sentinel.robot_width / 2.0;

    // The shipped config protects the whole HIVE frame, so there is exactly one zone.
    assertEquals(1, sentinel.getProtectedZones().size());
    var zone = sentinel.getProtectedZones().get(0);
    double midY = (zone.getMinY() + zone.getMaxY()) / 2.0;
    double midX = (zone.getMinX() + zone.getMaxX()) / 2.0;

    // --- CASE 1: Depth (X-axis) Protection Check ---
    // Place robot inside the depth hard stop boundary dynamically, driving toward the frame.
    double poseX = zone.getMaxX() + halfWidth + (0.5 * Casablanca.depthHardStop);
    double[] outputX =
        casablanca.adjustDriveInput(
            new Pose(poseX, midY, 0), com.pedropathing.math.Vector2D.zero(), 0.0, 0.0, -1.0, 0.0);
    // Adjusted forward (index 1) should be scaled down significantly
    assertTrue(Math.abs(outputX[1]) < 0.8);

    // --- CASE 2: Side (Y-axis) Protection Check ---
    // Place robot inside the side hard stop boundary dynamically, strafing toward the frame.
    double poseY = zone.getMinY() - halfWidth - (0.5 * Casablanca.sideHardStop);
    double[] outputY =
        casablanca.adjustDriveInput(
            new Pose(midX, poseY, 0), com.pedropathing.math.Vector2D.zero(), 0.0, 1.0, 0.0, 0.0);
    // Adjusted strafe (index 0) should be scaled down significantly
    assertTrue(Math.abs(outputY[0]) < 0.8);
  }

  @Test
  public void testCasablancaGoalHeadingLock() {
    Sentinel sentinel = new Sentinel(Alliance.RED);
    Casablanca casablanca = new Casablanca(sentinel);

    Casablanca.enableInputSmoothing = false;
    Casablanca.enableFrictionComp = false;
    Casablanca.fieldCentric = false;
    // The driver's face-the-CELL button is an explicit request, so it must engage even with the
    // automatic "hold the last heading you were left at" lock switched off in config.
    Casablanca.enableHeadingLock = false;

    // Open floor on the alliance side, clear of the HIVE frame, so the rotation-safety gate cannot
    // zero the turn.
    Pose pose = new Pose(20, 20, 0.0);
    com.pedropathing.math.Vector2D rest = com.pedropathing.math.Vector2D.zero();

    // Lock off: no turn stick means no turn, because the automatic lock is disabled.
    assertEquals(0.0, casablanca.adjustDriveInput(pose, rest, 0.0, 0.0, 0.0, 0.0)[2], 1e-9);
    assertFalse(casablanca.isTargetHeadingLockActive());

    // Target 90 deg to the robot's left. The bearing comes from the raw target position, matching
    // what TeleOp publishes via Turret.alignPose -- no velocity lead, so a stationary robot and a
    // moving one at the same place get the same heading target.
    double goalBearing = Turret.alignPose(pose.x(), pose.y(), pose.x(), pose.y() + 28).heading();
    assertEquals(Math.PI / 2, goalBearing, 1e-6);

    casablanca.setTargetHeadingLock(goalBearing, true);
    assertTrue(casablanca.isTargetHeadingLockActive());

    // Corrects on the very first loop: unlike the latching lock, there is no wait for the robot to
    // stop turning before a target exists.
    double turnToGoal = casablanca.adjustDriveInput(pose, rest, 0.0, 0.0, 0.0, 0.0)[2];
    assertTrue(turnToGoal > 0);
    assertTrue(Math.abs(turnToGoal) <= Casablanca.headingLockMaxPower + 1e-9);

    // Same target while translating: the lock is a heading-only override, so a moving robot still
    // gets a correction toward the unmodified goal bearing rather than a lead-compensated one.
    double turnWhileMoving =
        casablanca
            .adjustDriveInput(
                pose, com.pedropathing.math.Vector2D.cartesian(30, 0), 0.0, 0.0, 1.0, 0.0)[2];
    assertTrue(turnWhileMoving > 0);

    // ShotController's armed aim outranks it: both channels active, the solved azimuth wins.
    casablanca.setArmedAimTarget(-Math.PI / 2, true);
    double turnArmed = casablanca.adjustDriveInput(pose, rest, 0.0, 0.0, 0.0, 0.0)[2];
    assertTrue(turnArmed < 0);

    // Clearing armed aim (stopShot) falls back to the driver's lock, not to no lock at all.
    casablanca.setArmedAimTarget(0.0, false);
    assertTrue(casablanca.adjustDriveInput(pose, rest, 0.0, 0.0, 0.0, 0.0)[2] > 0);

    // Toggling the lock off hands the turn axis back to the driver.
    casablanca.setTargetHeadingLock(goalBearing, false);
    assertFalse(casablanca.isTargetHeadingLockActive());
    assertEquals(0.0, casablanca.adjustDriveInput(pose, rest, 0.0, 0.0, 0.0, 0.0)[2], 1e-9);

    Casablanca.enableHeadingLock = true;
  }

  @Test
  public void testTurretHoldAngleIsChassisRelativeAndHeadingInvariant() {
    // The analog encoder reads the turret's angle relative to the chassis directly (unlike the
    // old IMU, which read an absolute field heading). Physically offset the turret from the
    // chassis and hold it there.
    //
    // Derived from configured travel rather than hardcoded: setHoldAngle() clamps to
    // [min_angle, max_angle], so a literal angle outside the currently calibrated travel would
    // clamp the target away from the physical angle and fail this assert for reasons that have
    // nothing to do with heading invariance.
    double offsetAngle = config.turret.travel.max_angle * 0.5;
    setPhysicalTurretAngle(offsetAngle);
    turret.setHoldAngle(offsetAngle);
    turret.setAimMode(Turret.AimMode.HOLD);
    turret.periodic();
    Mockito.reset(mockServo);

    // Error must be 0 regardless of the robot chassis's field heading -- the turret servo never
    // moved, so its angle relative to the chassis is unaffected by the whole robot rotating.
    Pose facingNorth = new Pose(20, 20, Math.PI / 2);
    assertEquals(0.0, turret.getAimError(facingNorth), 1e-6);
    assertTrue(turret.isAimed(facingNorth));
    turret.updateTurret(facingNorth);
    Mockito.verify(mockServo).setPower(0.0);

    Mockito.reset(mockServo);
    Pose facingSouth = new Pose(20, 20, -Math.PI / 2);
    assertEquals(0.0, turret.getAimError(facingSouth), 1e-6);
    assertTrue(turret.isAimed(facingSouth));
    turret.updateTurret(facingSouth);
    Mockito.verify(mockServo).setPower(0.0);
  }

  @Test
  public void testManualModeRespectsBoundarySafety() {
    // MANUAL mode (and therefore the relay autotuner, which drives raw power with no PIDF
    // target) has no closed-loop target to bound it, so setTurretPowerRaw must refuse to drive
    // further past the +/-45 deg mechanical limit on its own.
    setPhysicalTurretAngle(config.turret.travel.max_angle + 1e-3);
    turret.setTurretPowerRaw(0.4);
    Mockito.verify(mockServo).setPower(0.0);

    // Driving back the other way off the limit must still be allowed.
    turret.setTurretPowerRaw(-0.4);
    Mockito.verify(mockServo).setPower(-0.4);
  }

  @Test
  public void testManualModeAllowsFullTravelWhenCentered() {
    // Centered turret must retain its full +/-45 deg range in MANUAL mode.
    setPhysicalTurretAngle(0.0);
    turret.setTurretPowerRaw(0.4);
    Mockito.verify(mockServo).setPower(0.4);

    Mockito.reset(mockServo);
    turret.setTurretPowerRaw(-0.4);
    Mockito.verify(mockServo).setPower(-0.4);
  }

  @Test
  public void testTurretIdleModeOnStartup() {
    turret.setAimMode(Turret.AimMode.IDLE);
    turret.periodic();
    Mockito.verify(mockServo).setPower(0.0);
  }

  @Test
  public void testHeadingErrorAngleWrapping() {
    // Target near +179 degrees, current near -179 degrees
    double targetHeading = Math.toRadians(179);
    double currentHeading = Math.toRadians(-179);

    // Signed shortest path error should be -2 degrees (-0.0349 rad)
    double error = AngleUnit.normalizeRadians(targetHeading - currentHeading);
    assertEquals(Math.toRadians(-2), error, 1e-4);

    // Reversed case: target near -179 deg, current near +179 deg -> error should be +2 deg
    double error2 = AngleUnit.normalizeRadians(-targetHeading - (-currentHeading));
    assertEquals(Math.toRadians(2), error2, 1e-4);
  }

  @Test
  public void testCasablancaLockAuthorityFade() {
    Sentinel sentinel = new Sentinel(Alliance.RED);
    new Casablanca(sentinel);
    Casablanca.headingLockIntentThreshold = 0.05;

    // Test authority at 0 stick intent: 1.0 - clamp(0 / 0.05, 0, 1) = 1.0
    double authZero = 1.0 - Math.clamp(0.0 / Casablanca.headingLockIntentThreshold, 0.0, 1.0);
    assertEquals(1.0, authZero, 1e-6);

    // Test authority at midpoint (0.025): 1.0 - clamp(0.025 / 0.05, 0, 1) = 0.5
    double authMid = 1.0 - Math.clamp(0.025 / Casablanca.headingLockIntentThreshold, 0.0, 1.0);
    assertEquals(0.5, authMid, 1e-6);

    // Test authority at/above threshold (0.05): 1.0 - clamp(0.05 / 0.05, 0, 1) = 0.0
    double authFull = 1.0 - Math.clamp(0.05 / Casablanca.headingLockIntentThreshold, 0.0, 1.0);
    assertEquals(0.0, authFull, 1e-6);
  }

  @Test
  public void testCasablancaKsSpeedInterpolation() {
    Sentinel sentinel = new Sentinel(Alliance.RED);
    new Casablanca(sentinel);
    Casablanca.frictionRot = 0.08;
    Casablanca.headingLockKsMoving = 0.02;
    Casablanca.headingLockMovingSpeedThreshold = 10.0;

    // Helper lerp matching Casablanca: frictionRot + speedRatio * (ks_moving - frictionRot)
    // Speed = 0.0 -> frictionRot (0.08)
    double speedRatio0 = Math.clamp(0.0 / Casablanca.headingLockMovingSpeedThreshold, 0.0, 1.0);
    double ks0 =
        Casablanca.frictionRot
            + speedRatio0 * (Casablanca.headingLockKsMoving - Casablanca.frictionRot);
    assertEquals(0.08, ks0, 1e-6);

    // Speed = 5.0 (midpoint) -> 0.08 + 0.5 * (0.02 - 0.08) = 0.05
    double speedRatioMid = Math.clamp(5.0 / Casablanca.headingLockMovingSpeedThreshold, 0.0, 1.0);
    double ksMid =
        Casablanca.frictionRot
            + speedRatioMid * (Casablanca.headingLockKsMoving - Casablanca.frictionRot);
    assertEquals(0.05, ksMid, 1e-6);

    // Speed = 10.0 or higher -> ks_moving (0.02)
    double speedRatioMax = Math.clamp(12.0 / Casablanca.headingLockMovingSpeedThreshold, 0.0, 1.0);
    double ksMax =
        Casablanca.frictionRot
            + speedRatioMax * (Casablanca.headingLockKsMoving - Casablanca.frictionRot);
    assertEquals(0.02, ksMax, 1e-6);
  }

  @Test
  public void testCasablancaHeadingLockHardGate() {
    Sentinel sentinel = new Sentinel(Alliance.RED);
    Casablanca casablanca = new Casablanca(sentinel);

    Casablanca.enableHeadingLock = true;
    Casablanca.enableFrictionComp = false;
    Casablanca.enableInputSmoothing = false;
    Casablanca.fieldCentric = false;
    Casablanca.headingLockIntentThreshold = 0.05;

    // First call to initialize target heading at pose (20, 20, 0)
    casablanca.adjustDriveInput(
        new Pose(20, 20, 0), com.pedropathing.math.Vector2D.zero(), 0.0, 0.0, 0.0, 0.0);

    // Below threshold (0.0499): driver is considered "released" -> heading lock takes over and
    // corrects towards the latched target, bounded by headingLockMaxPower, not equal to raw stick.
    double[] outputBelow =
        casablanca.adjustDriveInput(
            new Pose(20, 20, 0.01), com.pedropathing.math.Vector2D.zero(), 0.0, 0.0, 0.0, 0.0499);
    assertTrue(Math.abs(outputBelow[2]) <= Casablanca.headingLockMaxPower + 1e-9);
    assertTrue(outputBelow[2] != 0.0499);

    // Re-initialize to same target pose (20, 20, 0)
    casablanca.reset();
    casablanca.adjustDriveInput(
        new Pose(20, 20, 0), com.pedropathing.math.Vector2D.zero(), 0.0, 0.0, 0.0, 0.0);

    // At/above threshold (0.0501): driver has full, unfought authority -> raw stick passes through
    // untouched by the lock (friction comp disabled in this test).
    double[] outputAbove =
        casablanca.adjustDriveInput(
            new Pose(20, 20, 0.01), com.pedropathing.math.Vector2D.zero(), 0.0, 0.0, 0.0, 0.0501);
    assertEquals(0.0501, outputAbove[2], 1e-9);
  }

  @Test
  public void testCasablancaHeadingLockRelatchAndReset() {
    Sentinel sentinel = new Sentinel(Alliance.RED);
    Casablanca casablanca = new Casablanca(sentinel);

    Casablanca.enableHeadingLock = true;
    Casablanca.enableFrictionComp = false;
    Casablanca.enableInputSmoothing = false;
    Casablanca.fieldCentric = false;
    Casablanca.headingLockIntentThreshold = 0.05;

    // Call 1: Start at heading = 0 with no stick input -> targetHeading initialized to 0
    casablanca.adjustDriveInput(
        new Pose(20, 20, 0), com.pedropathing.math.Vector2D.zero(), 0.0, 0.0, 0.0, 0.0);

    // Call 2: Driver actively steers (turn = 0.1 > threshold 0.05) to heading = Math.PI / 4 (45
    // deg)
    casablanca.adjustDriveInput(
        new Pose(20, 20, Math.PI / 4), com.pedropathing.math.Vector2D.zero(), 0.0, 0.0, 0.0, 0.1);

    // Call 3: Driver releases stick (turn = 0.0) while robot is at heading = Math.PI / 4
    // Target heading should have re-latched to Math.PI / 4, so error is 0 and turn output is 0
    double[] outputLocked =
        casablanca.adjustDriveInput(
            new Pose(20, 20, Math.PI / 4),
            com.pedropathing.math.Vector2D.zero(),
            0.0,
            0.0,
            0.0,
            0.0);

    assertEquals(0.0, outputLocked[2], 1e-4);
  }

  @Test
  public void testCasablancaLiveRotationLookaheadTime() {
    Sentinel sentinel = new Sentinel(Alliance.RED);
    new Casablanca(sentinel);
    Casablanca.rotationLookaheadTimeSeconds = 0.075;

    // At angular velocity = 0, lookahead angle is 0
    double lookaheadRad0 = Math.abs(0.0) * Casablanca.rotationLookaheadTimeSeconds;
    assertEquals(0.0, lookaheadRad0, 1e-6);

    // At angular velocity = 6.0 rad/s, lookahead angle is 6.0 * 0.075 = 0.45 rad
    double lookaheadRadMax = Math.abs(6.0) * Casablanca.rotationLookaheadTimeSeconds;
    assertEquals(0.45, lookaheadRadMax, 1e-6);

    // At low angular velocity = 1.0 rad/s, lookahead angle scales down to 0.075 rad
    double lookaheadRadLow = Math.abs(1.0) * Casablanca.rotationLookaheadTimeSeconds;
    assertEquals(0.075, lookaheadRadLow, 1e-6);
  }

  @Test
  public void testCasablancaProximityVsPhysicsBrakingScale() {
    Sentinel sentinel = new Sentinel(Alliance.RED);
    Casablanca casablanca = new Casablanca(sentinel);
    Casablanca.enableHeadingLock = false;
    Casablanca.enableInputSmoothing = false;
    Casablanca.enableFrictionComp = false;
    Casablanca.fieldCentric = false;

    // Dynamically calculate test position to be at the midpoint of the proximity braking zone
    double hardStop = Casablanca.sideHardStop;
    double slowDown = Casablanca.sideSlowDown;
    double dMid = hardStop + 0.5 * (slowDown - hardStop);

    // Protected zone boundary (HIVE frame Y min), approached from the audience side in the lane
    // centred on the frame so the side protection has full lane authority.
    var zone = sentinel.getProtectedZones().get(0);
    double protectedMinY = zone.getMinY();
    double laneX = (zone.getMinX() + zone.getMaxX()) / 2.0;
    // Distance from robot center to robot top edge
    double halfWidth = config.sentinel.robot_width / 2.0;
    // Set robot Y so distance to hard stop is dMid
    double poseY = protectedMinY - halfWidth - dMid;

    double[] output =
        casablanca.adjustDriveInput(
            new Pose(laneX, poseY, 0), com.pedropathing.math.Vector2D.zero(), 0.0, 1.0, 0.0, 0.0);

    // Dynamic expected scale: (dMid - hardStop) / (slowDown - hardStop) = 0.5
    double expectedScale = (dMid - hardStop) / (slowDown - hardStop);
    assertEquals(expectedScale, output[0], 1e-3);
  }

  @Test
  public void testCasablancaFieldCentricIsHeadingInvariant() {
    Sentinel sentinel = new Sentinel(Alliance.RED);
    Casablanca casablanca = new Casablanca(sentinel);

    Casablanca.enableHeadingLock = false;
    Casablanca.enableInputSmoothing = false;
    Casablanca.enableFrictionComp = false;
    Casablanca.fieldCentric = true;
    Casablanca.fieldCentricOffsetRad = Math.toRadians(90.0);

    // The whole contract of field-centric drive: one stick direction maps to one field direction
    // no matter which way the robot is pointing. Output is {fieldY, fieldX, turn}.
    double[] reference = null;
    for (double headingDeg : new double[] {0.0, 90.0, 180.0, 270.0, 37.0}) {
      casablanca.reset();
      double[] out =
          casablanca.adjustDriveInput(
              new Pose(20, 20, Math.toRadians(headingDeg)),
              com.pedropathing.math.Vector2D.zero(),
              0.0,
              0.0,
              1.0,
              0.0);
      if (reference == null) {
        reference = out;
      } else {
        assertEquals(reference[0], out[0], 1e-6);
        assertEquals(reference[1], out[1], 1e-6);
      }
    }

    // The 90 degree offset means a fully-forward stick drives along field +Y -- away from the
    // driver wall at y = 0 -- rather than along field +X.
    assertEquals(0.0, reference[1], 1e-6);
    assertTrue(reference[0] > 0.0);

    // Teeth for the assertions above: the same stick IS heading-dependent with field-centric off,
    // so heading-invariance is the transform doing work and not a degenerate all-zero output.
    Casablanca.fieldCentric = false;
    casablanca.reset();
    double[] robotAt0 =
        casablanca.adjustDriveInput(
            new Pose(20, 20, 0.0), com.pedropathing.math.Vector2D.zero(), 0.0, 0.0, 1.0, 0.0);
    casablanca.reset();
    double[] robotAt90 =
        casablanca.adjustDriveInput(
            new Pose(20, 20, Math.PI / 2),
            com.pedropathing.math.Vector2D.zero(),
            0.0,
            0.0,
            1.0,
            0.0);
    assertTrue(Math.abs(robotAt0[0] - robotAt90[0]) > 0.5);
  }

  @Test
  public void testCasablancaRefusesToDriveOnNonFinitePose() {
    Sentinel sentinel = new Sentinel(Alliance.RED);
    Casablanca casablanca = new Casablanca(sentinel);

    Casablanca.enableHeadingLock = false;
    Casablanca.enableInputSmoothing = false;
    Casablanca.enableFrictionComp = false;

    // A NaN heading is a dead localizer. Vector.rotateVector(NaN) produces NaN components even for
    // a zero-magnitude vector, and every JTS Envelope comparison against NaN is false, so the zone
    // protections would silently disengage while NaN reached the drivetrain. Refuse instead.
    for (boolean fieldCentric : new boolean[] {true, false}) {
      Casablanca.fieldCentric = fieldCentric;
      casablanca.reset();
      double[] out =
          casablanca.adjustDriveInput(
              new Pose(20, 20, Double.NaN),
              com.pedropathing.math.Vector2D.zero(),
              0.0,
              0.5,
              1.0,
              0.5);
      assertEquals(0.0, out[0], 1e-9);
      assertEquals(0.0, out[1], 1e-9);
      assertEquals(0.0, out[2], 1e-9);
      assertTrue(casablanca.wasPoseUntrusted());
    }

    // And it clears again once the localizer recovers.
    Casablanca.fieldCentric = false;
    casablanca.reset();
    casablanca.adjustDriveInput(
        new Pose(20, 20, 0.0), com.pedropathing.math.Vector2D.zero(), 0.0, 0.0, 1.0, 0.0);
    assertFalse(casablanca.wasPoseUntrusted());
  }

  @Test
  public void testCasablancaPredictiveBrakingSanityCheck() {
    Sentinel sentinel = new Sentinel(Alliance.RED);
    new Casablanca(sentinel); // Runs the startup braking check.

    var p = org.firstinspires.ftc.teamcode.robot.config.generated.config.pedro;
    double forward =
        Casablanca.brakingDistance(
            p.motion.max_forward_velocity, p.braking.forward.linear, p.braking.forward.quadratic);
    double strafe =
        Casablanca.brakingDistance(
            p.motion.max_strafe_velocity, p.braking.strafe.linear, p.braking.strafe.quadratic);
    assertTrue(forward / Casablanca.decelSafetyFactor > 0);
    assertTrue(strafe / Casablanca.decelSafetyFactor > 0);
    assertEquals(
        -forward,
        Casablanca.brakingDistance(
            -p.motion.max_forward_velocity, p.braking.forward.linear, p.braking.forward.quadratic),
        1e-9);
    assertEquals(
        0,
        Casablanca.brakingDistance(0, p.braking.forward.linear, p.braking.forward.quadratic),
        1e-9);
  }

  @Test
  public void testShotControllerStopShotResetsHoldAngleToZero() {
    org.firstinspires.ftc.teamcode.robot.Shooter mockShooter =
        Mockito.mock(org.firstinspires.ftc.teamcode.robot.Shooter.class);
    org.firstinspires.ftc.teamcode.robot.Intake mockIntake =
        Mockito.mock(org.firstinspires.ftc.teamcode.robot.Intake.class);
    Telemetry mockTelemetry = Mockito.mock(Telemetry.class);

    // Robot has non-zero heading (e.g. 50 degrees)
    Pose nonZeroHeadingPose = new Pose(20, 20, Math.toRadians(50.0));
    org.firstinspires.ftc.teamcode.robot.ShotController shotController =
        new org.firstinspires.ftc.teamcode.robot.ShotController(
            mockShooter,
            turret,
            mockIntake,
            () -> nonZeroHeadingPose,
            Velocity::zero,
            null,
            new FieldModel(Alliance.BLUE),
            HiveTracker.fromConfig(() -> 0L),
            mockTelemetry);

    try {
      shotController.startShot(0.8, true);
      assertEquals(Turret.AimMode.AIM_AT_TARGET, turret.getAimMode());
      shotController.stopShot();

      // Verify hold angle is set to 0.0 chassis-relative center, NOT 50.0 degrees
      assertEquals(0.0, turret.getHoldAngle(), 1e-6);
      assertEquals(Turret.AimMode.HOLD, turret.getAimMode());
      Mockito.verify(mockShooter).setTargetPower(0.0);
      Mockito.verify(mockIntake).stop();
    } finally {
      shotController.shutdown();
    }
  }

  @Test
  public void testTurretDisabledBehavior() {
    assertTrue(turret.isEnabled());

    // 1. Flip via config flag
    config.turret.enabled = false;
    assertFalse(turret.isEnabled());
    assertTrue(turret.isTurnDone());

    turret.setTurretPowerRaw(0.5);
    assertEquals("TURRET DISABLED", turret.getPowerBlockedReason());
    Mockito.verify(mockServo, Mockito.never()).setPower(0.5);

    config.turret.enabled = true;
    assertTrue(turret.isEnabled());

    // 2. Flip dynamically via turret.setEnabled(false)
    turret.setEnabled(false);
    assertFalse(turret.isEnabled());

    turret.setEnabled(true);
    assertTrue(turret.isEnabled());
  }

  /**
   * With no turret the shooter is fixed to the chassis, so "aimed" can only mean the robot itself
   * faces the CELL. A disabled turret must never wave a robot that faces away through the aim gate.
   */
  @Test
  public void testDisabledTurretIsAimedOnlyWhenChassisFacesTarget() {
    config.turret.enabled = false;
    try {
      turret.setTargetPoint(100.0, 0.0);
      Pose here = new Pose(0, 0, 0);
      double tolerance =
          turret.calculateDynamicTolerance(100.0)
              + config.casablanca.heading_lock.error_deadband_deg;
      assertTrue(tolerance > 0.0, "tolerance must be a real, positive window");

      // Facing it, and facing within the allowed window either side.
      assertEquals(0.0, turret.getChassisAimErrorDeg(here), 1e-9);
      assertTrue(turret.isAimed(here));
      assertTrue(turret.isAimed(new Pose(0, 0, Math.toRadians(tolerance - 0.25))));
      assertTrue(turret.isAimed(new Pose(0, 0, Math.toRadians(-(tolerance - 0.25)))));

      // Just outside the window, then well off, then directly away from the target.
      assertFalse(turret.isAimed(new Pose(0, 0, Math.toRadians(tolerance + 0.25))));
      assertFalse(turret.isAimed(new Pose(0, 0, Math.toRadians(-(tolerance + 0.25)))));
      assertFalse(turret.isAimed(new Pose(0, 0, Math.PI / 2)));
      assertFalse(turret.isAimed(new Pose(0, 0, Math.PI)));
      assertEquals(180.0, Math.abs(turret.getChassisAimErrorDeg(new Pose(0, 0, Math.PI))), 1e-9);

      // The error is signed: target to the robot's left is positive.
      turret.setTargetPoint(0.0, 100.0);
      assertEquals(90.0, turret.getChassisAimErrorDeg(here), 1e-9);
      assertFalse(turret.isAimed(here));
      assertTrue(turret.isAimed(new Pose(0, 0, Math.PI / 2)));

      // A lead-compensated azimuth, once the solver publishes one, outranks the raw target point...
      turret.setTargetAzimuth(Math.PI);
      assertEquals(180.0, Math.abs(turret.getChassisAimErrorDeg(here)), 1e-9);
      assertFalse(turret.isAimed(new Pose(0, 0, Math.PI / 2)));
      assertTrue(turret.isAimed(new Pose(0, 0, Math.PI)));
      // ...and a NaN azimuth (no valid solution) falls back to the raw bearing.
      turret.setTargetAzimuth(Double.NaN);
      assertTrue(turret.isAimed(new Pose(0, 0, Math.PI / 2)));
    } finally {
      config.turret.enabled = true;
    }
  }

  /** The enabled turret keeps judging aim from its own angle, not from the chassis heading. */
  @Test
  public void testEnabledTurretAimIgnoresChassisHeading() {
    turret.setTargetPoint(100.0, 0.0);
    setPhysicalTurretAngle(0.0);
    turret.setTargetTurnAngle(0.0);
    // Chassis facing directly away from the target, but the turret is already at its commanded
    // angle: the turret, not the chassis, is what is being aimed.
    assertTrue(turret.isAimed(new Pose(0, 0, Math.PI)));
    // Commanded 10 degrees away from where it physically is: not aimed, whatever the heading.
    turret.setTargetTurnAngle(10.0);
    assertFalse(turret.isAimed(new Pose(0, 0, 0)));
  }

  @Test
  public void testShotControllerSetInitialSolution() {
    HardwareMap hardwareMap = Mockito.mock(HardwareMap.class);
    DcMotorEx mockShooter1 = Mockito.mock(DcMotorEx.class);
    DcMotorEx mockShooter2 = Mockito.mock(DcMotorEx.class);
    com.qualcomm.robotcore.hardware.Servo mockHood =
        Mockito.mock(com.qualcomm.robotcore.hardware.Servo.class);
    com.qualcomm.robotcore.hardware.VoltageSensor mockVoltage =
        Mockito.mock(com.qualcomm.robotcore.hardware.VoltageSensor.class);
    Mockito.when(hardwareMap.get(DcMotorEx.class, "shooter")).thenReturn(mockShooter1);
    Mockito.when(hardwareMap.get(DcMotorEx.class, "shooter2")).thenReturn(mockShooter2);
    Mockito.when(hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "hood"))
        .thenReturn(mockHood);
    java.util.List<com.qualcomm.robotcore.hardware.VoltageSensor> sensors =
        java.util.Collections.singletonList(mockVoltage);
    hardwareMap.voltageSensor =
        Mockito.mock(com.qualcomm.robotcore.hardware.HardwareMap.DeviceMapping.class);
    Mockito.when(hardwareMap.voltageSensor.iterator()).thenAnswer(inv -> sensors.iterator());

    org.firstinspires.ftc.teamcode.robot.Shooter shooter =
        new org.firstinspires.ftc.teamcode.robot.Shooter(hardwareMap);
    org.firstinspires.ftc.teamcode.robot.ShotController shotController =
        new org.firstinspires.ftc.teamcode.robot.ShotController(
            shooter,
            null,
            null,
            null,
            null,
            null,
            new FieldModel(Alliance.RED),
            HiveTracker.fromConfig(() -> 0L),
            null);

    try {
      // A fresh controller refuses to authorize anything until a solution has been computed.
      assertFalse(shotController.getLastSolution().isValid());

      ShotSolution primedSolution =
          new ShotSolution(
              0.65, 1150.0, 220.0, 0.0, 0.4, 0.0, 65.0, true, "Valid", 0, BallType.POLLEN);

      shotController.setInitialSolution(primedSolution);

      assertEquals(primedSolution, shotController.getLastSolution());
      assertEquals(0.65, shooter.getTargetHoodPosition(), 1e-6);

      // A null seed is ignored rather than clobbering the primed one.
      shotController.setInitialSolution(null);
      assertEquals(primedSolution, shotController.getLastSolution());
    } finally {
      shotController.shutdown();
    }
  }
}
