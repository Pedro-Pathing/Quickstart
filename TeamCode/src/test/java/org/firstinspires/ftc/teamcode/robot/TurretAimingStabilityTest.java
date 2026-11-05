package org.firstinspires.ftc.teamcode.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mockito;

/**
 * Closed-loop turret behaviour against a CRServo plant: power sets speed (K deg/s per unit power
 * above a static-friction deadband). Each test reproduces a failure the pre-BIOBUZZ loop showed in
 * the same simulation: a target outside travel reported as aimed, a 9 deg overshoot on the first
 * aim after an idle period, ~10 deg lag behind a turning chassis, and a stall fault that never
 * cleared.
 */
public class TurretAimingStabilityTest {

  private static final double K = 120.0;
  private static final double STICTION = 0.06;
  private static final long LOOP_NANOS = 20_000_000L;

  private double angle;
  private double power;
  private boolean jammed;
  private boolean plantInverted;
  private long nanos;
  private Pose pose;
  private Velocity velocity;
  private Turret turret;

  private double savedKv;
  private double savedPivotForward;
  private boolean savedRunaway;

  @BeforeEach
  public void setUp() {
    savedKv = config.turret.kv;
    savedPivotForward = config.turret.pivot_forward_in;
    savedRunaway = config.turret.runaway.enabled;
    config.turret.enabled = true;
    angle = 0.0;
    power = 0.0;
    jammed = false;
    plantInverted = false;
    nanos = 1_000_000_000L;
    pose = new Pose(0, 0, 0);
    velocity = Velocity.zero();

    HardwareMap hw = Mockito.mock(HardwareMap.class);
    CRServo servo = Mockito.mock(CRServo.class);
    AnalogInput analog = Mockito.mock(AnalogInput.class);
    Mockito.when(hw.get(CRServo.class, "turn")).thenReturn(servo);
    Mockito.when(hw.get(AnalogInput.class, "turnanalog")).thenReturn(analog);
    Mockito.doAnswer(
            inv -> {
              power = inv.getArgument(0);
              return null;
            })
        .when(servo)
        .setPower(Mockito.anyDouble());
    Mockito.when(servo.getPower()).thenAnswer(inv -> power);
    var enc = config.turret.analog_encoder;
    Mockito.when(analog.getVoltage())
        .thenAnswer(
            inv -> enc.zero_voltage + (enc.inverted ? -angle : angle) / enc.degrees_per_volt);

    turret = new Turret(hw, Mockito.mock(Telemetry.class), () -> pose, () -> velocity, () -> nanos);
  }

  @AfterEach
  public void restore() {
    config.turret.kv = savedKv;
    config.turret.pivot_forward_in = savedPivotForward;
    config.turret.runaway.enabled = savedRunaway;
  }

  /** One 20 ms loop: control update, then the plant and the chassis move. */
  private void step(double chassisRateDps) {
    nanos += LOOP_NANOS;
    turret.periodic();
    double effective =
        Math.abs(power) <= STICTION ? 0.0 : Math.signum(power) * (Math.abs(power) - STICTION);
    if (!jammed) {
      angle += (plantInverted ? -K : K) * effective * LOOP_NANOS * 1e-9;
    }
    velocity = new Velocity(0, 0, Math.toRadians(chassisRateDps));
    pose =
        new Pose(
            pose.x(),
            pose.y(),
            pose.heading() + Math.toRadians(chassisRateDps) * LOOP_NANOS * 1e-9);
  }

  private void aimAtBearing(double distance, double bearingDeg) {
    double b = Math.toRadians(bearingDeg);
    turret.setTargetPoint(distance * Math.cos(b), distance * Math.sin(b));
    turret.setAimMode(Turret.AimMode.AIM_AT_TARGET);
  }

  @Test
  public void invalidLocalizationCutsPowerAndControlRecoversWithoutReinitializing() {
    aimAtBearing(40, 20);
    step(0);
    assertTrue(Math.abs(power) > 0);
    Pose good = pose;
    for (double bad :
        new double[] {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
      for (Pose broken :
          new Pose[] {
            new Pose(bad, good.y(), good.heading()),
            new Pose(good.x(), bad, good.heading()),
            new Pose(good.x(), good.y(), bad),
            null
          }) {
        pose = broken;
        turret.periodic();
        assertEquals(
            0.0, power, 0.0, "bad pose must stop the servo instead of retaining its old command");
        pose = good;
        velocity = Velocity.zero();
        nanos += LOOP_NANOS;
        turret.periodic();
        assertTrue(Double.isFinite(power) && Math.abs(power) > 0, "the PID state must recover too");
      }
      for (Velocity broken :
          new Velocity[] {
            new Velocity(bad, 0, 0), new Velocity(0, bad, 0), new Velocity(0, 0, bad)
          }) {
        velocity = broken;
        turret.periodic();
        assertEquals(0.0, power, 0.0);
        velocity = Velocity.zero();
        nanos += LOOP_NANOS;
        turret.periodic();
        assertTrue(Double.isFinite(power) && Math.abs(power) > 0);
      }
    }
  }

  @Test
  public void nonFiniteManualPowerNeverReachesTheServo() {
    turret.setAimMode(Turret.AimMode.MANUAL);
    for (double bad :
        new double[] {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY}) {
      turret.setManualPower(bad);
      turret.periodic();
      assertEquals(0.0, power, 0.0);
    }
    turret.setManualPower(0.2);
    turret.periodic();
    assertEquals(0.2, power, 1e-12);
  }

  @Test
  public void targetOutsideTravelIsNeverReportedAimed() {
    aimAtBearing(100, 60);
    for (int i = 0; i < 60; i++) {
      step(0);
    }
    assertTrue(angle > config.turret.travel.max_angle - 3, "turret should be parked near its stop");
    assertTrue(turret.isTargetOutsideTravel());
    assertFalse(
        turret.isAimed(pose),
        "a CELL 60 deg off is not aimed just because the turret hit its stop");

    // Turning the chassis toward it brings it back inside travel, and then it aims.
    pose = new Pose(0, 0, Math.toRadians(50));
    for (int i = 0; i < 60; i++) {
      step(0);
    }
    assertFalse(turret.isTargetOutsideTravel());
    assertTrue(turret.isAimed(pose));
  }

  @Test
  public void firstAimAfterAnIdlePeriodDoesNotOvershoot() {
    aimAtBearing(140, 15);
    turret.setAimMode(Turret.AimMode.IDLE);
    turret.periodic();
    nanos += 3_000_000_000L; // three seconds idle, e.g. INIT to the first aim
    turret.setAimMode(Turret.AimMode.AIM_AT_TARGET);

    double max = 0.0;
    for (int i = 0; i < 100; i++) {
      step(0);
      max = Math.max(max, angle);
    }
    assertTrue(max < 15.0 + 1.5, "overshoot was " + (max - 15.0) + " deg");
    assertTrue(Math.abs(angle - 15.0) < 2.0, "did not settle: " + angle);
  }

  @Test
  public void feedforwardKeepsTheTurretOnTargetWhileTheChassisTurns() {
    config.turret.kv = 1.0 / K;
    double withFf = worstLagWhileTurning(30.0);
    config.turret.kv = 0.0;
    double without = worstLagWhileTurning(30.0);

    assertTrue(withFf < 3.0, "with kv the lag was " + withFf + " deg");
    assertTrue(
        withFf < without / 2.0,
        "feedforward should cut the lag substantially (" + without + " -> " + withFf + ")");
  }

  private double worstLagWhileTurning(double rateDps) {
    angle = 0.0;
    pose = new Pose(0, 0, 0);
    velocity = Velocity.zero();
    aimAtBearing(40, 0);
    for (int i = 0; i < 10; i++) {
      step(0);
    }
    double worst = 0.0;
    for (int i = 0; i < 40; i++) {
      step(rateDps);
      if (i >= 10) {
        double desired = -Math.toDegrees(pose.heading());
        worst = Math.max(worst, Math.abs(desired - angle));
      }
    }
    return worst;
  }

  @Test
  public void requiredRateCombinesChassisRotationAndTranslation() {
    // Target straight ahead 50 in away; strafing left at 10 in/s swings the bearing right at
    // 10/50 rad/s, and a 0.1 rad/s left turn adds another 0.1 rad/s the turret must undo.
    Pose robot = new Pose(0, 0, 0);
    double rate = Turret.requiredRateDps(robot, new Velocity(0, 10, 0.1), 50, 0);
    assertEquals(Math.toDegrees(-0.2 - 0.1), rate, 1e-9);
    assertEquals(0.0, Turret.requiredRateDps(robot, new Velocity(10, 0, 0), 50, 0), 1e-9);
  }

  @Test
  public void pivotOffsetMovesTheAimPointWithTheChassis() {
    config.turret.pivot_forward_in = 4.0;
    Pose robot = new Pose(10, 20, Math.toRadians(90));
    Pose pivot = Turret.pivotPose(robot);
    assertEquals(10.0, pivot.x(), 1e-9);
    assertEquals(24.0, pivot.y(), 1e-9);
    Pose back = Turret.robotPoseForPivot(pivot);
    assertEquals(robot.x(), back.x(), 1e-9);
    assertEquals(robot.y(), back.y(), 1e-9);
    // Spinning in place moves an offset pivot sideways: omega x r.
    Velocity v = Turret.pivotVelocity(robot, new Velocity(0, 0, 1.0));
    assertEquals(-4.0, v.vx, 1e-9);
    assertEquals(0.0, v.vy, 1e-9);
  }

  @Test
  public void stallFaultClearsItselfAfterTheRetryDelay() {
    jammed = true;
    aimAtBearing(60, 20);
    for (int i = 0; i < 40; i++) {
      step(0);
    }
    assertTrue(turret.isFaulted(), "a jammed turret must trip the stall watchdog");
    assertNotNull(turret.getPowerBlockedReason());
    assertEquals(0.0, power, 0.0);

    jammed = false;
    int loops = (int) Math.ceil(config.turret.stall.retry_after_sec / 0.02) + 2;
    for (int i = 0; i < loops; i++) {
      step(0);
    }
    assertFalse(turret.isFaulted(), "the fault must clear after retry_after_sec");
    for (int i = 0; i < 80; i++) {
      step(0);
    }
    assertNull(turret.getFaultReason());
    assertTrue(turret.isAimed(pose), "aiming must work again after the jam clears");
  }

  @Test
  public void settledTurretDoesNotChaseAimNoiseInsideTheExitBand() {
    aimAtBearing(30, 10);
    for (int i = 0; i < 80; i++) {
      step(0);
    }
    assertTrue(turret.isAimed(pose));
    double settledAngle = angle;
    // Pose noise of a degree or so swings the desired angle; inside the exit band the turret stays.
    double tol = turret.calculateDynamicTolerance(30.0);
    for (int i = 0; i < 50; i++) {
      double wobble = (i % 2 == 0 ? 1 : -1) * Math.min(1.0, tol * 0.4);
      pose = new Pose(0, 0, Math.toRadians(wobble));
      nanos += LOOP_NANOS;
      turret.periodic();
      assertEquals(0.0, power, 0.0, "held turret must not be driven by noise");
    }
    assertEquals(settledAngle, angle, 0.0);
  }

  /**
   * A new target that lands inside the settled hold band (but outside the aim tolerance) must still
   * be driven to. The hold band exists to ignore pose noise around the same target, not to swallow
   * a real step such as a mode switch or the solver's lead azimuth.
   */
  @Test
  public void targetStepInsideTheHoldBandIsStillTracked() {
    aimAtBearing(60, 0);
    for (int i = 0; i < 100; i++) {
      step(0);
    }
    double tol = turret.calculateDynamicTolerance(60);
    aimAtBearing(60, 1.3 * tol);
    for (int i = 0; i < 100; i++) {
      step(0);
    }
    assertEquals(1.3 * tol, angle, tol, "the turret ignored a " + 1.3 * tol + " deg target step");
  }

  /**
   * The shot gate reads isAimed() before the turret's own periodic() in the same loop. It must
   * judge the target as it is now (e.g. a lead azimuth just published), not last loop's.
   */
  @Test
  public void isAimedJudgesTheTargetAsItIsNowNotLastLoop() {
    aimAtBearing(60, 0);
    for (int i = 0; i < 100; i++) {
      step(0);
    }
    assertTrue(turret.isAimed(pose));
    double tol = turret.calculateDynamicTolerance(60);
    turret.setTargetAzimuth(Math.toRadians(1.4 * tol));
    assertFalse(turret.isAimed(pose), "aimed at a lead azimuth the turret has not moved to");
  }

  /**
   * Power the travel limit already cut is not power the servo is failing to deliver: holding a
   * manual command against the stop must not trip the stall watchdog and then block reversing.
   */
  @Test
  public void powerCutAtTheTravelLimitDoesNotTripTheStallWatchdog() {
    angle = config.turret.travel.max_angle + 0.1;
    turret.setAimMode(Turret.AimMode.MANUAL);
    turret.setManualPower(0.3);
    for (int i = 0; i < 40; i++) {
      step(0);
    }
    assertFalse(turret.isFaulted(), turret.getFaultReason());
    turret.setManualPower(-0.3);
    step(0);
    assertTrue(power < 0, "reversing off the stop was blocked");
  }

  /**
   * A servo wired backwards drives away from every target. While aiming, odometry noise moves the
   * target a hair every loop; that must not keep re-arming the runaway watchdog, or a reversed
   * servo cycles stall faults against the stop forever instead of latching RUNAWAY.
   */
  @Test
  public void reversedServoLatchesRunawayWhileAiming() {
    config.turret.runaway.enabled = true;
    plantInverted = true;
    aimAtBearing(40, 10);
    for (int i = 0; i < 100; i++) {
      pose = new Pose(0, 0, (i % 2 == 0 ? 1 : -1) * 1e-4);
      step(0);
    }
    assertTrue(turret.isFaulted());
    assertTrue(
        turret.getFaultReason().startsWith("RUNAWAY"), "fault was: " + turret.getFaultReason());
  }

  /** A jam anywhere in the travel trips the stall watchdog, not just one at the zero angle. */
  @Test
  public void aJamAwayFromZeroTripsTheStallWatchdog() {
    angle = 15.0;
    jammed = true;
    aimAtBearing(60, -10);
    for (int i = 0; i < 40; i++) {
      step(0);
    }
    assertTrue(turret.isFaulted(), "jammed at 15 deg with power on, no fault");
    assertTrue(turret.getFaultReason().startsWith("STALLED"), turret.getFaultReason());
  }
}
