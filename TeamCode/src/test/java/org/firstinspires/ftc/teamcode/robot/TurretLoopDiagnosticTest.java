package org.firstinspires.ftc.teamcode.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.Mockito;

/** Diagnostic unit test suite verifying closed-loop control of the simplified {@link Turret}. */
public class TurretLoopDiagnosticTest {

  private static final long LOOP_MS = 20;
  private static final double DEG_PER_LOOP_AT_FULL = 2.4;
  private static final double STICTION = 0.06;

  private Turret turret;
  private double[] voltageBox;
  private double[] cmdPower;

  /** The turret's clock: loops advance it instead of sleeping. */
  private long nanos = 1_000_000_000L;

  private CRServo.Direction[] cmdDir;

  @BeforeEach
  public void setUp() {
    HardwareMap hardwareMap = Mockito.mock(HardwareMap.class);
    DcMotorEx mockMotor = Mockito.mock(DcMotorEx.class);

    // Plain fakes for the devices every simulated loop reads and writes: a Mockito invocation
    // records a stack trace each call, which dominated this suite's run time.
    cmdPower = new double[] {0.0};
    cmdDir = new CRServo.Direction[] {CRServo.Direction.FORWARD};
    CRServo servo =
        new CRServoImpl(null, 0) {
          @Override
          public void setPower(double power) {
            cmdPower[0] = power;
          }

          @Override
          public double getPower() {
            return cmdPower[0];
          }

          @Override
          public synchronized void setDirection(Direction direction) {
            cmdDir[0] = direction;
          }

          @Override
          public synchronized Direction getDirection() {
            return cmdDir[0];
          }
        };
    Mockito.when(hardwareMap.get(CRServo.class, "turn")).thenReturn(servo);

    voltageBox = new double[] {config.turret.analog_encoder.zero_voltage};
    AnalogInput analog =
        new AnalogInput(null, 0) {
          @Override
          public double getVoltage() {
            return voltageBox[0];
          }
        };
    Mockito.when(hardwareMap.get(AnalogInput.class, "turnanalog")).thenReturn(analog);

    Mockito.when(hardwareMap.get(DcMotorEx.class, "leftFront")).thenReturn(mockMotor);
    Mockito.when(hardwareMap.get(DcMotorEx.class, "leftBack")).thenReturn(mockMotor);
    Mockito.when(hardwareMap.get(DcMotorEx.class, "rightFront")).thenReturn(mockMotor);
    Mockito.when(hardwareMap.get(DcMotorEx.class, "rightBack")).thenReturn(mockMotor);

    // config.turret.enabled defaults to false on a freshly-built robot; this suite exercises the
    // closed-loop control math, not the enabled flag, so force it on regardless of test order.
    config.turret.enabled = true;

    turret =
        new Turret(
            hardwareMap, (Telemetry) null, () -> new Pose(0, 0, 0), Velocity::zero, () -> nanos);
  }

  private void advance(long ms) {
    nanos += ms * 1_000_000L;
  }

  private void setPhysicalTurretAngle(double deg) {
    var enc = config.turret.analog_encoder;
    double delta = enc.inverted ? -deg / enc.degrees_per_volt : deg / enc.degrees_per_volt;
    voltageBox[0] = enc.zero_voltage + delta;
  }

  private double signedCommandedPower() {
    return cmdPower[0];
  }

  private double steadyStatePowerForError(double errorDeg, Pose pose) {
    setPhysicalTurretAngle(0.0);
    turret.setTargetTurnAngle(errorDeg);
    advance(LOOP_MS);
    turret.updateTurret(pose);
    setPhysicalTurretAngle(0.0);
    advance(LOOP_MS);
    turret.updateTurret(pose);
    return signedCommandedPower();
  }

  @Test
  public void commandedPowerScalesWithError() {
    turret.setTargetPoint(10000.0, 0.0);
    Pose pose = new Pose(0, 0, 0);

    double smallErr = 5.0;
    double midErr = Math.min(15.0, config.turret.travel.max_angle * 0.5);
    double maxErr = config.turret.travel.max_angle;

    double atSmall = steadyStatePowerForError(smallErr, pose);
    double atMid = steadyStatePowerForError(midErr, pose);
    double atMax = steadyStatePowerForError(maxErr, pose);

    assertTrue(
        atMid > atSmall, "power must grow with error (small=" + atSmall + ", mid=" + atMid + ")");
    assertTrue(atMax >= atMid, "power must grow with error (mid=" + atMid + ", max=" + atMax + ")");
    // Deliberately not asserting an absolute power for a given error: that is a statement about
    // how aggressively the turret happens to be tuned, and it fails whenever p is legitimately
    // retuned. What must hold regardless of gains is that power never leaves the clamp.
    for (double p : new double[] {atSmall, atMid, atMax}) {
      assertTrue(
          Math.abs(p) <= config.turret.max_power_output + 1e-9,
          "commanded power must stay within max_power_output, got " + p);
    }
  }

  @Test
  public void closedLoopHoldConvergesWithoutOscillating() {
    turret.setTargetPoint(10000.0, 0.0);
    turret.setHoldAngle(20.0);
    turret.setAimMode(Turret.AimMode.HOLD);

    double angle = 0.0;
    setPhysicalTurretAngle(angle);

    final int loops = 120;
    final int settleFrom = 80;
    double minAfterSettle = Double.MAX_VALUE;
    double maxAfterSettle = -Double.MAX_VALUE;

    for (int i = 0; i < loops; i++) {
      advance(LOOP_MS);
      turret.periodic();
      double signed = signedCommandedPower();
      angle += Math.abs(signed) < STICTION ? 0.0 : signed * DEG_PER_LOOP_AT_FULL;
      setPhysicalTurretAngle(angle);

      if (i >= settleFrom) {
        minAfterSettle = Math.min(minAfterSettle, angle);
        maxAfterSettle = Math.max(maxAfterSettle, angle);
      }
    }

    assertEquals(20.0, angle, 3.5, "turret should settle near its 20 deg target");
    assertEquals(0.0, maxAfterSettle - minAfterSettle, 1e-9, "settled turret must not oscillate");
  }

  /**
   * Guards the live-tuner bug: {@code PIDFController.run()} re-reads its gains from the supplier it
   * was constructed with, so handing it a brand new coefficients object had no effect and the
   * turret kept driving on the gains baked in at construction even with every dashboard gain at 0.
   */
  @Test
  public void zeroedGainsFromTheLiveTunerStopTheTurret() {
    turret.setTargetPoint(10000.0, 0.0);
    Pose pose = new Pose(0, 0, 0);

    double movingPower = steadyStatePowerForError(config.turret.travel.max_angle, pose);
    assertTrue(Math.abs(movingPower) > 0, "baseline: config gains must actually drive the turret");

    // kS is added outside the PIDF, so zeroing the gains leaves exactly the stiction term.
    turret.setPIDF(0.0, 0.0, 0.0, 0.0);
    turret.setKs(0.0, 0.0);
    double zeroedPower = steadyStatePowerForError(config.turret.travel.max_angle, pose);

    assertEquals(0.0, zeroedPower, 1e-9, "all-zero gains must command no power");
  }

  /** kS is a directional stiction term added on top of the PIDF output. */
  @Test
  public void ksAloneDrivesTheTurretTowardTheTarget() {
    turret.setTargetPoint(10000.0, 0.0);
    Pose pose = new Pose(0, 0, 0);

    turret.setPIDF(0.0, 0.0, 0.0, 0.0);
    turret.setKs(0.08, 0.05);

    double positivePower = steadyStatePowerForError(config.turret.travel.max_angle, pose);
    assertEquals(0.08, positivePower, 1e-9, "a positive target must command +ks_positive");

    double negativePower = steadyStatePowerForError(config.turret.travel.min_angle, pose);
    assertEquals(-0.05, negativePower, 1e-9, "a negative target must command -ks_negative");
  }

  /**
   * kS must not also be routed through the PIDF's F term. {@code run()} computes {@code
   * feedForwardInput * F}, and the loop already adds the same kS to the output separately, so a
   * non-zero f silently double-counts it.
   */
  @Test
  public void nonZeroFGainDoesNotDoubleCountKs() {
    turret.setTargetPoint(10000.0, 0.0);
    Pose pose = new Pose(0, 0, 0);

    turret.setKs(0.08, 0.08);
    turret.setPIDF(0.0, 0.0, 0.0, 0.0);
    double withoutF = steadyStatePowerForError(config.turret.travel.max_angle, pose);

    turret.setPIDF(0.0, 0.0, 0.0, 0.05);
    double withF = steadyStatePowerForError(config.turret.travel.max_angle, pose);

    assertEquals(withoutF + 0.05, withF, 1e-9, "f must add its own constant, not scale kS by f");
  }

  /**
   * An out-of-range reading must cut power in BOTH directions. Deciding which way is "back toward
   * travel" needs the encoder angle, and this is precisely the case where that reading cannot be
   * trusted -- letting the loop auto-recover here drove the turret into its hard stop.
   */
  @Test
  public void encoderReadingOutsideFaultWindowCutsPowerInBothDirections() {
    for (double faulted :
        new double[] {
          config.turret.travel.min_angle - Turret.FAULT_MARGIN_DEG - 2.0,
          config.turret.travel.max_angle + Turret.FAULT_MARGIN_DEG + 2.0
        }) {
      setPhysicalTurretAngle(faulted);
      assertTrue(!turret.isReadingWithinTravel(), "test setup must be outside the fault window");

      for (double power : new double[] {0.4, -0.4}) {
        turret.setTurretPowerRaw(power);
        assertEquals(
            0.0,
            signedCommandedPower(),
            1e-9,
            "power must be cut at " + faulted + " deg for command " + power);
        assertNotNull(turret.getPowerBlockedReason(), "blocking must be reported");
      }
    }
  }

  /**
   * The protection that was missing when the turret was driven into its hard stop: every
   * position-based guard trusts the encoder, so none of them fire when the encoder is the thing
   * that is wrong. Lack of progress under power does not depend on the reading being correct.
   */
  @Test
  public void stallWatchdogCutsPowerWhenDrivenWithoutMoving() {
    var stall = config.turret.stall;
    org.junit.jupiter.api.Assumptions.assumeTrue(stall.enabled, "stall watchdog must be enabled");

    // Well inside the valid window, so only the watchdog can stop this.
    setPhysicalTurretAngle(0.0);
    double power = Math.max(stall.power_threshold + 0.05, 0.2);

    long deadline = nanos + (long) ((stall.timeout_sec + 0.5) * 1e9);
    while (nanos < deadline && !turret.isFaulted()) {
      turret.setTurretPowerRaw(power); // angle never changes -> no progress
      advance(10);
    }

    assertTrue(turret.isFaulted(), "watchdog must latch a stall");
    assertEquals(0.0, signedCommandedPower(), 1e-9, "stalled turret must be unpowered");
    assertNotNull(turret.getPowerBlockedReason(), "stall must be reported");

    turret.clearFault();
    turret.setTurretPowerRaw(power);
    assertEquals(power, signedCommandedPower(), 1e-9, "clearFault must restore control");
  }

  @Test
  public void stallWatchdogDoesNotTripWhileTheTurretIsMoving() {
    var stall = config.turret.stall;
    org.junit.jupiter.api.Assumptions.assumeTrue(stall.enabled, "stall watchdog must be enabled");

    double power = Math.max(stall.power_threshold + 0.05, 0.2);
    double angle = 0.0;
    double step = Math.max(stall.min_progress_deg * 2.0, 0.1);

    long deadline = nanos + (long) ((stall.timeout_sec * 3.0) * 1e9);
    while (nanos < deadline) {
      angle += step;
      if (angle > config.turret.travel.max_angle) {
        angle = config.turret.travel.min_angle;
      }
      setPhysicalTurretAngle(angle);
      turret.setTurretPowerRaw(power);
      advance(10);
    }

    assertTrue(!turret.isFaulted(), "a moving turret must never be flagged stalled");
  }

  @Test
  public void angleIsContinuousAcrossThePotentiometerWrapPoint() {
    var enc = config.turret.analog_encoder;
    org.junit.jupiter.api.Assumptions.assumeTrue(
        enc.full_scale_voltage > 0, "wrap handling must be enabled");

    double center = enc.zero_voltage;
    assertEquals(0.0, turret.angleForVoltage(center), 0.5, "center must read zero");

    double atLowStop = turret.angleForVoltage(enc.min_voltage);
    double atHighStop = turret.angleForVoltage(enc.max_voltage);

    assertTrue(
        Math.signum(atLowStop) != Math.signum(atHighStop),
        "stops must straddle center, got " + atLowStop + " and " + atHighStop);
    assertEquals(
        Math.abs(atLowStop), Math.abs(atHighStop), 2.0, "stops must be symmetric about center");

    var travel = config.turret.travel;
    double limit = travel.max_angle + Turret.FAULT_MARGIN_DEG;
    assertTrue(Math.abs(atLowStop) <= limit, "stop " + atLowStop + " must be inside +/-" + limit);
    assertTrue(Math.abs(atHighStop) <= limit, "stop " + atHighStop + " must be inside +/-" + limit);
  }

  @Test
  public void encoderRangeWarningFlagsUnreachableTravelLimit() {
    double[] range = turret.getReachableAngleRange();
    assertTrue(range[0] < range[1], "reachable range should be ordered");

    var travel = config.turret.travel;
    boolean limitsReachable = range[0] <= travel.min_angle && range[1] >= travel.max_angle;
    if (limitsReachable) {
      String warning = turret.getEncoderRangeWarning();
      if (warning != null) {
        assertFalse(
            warning.contains("unreachable"),
            "travel limits should not be reported unreachable when reachable");
      }
    } else {
      assertNotNull(turret.getEncoderRangeWarning(), "unreachable travel limits must be reported");
    }
  }

  /**
   * An inverted loop (servo_direction_inverted disagreeing with analog_encoder.inverted) drives the
   * turret away from its target and into a hard stop, with every reading staying plausible the
   * whole way. Only the growing error gives it away.
   */
  @Test
  public void runawayWatchdogCutsPowerWhenErrorGrowsUnderPower() {
    org.junit.jupiter.api.Assumptions.assumeTrue(
        config.turret.runaway.enabled, "runaway watchdog must be enabled");
    turret.setTargetPoint(10000.0, 0.0);
    Pose pose = new Pose(0, 0, 0);

    turret.setTargetTurnAngle(config.turret.travel.max_angle);

    // Simulate an inverted plant: the turret walks away from the target instead of toward it.
    double angle = 0.0;
    for (int i = 0; i < 200 && !turret.isFaulted(); i++) {
      setPhysicalTurretAngle(angle);
      advance(5);
      turret.updateTurret(pose);
      angle = Math.max(angle - 1.0, config.turret.travel.min_angle);
    }

    assertTrue(turret.isFaulted(), "watchdog must latch on a diverging error");
    assertEquals(0.0, signedCommandedPower(), 1e-9, "runaway turret must be unpowered");
    assertTrue(
        turret.getFaultReason().contains("RUNAWAY"),
        "fault must name the inverted-loop cause, got " + turret.getFaultReason());
  }

  @Test
  public void runawayWatchdogToleratesNormalConvergence() {
    org.junit.jupiter.api.Assumptions.assumeTrue(
        config.turret.runaway.enabled, "runaway watchdog must be enabled");
    turret.setTargetPoint(10000.0, 0.0);
    turret.setHoldAngle(20.0);
    turret.setAimMode(Turret.AimMode.HOLD);

    double angle = 0.0;
    setPhysicalTurretAngle(angle);
    for (int i = 0; i < 120; i++) {
      advance(LOOP_MS);
      turret.periodic();
      double signed = signedCommandedPower();
      angle += Math.abs(signed) < STICTION ? 0.0 : signed * DEG_PER_LOOP_AT_FULL;
      setPhysicalTurretAngle(angle);
    }

    assertTrue(
        !turret.isFaulted(),
        "a converging turret must never fault, got " + turret.getFaultReason());
  }
}
