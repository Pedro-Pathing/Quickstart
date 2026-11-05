package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.Nullable;
import com.pedropathing.controllers.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Locale;
import java.util.function.LongSupplier;
import java.util.function.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.records.ShotInputs;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.AntiWindupIntegrator;

/**
 * CRServo turret with an absolute analog encoder.
 *
 * <p>The servo commands speed, not position, so the turret is an integrator: power sets angular
 * velocity. The loop is therefore P (plus a static-friction step) to close position, an optional
 * velocity feedforward {@code turret.kv} for the rate the chassis motion demands, and a small
 * band-limited I term ({@link AntiWindupIntegrator}) for any steady load. Pedro's {@link
 * PIDController} carries only P and D here: its own integral is unclamped and integrates the whole
 * gap since its last call, which made the first aim after an idle period overshoot.
 *
 * <p>Aim is judged against the unclamped desired angle, so a target outside the turret's travel is
 * never reported as aimed.
 *
 * <p>Every decision is made by {@link TurretLaw}, whose contracts are machine-checked ({@code
 * ./gradlew verifyTurret}); this class reads config, hardware and the clock, calls it, keeps the
 * loop's state and the watchdogs, and writes the servo.
 */
public class Turret {
  public static final double FAULT_MARGIN_DEG = 10.0;

  private double p;
  private double i;
  private double d;
  private double f;
  private double ksPositive;
  private double ksNegative;
  private double maxPower;

  private final CRServo turnServo;
  private final AnalogInput turnAnalog;
  private final Telemetry telemetry;

  private final PIDController pidfController;
  private final AntiWindupIntegrator integrator = new AntiWindupIntegrator();

  /** Longest gap between control updates (seconds) before the loop restarts from rest. */
  public static final double MAX_CONTROL_GAP_SEC = 0.1;

  /** Where the turret is told to go: {@link #desiredTurnAngle} clamped to the travel limits. */
  private double targetTurnAngle = 0;

  /** Where the turret would need to point, unclamped. Aim is judged against this. */
  private double desiredTurnAngle = 0;

  /** Turret rate (deg/s) the chassis motion demands, for the {@code kv} feedforward. */
  private double feedforwardRateDps = 0;

  private boolean isTurnDone = false;
  private boolean settled = false;

  /** The target the turret last settled on; the wider exit band only applies around it. */
  private double settledTarget = Double.NaN;

  private long lastControlNanos = Long.MIN_VALUE;
  private int lastSaturationSign = 0;
  private double targetX;
  private double targetY;
  private final Supplier<Pose> poseSupplier;
  private final Supplier<Velocity> velocitySupplier;
  private final LongSupplier nanoClock;

  public enum AimMode {
    IDLE,
    HOLD,
    AIM_AT_TARGET,
    MANUAL
  }

  private AimMode mode = AimMode.IDLE;
  private double holdAngle = 0.0;
  private double manualPower = 0.0;

  public Turret(HardwareMap hardwareMap, Telemetry telemetry, Follower follower) {
    this(hardwareMap, telemetry, follower::pose, follower::velocity);
  }

  public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
    this(hardwareMap, telemetry, () -> new Pose(0, 0, 0));
  }

  public Turret(HardwareMap hardwareMap, Telemetry telemetry, Supplier<Pose> poseSupplier) {
    this(hardwareMap, telemetry, poseSupplier, Velocity::zero, System::nanoTime);
  }

  /**
   * @param velocitySupplier field-frame robot velocity: vx/vy in in/s, omega in rad/s CCW
   */
  public Turret(
      HardwareMap hardwareMap,
      Telemetry telemetry,
      Supplier<Pose> poseSupplier,
      Supplier<Velocity> velocitySupplier) {
    this(hardwareMap, telemetry, poseSupplier, velocitySupplier, System::nanoTime);
  }

  Turret(
      HardwareMap hardwareMap,
      Telemetry telemetry,
      Supplier<Pose> poseSupplier,
      Supplier<Velocity> velocitySupplier,
      LongSupplier nanoClock) {
    this.telemetry = telemetry;
    this.poseSupplier = poseSupplier;
    this.velocitySupplier = velocitySupplier;
    this.nanoClock = nanoClock;

    // A robot built without a turret sets turret.enabled false and may omit both devices.
    boolean required = config.turret == null || config.turret.enabled;
    turnServo =
        required
            ? hardwareMap.get(CRServo.class, "turn")
            : hardwareMap.tryGet(CRServo.class, "turn");
    turnAnalog =
        required
            ? hardwareMap.get(AnalogInput.class, "turnanalog")
            : hardwareMap.tryGet(AnalogInput.class, "turnanalog");

    if (turnServo != null && config.turret != null) {
      boolean flip = config.turret.servo_direction_inverted;
      turnServo.setDirection(flip ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);
    }

    reloadFromConfig();

    // P and D only: the I term lives in the clamped AntiWindupIntegrator.
    pidfController = new PIDController(p, 0.0, d);

    if (config.turret != null && config.turret.analog_encoder != null) {
      this.zeroVoltageOffset = config.turret.analog_encoder.zero_voltage;
    }
  }

  public final void reloadFromConfig() {
    if (config.turret != null) {
      if (config.turret.pidf != null) {
        this.p = config.turret.pidf.p;
        this.i = config.turret.pidf.i;
        this.d = config.turret.pidf.d;
        this.f = config.turret.pidf.f;
      }
      this.ksPositive = config.turret.ks_positive;
      this.ksNegative = config.turret.ks_negative;
      this.maxPower = config.turret.max_power_output;
    }
    // Null during the constructor's own call, before the coefficients exist.
    if (pidfController != null) {
      applyPIDFCoefficients();
    }
  }

  public void setPIDF(double p, double i, double d, double f) {
    this.p = p;
    this.i = i;
    this.d = d;
    this.f = f;
    applyPIDFCoefficients();
  }

  public void setKs(double ksPositive, double ksNegative) {
    this.ksPositive = ksPositive;
    this.ksNegative = ksNegative;
  }

  public double getKsPositive() {
    return ksPositive;
  }

  public double getKsNegative() {
    return ksNegative;
  }

  public void setMaxPower(double maxPower) {
    this.maxPower = maxPower;
  }

  public double getP() {
    return p;
  }

  public double getI() {
    return i;
  }

  public double getD() {
    return d;
  }

  public double getF() {
    return f;
  }

  public double getMaxPower() {
    return maxPower;
  }

  private void applyPIDFCoefficients() {
    pidfController.kP = p;
    pidfController.kI = 0.0;
    pidfController.kD = d;
  }

  private double zeroVoltageOffset = 0.0;

  public void zeroAtCurrentPosition() {
    if (turnAnalog != null) {
      this.zeroVoltageOffset = turnAnalog.getVoltage();
    }
  }

  public double getZeroVoltage() {
    return (zeroVoltageOffset != 0.0)
        ? zeroVoltageOffset
        : (config.turret != null && config.turret.analog_encoder != null
            ? config.turret.analog_encoder.zero_voltage
            : 0.0);
  }

  public void setTargetPoint(double x, double y) {
    this.targetX = x;
    this.targetY = y;
  }

  /** Sets where the turret should point (chassis-relative degrees); the command is clamped. */
  public void setTargetTurnAngle(double degrees) {
    this.desiredTurnAngle = degrees;
    this.targetTurnAngle = clampToTravel(degrees);
  }

  /** Clamps to the travel limits, in whichever order they are configured. */
  private static double clampToTravel(double degrees) {
    var travel = config.turret.travel;
    return TurretLaw.toTravel(degrees, travel.min_angle, travel.max_angle);
  }

  /** True when the desired angle is beyond the travel limits: turn the chassis instead. */
  public boolean isTargetOutsideTravel() {
    var travel = config.turret.travel;
    return desiredTurnAngle < travel.min_angle || desiredTurnAngle > travel.max_angle;
  }

  public void setHoldAngle(double angle) {
    this.holdAngle = clampToTravel(angle);
  }

  public double getHoldAngle() {
    return holdAngle;
  }

  public double getRawVoltage() {
    if (turnAnalog != null && config.turret.analog_encoder.enabled) {
      return turnAnalog.getVoltage();
    }
    return Double.NaN;
  }

  /**
   * Whether the encoder reading is believed: within {@link #FAULT_MARGIN_DEG} of the travel. A NaN
   * reading is not. With no encoder the angle reads 0, which is.
   */
  public boolean isReadingWithinTravel() {
    var travel = config.turret.travel;
    return TurretLaw.readingTrusted(
        getCurrentTurnAngle(), travel.min_angle, travel.max_angle, FAULT_MARGIN_DEG);
  }

  public double getCurrentTurnAngle() {
    if (turnAnalog != null && config.turret.analog_encoder.enabled) {
      return angleForVoltage(turnAnalog.getVoltage());
    }
    return 0.0;
  }

  public double angleForVoltage(double voltage) {
    return angleForVoltage(voltage, getZeroVoltage());
  }

  /**
   * Turret angle for an encoder voltage, given where zero is; config-only, so INIT can check it.
   */
  public static double angleForVoltage(double voltage, double zeroVoltage) {
    var enc = config.turret.analog_encoder;
    return TurretLaw.angleForVoltage(
        voltage, zeroVoltage, enc.full_scale_voltage, enc.inverted, enc.degrees_per_volt);
  }

  public double[] getReachableAngleRange() {
    return reachableAngleRange(getZeroVoltage());
  }

  private static double[] reachableAngleRange(double zeroVoltage) {
    var enc = config.turret.analog_encoder;
    double atMin = angleForVoltage(enc.min_voltage, zeroVoltage);
    double atMax = angleForVoltage(enc.max_voltage, zeroVoltage);
    return new double[] {Math.min(atMin, atMax), Math.max(atMin, atMax)};
  }

  public @Nullable String getEncoderRangeWarning() {
    return encoderRangeWarning(getZeroVoltage());
  }

  /**
   * Why the encoder calibration cannot cover the configured travel, or null: the wrap falls between
   * the stops, or a travel limit is beyond what the encoder can read.
   */
  public static @Nullable String encoderRangeWarning(double zeroVoltage) {
    var travel = config.turret.travel;
    var enc = config.turret.analog_encoder;
    double[] range = reachableAngleRange(zeroVoltage);
    StringBuilder sb = new StringBuilder();

    double expectedSpan = (enc.max_voltage - enc.min_voltage) * enc.degrees_per_volt;
    double actualSpan = range[1] - range[0];
    if (enc.full_scale_voltage > 0 && Math.abs(actualSpan - expectedSpan) > 1.0) {
      sb.append(
          String.format(
              Locale.ROOT,
              "zero_voltage %.4f puts the encoder wrap inside the turret's travel "
                  + "(stops span %.1f deg, expected %.1f deg) - angle will jump mid-travel. ",
              zeroVoltage,
              actualSpan,
              expectedSpan));
    }

    if (range[1] < travel.max_angle) {
      sb.append(
          String.format(
              Locale.ROOT,
              "max_angle %.1f unreachable (encoder tops out at %.1f). ",
              travel.max_angle,
              range[1]));
    }
    if (range[0] > travel.min_angle) {
      sb.append(
          String.format(
              Locale.ROOT,
              "min_angle %.1f unreachable (encoder bottoms out at %.1f). ",
              travel.min_angle,
              range[0]));
    }
    return sb.length() == 0 ? null : sb.toString().trim();
  }

  private @Nullable Boolean enabledOverride = null;

  public boolean isEnabled() {
    if (enabledOverride != null) {
      return enabledOverride;
    }
    return config.turret != null && config.turret.enabled;
  }

  public void setEnabled(boolean enabled) {
    this.enabledOverride = enabled;
  }

  public boolean isTurnDone() {
    if (!isEnabled()) {
      return true;
    }
    return isTurnDone;
  }

  private boolean wouldExceedTurnBoundary(double power) {
    var travel = config.turret.travel;
    return TurretLaw.pushesPastTravel(
        power, getCurrentTurnAngle(), travel.min_angle, travel.max_angle);
  }

  public @Nullable String getPowerBlockedReason() {
    return powerBlockedReason;
  }

  private @Nullable String powerBlockedReason = null;

  public void clearFault() {
    faultReason = null;
    faultIsStall = false;
    stallReferenceAngle = Double.NaN;
    runawayTarget = Double.NaN;
  }

  public boolean isFaulted() {
    return faultReason != null;
  }

  public @Nullable String getFaultReason() {
    return faultReason;
  }

  private @Nullable String faultReason = null;
  private boolean faultIsStall = false;
  private long faultNanos = 0L;
  private double stallReferenceAngle = Double.NaN;
  private long stallReferenceTimeNano = 0L;
  private double runawayTarget = Double.NaN;
  private double runawayBestAbsError = Double.MAX_VALUE;

  private void updateRunawayWatchdog(double target, double error, double command) {
    var runaway = config.turret.runaway;
    if (!runaway.enabled || Double.isNaN(getRawVoltage()) || isFaulted()) {
      return;
    }

    // Re-arm only on a real target change. In AIM mode odometry noise moves the target a hair every
    // loop; re-arming on that kept a reversed servo from ever latching RUNAWAY. Half the divergence
    // keeps a lagging-but-correct turret on a moving target from tripping it.
    if (Double.isNaN(runawayTarget)
        || Math.abs(target - runawayTarget) > runaway.divergence_deg / 2.0) {
      runawayTarget = target;
      runawayBestAbsError = Math.abs(error);
      return;
    }

    if (Math.abs(command) < config.turret.stall.power_threshold) {
      return;
    }

    double absError = Math.abs(error);
    runawayBestAbsError = Math.min(runawayBestAbsError, absError);
    if (absError > runawayBestAbsError + runaway.divergence_deg) {
      faultReason =
          String.format(
              Locale.ROOT,
              "RUNAWAY - error grew %.1f -> %.1f deg under power. The loop is driving away from "
                  + "its target: check servo_direction_inverted vs analog_encoder.inverted.",
              runawayBestAbsError,
              absError);
    }
  }

  private void updateStallWatchdog(double commandedPower) {
    var stall = config.turret.stall;
    if (!stall.enabled || Double.isNaN(getRawVoltage()) || isFaulted()) {
      return;
    }

    long now = nanoClock.getAsLong();
    double angle = getCurrentTurnAngle();

    if (Math.abs(commandedPower) < stall.power_threshold) {
      stallReferenceAngle = Double.NaN;
      return;
    }

    if (Double.isNaN(stallReferenceAngle)) {
      stallReferenceAngle = angle;
      stallReferenceTimeNano = now;
      return;
    }

    if (Math.abs(angle - stallReferenceAngle) >= stall.min_progress_deg) {
      stallReferenceAngle = angle;
      stallReferenceTimeNano = now;
      return;
    }

    if ((now - stallReferenceTimeNano) / 1e9 >= stall.timeout_sec) {
      faultIsStall = true;
      faultNanos = now;
      faultReason =
          String.format(
              Locale.ROOT,
              "STALLED at %.1f deg - power commanded but the turret is not moving.",
              getCurrentTurnAngle());
    }
  }

  /**
   * A stall fault protects the servo while something holds the turret; it clears itself after
   * {@code stall.retry_after_sec} so a ball jam or a defender does not disable aiming for the rest
   * of the match. A runaway fault (inverted loop) stays latched.
   */
  private void retryStallFaultIfDue() {
    double retry = config.turret.stall.retry_after_sec;
    if (faultIsStall && retry > 0 && (nanoClock.getAsLong() - faultNanos) / 1e9 >= retry) {
      clearFault();
    }
  }

  /**
   * Gives the servo {@code requested} power through {@link TurretLaw#gate}: zero when the request
   * is not finite, the turret is disabled or faulted, the encoder is not believed, or the power
   * would push past a travel limit; otherwise the request, within the servo's +/-1. The stall
   * watchdog then judges the power actually delivered.
   */
  public void setTurretPowerRaw(double requested) {
    boolean enabled = isEnabled();
    boolean finite = Double.isFinite(requested);
    if (finite && enabled) {
      retryStallFaultIfDue();
    }
    var travel = config.turret.travel;
    double angle = getCurrentTurnAngle();
    double power =
        TurretLaw.gate(
            requested,
            SERVO_POWER_LIMIT,
            enabled,
            isFaulted(),
            angle,
            travel.min_angle,
            travel.max_angle,
            FAULT_MARGIN_DEG);
    powerBlockedReason = blockedReason(finite, enabled, angle, requested);

    if (finite && enabled) {
      // Judge only the power actually delivered: power the travel limit already cut is not the
      // servo failing to move, and faulting on it blocked reversing off the stop.
      updateStallWatchdog(power);
      if (isFaulted() && power != 0) {
        powerBlockedReason = faultReason;
        power = 0;
      }
    }

    if (turnServo != null) {
      boolean flip = config.turret != null && config.turret.servo_direction_inverted;
      CRServo.Direction targetDir = flip ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD;
      if (turnServo.getDirection() != targetDir) {
        turnServo.setDirection(targetDir);
      }
      turnServo.setPower(power);
    }
  }

  /** The CRServo's own power range; closed-loop commands are already within max_power_output. */
  private static final double SERVO_POWER_LIMIT = 1.0;

  /** Why {@link TurretLaw#gate} cut the power, in the order it checks; null if it did not. */
  private @Nullable String blockedReason(
      boolean finite, boolean enabled, double angle, double requested) {
    var travel = config.turret.travel;
    if (!finite) {
      return "INVALID TURRET COMMAND";
    }
    if (!enabled) {
      return "TURRET DISABLED";
    }
    if (isFaulted()) {
      return faultReason;
    }
    if (!TurretLaw.readingTrusted(angle, travel.min_angle, travel.max_angle, FAULT_MARGIN_DEG)) {
      return String.format(
          Locale.ROOT,
          "ENCODER OUT OF RANGE (%.1f deg) - reading is not trusted, power cut",
          angle);
    }
    if (TurretLaw.pushesPastTravel(requested, angle, travel.min_angle, travel.max_angle)) {
      return "AT TRAVEL LIMIT";
    }
    return null;
  }

  public static Pose alignPose(double x, double y, double targetX, double targetY) {
    double deltaX = targetX - x;
    double deltaY = targetY - y;
    double angleRadians = Math.atan2(deltaY, deltaX);
    return new Pose(x, y, angleRadians);
  }

  /**
   * Aim tolerance at a distance (see {@link TurretLaw#tolerance}). A tuning pass that sets max_deg
   * below min_deg degrades instead of throwing from periodic(); ConfigSanity reports it at INIT.
   */
  public double calculateDynamicTolerance(double distanceInches) {
    var tol = config.turret.tolerance;
    return TurretLaw.tolerance(
        distanceInches, tol.near_cutoff, tol.near_val, tol.max_drift, tol.min_deg, tol.max_deg);
  }

  /**
   * One control update toward {@link #getTargetTurnAngle()}. The loop restarts from rest if it has
   * not run for {@link #MAX_CONTROL_GAP_SEC}, so time spent idle is never integrated.
   */
  public void updateTurret(Pose currentPose) {
    double relativeTurretAngle = getCurrentTurnAngle();
    if (!ShotInputs.isFinitePose(currentPose)
        || !Double.isFinite(targetTurnAngle)
        || !Double.isFinite(relativeTurretAngle)
        || !Double.isFinite(feedforwardRateDps)) {
      resetControl();
      lastControlNanos = Long.MIN_VALUE;
      isTurnDone = false;
      setTurretPowerRaw(0);
      powerBlockedReason = "INVALID AIM INPUT";
      return;
    }
    long now = nanoClock.getAsLong();
    double dt =
        lastControlNanos == Long.MIN_VALUE
            ? Double.POSITIVE_INFINITY
            : (now - lastControlNanos) * 1e-9;
    lastControlNanos = now;
    if (dt > MAX_CONTROL_GAP_SEC) {
      resetControl();
      dt = 0.0;
    }

    double relativeTargetAngle = targetTurnAngle;

    double error = TurretLaw.error(relativeTargetAngle, relativeTurretAngle);
    double tolerance = calculateDynamicTolerance(distanceToTarget(currentPose));
    double absError = Math.abs(error);

    // Hysteresis: once settled, leave the turret alone until the error clearly grows again. The
    // wider exit band only covers noise around the target it settled on: a new target more than a
    // tolerance away (a mode switch, the solver's lead azimuth) is judged afresh and driven to.
    double nextSettledTarget =
        TurretLaw.settledTargetAfter(
            settled, settledTarget, relativeTargetAngle, absError, tolerance);
    settled =
        TurretLaw.settledAfter(
            settled, settledTarget, relativeTargetAngle, absError, tolerance, exitFactor());
    settledTarget = nextSettledTarget;

    var turret = config.turret;
    double command;
    if (settled) {
      // Hold (TurretLaw.settledCommand): follow a moving target, leave a still one alone.
      pidfController.reset();
      integrator.reset();
      command =
          TurretLaw.settledCommand(
              error,
              feedforwardRateDps,
              turret.kv,
              turret.ff_min_rate_dps,
              p,
              ksPositive,
              ksNegative);
    } else {
      double integral =
          integrator.update(
              error,
              dt,
              i,
              turret.integral.band_deg,
              turret.integral.limit_power,
              lastSaturationSign);
      double raw =
          TurretLaw.drivingCommand(
              error,
              pidfController.calculate(relativeTargetAngle, error),
              integral,
              feedforwardRateDps,
              turret.kv,
              f,
              ksPositive,
              ksNegative);
      lastSaturationSign = TurretLaw.saturationSign(raw, maxPower);
      command = raw;
    }
    command = TurretLaw.limit(command, maxPower);

    updateRunawayWatchdog(relativeTargetAngle, error, command);

    boolean isBoundaryViolated = wouldExceedTurnBoundary(command);
    if (isBoundaryViolated) {
      integrator.reset();
      command = 0.0;
    }
    isTurnDone = settled;
    setTurretPowerRaw(command);

    if (telemetry != null) {
      telemetry.addData("Robot Heading", Math.toDegrees(currentPose.heading()));
      telemetry.addData("Target Relative", relativeTargetAngle);
      telemetry.addData("Turret Desired", desiredTurnAngle);
      telemetry.addData("Turret Relative", relativeTurretAngle);
      telemetry.addData("Turret Power", turnServo != null ? turnServo.getPower() : 0.0);
      telemetry.addData("Turret Error", error);
      telemetry.addData("Turret Tolerance", tolerance);
      telemetry.addData("Turret Done", isTurnDone);
      telemetry.addData("Turret FF Rate (deg/s)", feedforwardRateDps);
      telemetry.addData("Turret At Limit", isBoundaryViolated);
      telemetry.addData("Turret Target Outside Travel", isTargetOutsideTravel());
      telemetry.addData("Turret Encoder Fault", !isReadingWithinTravel());
      telemetry.addData(
          "Turret Power Blocked", powerBlockedReason == null ? "no" : powerBlockedReason);
      telemetry.addData("Encoder Raw Voltage", getRawVoltage());
      telemetry.addData("Encoder Live Angle", getCurrentTurnAngle());
    }
  }

  private void resetControl() {
    pidfController.reset();
    integrator.reset();
    settled = false;
    lastSaturationSign = 0;
  }

  private static double exitFactor() {
    return TurretLaw.exitFactor(config.turret.tolerance.exit_factor);
  }

  /** Field position of the turret axis for a robot pose (heading unchanged). */
  public static Pose pivotPose(Pose robot) {
    double c = Math.cos(robot.heading());
    double s = Math.sin(robot.heading());
    double fwd = config.turret.pivot_forward_in;
    double left = config.turret.pivot_left_in;
    return new Pose(
        robot.x() + fwd * c - left * s, robot.y() + fwd * s + left * c, robot.heading());
  }

  /** Robot pose that puts the turret axis at {@code pivot} (heading unchanged). */
  public static Pose robotPoseForPivot(Pose pivot) {
    double c = Math.cos(pivot.heading());
    double s = Math.sin(pivot.heading());
    double fwd = config.turret.pivot_forward_in;
    double left = config.turret.pivot_left_in;
    return new Pose(
        pivot.x() - fwd * c + left * s, pivot.y() - fwd * s - left * c, pivot.heading());
  }

  /**
   * Field velocity of the turret axis: the robot centre's velocity plus omega x offset.
   *
   * <p>Velocities are Pedro {@link Velocity}s, never {@link Pose}s: a Pose wraps its heading into
   * [0, 2pi), which turns a clockwise yaw rate of -1 rad/s into +5.28 rad/s.
   */
  public static Velocity pivotVelocity(Pose robot, Velocity robotVelocity) {
    Pose pivot = pivotPose(robot);
    double ox = pivot.x() - robot.x();
    double oy = pivot.y() - robot.y();
    double omega = robotVelocity.omega;
    return new Velocity(robotVelocity.vx - omega * oy, robotVelocity.vy + omega * ox, omega);
  }

  /**
   * Turret rate (deg/s, chassis-relative) needed to stay on a fixed field point while the robot
   * moves: the bearing's rate of change minus the chassis rotation.
   */
  public static double requiredRateDps(
      Pose pivot, Velocity pivotVelocity, double targetX, double targetY) {
    double rx = targetX - pivot.x();
    double ry = targetY - pivot.y();
    double r2 = rx * rx + ry * ry;
    double bearingRate = r2 > 1e-6 ? (ry * pivotVelocity.vx - rx * pivotVelocity.vy) / r2 : 0.0;
    return Math.toDegrees(bearingRate - pivotVelocity.omega);
  }

  private double distanceToTarget(Pose currentPose) {
    Pose pivot = pivotPose(currentPose);
    return Math.hypot(targetX - pivot.x(), targetY - pivot.y());
  }

  public double getTargetTurnAngle() {
    return targetTurnAngle;
  }

  /** Degrees between where the turret points and where it should point (unclamped). */
  public double getAimError(Pose currentPose) {
    return TurretLaw.error(desiredTurnAngle, getCurrentTurnAngle());
  }

  /**
   * Whether the shooter is pointed at the target. With the turret disabled the shooter is fixed to
   * the chassis, so this checks the robot heading against the target bearing instead (Casablanca's
   * armed aim turns the chassis there), allowing for the heading lock's own deadband. A disabled
   * turret must never report aimed while the chassis faces away from the CELL.
   */
  public boolean isAimed(Pose currentPose) {
    if (!isEnabled()) {
      return Math.abs(getChassisAimErrorDeg(currentPose))
          < calculateDynamicTolerance(distanceToTarget(currentPose))
              + config.casablanca.heading_lock.error_deadband_deg;
    }
    // The shot gate calls this before this loop's periodic(), so judge the target as it is now (a
    // lead azimuth or hold angle set this loop), not the one periodic() last computed.
    double desired = currentDesiredAngle(currentPose);
    var travel = config.turret.travel;
    // Same band the controller holds in, so the shot gate does not flicker at the edge.
    return TurretLaw.aimed(
        desired,
        getCurrentTurnAngle(),
        travel.min_angle,
        travel.max_angle,
        calculateDynamicTolerance(distanceToTarget(currentPose)),
        settled,
        settledTarget,
        exitFactor());
  }

  /** Chassis-relative angle the turret should point at for this pose in the current mode. */
  private double currentDesiredAngle(Pose currentPose) {
    if (mode == AimMode.HOLD) {
      return holdAngle;
    }
    if (mode == AimMode.AIM_AT_TARGET && ShotInputs.isFinitePose(currentPose)) {
      return TurretLaw.wrap(
          worldBearingDegrees(currentPose) - Math.toDegrees(currentPose.heading()));
    }
    return desiredTurnAngle;
  }

  /**
   * Degrees from the robot heading to the target: the lead-compensated azimuth when one is set,
   * otherwise the raw bearing to the target point.
   */
  public double getChassisAimErrorDeg(Pose currentPose) {
    return TurretLaw.wrap(worldBearingDegrees(currentPose) - Math.toDegrees(currentPose.heading()));
  }

  /** Field bearing (deg) from the turret axis to the target: the lead azimuth when one is set. */
  private double worldBearingDegrees(Pose currentPose) {
    if (!Double.isNaN(targetAzimuthRad)) {
      return Math.toDegrees(targetAzimuthRad);
    }
    Pose pivot = pivotPose(currentPose);
    return Math.toDegrees(Math.atan2(targetY - pivot.y(), targetX - pivot.x()));
  }

  public void setAimMode(AimMode mode) {
    this.mode = mode;
  }

  public AimMode getAimMode() {
    return mode;
  }

  private double targetAzimuthRad = Double.NaN;

  public void setTargetAzimuth(double azimuthRad) {
    this.targetAzimuthRad = azimuthRad;
  }

  public void setManualPower(double power) {
    this.manualPower = power;
  }

  public void periodic() {
    if (!isEnabled()) {
      lastControlNanos = Long.MIN_VALUE;
      setTurretPowerRaw(0);
      return;
    }
    switch (mode) {
      case IDLE:
        lastControlNanos = Long.MIN_VALUE;
        setTurretPowerRaw(0);
        break;
      case HOLD:
        setTargetTurnAngle(holdAngle);
        feedforwardRateDps = 0.0;
        updateTurret(poseSupplier.get());
        break;
      case AIM_AT_TARGET:
        Pose pose = poseSupplier.get();
        if (!ShotInputs.isFinitePose(pose)) {
          updateTurret(pose);
          break;
        }
        if (pose != null) {
          double robotWorldHeadingDegrees = Math.toDegrees(pose.heading());
          setTargetTurnAngle(TurretLaw.wrap(worldBearingDegrees(pose) - robotWorldHeadingDegrees));
          Velocity velocity = velocitySupplier.get();
          feedforwardRateDps =
              velocity == null || isTargetOutsideTravel()
                  ? 0.0
                  : requiredRateDps(
                      pivotPose(pose), pivotVelocity(pose, velocity), targetX, targetY);
          updateTurret(pose);
        }
        break;
      case MANUAL:
        lastControlNanos = Long.MIN_VALUE;
        setTurretPowerRaw(manualPower);
        break;
    }
  }
}
