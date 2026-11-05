package org.firstinspires.ftc.teamcode.utilities;

import androidx.annotation.Nullable;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.controllers.PIDController;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.SlewProfile;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.locationtech.jts.geom.Envelope;

@Configurable
public class Casablanca {

  public static boolean enableFrictionComp = true;
  public static double frictionX;
  public static double frictionY;
  public static double frictionRot;

  public static boolean enableInputSmoothing = true;

  public static double wallRepulsionPower;
  public static double decelSafetyFactor;

  public static boolean enableDepthProtection = true;
  public static double depthSlowDown;
  public static double depthHardStop;

  public static boolean enableSideProtection = true;
  public static double sideSlowDown;
  public static double sideHardStop;

  public static double laneBlendDistance;
  public static double rotationLookaheadTimeSeconds;

  public static boolean fieldCentric;
  public static double fieldCentricOffsetRad;

  public static boolean enableHeadingLock;
  public static double headingLockIntentThreshold;
  public static double headingLockKsMoving;
  public static double headingLockMovingSpeedThreshold;
  public static double headingLockMaxPower;
  public static double headingLockErrorDeadbandRad;
  public static double headingLockSettleRateRad;

  private double targetHeading = 0.0;
  private boolean headingLockInitialized = false;
  private final PIDController headingPidf;

  private final Timer timer = new Timer();
  private final DriveSlewLimiter slewLimiter = new DriveSlewLimiter();
  private @Nullable SlewProfile slewTuning;
  private @Nullable Boolean teleopSlewOverride;

  private final Sentinel sentinel;

  private double lastLookaheadRad = 0.0;
  private boolean lastRotationSafe = true;
  private double lastAngularVelocityUsed = 0.0;

  private double lastLaneFadeX = 0.0;
  private double lastLaneFadeY = 0.0;
  private double lastDepthScale = 1.0;
  private double lastSideScale = 1.0;
  private boolean lastDepthRepulsion = false;
  private boolean lastSideRepulsion = false;
  private boolean lastPoseUntrusted = false;
  private @Nullable Envelope lastRobotBounds;
  private @Nullable Envelope lastProtectedZone;

  /** The per-match geometry this instance protects, for callers that plan their own motion. */
  public Sentinel sentinel() {
    return sentinel;
  }

  public boolean wasPoseUntrusted() {
    return lastPoseUntrusted;
  }

  public double getLastLaneFadeX() {
    return lastLaneFadeX;
  }

  public double getLastLaneFadeY() {
    return lastLaneFadeY;
  }

  public double getLastDepthScale() {
    return lastDepthScale;
  }

  public double getLastSideScale() {
    return lastSideScale;
  }

  public boolean getLastDepthRepulsion() {
    return lastDepthRepulsion;
  }

  public boolean getLastSideRepulsion() {
    return lastSideRepulsion;
  }

  public @Nullable Envelope getLastRobotBounds() {
    return lastRobotBounds;
  }

  public @Nullable Envelope getLastProtectedZone() {
    return lastProtectedZone;
  }

  public double getLastLookaheadRad() {
    return lastLookaheadRad;
  }

  public boolean getLastRotationSafe() {
    return lastRotationSafe;
  }

  public double getLastAngularVelocityUsed() {
    return lastAngularVelocityUsed;
  }

  public Casablanca(Sentinel sentinel) {
    this.sentinel = sentinel;

    var c = config.casablanca;
    frictionX = c.friction.x;
    frictionY = c.friction.y;
    frictionRot = c.friction.rot;

    enableInputSmoothing = c.smoothing.teleop_enabled;

    wallRepulsionPower = c.repulsion.power;
    decelSafetyFactor = c.repulsion.decel_safety_factor;

    enableDepthProtection = c.enable_depth_protection;
    depthSlowDown = c.depth.slow_down;
    depthHardStop = c.depth.hard_stop;

    enableSideProtection = c.enable_side_protection;
    sideSlowDown = c.side.slow_down;
    sideHardStop = c.side.hard_stop;

    laneBlendDistance = c.lane_blend_distance;
    rotationLookaheadTimeSeconds = config.sentinel.rotation_lookahead_time;

    fieldCentric = config.teleop.field_centric;
    fieldCentricOffsetRad = Math.toRadians(config.teleop.field_centric_offset_deg);

    var hl = c.heading_lock;
    enableHeadingLock = hl.enabled;
    headingLockIntentThreshold = hl.intent_threshold;
    headingLockKsMoving = hl.ks_moving;
    headingLockMovingSpeedThreshold = hl.moving_speed_threshold;
    headingLockMaxPower = hl.max_power;
    headingLockErrorDeadbandRad = Math.toRadians(hl.error_deadband_deg);
    headingLockSettleRateRad = Math.toRadians(hl.settle_rate_dps);

    this.headingPidf = new PIDController(hl.pid.p, hl.pid.i, hl.pid.d);

    performBrakingSanityCheck();

    reset();
  }

  private void performBrakingSanityCheck() {
    double maxVelX = config.pedro.motion.max_forward_velocity;
    double maxVelY = config.pedro.motion.max_strafe_velocity;
    double minBrakingX =
        Math.abs(
                brakingDistance(
                    maxVelX,
                    config.pedro.braking.forward.linear,
                    config.pedro.braking.forward.quadratic))
            / decelSafetyFactor;
    double minBrakingY =
        Math.abs(
                brakingDistance(
                    maxVelY,
                    config.pedro.braking.strafe.linear,
                    config.pedro.braking.strafe.quadratic))
            / decelSafetyFactor;

    com.qualcomm.robotcore.util.RobotLog.ii(
        "Casablanca",
        "Init check: depthHardStop=%.2f in (physics stopping dist at max %.1f in/s is %.2f in), sideHardStop=%.2f in (physics stopping dist at max %.1f in/s is %.2f in)",
        depthHardStop,
        maxVelX,
        minBrakingX,
        sideHardStop,
        maxVelY,
        minBrakingY);
  }

  public final void reset() {
    timer.reset();
    slewLimiter.reset();
    headingLockInitialized = false;
    armedAimActive = false;
    targetLockActive = false;
    if (headingPidf != null) {
      headingPidf.reset();
    }
  }

  private double armedAimHeadingTarget = 0.0;
  private boolean armedAimActive = false;

  private double targetLockHeading = 0.0;
  private boolean targetLockActive = false;

  public void setArmedAimTarget(double targetHeadingRad, boolean armed) {
    this.armedAimHeadingTarget = targetHeadingRad;
    this.armedAimActive = armed;
  }

  public void setTargetHeadingLock(double targetHeadingRad, boolean active) {
    if (active && !targetLockActive) {
      headingPidf.reset();
    }
    this.targetLockHeading = targetHeadingRad;
    this.targetLockActive = active;
  }

  public boolean isTargetHeadingLockActive() {
    return targetLockActive;
  }

  public double[] adjustDriveInput(
      Pose pose,
      Vector2D currentVelocity,
      double currentAngularVelocity,
      double strafe,
      double forward,
      double turn) {
    return adjustDriveInput(
        pose, currentVelocity, currentAngularVelocity, strafe, forward, turn, turn);
  }

  public double[] adjustDriveInput(
      Pose pose,
      Vector2D currentVelocity,
      double currentAngularVelocity,
      double strafe,
      double forward,
      double turn,
      double rawTurnIntent) {
    if (!Double.isFinite(currentAngularVelocity)) {
      currentAngularVelocity = 0.0;
    }
    if (!Double.isFinite(currentVelocity.x()) || !Double.isFinite(currentVelocity.y())) {
      currentVelocity = Vector2D.zero();
    }
    boolean poseFinite =
        Double.isFinite(pose.x()) && Double.isFinite(pose.y()) && Double.isFinite(pose.heading());

    lastPoseUntrusted = !poseFinite;
    if (!poseFinite) {
      slewLimiter.reset();
      headingLockInitialized = false;
      return new double[] {0.0, 0.0, 0.0};
    }

    if (fieldCentric) {
      // Stick-forward points away from this alliance's drivers: +x for RED, whose wall is x = 0,
      // and
      // -x for BLUE, whose wall is x = field size (the field is a half-turn, not a mirror).
      double allianceRad = sentinel.getAlliance() == Alliance.BLUE ? Math.PI : 0.0;
      Vector2D stick =
          Vector2D.cartesian(forward, strafe)
              .rotate(fieldCentricOffsetRad + allianceRad - pose.heading());
      forward = stick.x();
      strafe = stick.y();
    }

    if (enableFrictionComp) {
      forward = applyFriction(forward, frictionX);
      strafe = applyFriction(strafe, frictionY);
      turn = applyFriction(turn, frictionRot);
    }

    boolean stickReleased = Math.abs(rawTurnIntent) < headingLockIntentThreshold;

    if (armedAimActive && Double.isFinite(armedAimHeadingTarget)) {
      targetHeading = armedAimHeadingTarget;
      headingLockInitialized = true;
    } else if (targetLockActive && Double.isFinite(targetLockHeading)) {
      targetHeading = targetLockHeading;
      headingLockInitialized = true;
    }

    if (armedAimActive || targetLockActive || enableHeadingLock && stickReleased) {
      if (!headingLockInitialized) {
        if (Math.abs(currentAngularVelocity) < headingLockSettleRateRad) {
          targetHeading = pose.heading();
          headingLockInitialized = true;
          headingPidf.reset();
        }
        turn = 0.0;
      } else {
        double headingError = AngleUnit.normalizeRadians(targetHeading - pose.heading());

        if (Math.abs(headingError) < headingLockErrorDeadbandRad) {
          headingPidf.reset();
          turn = 0.0;
        } else if (willCoastToHeading(headingError, currentAngularVelocity)) {
          // Already turning toward the target fast enough that the measured stopping model says
          // the chassis will coast the rest of the way. Pushing on (the friction step alone
          // saturates the output at the deadband edge) is what made the lock overshoot and
          // ring around its deadband.
          headingPidf.reset();
          turn = 0.0;
        } else {
          double speedMag = currentVelocity.magnitude();
          double speedRatio = Math.clamp(speedMag / headingLockMovingSpeedThreshold, 0.0, 1.0);
          double ks = frictionRot + speedRatio * (headingLockKsMoving - frictionRot);

          double correction =
              headingPidf.calculate(0, headingError, currentAngularVelocity)
                  + Math.copySign(ks + config.casablanca.heading_lock.ks, headingError);
          double limit = Math.min(1.0, Math.abs(headingLockMaxPower));
          turn = Math.clamp(correction, -limit, limit);
        }
      }
    } else {
      headingLockInitialized = false;
    }

    double dt = timer.seconds();
    timer.reset();
    DrivePowers requested = new DrivePowers(forward, strafe, turn);
    if (slewTuning != null || isTeleopSlewEnabled()) {
      var s = config.casablanca.smoothing;
      DrivePowers smoothed =
          slewLimiter.calculate(
              requested,
              Math.min(dt, s.max_loop_dt),
              slewTuning == null ? SlewProfile.configured() : slewTuning);
      forward = smoothed.forward();
      strafe = smoothed.strafe();
      turn = smoothed.turn();
    } else {
      slewLimiter.reset(requested);
    }

    Vector2D inputField = Vector2D.cartesian(forward, strafe).rotate(pose.heading());
    double adjFieldX = inputField.x();
    double adjFieldY = inputField.y();

    Envelope robotBounds = sentinel.getRobotBounds(pose);

    double currentVelX = currentVelocity.x();
    double currentVelY = currentVelocity.y();
    Vector2D stop = fieldStoppingDistance(currentVelocity, pose.heading());

    lastRobotBounds = robotBounds;
    lastProtectedZone = null;
    lastLaneFadeX = 0.0;
    lastLaneFadeY = 0.0;
    lastDepthScale = 1.0;
    lastSideScale = 1.0;
    lastDepthRepulsion = false;
    lastSideRepulsion = false;

    // Each protected zone (the HIVE frame, or its two bases) is handled by the same per-axis math.
    // X uses the zone's X bounds faded by Y-lane overlap and Y the mirror image, so depth and side
    // protection stay symmetric however many zones there are.
    for (Envelope protectedZone : sentinel.getProtectedZones()) {
      double laneFadeY =
          calculateLaneFade(
              robotBounds.getMinY(),
              robotBounds.getMaxY(),
              protectedZone.getMinY(),
              protectedZone.getMaxY(),
              laneBlendDistance);
      double laneFadeX =
          calculateLaneFade(
              robotBounds.getMinX(),
              robotBounds.getMaxX(),
              protectedZone.getMinX(),
              protectedZone.getMaxX(),
              laneBlendDistance);

      if (lastProtectedZone == null
          || Math.max(laneFadeX, laneFadeY) > Math.max(lastLaneFadeX, lastLaneFadeY)) {
        lastProtectedZone = protectedZone;
        lastLaneFadeX = laneFadeX;
        lastLaneFadeY = laneFadeY;
      }

      boolean overlapsX =
          robotBounds.getMaxX() > protectedZone.getMinX()
              && robotBounds.getMinX() < protectedZone.getMaxX();
      boolean overlapsY =
          robotBounds.getMaxY() > protectedZone.getMinY()
              && robotBounds.getMinY() < protectedZone.getMaxY();

      if (enableDepthProtection && laneFadeY > 0) {
        AxisState xState =
            calculateAxisState(
                robotBounds.getMinX(),
                robotBounds.getMaxX(),
                protectedZone.getMinX(),
                protectedZone.getMaxX(),
                overlapsY,
                currentVelX,
                adjFieldX,
                depthSlowDown,
                depthHardStop,
                Math.abs(stop.x()));

        lastDepthScale = Math.min(lastDepthScale, xState.scale);
        adjFieldX *= (1.0 + laneFadeY * (xState.scale - 1.0));
        if (laneFadeY >= 0.9 && xState.triggerRepulsion) {
          adjFieldX = repel(xState.repulsionDir, adjFieldX);
          lastDepthRepulsion = true;
        }
      }

      if (enableSideProtection && laneFadeX > 0) {
        AxisState yState =
            calculateAxisState(
                robotBounds.getMinY(),
                robotBounds.getMaxY(),
                protectedZone.getMinY(),
                protectedZone.getMaxY(),
                overlapsX,
                currentVelY,
                adjFieldY,
                sideSlowDown,
                sideHardStop,
                Math.abs(stop.y()));

        lastSideScale = Math.min(lastSideScale, yState.scale);
        adjFieldY *= (1.0 + laneFadeX * (yState.scale - 1.0));
        if (laneFadeX >= 0.9 && yState.triggerRepulsion) {
          adjFieldY = repel(yState.repulsionDir, adjFieldY);
          lastSideRepulsion = true;
        }
      }
    }

    double lookaheadRad = Math.abs(currentAngularVelocity) * rotationLookaheadTimeSeconds;
    lastLookaheadRad = lookaheadRad;
    lastAngularVelocityUsed = currentAngularVelocity;
    boolean rotationSafe =
        Double.isFinite(turn)
            && Double.isFinite(lookaheadRad)
            && sentinel.isRotationSafe(pose, turn, lookaheadRad);
    lastRotationSafe = rotationSafe;
    if (turn != 0 && !rotationSafe) {
      turn = 0;
    }

    return new double[] {adjFieldY, adjFieldX, turn};
  }

  public static double applyFriction(double input, double kS) {
    if (Math.abs(input) < 0.01) return 0;
    return Math.signum(input) * (kS + Math.abs(input) * (1.0 - kS));
  }

  public static double applySlewLimit(double current, double target, double maxChange) {
    return DriveSlewLimiter.limit(current, target, maxChange);
  }

  /** Temporary per-instance calibration; null restores configured behavior. */
  public void setSlewTuning(@Nullable SlewProfile tuning) {
    slewTuning = tuning;
  }

  public DrivePowers getSlewOutput() {
    return slewLimiter.output();
  }

  /** Per-instance TeleOp command; does not change Pedro's autonomous slew setting. */
  public Command setTeleopSlewCommand(boolean enabled) {
    return Command.build()
        .setStart(() -> setTeleopSlewEnabled(enabled))
        .setDone(() -> true)
        .requiring(this);
  }

  public void setTeleopSlewEnabled(boolean enabled) {
    teleopSlewOverride = enabled;
  }

  public boolean isTeleopSlewEnabled() {
    return teleopSlewOverride != null
        ? teleopSlewOverride
        : config.casablanca.smoothing.teleop_enabled && enableInputSmoothing;
  }

  /** Calibration override; null restores TeleOp YAML gains. Pedro auto gains are independent. */
  public void setHeadingPid(@Nullable PIDFCoefficients gains) {
    PIDFCoefficients p = gains == null ? config.casablanca.heading_lock.pid : gains;
    headingPidf.kP = p.p;
    headingPidf.kI = p.i;
    headingPidf.kD = p.d;
    headingPidf.reset();
  }

  public static double calculateLaneFade(
      double botMin, double botMax, double zoneMin, double zoneMax, double buffer) {
    double distToStrict = Math.max(zoneMin - botMax, botMin - zoneMax);
    if (distToStrict <= 0) return 1.0;
    if (distToStrict >= buffer) return 0.0;
    return 1.0 - (distToStrict / buffer);
  }

  /**
   * Repulsion away from a zone, but never weaker than what the driver is already commanding in that
   * same direction: a driver backing out at full power keeps full power.
   */
  private static double repel(double repulsionDir, double commanded) {
    return repulsionDir * Math.max(wallRepulsionPower, repulsionDir * commanded);
  }

  /**
   * Protection along one axis for one zone.
   *
   * @param overlapsOtherAxis whether the robot's footprint overlaps the zone on the other axis;
   *     only then does overlapping on this axis mean the robot is inside the zone. Overlapping on
   *     this axis alone means it is beside the zone, driving parallel to its face.
   */
  private AxisState calculateAxisState(
      double botMin,
      double botMax,
      double zoneMin,
      double zoneMax,
      boolean overlapsOtherAxis,
      double currentVel,
      double inputVel,
      double slowDownDist,
      double hardStopDist,
      double stoppingDistance) {
    AxisState state = new AxisState();
    double distToStop;
    boolean movingTowards = false;
    double repulsionDir;

    if (botMax <= zoneMin) {
      distToStop = zoneMin - botMax;
      repulsionDir = -1.0;
      if (currentVel > 0 || inputVel > 0) movingTowards = true;
    } else if (botMin >= zoneMax) {
      distToStop = botMin - zoneMax;
      repulsionDir = 1.0;
      if (currentVel < 0 || inputVel < 0) movingTowards = true;
    } else if (!overlapsOtherAxis) {
      // Beside the zone and moving along its face: this axis is not the approach axis.
      state.scale = 1.0;
      return state;
    } else {
      state.triggerRepulsion = true;
      double zoneCenter = (zoneMin + zoneMax) / 2.0;
      double botCenter = (botMin + botMax) / 2.0;
      state.repulsionDir = (botCenter < zoneCenter) ? -1.0 : 1.0;
      state.scale = 0.0;
      return state;
    }

    state.repulsionDir = repulsionDir;

    // An input that already points away from the zone (the driver braking or backing out) is never
    // attenuated, even while the robot still coasts toward it.
    boolean inputAway = repulsionDir * inputVel > 0;
    if (!movingTowards || inputAway) {
      state.scale = 1.0;
      state.triggerRepulsion = inputAway && movingTowards && distToStop <= hardStopDist;
      return state;
    }

    if (distToStop <= hardStopDist) {
      state.scale = 0.0;
      state.triggerRepulsion = true;
      return state;
    }

    double proximityScale = 1.0;
    if (distToStop < slowDownDist) {
      proximityScale = (distToStop - hardStopDist) / (slowDownDist - hardStopDist);
    }

    double physicsScale = 1.0;
    if (Math.abs(currentVel) > 0.2) {
      double brakingRoom = Math.max(0, distToStop - hardStopDist);
      double predictedBrakingDist = stoppingDistance / decelSafetyFactor;

      if (predictedBrakingDist > brakingRoom && predictedBrakingDist > 0) {
        physicsScale = Math.sqrt(brakingRoom / predictedBrakingDist);
      }
    }
    state.scale = Math.min(proximityScale, physicsScale);
    return state;
  }

  /**
   * True when the chassis is already rotating toward {@code headingErrorRad} and its predicted
   * stopping rotation (Foresight's measured heading braking model, {@code pedro.braking.heading})
   * covers the remaining error.
   */
  public static boolean willCoastToHeading(double headingErrorRad, double angularVelocity) {
    if (headingErrorRad == 0.0 || Math.signum(angularVelocity) != Math.signum(headingErrorRad)) {
      return false;
    }
    double coast =
        Math.abs(
            brakingDistance(
                angularVelocity,
                config.pedro.braking.heading.linear,
                config.pedro.braking.heading.quadratic));
    return coast >= Math.abs(headingErrorRad);
  }

  /** Signed stopping distance used by the protected-zone drive limiter. */
  public static double brakingDistance(double velocity, double linear, double quadratic) {
    return velocity * linear + velocity * Math.abs(velocity) * quadratic;
  }

  /** Apply the measured chassis-axis stopping model, then express its stop vector in the field. */
  public static Vector2D fieldStoppingDistance(Vector2D fieldVelocity, double heading) {
    Vector2D robotVelocity = fieldVelocity.rotate(-heading);
    return Vector2D.cartesian(
            brakingDistance(
                robotVelocity.x(),
                config.pedro.braking.forward.linear,
                config.pedro.braking.forward.quadratic),
            brakingDistance(
                robotVelocity.y(),
                config.pedro.braking.strafe.linear,
                config.pedro.braking.strafe.quadratic))
        .rotate(heading);
  }

  private static class AxisState {
    double scale = 1.0;
    boolean triggerRepulsion = false;
    double repulsionDir = 0.0;
  }
}
