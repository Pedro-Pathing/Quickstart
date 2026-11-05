package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.List;
import java.util.Optional;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.game.HiveTracker;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.records.MatchProfile;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.Casablanca;
import org.firstinspires.ftc.teamcode.utilities.HiveVision;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;

/**
 * Composition root for one match: builds every subsystem, the per-match field model, HIVE tracker
 * and safety instances from one {@link MatchProfile}, and runs the control loop.
 */
public final class Robot {

  public final Follower follower;
  public final Intake intake;
  public final Shooter shooter;
  public final Turret turret;
  public final HiveVision vision;
  public final FieldModel field;
  public final HiveTracker hive;
  public final Sentinel sentinel;
  public final Casablanca casablanca;
  public final ShotController shotController;
  public final List<LynxModule> allHubs;
  public final Telemetry telemetry;

  private boolean shutDown = false;
  private long lastVisionFrameNanos = Long.MIN_VALUE;

  public Robot(HardwareMap hardwareMap, Telemetry telemetry, MatchProfile profile) {
    this.telemetry = telemetry;

    allHubs = hardwareMap.getAll(LynxModule.class);
    for (LynxModule module : allHubs) {
      module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    follower = Constants.create(hardwareMap);

    field = new FieldModel(profile.alliance());
    hive = HiveTracker.fromConfig(Robot::nowMs);

    intake = new Intake(hardwareMap);
    shooter = new Shooter(hardwareMap);
    turret = new Turret(hardwareMap, telemetry, follower::pose, follower::velocity);

    sentinel = new Sentinel(field);
    casablanca = new Casablanca(sentinel);
    shotController =
        new ShotController(
            shooter,
            turret,
            intake,
            follower::pose,
            follower::velocity,
            casablanca,
            field,
            hive,
            telemetry);
    turret.setTargetPoint(shotController.currentTarget().x(), shotController.currentTarget().y());

    vision = new HiveVision();
    vision.init(hardwareMap);
  }

  /** Clock shared by the HIVE tracker and camera frame timestamps (System.nanoTime domain). */
  public static long nowMs() {
    return System.nanoTime() / 1_000_000L;
  }

  /**
   * One control loop: read sensors, update estimates, run commands, decide, then write. Commands
   * and {@link ShotController} only set targets; the subsystems' periodic() calls after them are
   * the only hardware writes, so every target set this loop reaches the motors this loop.
   */
  public void update() {
    for (LynxModule module : allHubs) {
      module.clearBulkCache();
    }

    follower.update();
    updateHiveFromVision();

    Scheduler.execute();

    shotController.periodic();
    intake.periodic();
    shooter.periodic();
    turret.periodic();
  }

  /**
   * Feeds this alliance's CELL sightings to the HIVE tracker, once per camera frame. The loop runs
   * faster than the camera, and the tracker's N-consecutive-frame confirmation only means anything
   * if one frame is never counted twice.
   */
  private void updateHiveFromVision() {
    double threshold = config.hive.up_height_threshold_in;
    long newestFrame = lastVisionFrameNanos;
    for (HiveVision.CellSighting s : vision.poll()) {
      if (s.capturedAtNanos() <= lastVisionFrameNanos) {
        continue;
      }
      newestFrame = Math.max(newestFrame, s.capturedAtNanos());
      if (s.owner() == field.alliance()) {
        hive.observe(s.cell(), HiveVision.isUpward(s, threshold), s.capturedAtNanos() / 1_000_000L);
      }
    }
    lastVisionFrameNanos = newestFrame;
  }

  /**
   * Re-anchors odometry on a CELL opening the camera can see, if the HIVE is settled and the fix is
   * plausible. Only this alliance's upward CELL is used: its field position is known only while the
   * HIVE rests in the state the tracker believes.
   *
   * @return the corrected pose, or empty if no trustworthy fix was available
   */
  public Optional<Pose> relocalizeFromHive() {
    if (!hive.isSettled()) {
      return Optional.empty();
    }
    Pose odometry = follower.pose();
    var relocalize = config.vision.relocalize;
    for (HiveVision.CellSighting s : vision.poll()) {
      if (!HiveVision.isFresh(s, nowMs(), config.hive.max_observation_age_ms)
          || s.owner() != field.alliance()
          || s.cell() != hive.upCell()
          || !HiveVision.isUpward(s, config.hive.up_height_threshold_in)
          || s.rangeIn() > relocalize.max_range_in) {
        continue;
      }
      Pose fix = HiveVision.relocalize(s, field.upOpening(s.cell()), odometry.heading());
      if (Math.hypot(fix.x() - odometry.x(), fix.y() - odometry.y())
          > relocalize.max_correction_in) {
        continue;
      }
      follower.setPose(fix);
      return Optional.of(fix);
    }
    return Optional.empty();
  }

  /**
   * Stops everything this Robot owns: the shot, the intake, the solver thread and the camera. Must
   * be called from the owning OpMode's stop(); a new Robot is built on every init, so skipping this
   * leaks a solver thread and holds the webcam open. Safe to call more than once.
   */
  public void shutdown() {
    if (shutDown) {
      return;
    }
    shutDown = true;
    Scheduler.reset();
    shotController.stopShot();
    shotController.shutdown();
    vision.close();
  }
}
