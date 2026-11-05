package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.auto.PathUtil.pline;

import androidx.annotation.Nullable;
import com.bylazar.field.FieldManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.ballistics.ShotSolver;
import org.firstinspires.ftc.teamcode.ballistics.ShotTable;
import org.firstinspires.ftc.teamcode.config.ConfigLoader;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.game.HiveTracker;
import org.firstinspires.ftc.teamcode.game.MatchClock;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.BallType;
import org.firstinspires.ftc.teamcode.records.BallisticsParameters;
import org.firstinspires.ftc.teamcode.records.CellTarget;
import org.firstinspires.ftc.teamcode.records.HiveCell;
import org.firstinspires.ftc.teamcode.records.MatchProfile;
import org.firstinspires.ftc.teamcode.records.ShotInputs;
import org.firstinspires.ftc.teamcode.records.ShotSolution;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.DrawingUtil;
import org.firstinspires.ftc.teamcode.utilities.OpModeUtil;
import org.firstinspires.ftc.teamcode.utilities.ParkTimeEstimate;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;

/**
 * Alliance-agnostic BIOBUZZ autonomous base: hardware lifecycle, pose and HIVE handoff, CELL-aware
 * volleys and the end-of-AUTO park.
 *
 * <p>Routes are not checked on the robot. {@code AutoRouteSafetyTest} checks every configured
 * start, park and path for both alliances when the APK is built ({@code assemble*} / {@code
 * install*}) and before {@code pushConfig} sends a config to the robot ({@code checkPushConfig}),
 * so a config that would cross the centre line, hit the HIVE frame or miss LEAVE + PARK never
 * reaches the robot.
 *
 * <p>Every routine ends the same way. LEAVE (3) plus AUTO PARK (5) from both alliance robots is
 * exactly the 16-point SWARM ranking-point threshold at standard events, so {@link
 * #updateParkGuard} cancels whatever the routine is doing once the time left is no more than the
 * estimated drive to the park pose (stopping from the current speed, then the HIVE-avoiding route)
 * plus {@code auto.park_margin_ms}, and never later than {@code auto.park_reserve_ms} before the
 * end. It then drives to the routine's park pose, partly inside the LOADING ZONE and clear of the
 * wall.
 *
 * <p>The robot never launches at a CELL it does not believe is upward-facing: {@link #volley} waits
 * up to {@code auto.tip_wait_ms} for the HIVE tracker to report the CELL up and skips the volley
 * otherwise, because launching into the outside of a CELL is a G417 violation.
 *
 * @param <T> the generated {@code config.*Auto} pose class for this routine
 */
public abstract class AllianceAutoBase<T> extends OpMode {

  protected final Alliance alliance;
  protected final Class<T> poseClass;
  protected final String posePrefix;
  protected T poses;

  protected Robot robot;
  protected Intake intake;
  protected Shooter shooter;
  protected Turret turret;
  protected Follower follower;
  protected Sentinel sentinel;
  protected FieldModel field;
  protected HiveTracker hive;
  protected FieldManager panels;

  private MatchClock clock;
  private boolean parking;
  // Park deadline state (see updateParkGuard).
  private boolean parkEstimateValid = false;
  private long parkEstimatedAtMs = 0;
  private double parkDriveSeconds = Double.NaN;
  private String status = "waiting for start";
  private final List<String> warnings = new ArrayList<>();

  protected AllianceAutoBase(Alliance alliance, Class<T> poseClass, String posePrefix) {
    this.alliance = alliance;
    this.poseClass = poseClass;
    this.posePrefix = posePrefix;
  }

  /** Builds this routine's paths from {@link #poses}. Called once at init. */
  protected abstract void buildPaths();

  /** The CELL the first volley is aimed at; the shooter is only pre-solved if it starts upward. */
  protected HiveCell firstVolleyCell() {
    return HiveCell.A;
  }

  /**
   * Whether the robot starts AUTO holding {@code inventory.preload_count} elements. A routine that
   * never shoots may run with or without them, so it reports the count as unknown: an unknown count
   * never blocks the intake, while a wrong count of four would hold it off for all of TeleOp.
   */
  protected boolean startsWithPreload() {
    return true;
  }

  /** The routine itself; the park guard replaces it near the end of AUTO. */
  protected abstract CommandBuilder buildAuto();

  protected abstract Pose startPose();

  /** Partly inside the LOADING ZONE, clear of the wall: earns LEAVE and PARK. */
  protected abstract Pose parkPose();

  /** Where the first volley is fired, to pre-position the hood at init; null if none. */
  protected abstract @Nullable Pose firstVolleyPose();

  @Override
  public void init() {
    config.reload();
    String allianceKey = alliance == Alliance.RED ? "red" : "blue";
    poses = ConfigLoader.loadMerged(poseClass, posePrefix + "." + allianceKey, "auto");

    panels = OpModeUtil.initPanelsField();

    blackboard.clear();
    blackboard.put("ALLIANCE", alliance);

    robot = new Robot(hardwareMap, telemetry, MatchProfile.loadMatchProfile(alliance));
    follower = robot.follower;
    intake = robot.intake;
    shooter = robot.shooter;
    turret = robot.turret;
    sentinel = robot.sentinel;
    field = robot.field;
    hive = robot.hive;
    clock = MatchClock.fromConfig(MatchClock.Period.AUTO, Robot::nowMs);

    if (startsWithPreload()) {
      intake.inventory().set(config.inventory.preload_count);
    } else {
      intake.inventory().markUnknown();
    }
    robot.shotController.setBallType(BallType.POLLEN);
    OpModeUtil.setupTurretAndShooter(turret, shooter);
    primeShooter(firstVolleyPose());

    follower.setPose(startPose());
    buildPaths();
    Scheduler.reset();
  }

  /** Pre-solves the first volley; the hood target is applied by periodic() after START. */
  private void primeShooter(@Nullable Pose volleyPose) {
    // Only pre-solve against the CELL that is up now. A solution for the other CELL would carry the
    // current epoch and count as current, so never create one.
    if (volleyPose == null || firstVolleyCell() != hive.upCell()) {
      return;
    }
    try {
      robot.shotController.setInitialSolution(
          solveInitialShot(
              volleyPose,
              robot.shotController.currentTarget(),
              robot.shotController.getBallType()));
    } catch (RuntimeException e) {
      // A bad shot table stops volleys (they time out), not driving: LEAVE and PARK still count.
      warnings.add("shot table: " + e.getMessage());
    }
  }

  static ShotSolution solveInitialShot(Pose volleyPose, CellTarget target, BallType ballType) {
    return ShotSolver.solve(
        new ShotInputs(Turret.pivotPose(volleyPose), Velocity.zero(), target, ballType),
        ShotTable.fromConfig(ballType),
        BallisticsParameters.fromConfig(),
        0.0);
  }

  @Override
  public void init_loop() {
    telemetry.addData("Alliance", alliance);
    telemetry.addData("Config", ConfigLoader.loadedSource());
    OpModeUtil.reportConfig(telemetry);
    if (!config.shooter.shot_table.calibrated) {
      telemetry.addLine("!! Shot tables are PLACEHOLDERS: volleys will time out without firing");
    }
    for (String warning : warnings) {
      telemetry.addLine("!! " + warning);
    }
    telemetry.update();
  }

  @Override
  public void start() {
    parking = false;
    parkEstimateValid = false;
    parkDriveSeconds = Double.NaN;
    clock.start();
    status = "running";
    schedule(buildAuto());
  }

  @Override
  public void loop() {
    updateParkGuard();
    robot.update();

    OpModeUtil.savePose(alliance, follower.pose());
    OpModeUtil.saveHandoff(blackboard, alliance, hive, heldForHandoff(), Robot.nowMs());
    OpModeUtil.drawRobot(panels, follower, turret, robot.shotController.currentTarget());
    DrawingUtil.drawFieldZones(panels, sentinel);

    telemetry.addData("Status", status);
    telemetry.addData("AUTO left", "%.1f s", clock.remainingMs() / 1000.0);
    telemetry.addData(
        "Park deadline",
        Double.isFinite(parkDriveSeconds)
            ? String.format(
                Locale.ROOT,
                "~%.1f s drive + %.1f s margin",
                parkDriveSeconds,
                config.auto.park_margin_ms / 1000.0)
            : String.format(
                Locale.ROOT, "no estimate: %.1f s reserve", config.auto.park_reserve_ms / 1000.0));
    telemetry.addData(
        "Pose",
        "(%.1f, %.1f, %.0f deg)",
        follower.pose().x(),
        follower.pose().y(),
        Math.toDegrees(follower.pose().heading()));
    telemetry.addData(
        "HIVE", "CELL %s up, %d TIPS (%s)", hive.upCell(), hive.tips(), hive.source());
    telemetry.addData(
        "Inventory",
        intake.inventory().isKnown() ? Integer.toString(intake.inventory().count()) : "unknown");
    telemetry.addData("Shot", robot.shotController.getGateReason());
    telemetry.addData(
        "Scoring at park",
        sentinel.earnsLeaveAndPark(follower.pose()) ? "LEAVE + PARK" : "not yet");
    telemetry.update();
  }

  /**
   * Every {@code auto.park_update_ms}, re-estimates how long parking from here would take: stopping
   * from the current speed, then {@link Sentinel#parkRoute} to the park pose ({@link
   * ParkTimeEstimate}). Once {@link MatchClock#isParkDue} says the time left is no more than that
   * plus {@code auto.park_margin_ms} (or {@code auto.park_reserve_ms}, whichever is earlier), it
   * cancels the routine and drives the route.
   *
   * <p>A moment with no clear route keeps the last estimate, so it never cancels the routine early.
   * If the deadline arrives where no route is clear, the routine keeps running and the route is
   * re-planned every update until one is clear: the configured paths are checked to have a park
   * route all along them ({@code AutoRouteSafetyTest}), so a moving robot soon reaches one.
   */
  private void updateParkGuard() {
    if (parking) {
      return;
    }
    var auto = config.auto;
    long now = Robot.nowMs();
    boolean refresh = !parkEstimateValid || now - parkEstimatedAtMs >= auto.park_update_ms;
    if (!refresh && !clock.isParkDue(parkDriveSeconds, auto.park_margin_ms, auto.park_reserve_ms)) {
      return;
    }
    Pose here = follower.pose();
    List<Pose> route = isFinite(here) ? sentinel.parkRoute(here, parkPose()) : List.of();
    if (refresh) {
      parkEstimateValid = true;
      parkEstimatedAtMs = now;
      double speed = follower.velocity().toVector2D().magnitude();
      double estimate =
          ParkTimeEstimate.stoppingSeconds(speed) + ParkTimeEstimate.seconds(here, route);
      if (Double.isFinite(estimate)) {
        parkDriveSeconds = estimate;
      }
    }
    if (!clock.isParkDue(parkDriveSeconds, auto.park_margin_ms, auto.park_reserve_ms)) {
      return;
    }
    if (route.isEmpty()) {
      status = "park due: no clear route from here yet, retrying";
      parkEstimatedAtMs = now - auto.park_update_ms; // re-plan on the next loop
      return;
    }
    parking = true;
    Scheduler.reset();
    robot.shotController.stopShot();
    intake.stop();
    shooter.setTargetPower(0);
    turret.setAimMode(Turret.AimMode.IDLE);
    status = String.format(Locale.ROOT, "parking via %d waypoint(s)", route.size());
    schedule(followRoute(here, route));
  }

  private static boolean isFinite(Pose p) {
    return p != null
        && Double.isFinite(p.x())
        && Double.isFinite(p.y())
        && Double.isFinite(p.heading());
  }

  // ---------------------------------------------------------------------------------------------
  // Building blocks for routines

  /** Follows straight segments through {@code waypoints}, starting from {@code from}. */
  protected CommandBuilder followRoute(Pose from, List<Pose> waypoints) {
    if (waypoints.isEmpty()) {
      return instant(follower::stop).requiring(follower);
    }
    Command[] legs = new Command[waypoints.size()];
    Pose previous = from;
    for (int i = 0; i < waypoints.size(); i++) {
      Pose to = waypoints.get(i);
      legs[i] =
          previous.distance(to) == 0.0
              ? instant(() -> follower.hold(to)).requiring(follower)
              : follow(follower, pline(previous, to));
      previous = waypoints.get(i);
    }
    return sequential(legs);
  }

  /** Spins the flywheel up and starts collecting, then drives {@code path}. */
  protected CommandBuilder intakeAndFollow(Path path) {
    return sequential(
        instant(
            () -> {
              shooter.setTargetPower(Shooter.constantPower());
              intake.run(1.0, config.auto.transfer_power);
            }),
        follow(follower, path));
  }

  /** Drives into a FLOWER's retrieval opening with the intake running, then dwells there. */
  protected CommandBuilder collectAt(Path path) {
    return sequential(
        intakeAndFollow(path),
        waitMs(config.auto.collect_dwell_ms),
        // Stop collecting once the dwell is over. Without an entry sensor the inventory count is
        // unknown once the intake runs, so nothing else would stop a fifth element (G407) being
        // picked up on the drive to the next volley.
        instant(
            () -> {
              intake.stop();
              status = "collected";
            }));
  }

  /**
   * Fires one volley into {@code cell} if it is (or within {@code auto.tip_wait_ms} becomes)
   * upward-facing and settled. Otherwise skips: the robot never launches at a downward CELL.
   */
  protected CommandBuilder volley(HiveCell cell) {
    return sequential(
        instant(() -> status = "waiting for CELL " + cell + " up"),
        race(
            waitUntil(() -> hive.upCell() == cell && hive.isSettled()),
            waitMs(config.auto.tip_wait_ms)),
        lazy(
            () -> {
              if (hive.upCell() != cell) {
                status = "skipped volley: CELL " + cell + " never came up";
                return instant(() -> {});
              }
              status = "volley into CELL " + cell;
              return robot.shotController.volleyCommand(
                  follower, config.auto.balls_per_volley_count);
            }));
  }

  /** Stops the intake and flywheel. */
  protected CommandBuilder idleMechanisms() {
    return instant(
        () -> {
          intake.stop();
          shooter.setTargetPower(0);
        });
  }

  /** Inventory count to hand to the next OpMode; -1 when it is not known. */
  private int heldForHandoff() {
    return intake.inventory().isKnown() ? intake.inventory().count() : -1;
  }

  @Override
  public void stop() {
    robot.shutdown();
  }
}
