package org.firstinspires.ftc.teamcode.teleop;

import androidx.annotation.Nullable;
import com.bylazar.field.FieldManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.Locale;
import java.util.Optional;
import org.firstinspires.ftc.teamcode.config.ConfigLoader;
import org.firstinspires.ftc.teamcode.game.FieldModel;
import org.firstinspires.ftc.teamcode.game.HiveTracker;
import org.firstinspires.ftc.teamcode.game.MatchClock;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.MatchProfile;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.ShotController;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.BallInventory;
import org.firstinspires.ftc.teamcode.utilities.Casablanca;
import org.firstinspires.ftc.teamcode.utilities.DrawingUtil;
import org.firstinspires.ftc.teamcode.utilities.OpModeUtil;
import org.firstinspires.ftc.teamcode.utilities.ParkTimeEstimate;
import org.firstinspires.ftc.teamcode.utilities.Sentinel;

/**
 * Season-agnostic TeleOp plumbing: robot lifecycle, alliance selection, AUTO handoff, gamepad edge
 * tracking, default drive, match-time cues, the park alert, vision relocalization, telemetry and
 * field drawing. Subclasses bind buttons in {@link #buildCommands()} and {@link #onLoop()}.
 */
public abstract class TeleOpBase extends OpMode {

  public static class ButtonTracker {
    private boolean currentState = false;
    private boolean previousState = false;

    public void update(boolean state) {
      previousState = currentState;
      currentState = state;
    }

    public boolean onTrue() {
      return currentState && !previousState;
    }

    public boolean onFalse() {
      return !currentState && previousState;
    }

    public boolean state() {
      return currentState;
    }
  }

  protected Robot robot;
  protected Follower follower;
  protected Intake intake;
  protected Shooter shooter;
  protected Turret turret;
  protected ShotController shotController;
  protected Sentinel sentinel;
  protected Casablanca casablanca;
  protected FieldModel field;
  protected HiveTracker hive;
  protected MatchProfile profile;
  protected MatchClock clock;

  protected Gamepad driver;
  protected Gamepad operator;

  protected ButtonTracker operatorA = new ButtonTracker();
  protected ButtonTracker operatorB = new ButtonTracker();
  protected ButtonTracker operatorX = new ButtonTracker();
  protected ButtonTracker operatorY = new ButtonTracker();
  protected ButtonTracker operatorDpadUp = new ButtonTracker();
  protected ButtonTracker operatorDpadDown = new ButtonTracker();
  protected ButtonTracker operatorDpadRight = new ButtonTracker();
  protected ButtonTracker operatorRightTrigger = new ButtonTracker();
  protected ButtonTracker operatorLeftTrigger = new ButtonTracker();
  protected ButtonTracker operatorLeftBumper = new ButtonTracker();
  protected ButtonTracker operatorRightBumper = new ButtonTracker();
  protected ButtonTracker operatorBack = new ButtonTracker();
  protected ButtonTracker operatorLeftStick = new ButtonTracker();

  protected ButtonTracker driverX = new ButtonTracker();
  protected ButtonTracker driverDpadLeft = new ButtonTracker();
  protected ButtonTracker driverDpadRight = new ButtonTracker();
  protected ButtonTracker driverRightTrigger = new ButtonTracker();
  protected ButtonTracker driverLeftTrigger = new ButtonTracker();

  private final ButtonTracker initSelectRed = new ButtonTracker();
  private final ButtonTracker initSelectBlue = new ButtonTracker();

  protected FieldManager panels;
  protected TelemetryManager telemetryM;

  protected Command teleOpDriveCommand;

  /** True when no AUTO handed over an alliance and the drivers have not picked one either. */
  private boolean allianceUnconfirmed = false;

  private String handoffStatus = "none";
  private String poseStatus = "profile start";
  private String relocalizeStatus = "";
  private String driveAssistStatus = "idle";

  // Park alert state (see handleParkAlert).
  private boolean parkEstimateValid = false;
  private long parkEstimatedAtMs = 0;
  private double parkDriveSeconds = Double.POSITIVE_INFINITY;
  private boolean parked = false;
  private boolean parkAlertActive = false;
  private long parkAlertPulsedAtMs = 0;

  protected abstract void buildCommands();

  protected abstract void onLoop();

  @Override
  public void init() {
    config.reload();
    Object saved = blackboard.get("ALLIANCE");
    Alliance alliance = saved instanceof Alliance a ? a : Alliance.RED;
    driver = gamepad1;
    operator = gamepad2;
    buildRobot(alliance, false);
  }

  /**
   * (Re)builds everything alliance-dependent. Called at init and when the drivers switch alliance.
   */
  private void buildRobot(Alliance alliance, boolean driverSelected) {
    if (robot != null) {
      robot.shutdown();
    }
    profile = config.loadMatchProfile(alliance);
    OpModeUtil.TeleOpStart start =
        OpModeUtil.resolveTeleOpStart(blackboard, profile, Robot.nowMs());
    allianceUnconfirmed = !driverSelected && !start.allianceConfirmed();

    robot = new Robot(hardwareMap, telemetry, profile);
    follower = robot.follower;
    intake = robot.intake;
    shooter = robot.shooter;
    turret = robot.turret;
    shotController = robot.shotController;
    sentinel = robot.sentinel;
    casablanca = robot.casablanca;
    field = robot.field;
    hive = robot.hive;
    clock = MatchClock.fromConfig(MatchClock.Period.TELEOP, Robot::nowMs);

    panels = OpModeUtil.initPanelsField();
    telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    follower.setPose(start.pose());
    poseStatus = start.poseRestored() ? "saved pose (fresh handoff)" : "profile start";
    restoreHandoff(start.handoff());
    casablanca.reset();
    shooter.setShooterPIDFCoefficients();

    double maxSpeed = config.teleop.max_speed;
    teleOpDriveCommand =
        Command.build()
            .setStart(() -> follower.manual(0, 0, 0))
            .setExecute(
                () -> {
                  double y = Math.clamp(-Math.pow(driver.left_stick_y, 3), -maxSpeed, maxSpeed);
                  double x = Math.clamp(-Math.pow(driver.left_stick_x, 3), -maxSpeed, maxSpeed);
                  double r = Math.clamp(-Math.pow(driver.right_stick_x, 3), -maxSpeed, maxSpeed);

                  double[] adjusted =
                      casablanca.adjustDriveInput(
                          follower.pose(),
                          follower.velocity().toVector2D(),
                          follower.velocity().omega,
                          x,
                          y,
                          r,
                          -driver.right_stick_x);
                  follower.manual(
                      ManualDrive.fieldCentric(
                          new DrivePowers(adjusted[1], adjusted[0], adjusted[2]),
                          follower.pose().heading()));
                })
            .requiring(follower);

    buildCommands();
  }

  /** Restores the HIVE state and inventory AUTO left behind, if they are from this match. */
  void restoreHandoff(@Nullable OpModeUtil.Handoff handoff) {
    BallInventory inventory = intake.inventory();
    if (handoff != null) {
      hive.restore(handoff.upCell(), handoff.tips());
      inventory.restore(handoff.inventory());
      handoffStatus =
          String.format(
              Locale.ROOT,
              "fresh handoff: CELL %s up, %d TIPS, %s held",
              handoff.upCell(),
              handoff.tips(),
              handoff.inventory() < 0 ? "unknown" : Integer.toString(handoff.inventory()));
    } else {
      inventory.restore(-1);
      handoffStatus = "none (match start state assumed)";
    }
  }

  @Override
  public void init_loop() {
    initSelectRed.update(gamepad1.b || gamepad2.b);
    initSelectBlue.update(gamepad1.x || gamepad2.x);
    if (allianceUnconfirmed && (initSelectRed.onTrue() || initSelectBlue.onTrue())) {
      Alliance chosen = initSelectRed.onTrue() ? Alliance.RED : Alliance.BLUE;
      blackboard.put("ALLIANCE", chosen);
      allianceUnconfirmed = false;
      if (chosen != profile.alliance()) {
        buildRobot(chosen, true);
      }
    }

    telemetry.addData("Alliance", profile.alliance());
    if (allianceUnconfirmed) {
      telemetry.addLine("!! No fresh handoff: confirm alliance. Press B = RED, X = BLUE.");
    }
    telemetry.addData("Alliance confirmation", allianceUnconfirmed ? "required" : "confirmed");
    telemetry.addData("Starting pose", poseStatus);
    telemetry.addData("Handoff", handoffStatus);
    telemetry.addData("Config", ConfigLoader.loadedSource());
    OpModeUtil.reportConfig(telemetry);
    telemetry.update();
  }

  @Override
  public void start() {
    clock.start();
    follower.manual(0, 0, 0);
    OpModeUtil.setupTurretAndShooter(turret, shooter);
    teleOpDriveCommand.schedule();
  }

  private void updateGamepads() {
    operatorA.update(operator.a);
    operatorB.update(operator.b);
    operatorX.update(operator.x);
    operatorY.update(operator.y);
    operatorDpadUp.update(operator.dpad_up);
    operatorDpadDown.update(operator.dpad_down);
    operatorDpadRight.update(operator.dpad_right);
    operatorRightTrigger.update(operator.right_trigger > 0.5);
    operatorLeftTrigger.update(operator.left_trigger > 0.5);
    operatorLeftBumper.update(operator.left_bumper);
    operatorRightBumper.update(operator.right_bumper);
    operatorBack.update(operator.back);
    operatorLeftStick.update(operator.left_stick_button);

    driverX.update(driver.x);
    driverDpadLeft.update(driver.dpad_left);
    driverDpadRight.update(driver.dpad_right);
    driverRightTrigger.update(driver.right_trigger > 0.5);
    driverLeftTrigger.update(driver.left_trigger > 0.5);
  }

  @Override
  public void loop() {
    updateGamepads();

    robot.update();

    onLoop();

    if (!Scheduler.isScheduled(teleOpDriveCommand) && !hasActiveDriveOverride()) {
      teleOpDriveCommand.schedule();
    }

    handleMatchCues();
    handleParkAlert();
    handleRelocalize();

    OpModeUtil.drawRobot(panels, follower, turret, shotController.currentTarget());
    DrawingUtil.drawFieldZones(panels, sentinel);

    OpModeUtil.savePose(profile.alliance(), follower.pose());
    OpModeUtil.saveHandoff(blackboard, profile.alliance(), hive, heldForHandoff(), Robot.nowMs());

    addTelemetry();
  }

  /** Two rumble blips on both gamepads when NECTAR unlocks (1:00 left). */
  private void handleMatchCues() {
    if (clock.pollCue() == MatchClock.Cue.FLOWER_UNLOCK) {
      driver.rumbleBlips(2);
      operator.rumbleBlips(2);
    }
  }

  /**
   * From {@code match.final_warning_remaining_ms} on, re-estimates the drive from here to the park
   * pose every {@code teleop.park_alert_update_ms}. Once the time left is no more than that drive
   * plus {@code teleop.park_alert_margin_ms} and the robot is not PARKED, it repeats a rumble and a
   * red gamepad light every {@code teleop.park_alert_repeat_ms}, says "park now" once on the Driver
   * Station, and shows PARK NOW at the top of telemetry. It only alerts: it never drives the robot.
   *
   * <p>The estimate is {@link ParkTimeEstimate} over {@link Sentinel#parkRoute}, the HIVE-avoiding
   * route; no clear route, or a lost pose, alerts as soon as the window opens. F310 gamepads have
   * no rumble motors or lights, so on those only the voice and telemetry reach the drivers.
   */
  private void handleParkAlert() {
    if (!clock.isFinalWarning()) {
      parkEstimateValid = false;
      parkAlertActive = false;
      return;
    }
    var t = config.teleop;
    long now = Robot.nowMs();
    Pose here = follower.pose();
    boolean poseFinite =
        here != null
            && Double.isFinite(here.x())
            && Double.isFinite(here.y())
            && Double.isFinite(here.heading());
    parked = poseFinite && sentinel.isParked(here);
    if (!parkEstimateValid || now - parkEstimatedAtMs >= t.park_alert_update_ms) {
      parkEstimateValid = true;
      parkEstimatedAtMs = now;
      if (parked) {
        parkDriveSeconds = 0.0;
      } else if (poseFinite) {
        parkDriveSeconds =
            ParkTimeEstimate.seconds(here, sentinel.parkRoute(here, profile.parkPose()));
      } else {
        parkDriveSeconds = Double.POSITIVE_INFINITY;
      }
    }

    boolean alert = !parked && clock.isParkAlert(parkDriveSeconds, t.park_alert_margin_ms);
    boolean onset = alert && !parkAlertActive;
    parkAlertActive = alert;
    if (!alert) {
      return;
    }
    if (onset) {
      telemetry.speak("Park now");
    }
    if (onset || now - parkAlertPulsedAtMs >= t.park_alert_repeat_ms) {
      parkAlertPulsedAtMs = now;
      driver.rumble(t.rumble_ms);
      operator.rumble(t.rumble_ms);
      driver.setLedColor(1.0, 0.0, 0.0, t.rumble_ms);
      operator.setLedColor(1.0, 0.0, 0.0, t.rumble_ms);
    }
  }

  /** Operator holds dpad down to re-anchor odometry on the upward CELL. */
  private void handleRelocalize() {
    if (!config.vision.continuous) {
      if (operatorDpadDown.onTrue()) {
        robot.vision.resumeStreaming();
      }
      if (operatorDpadDown.onFalse()) {
        robot.vision.stopStreaming();
      }
    }
    if (operatorDpadDown.onTrue()) {
      relocalizeStatus = "searching";
    }
    if (operatorDpadDown.state()) {
      Optional<Pose> fix = robot.relocalizeFromHive();
      fix.ifPresent(
          p ->
              relocalizeStatus =
                  String.format(
                      Locale.ROOT,
                      "fixed at (%.1f, %.1f, %.0f deg)",
                      p.x(),
                      p.y(),
                      Math.toDegrees(p.heading())));
    }
  }

  private void addTelemetry() {
    boolean unlocked = clock.isFlowerUnlocked();
    if (parkAlertActive) {
      telemetryM.addLine("!!!!!!!!!! PARK NOW !!!!!!!!!!");
    }
    telemetryM.addData("Alliance", profile.alliance());
    telemetryM.addData(
        "Time left",
        String.format(
            Locale.ROOT,
            "%d:%02d%s",
            clock.remainingMs() / 60_000,
            (clock.remainingMs() / 1000) % 60,
            unlocked ? "  NECTAR->FLOWERS OK" : ""));
    telemetryM.addData(
        "HIVE",
        String.format(
            Locale.ROOT,
            "CELL %s up, %d TIPS (%s%s)",
            hive.upCell(),
            hive.tips(),
            hive.source(),
            hive.isSettled() ? "" : ", settling"));
    telemetryM.addData(
        "Human NECTAR allowed",
        String.format(
            Locale.ROOT,
            "%d of %d",
            MatchClock.nectarAllowance(hive.tips(), unlocked),
            MatchClock.STAGED_NECTAR));
    if (clock.isFinalWarning()) {
      telemetryM.addData(
          "Park",
          parked
              ? "PARKED"
              : Double.isFinite(parkDriveSeconds)
                  ? String.format(
                      Locale.ROOT,
                      "~%.1f s drive, %.1f s left",
                      parkDriveSeconds,
                      clock.remainingMs() / 1000.0)
                  : "no clear route from here: park now");
    }
    BallInventory inventory = intake.inventory();
    telemetryM.addData(
        "Holding",
        inventory.isKnown()
            ? String.format(
                Locale.ROOT,
                "%d / %d%s",
                inventory.count(),
                inventory.capacity(),
                intake.isBlockedByCapacity() ? "  FULL, intake held" : "")
            : "unknown (no entry sensor)");
    telemetryM.addData("Ball type", shotController.getBallType());
    telemetryM.addData("Shot", shotController.getGateReason());
    telemetryM.addData("Pose", follower.pose());
    telemetryM.addData("Pose assist", driveAssistStatus);
    if (!relocalizeStatus.isEmpty()) {
      telemetryM.addData("Vision fix", relocalizeStatus);
    }
    telemetryM.addData("Drive Mode", Casablanca.fieldCentric ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
    telemetryM.addData(
        "Heading Lock",
        casablanca.isTargetHeadingLockActive() ? "UPWARD CELL (driver X)" : "last heading");
    if (casablanca.wasPoseUntrusted()) {
      telemetryM.addLine("!! DRIVE REFUSED: localizer pose is non-finite !!");
    }
    if (ConfigLoader.overrideRejection() != null) {
      telemetryM.addLine("!! Pushed config ignored: " + ConfigLoader.overrideRejection());
    }
    telemetryM.update(telemetry);
  }

  /** Refuses a pose hold whose direct approach would cross the HIVE or the walls. */
  protected void holdPoseIfClear(Pose target) {
    Pose from = follower.pose();
    Optional<String> problem =
        from.distance(target) == 0.0
            ? sentinel.rotationProblem(from, target.heading(), false)
            : sentinel.routeProblem(
                org.firstinspires.ftc.teamcode.auto.PathUtil.pline(from, target),
                config.sentinel.route_sample_step,
                false);
    if (problem.isPresent()) {
      driveAssistStatus = "blocked: " + problem.get();
      follower.stop();
    } else {
      driveAssistStatus = "holding pose";
      follower.hold(target);
    }
  }

  protected boolean hasActiveDriveOverride() {
    return false;
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
