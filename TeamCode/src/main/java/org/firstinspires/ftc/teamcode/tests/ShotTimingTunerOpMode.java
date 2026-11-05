package org.firstinspires.ftc.teamcode.tests;

import android.os.Environment;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.ballistics.ShotTable;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.HiveCell;
import org.firstinspires.ftc.teamcode.records.MatchProfile;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.CalibrationRay;
import org.firstinspires.ftc.teamcode.utilities.FlywheelDipDetector;
import org.firstinspires.ftc.teamcode.utilities.OpModeUtil;

@TeleOp(name = "Shot Timing Tuner", group = "Calibration")
@Configurable
public class ShotTimingTunerOpMode extends LinearOpMode {

  /** One accepted measurement for one distance. */
  public record TimingPoint(
      double distanceInches,
      int shootWindowMs,
      int firstBallMs,
      int lastBallMs,
      int ballCount,
      long timestampMs) {}

  /** One control loop inside one shot. Written straight out as a CSV row. */
  private record TraceRow(
      int shotId,
      double distanceInches,
      int elapsedMs,
      double velocity,
      double targetVelocity,
      double dipFraction,
      double baseline,
      boolean ballEvent,
      boolean feedCommanded,
      boolean flywheelReady,
      boolean aimed,
      boolean solverValid,
      double hoodTarget,
      double solvedRpm) {}

  /** Distances (inches from RED's CELL A opening centre) the run walks through. */
  private static final double[] CALIBRATION_DISTANCES = new double[] {24.0, 30.0, 36.0, 42.0, 48.0};

  /** Same ray Ballistics Calibration uses, so the two tables describe the same shots. */
  private static final double RAY_OFF_AXIS_DEG = -40.0;

  /**
   * Time budget for one shot, sized from what the robot actually does.
   *
   * <p>A ball costs the flywheel about a quarter of its speed and the controller needs well over a
   * second to earn it back before the feed gate will re-arm, so three balls is a multi-second
   * affair however the window is set. The cap exists to end a failed shot, not to be the answer.
   */
  private static final long PER_BALL_BUDGET_MS = 2000;

  private static final long BASE_BUDGET_MS = 1500;

  /** Added to the last ball's timestamp to get the window this distance needs. */
  private static final int WINDOW_MARGIN_MS = 250;

  /**
   * How deep a dip has to be to count as a ball, as a fraction of the reference speed.
   *
   * <p>Measured, not guessed. Across 11 recorded shots on two different flywheel controllers, a
   * real ball costs between 12.2% and 26.4% of flywheel speed, while the deepest thing that is not
   * a ball — the wheel coasting down from its post-shot overshoot, one encoder quantum at a time —
   * reaches 5.4%. There is nothing in between, so 9% sits in clear air with margin on both sides.
   *
   * <p>The old 5% default sat just under the coast-down and produced a false third ball in every
   * single shot of both runs, which cut each measurement short before the real third ball landed.
   */
  private static final double DIP_STEP = 0.005;

  private Robot robot;
  private List<LynxModule> allHubs;

  private final List<TimingPoint> trials = new ArrayList<>();
  private final List<TraceRow> trace = new ArrayList<>();

  /**
   * Starts at shooter.ball_detection.dip_fraction (9%, see above), the threshold the match code
   * counts with; the stick adjusts it for this run only. Set in runOpMode, after the config reload,
   * so a pushed value applies.
   */
  private double dipFraction;

  private int expectedBalls;
  private int shotId = 0;

  /**
   * Whether the flywheel idles at {@code constant_rpm} between shots.
   *
   * <p>Idling is the normal state — it matches what auto does while following a path, and it is the
   * only way a measurement starts from the same place a real scoring cycle does. The stop button
   * clears this latch; without one, the previous version re-commanded idle power on every
   * non-shooting loop, one line after stop had called {@code stopShot()}, and the wheel was back at
   * speed before the next tick.
   */
  private boolean flywheelIdling = true;

  @Override
  public void runOpMode() throws InterruptedException {
    config.reload();
    dipFraction = config.shooter.ball_detection.dip_fraction;
    expectedBalls = config.auto.balls_per_volley_count;

    allHubs = hardwareMap.getAll(LynxModule.class);
    for (LynxModule module : allHubs) {
      module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    MatchProfile profile = config.loadMatchProfile(Alliance.RED);
    robot = new Robot(hardwareMap, telemetry, profile);
    CalibrationRay ray = CalibrationRay.forCell(robot.field, HiveCell.A, RAY_OFF_AXIS_DEG);
    OpModeUtil.setupTurretAndShooter(robot.turret, robot.shooter);

    telemetry.addLine("Shot Timing Tuner");
    telemetry.addLine("");
    telemetry.addLine("The robot will NOT drive. Its pose is faked to each");
    telemetry.addLine("distance. It DOES fire and the turret DOES slew.");
    telemetry.addData("Load before each shot", "%d balls", expectedBalls);
    if (!config.shooter.shot_table.calibrated) {
      telemetry.addLine("!! Shot table not calibrated: every shot is refused. Run Ballistics");
      telemetry.addLine("!! Calibration first and set shooter.shot_table.calibrated: true.");
    }
    OpModeUtil.reportConfig(telemetry);
    telemetry.update();

    waitForStart();

    double tableMin = Double.NEGATIVE_INFINITY;
    double tableMax = Double.POSITIVE_INFINITY;
    try {
      ShotTable table = ShotTable.fromConfig(robot.shotController.getBallType());
      tableMin = table.minDistance();
      tableMax = table.maxDistance();
    } catch (RuntimeException e) {
      // A malformed table is named on the INIT config report; shots refuse until it is fixed.
    }
    try {
      for (double distance : CALIBRATION_DISTANCES) {
        if (!opModeIsActive()) break;
        if (distance > ray.maxDistance()) {
          continue;
        }
        // Inside the measured table: Ballistics Calibration records the distance it actually
        // shot from (say 24.3 in), and the solver refuses anything outside the table's rows, so
        // an exact 24.0 in endpoint could never fire.
        double at = Math.clamp(distance, tableMin, tableMax);
        robot.follower.setPose(Turret.robotPoseForPivot(ray.waypoint(at)));
        measureEndpoint(at);
      }
      finish();
    } finally {
      stopEverything();
      robot.shutdown();
    }
  }

  /** Full wind-down: no shot, no feed, no flywheel, and it stays down until the next fire. */
  private void stopEverything() {
    robot.shotController.stopShot();
    robot.intake.stop();
    robot.shooter.setTargetPower(0.0);
    flywheelIdling = false;
  }

  private void tick() {
    for (LynxModule module : allHubs) {
      module.clearBulkCache();
    }
    robot.update();
  }

  private long shotBudgetMs() {
    return BASE_BUDGET_MS + (long) expectedBalls * PER_BALL_BUDGET_MS;
  }

  /** Runs one distance until a measurement is accepted or the distance is skipped. */
  private void measureEndpoint(double distance) {
    FlywheelDipDetector detector = newDetector();
    List<Integer> ballTimesMs = new ArrayList<>();

    boolean shotRunning = false;
    boolean armed = false;
    boolean everReady = false;
    long shotStartMs = 0;

    int resultFirstMs = -1;
    int resultLastMs = -1;
    int resultCount = 0;
    int resultWindowMs = -1;
    String status = "load " + expectedBalls + " balls";

    boolean intakeOn = false;

    boolean aPrev = true;
    boolean bPrev = true;
    boolean xPrev = true;
    boolean upPrev = true;
    boolean downPrev = true;
    boolean lsPrev = true;
    boolean rsPrev = true;

    while (opModeIsActive()) {
      long nowMs = System.currentTimeMillis();

      if (gamepad1.dpad_up && !upPrev) {
        expectedBalls = Math.min(9, expectedBalls + 1);
      }
      upPrev = gamepad1.dpad_up;

      if (gamepad1.dpad_down && !downPrev) {
        expectedBalls = Math.max(1, expectedBalls - 1);
      }
      downPrev = gamepad1.dpad_down;

      // The one control to reach for when the ball count disagrees with what you watched. The
      // "deepest dip" readout says which way to move it.
      if (gamepad1.right_stick_button && !rsPrev) {
        dipFraction += DIP_STEP;
      }
      rsPrev = gamepad1.right_stick_button;

      if (gamepad1.left_stick_button && !lsPrev) {
        dipFraction = Math.max(DIP_STEP, dipFraction - DIP_STEP);
      }
      lsPrev = gamepad1.left_stick_button;

      if (gamepad1.x && !xPrev && !shotRunning) {
        intakeOn = !intakeOn;
        if (intakeOn) {
          robot.intake.run(config.teleop.intake_power, config.teleop.transfer_power);
        } else {
          robot.intake.stop();
        }
      }
      xPrev = gamepad1.x;

      // Stop is unconditional and wins over everything, including the idle latch below. Clearing
      // flywheelIdling in stopEverything() is what actually lets the wheel wind down.
      if (gamepad1.left_trigger > 0.3) {
        if (shotRunning) {
          shotRunning = false;
          status = "stopped by driver";
          saveProgress();
        }
        intakeOn = false;
        stopEverything();
      }

      boolean flywheelReady = robot.shotController.isFlywheelReady();
      everReady |= flywheelReady && !shotRunning;

      if (gamepad1.right_trigger > 0.3 && !shotRunning) {
        if (!flywheelIdling) {
          flywheelIdling = true;
          everReady = false;
          status = "spinning back up";
        } else if (!everReady) {
          status = "still spinning up — wait for READY";
        } else {
          detector = newDetector();
          ballTimesMs.clear();
          shotRunning = true;
          armed = false;
          shotStartMs = nowMs;
          intakeOn = false;
          shotId++;
          robot.shotController.startShot(Shooter.constantPower(), true);
          status = "firing";
        }
      }

      int elapsedMs = shotRunning ? (int) (nowMs - shotStartMs) : 0;

      if (shotRunning) {
        // Do not start counting until the flywheel has reached its arm window. Spin-up is one long
        // ramp far below the eventual reference, and feeding that to the detector scores it as a
        // fistful of phantom balls before the first real one moves.
        if (!armed && flywheelReady) {
          armed = true;
          detector.reset();
        }

        // Deliberately NOT gated on the feed being on. A ball keeps moving after the intake stops
        // — the gate shuts on the very dip the ball causes, and a ball left pressed against a
        // spinning flywheel with no intake behind it still works its way through, just late and
        // weak. That late ball is the one being measured. Depth is what separates a ball from
        // noise here, not timing; see shooter.ball_detection.dip_fraction.
        boolean ball = armed && detector.update(robot.shooter.getShooterVelocity(), nowMs);
        if (ball) {
          ballTimesMs.add(elapsedMs);
        }
        trace.add(row(shotId, distance, elapsedMs, detector, ball));

        if (ballTimesMs.size() >= expectedBalls) {
          shotRunning = false;
          status = String.format(Locale.ROOT, "all %d out", expectedBalls);
        } else if (elapsedMs >= shotBudgetMs()) {
          shotRunning = false;
          status =
              String.format(
                  Locale.ROOT,
                  "TIMED OUT with %d of %d — pull the csv",
                  ballTimesMs.size(),
                  expectedBalls);
        }

        if (!shotRunning) {
          robot.shotController.stopShot();
          robot.intake.stop();
          // Only a complete volley is a measurement: a timed-out one with fewer balls gives a
          // window too short for the balls that never came out.
          if (ballTimesMs.size() >= expectedBalls) {
            resultFirstMs = ballTimesMs.get(0);
            resultLastMs = ballTimesMs.get(ballTimesMs.size() - 1);
            resultCount = ballTimesMs.size();
            resultWindowMs = resultLastMs + WINDOW_MARGIN_MS;
          }
          saveProgress();
        }
      }

      // Single place the idle flywheel is written, and only when nothing else owns it.
      if (!shotRunning) {
        robot.shooter.setTargetPower(flywheelIdling ? Shooter.constantPower() : 0.0);
      }

      tick();
      render(
          distance,
          detector,
          armed,
          everReady,
          flywheelReady,
          shotRunning,
          elapsedMs,
          ballTimesMs,
          resultFirstMs,
          resultLastMs,
          resultCount,
          resultWindowMs,
          intakeOn,
          status);

      boolean aPressed = gamepad1.a && !aPrev;
      boolean bPressed = gamepad1.b && !bPrev;
      aPrev = gamepad1.a;
      bPrev = gamepad1.b;

      if (aPressed && !shotRunning && resultWindowMs > 0) {
        trials.add(
            new TimingPoint(
                distance,
                resultWindowMs,
                resultFirstMs,
                resultLastMs,
                resultCount,
                System.currentTimeMillis()));
        saveProgress();
        break;
      }
      if (bPressed && !shotRunning) {
        break;
      }
    }

    // Wind down on the way out. Leaving via `return` skipped this entirely, so the flywheel stayed
    // at constant_rpm from the last endpoint through the whole completion screen — minutes of a
    // spun-up shooter on a robot nobody is watching, which is also what made a finished run look
    // like a hung one.
    stopEverything();
  }

  private FlywheelDipDetector newDetector() {
    var d = config.shooter.ball_detection;
    return new FlywheelDipDetector(
        dipFraction, d.rebound_fraction, d.baseline_alpha, d.refractory_ms);
  }

  private TraceRow row(
      int id, double distance, int elapsedMs, FlywheelDipDetector detector, boolean ballEvent) {
    var solution = robot.shotController.getLastSolution();
    return new TraceRow(
        id,
        distance,
        elapsedMs,
        robot.shooter.getShooterVelocity(),
        robot.shotController.getFlywheelTarget(),
        detector.getLastDipFraction(),
        detector.getBaselineVelocity(),
        ballEvent,
        robot.shotController.isFeedCommanded(),
        robot.shotController.isFlywheelReady(),
        robot.turret.isAimed(robot.follower.pose()),
        solution.isValid(),
        robot.shooter.getTargetHoodPosition(),
        solution.targetRpm());
  }

  private void render(
      double distance,
      FlywheelDipDetector detector,
      boolean armed,
      boolean everReady,
      boolean flywheelReady,
      boolean shotRunning,
      int elapsedMs,
      List<Integer> ballTimesMs,
      int resultFirstMs,
      int resultLastMs,
      int resultCount,
      int resultWindowMs,
      boolean intakeOn,
      String status) {

    telemetry.addData("DISTANCE", "%.0f in", distance);
    telemetry.addData("LOAD", "%d balls", expectedBalls);

    String flywheel;
    if (!flywheelIdling) {
      flywheel = "STOPPED — RT to spin back up";
    } else if (everReady || flywheelReady) {
      flywheel = "READY";
    } else {
      flywheel =
          String.format(
              Locale.ROOT,
              "SPINNING UP  %.0f%%",
              robot.shotController.getFlywheelVelocityRatio() * 100);
    }
    telemetry.addData("FLYWHEEL", flywheel);

    telemetry.addData(
        "SHOT",
        shotRunning ? String.format(Locale.ROOT, "FIRING %d ms", elapsedMs) : "idle — " + status);
    telemetry.addData("Balls this shot", "%d %s", ballTimesMs.size(), ballTimesMs);

    if (resultWindowMs > 0) {
      telemetry.addLine("--- result ---");
      telemetry.addData(
          "WINDOW NEEDED", "%d ms   (config %d)", resultWindowMs, config.auto.shoot_wait_ms);
      telemetry.addData("First ball", "%d ms", resultFirstMs);
      telemetry.addData("Last ball", "%d ms   of %d balls", resultLastMs, resultCount);
      telemetry.addLine("A to accept");
    }

    telemetry.addLine("--- detector ---");
    telemetry.addData(
        "Dip now / deepest",
        "%.1f%% / %.1f%%   (trigger %.1f%%)",
        detector.getLastDipFraction() * 100,
        detector.getDeepestDipFraction() * 100,
        dipFraction * 100);
    telemetry.addData("State", armed ? (detector.isInDip() ? "DIP" : "watching") : "not armed");
    telemetry.addData("Gate", robot.shotController.getFlywheelGateDetail());
    telemetry.addData("Intake", intakeOn ? "LOADING" : "off");

    telemetry.addLine("--------------------------------------------------");
    telemetry.addLine("RT  fire     LT  stop     X  intake");
    telemetry.addLine("A   accept   B   skip     DPad U/D  ball count");
    telemetry.addLine("Stick clicks: dip trigger -/+ 0.5%");
    telemetry.update();
  }

  private void finish() {
    // Everything off before the completion screen: this loop only exits when the driver presses
    // STOP, and until then tick() keeps writing whatever the subsystems were last told to do.
    stopEverything();
    saveProgress();

    long idleStartMs = System.currentTimeMillis();
    while (opModeIsActive()) {
      tick();
      telemetry.addLine("Done — PRESS STOP");
      telemetry.addLine("========================================");
      for (TimingPoint p : trials) {
        telemetry.addLine(
            String.format(
                Locale.ROOT,
                "  %.0f in -> %d ms  (%d balls, first %d ms)",
                p.distanceInches(),
                p.shootWindowMs(),
                p.ballCount(),
                p.firstBallMs()));
      }
      telemetry.addLine("========================================");
      telemetry.addData("Trace rows", trace.size());
      telemetry.addLine(
          Environment.getExternalStorageDirectory() + "/FIRST/teamcode/shot_time_table.yaml");
      telemetry.addLine(
          Environment.getExternalStorageDirectory() + "/FIRST/teamcode/shot_timing_log.csv");
      telemetry.addData(
          "Safe to stop", "idle %ds", (System.currentTimeMillis() - idleStartMs) / 1000);
      telemetry.update();
      sleep(50);
    }
  }

  /**
   * Flushes both files after every shot.
   *
   * <p>Everything here lives in memory, and an OpMode that never reaches its last line never saves.
   * A dead battery or a mis-press part way through would otherwise discard the whole run.
   */
  private void saveProgress() {
    List<TimingPoint> sorted = new ArrayList<>(trials);
    sorted.sort(Comparator.comparingDouble(TimingPoint::distanceInches));
    saveTable(sorted);
    saveTrace();
  }

  private void saveTable(List<TimingPoint> points) {
    File dir = new File(Environment.getExternalStorageDirectory(), "FIRST/teamcode/");
    if (!dir.exists() && !dir.mkdirs()) {
      telemetry.addData("Error", "Failed to create " + dir);
    }
    try (FileWriter writer = new FileWriter(new File(dir, "shot_time_table.yaml"), false)) {
      writer.write("# Generated by Shot Timing Tuner\n");
      writer.write(
          "# Paste under auto: in config.yaml; AUTO volleys look this up by distance.\n\n");
      writer.write("  # distance_in, shoot_window_ms\n");
      writer.write("  shot_time_points: [\n");
      for (int i = 0; i < points.size(); i++) {
        TimingPoint p = points.get(i);
        writer.write(
            String.format(
                Locale.ROOT,
                "        %.1f, %d%s%n",
                p.distanceInches(),
                p.shootWindowMs(),
                (i == points.size() - 1) ? "" : ","));
      }
      writer.write("    ]\n");
      writer.write("\n# --- accepted measurements ---\n");
      for (TimingPoint p : trials) {
        writer.write(
            String.format(
                Locale.ROOT,
                "# %.0f in, window %d ms, first %d ms, last %d ms, %d balls, t=%d%n",
                p.distanceInches(),
                p.shootWindowMs(),
                p.firstBallMs(),
                p.lastBallMs(),
                p.ballCount(),
                p.timestampMs()));
      }
    } catch (IOException e) {
      telemetry.addData("Error", "Failed to write shot time table: " + e.getMessage());
    }
  }

  private void saveTrace() {
    File dir = new File(Environment.getExternalStorageDirectory(), "FIRST/teamcode/");
    if (!dir.exists() && !dir.mkdirs()) {
      telemetry.addData("Error", "Failed to create " + dir);
    }
    try (FileWriter writer = new FileWriter(new File(dir, "shot_timing_log.csv"), false)) {
      writer.write(
          "shot,distance_in,t_ms,velocity,target_velocity,dip_pct,baseline,ball,feeding,"
              + "flywheel_ready,aimed,solver_valid,hood,solved_rpm\n");
      for (TraceRow r : trace) {
        writer.write(
            String.format(
                Locale.ROOT,
                "%d,%.0f,%d,%.1f,%.1f,%.2f,%.1f,%d,%d,%d,%d,%d,%.4f,%.0f%n",
                r.shotId(),
                r.distanceInches(),
                r.elapsedMs(),
                r.velocity(),
                r.targetVelocity(),
                r.dipFraction() * 100,
                r.baseline(),
                r.ballEvent() ? 1 : 0,
                r.feedCommanded() ? 1 : 0,
                r.flywheelReady() ? 1 : 0,
                r.aimed() ? 1 : 0,
                r.solverValid() ? 1 : 0,
                r.hoodTarget(),
                r.solvedRpm()));
      }
    } catch (IOException e) {
      telemetry.addData("Error", "Failed to write trace log: " + e.getMessage());
    }
  }
}
