package org.firstinspires.ftc.teamcode.tests;

import androidx.annotation.Nullable;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.HeadingStepResponse;
import org.firstinspires.ftc.teamcode.utilities.OpModeUtil;

/** Non-relay PD search: measures the real Casablanca lock, including its slew and compensation. */
@TeleOp(name = "TeleOp Heading Lock Autotuner", group = "Calibration")
// Hardware is set up inside runOpMode's try block, so not every path assigns every field.
@SuppressWarnings("NullAway.Init")
public final class TeleopHeadingAutotuneOpMode extends LinearOpMode {
  private Robot robot;
  private GoBildaPinpointDriver pinpoint;
  private config.Casablanca.HeadingTuner settings;
  private @Nullable PIDFCoefficients best;
  private double bestScore = Double.POSITIVE_INFINITY;
  private String message = "Not started";
  private boolean abort;

  @Override
  public void runOpMode() throws InterruptedException {
    config.reload();
    OpModeUtil.reportConfig(telemetry); // a rejected push or broken tuning, at INIT
    settings = config.casablanca.heading_tuner;
    try {
      validateSettings(settings);
      Alliance alliance = (Alliance) blackboard.get("ALLIANCE");
      robot =
          new Robot(
              hardwareMap,
              telemetry,
              config.loadMatchProfile(alliance == null ? Alliance.RED : alliance));
      robot.turret.setAimMode(Turret.AimMode.IDLE);
      pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
      robot.follower.setPose(settings.calibration_pose);
      show();
      waitForStart();
      boolean released = false;
      while (opModeIsActive()) {
        stopDrive();
        if (!gamepad1.a) released = true;
        if (released && gamepad1.a && !gamepad1.b) {
          released = false;
          runSearch();
        }
        show();
        sleep(settings.sample_period_ms);
      }
    } catch (RuntimeException error) {
      message = "Stopped: " + error.getMessage();
      if (robot != null) stopDrive();
      while (!isStopRequested()) {
        show();
        idle();
      }
    } finally {
      if (robot != null) {
        robot.casablanca.setHeadingPid(null);
        try {
          stopDrive();
        } finally {
          robot.shutdown();
        }
      }
    }
  }

  private void runSearch() throws InterruptedException {
    best = null;
    bestScore = Double.POSITIVE_INFINITY;
    abort = false;
    robot.follower.setPose(settings.calibration_pose);
    for (int pi = 0; pi < settings.p_count && !abort; pi++) {
      for (int di = 0; di < settings.d_count && !abort; di++) {
        PIDFCoefficients candidate =
            new PIDFCoefficients(
                HeadingStepResponse.gainAt(settings.p_min, settings.p_max, settings.p_count, pi), 0,
                HeadingStepResponse.gainAt(settings.d_min, settings.d_max, settings.d_count, di),
                    0);
        robot.casablanca.setHeadingPid(candidate);
        double positive = trial(candidate, 1);
        if (abort) break;
        double negative = trial(candidate, -1);
        double score = Math.max(positive, negative);
        if (!abort && score < bestScore) {
          best = candidate;
          bestScore = score;
        }
      }
    }
    robot.casablanca.setHeadingPid(null);
    stopDrive();
    message = abort ? "Interrupted; only completed pairs are reported" : "Search complete";
  }

  private double trial(PIDFCoefficients candidate, int direction) throws InterruptedException {
    try {
      stopDrive();
      if (!settle()) {
        abort = true;
        return Double.POSITIVE_INFINITY;
      }
      Pose start = robot.follower.pose();
      double step = direction * Math.toRadians(settings.step_degrees);
      double target = AngleUnit.normalizeRadians(start.heading() + step);
      var response =
          new HeadingStepResponse(
              step,
              Math.toRadians(settings.heading_tolerance_degrees),
              Math.toRadians(settings.rate_tolerance_dps),
              Math.toRadians(settings.overshoot_limit_degrees),
              settings.settle_sec,
              settings.trial_timeout_sec);
      robot.casablanca.reset();
      robot.casablanca.setTargetHeadingLock(target, true);
      long previous = System.nanoTime();
      while (held()) {
        sleep(settings.sample_period_ms);
        if (!held()) break;
        long now = System.nanoTime();
        double dt = (now - previous) * 1e-9;
        previous = now;
        Pose pose = robot.follower.pose();
        var velocity = robot.follower.velocity();
        if (!healthy()
            || dt > settings.maximum_sample_gap_sec
            || pose.distance(start) > settings.position_drift_inches
            || Math.abs(
                    Math.toDegrees(AngleUnit.normalizeRadians(pose.heading() - start.heading())))
                > settings.maximum_sweep_degrees) {
          abort = true;
          return Double.POSITIVE_INFINITY;
        }
        double error = AngleUnit.normalizeRadians(target - pose.heading());
        var status = response.update(error, velocity.omega, dt);
        if (status == HeadingStepResponse.Status.INVALID) {
          abort = true;
          return Double.POSITIVE_INFINITY;
        }
        if (status == HeadingStepResponse.Status.FAIL) return Double.POSITIVE_INFINITY;
        if (status == HeadingStepResponse.Status.PASS) return response.settlingSeconds();
        double[] adjusted =
            robot.casablanca.adjustDriveInput(
                pose, velocity.toVector2D(), velocity.omega, 0, 0, 0, 0);
        if (robot.casablanca.wasPoseUntrusted()
            || !robot.casablanca.getLastRotationSafe()
            || robot.casablanca.getLastDepthRepulsion()
            || robot.casablanca.getLastSideRepulsion()
            || robot.casablanca.getLastDepthScale() < 1
            || robot.casablanca.getLastSideScale() < 1) {
          abort = true;
          return Double.POSITIVE_INFINITY;
        }
        robot.follower.manual(
            ManualDrive.fieldCentric(
                new DrivePowers(adjusted[1], adjusted[0], adjusted[2]), pose.heading()));
        robot.update();
        telemetry.addData("Testing TeleOp lock", "P=%.4f I=0 D=%.4f", candidate.p, candidate.d);
        telemetry.addData(
            "Error / overshoot",
            "%.2f / %.2f deg",
            Math.toDegrees(error),
            Math.toDegrees(response.overshoot()));
        telemetry.addLine("Hold A throughout. Release A or B aborts.");
        telemetry.update();
      }
      abort = true;
      return Double.POSITIVE_INFINITY;
    } finally {
      stopDrive();
    }
  }

  private boolean settle() throws InterruptedException {
    long previous = System.nanoTime();
    double elapsed = 0, still = 0;
    while (held()) {
      sleep(settings.sample_period_ms);
      robot.update();
      long now = System.nanoTime();
      double dt = (now - previous) * 1e-9;
      previous = now;
      elapsed += dt;
      if (!healthy() || dt > settings.maximum_sample_gap_sec) return false;
      var v = robot.follower.velocity();
      still =
          v.toVector2D().magnitude() <= settings.stopped_speed_ips
                  && Math.abs(v.omega) <= Math.toRadians(settings.rate_tolerance_dps)
              ? still + dt
              : 0;
      if (still >= settings.settle_sec) return true;
      if (elapsed > settings.trial_timeout_sec) return false;
    }
    return false;
  }

  private boolean healthy() {
    Pose p = robot.follower.pose();
    var v = robot.follower.velocity();
    return pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY
        && Double.isFinite(p.x())
        && Double.isFinite(p.y())
        && Double.isFinite(p.heading())
        && Double.isFinite(v.vx)
        && Double.isFinite(v.vy)
        && Double.isFinite(v.omega);
  }

  private boolean held() {
    return opModeIsActive() && gamepad1.a && !gamepad1.b;
  }

  private void stopDrive() {
    robot.casablanca.setTargetHeadingLock(0, false);
    robot.follower.stop();
    robot.update();
    robot.casablanca.reset();
  }

  private void show() {
    telemetry.addLine("TeleOp heading-lock PD tuning. Clear space for turns in both directions.");
    telemetry.addData("Place robot at", settings.calibration_pose);
    telemetry.addData("Status", message);
    telemetry.addLine(
        "Hold A to run the grid. Release A / B / STOP aborts. A new run clears results.");
    if (best != null) {
      telemetry.addLine("Copy into casablanca.heading_lock.pid (TeleOp only):");
      telemetry.addData("p", best.p);
      telemetry.addData("i", 0);
      telemetry.addData("d", best.d);
      telemetry.addData("Worst settling time", bestScore);
      telemetry.addLine(
          "Validated at rest with current friction, deadband, cap, and slew settings.");
    } else telemetry.addLine("No passing bidirectional candidate.");
    telemetry.update();
  }

  public static void validateSettings(config.Casablanca.HeadingTuner c) {
    double[] positive = {
      c.step_degrees,
      c.maximum_sweep_degrees,
      c.position_drift_inches,
      c.heading_tolerance_degrees,
      c.rate_tolerance_dps,
      c.overshoot_limit_degrees,
      c.settle_sec,
      c.trial_timeout_sec,
      c.stopped_speed_ips,
      c.sample_period_ms,
      c.maximum_sample_gap_sec
    };
    for (double x : positive)
      if (!Double.isFinite(x) || x <= 0)
        throw new IllegalArgumentException("Invalid heading tuner setting");
    HeadingStepResponse.gainAt(c.p_min, c.p_max, c.p_count, 0);
    HeadingStepResponse.gainAt(c.d_min, c.d_max, c.d_count, 0);
    if (c.step_degrees + c.overshoot_limit_degrees >= c.maximum_sweep_degrees
        || c.maximum_sweep_degrees >= 180
        || c.heading_tolerance_degrees >= c.step_degrees
        || c.heading_tolerance_degrees < config.casablanca.heading_lock.error_deadband_deg
        || c.settle_sec >= c.trial_timeout_sec
        || c.sample_period_ms * 1e-3 >= c.maximum_sample_gap_sec
        || c.calibration_pose == null
        || !Double.isFinite(c.calibration_pose.x())
        || !Double.isFinite(c.calibration_pose.y())
        || !Double.isFinite(c.calibration_pose.heading()))
      throw new IllegalArgumentException("Inconsistent heading tuner limits");
  }
}
