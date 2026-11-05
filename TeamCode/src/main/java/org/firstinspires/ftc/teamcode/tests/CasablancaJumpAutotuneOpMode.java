package org.firstinspires.ftc.teamcode.tests;

import androidx.annotation.Nullable;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import java.util.Objects;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.records.SlewProfile;
import org.firstinspires.ftc.teamcode.records.SlewProfile.Axis;
import org.firstinspires.ftc.teamcode.records.SlewTuning;
import org.firstinspires.ftc.teamcode.records.TiltReading;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.Turret;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.Casablanca;
import org.firstinspires.ftc.teamcode.utilities.JumpDetector;
import org.firstinspires.ftc.teamcode.utilities.JumpImu;
import org.firstinspires.ftc.teamcode.utilities.OpModeUtil;
import org.firstinspires.ftc.teamcode.utilities.SlewCalibrationSearch;
import org.firstinspires.ftc.teamcode.utilities.SlewTravelBudget;

/** Physical, non-PID calibration. Each A-held trial starts and stops in one direction. */
@TeleOp(name = "Casablanca Pod Contact Tuner", group = "Calibration")
// Hardware is set up inside runOpMode's try block, so not every path assigns every field.
@SuppressWarnings("NullAway.Init")
public final class CasablancaJumpAutotuneOpMode extends LinearOpMode {
  private enum Result {
    PASS,
    POD_LIFT,
    INCOMPLETE
  }

  private Robot robot;
  private Axis axis;
  private JumpImu imu;
  private config.Casablanca.JumpTuner settings;
  private String result = "No trials yet";

  @Override
  public void runOpMode() throws InterruptedException {
    config.reload();
    OpModeUtil.reportConfig(telemetry); // a rejected push or broken tuning, at INIT
    settings = config.casablanca.jump_tuner;
    axis = Axis.valueOf(settings.trial_axis);
    try {
      validateSettings(settings);
      Alliance alliance = (Alliance) blackboard.get("ALLIANCE");
      robot =
          new Robot(
              hardwareMap,
              telemetry,
              config.loadMatchProfile(alliance == null ? Alliance.RED : alliance));
      robot.turret.setAimMode(Turret.AimMode.IDLE);
      imu = new JumpImu(hardwareMap);
      SlewCalibrationSearch search =
          new SlewCalibrationSearch(
              SlewProfile.configured().axis(axis),
              settings.minimum_slew_time,
              settings.maximum_slew_time,
              settings.search_factor,
              settings.candidate_count);
      telemetry.addData("Tilt sensor", imu.source());
      telemetry.addData("Place at", startPose());
      telemetry.addLine("Clear the lane in BOTH directions.");
      telemetry.addLine("Hold A per trial. X = pod lifted. Release A / B / STOP aborts.");
      telemetry.addData(
          "Maximum travel per trial (including stopping)", settings.max_travel_inches);
      telemetry.addLine("Manually reposition between trials. No automatic return.");
      telemetry.update();
      waitForStart();
      boolean forward = true;
      boolean released = false;
      while (opModeIsActive()) {
        robot.follower.stop();
        robot.update();
        showSearch(search, forward);
        if (!gamepad1.a) released = true;
        if (!search.done() && released && gamepad1.a && !gamepad1.b) {
          released = false;
          Result trial = trial(search.candidate(), forward ? 1 : -1);
          if (trial == Result.POD_LIFT) {
            search.record(false);
            forward = true;
          } else if (trial == Result.PASS) {
            if (forward) forward = false;
            else {
              search.record(true);
              forward = true;
              SlewTuning best = Objects.requireNonNull(search.best()); // record(true) just set it
              RobotLog.ii(
                  "JumpTuner",
                  "Contact-confirmed pair: increase_time=%.6f decrease_time=%.6f",
                  best.increaseTime(),
                  best.decreaseTime());
            }
          }
        }
        sleep(settings.sample_period_ms);
      }
    } catch (RuntimeException error) {
      hardStop();
      RobotLog.ee("JumpTuner", error, "Calibration stopped");
      while (!isStopRequested()) {
        telemetry.addData("Calibration stopped", error.getMessage());
        telemetry.addLine("Fix setup/config and re-init. No result saved.");
        telemetry.update();
        sleep(settings.sample_period_ms > 0 ? settings.sample_period_ms : 1);
      }
    } finally {
      if (robot != null) {
        robot.casablanca.setSlewTuning(null);
        try {
          hardStop();
        } finally {
          robot.shutdown();
        }
      }
    }
  }

  /**
   * Where each trial starts. FORWARD drives along calibration_pose's heading; STRAFE starts at the
   * same spot turned 90 degrees, so it strafes along the same clear lane instead of across it (no
   * spot on the field has 48 in of clear floor both ways in both directions).
   */
  private Pose startPose() {
    Pose p = settings.calibration_pose;
    return axis == Axis.STRAFE ? new Pose(p.x(), p.y(), p.heading() - Math.PI / 2) : p;
  }

  private Result trial(SlewTuning tuning, int direction) throws InterruptedException {
    try {
      robot.casablanca.reset();
      robot.casablanca.setSlewTuning(SlewProfile.configured().with(axis, tuning));
      robot.follower.setPose(startPose());
      double started = seconds();
      TiltReading baseline = baseline(started);
      if (baseline == null) return incomplete("Baseline interrupted or robot not stationary");
      JumpDetector detector =
          new JumpDetector(
              baseline.pitch(),
              baseline.roll(),
              settings.jump_tilt_deg,
              settings.jump_rate_dps,
              settings.jump_hold_sec,
              settings.emergency_tilt_deg);
      TiltReading previousTilt = baseline;
      Pose start = robot.follower.pose();
      Pose previousPose = start;
      double distance = 0, angle = 0, held = 0, settled = 0;
      double lastProgress = seconds(), progressDistance = 0;
      boolean stopping = false, excited = false;
      while (opModeIsActive() && gamepad1.a && !gamepad1.b) {
        sleep(settings.sample_period_ms);
        if (!opModeIsActive() || !gamepad1.a || gamepad1.b) break;
        if (gamepad1.x) {
          result = "Observer reported pod lift";
          return Result.POD_LIFT;
        }
        // Inspect feedback before issuing the next command. The previous command remains active
        // during sampling; reserve includes reaction time and the following power-ramp step.
        TiltReading tilt = readTilt(previousTilt);
        double dt = tilt.secondsSince(previousTilt);
        if (dt > settings.reaction_time_sec)
          return incomplete("Loop exceeded stopping-budget reaction time");
        JumpDetector.Status status =
            detector.update(
                tilt.pitch(),
                tilt.roll(),
                tilt.pitchRate(previousTilt),
                tilt.rollRate(previousTilt),
                dt);
        previousTilt = tilt;
        if (status == JumpDetector.Status.INVALID || status == JumpDetector.Status.EMERGENCY) {
          return incomplete("Invalid IMU or emergency tilt: power removed");
        }
        Pose pose = robot.follower.pose();
        double speed = robot.follower.velocity().toVector2D().magnitude();
        if (!finite(pose)
            || !Double.isFinite(speed)
            || !Double.isFinite(robot.follower.velocity().omega)) {
          return incomplete("Invalid localization");
        }
        distance += pose.distance(previousPose);
        angle += Math.abs(AngleUnit.normalizeRadians(pose.heading() - previousPose.heading()));
        previousPose = pose;
        Vector2D offset =
            Vector2D.cartesian(pose.x() - start.x(), pose.y() - start.y()).rotate(-start.heading());
        if ((axis == Axis.FORWARD
                    ? Math.abs(offset.y())
                    : axis == Axis.STRAFE ? Math.abs(offset.x()) : pose.distance(start))
                > settings.lateral_drift_inches
            || axis != Axis.TURN
                && Math.abs(
                        Math.toDegrees(
                            AngleUnit.normalizeRadians(pose.heading() - start.heading())))
                    > settings.heading_drift_deg) return incomplete("Heading/lateral drift");
        double now = seconds();
        if (now - started > settings.trial_timeout_sec) return incomplete("Trial timeout");
        if (distance >= settings.max_travel_inches - settings.stop_margin_inches) {
          return incomplete("Travel guard: power removed before maximum travel");
        }
        double brakeLinear =
            axis == Axis.STRAFE
                ? config.pedro.braking.strafe.linear
                : config.pedro.braking.forward.linear;
        double brakeQuadratic =
            axis == Axis.STRAFE
                ? config.pedro.braking.strafe.quadratic
                : config.pedro.braking.forward.quadratic;
        double coastRoom =
            SlewTravelBudget.stoppingRoom(
                speed,
                0,
                tuning.increaseTime(),
                brakeLinear,
                brakeQuadratic,
                Casablanca.decelSafetyFactor,
                settings.reaction_time_sec,
                settings.stop_margin_inches);
        if (distance + coastRoom >= settings.max_travel_inches) {
          return incomplete("Braking reserve exhausted: power removed");
        }
        double progress = axis == Axis.TURN ? angle : distance;
        double minimumProgress =
            axis == Axis.TURN
                ? Math.toRadians(settings.minimum_progress_degrees)
                : settings.minimum_progress_inches;
        if (progress - progressDistance >= minimumProgress) {
          progressDistance = progress;
          lastProgress = now;
        }
        if (!stopping && now - lastProgress > settings.stall_timeout_sec) {
          return incomplete("No odometry progress: check carpet/pods");
        }
        double power = Math.abs(axisPower(robot.casablanca.getSlewOutput()));
        double requested = axis == Axis.TURN ? settings.rotation_trial_power : settings.trial_power;
        double friction =
            switch (axis) {
              case FORWARD -> Casablanca.frictionX;
              case STRAFE -> Casablanca.frictionY;
              case TURN -> Casablanca.frictionRot;
            };
        double target =
            Casablanca.enableFrictionComp
                ? Casablanca.applyFriction(requested, friction)
                : requested;
        double omega = Math.abs(robot.follower.velocity().omega);
        if (axis == Axis.TURN) {
          var h = config.pedro.braking.heading;
          double angularCoastRoom =
              SlewTravelBudget.stoppingRoom(
                  omega,
                  0,
                  0,
                  h.linear,
                  h.quadratic,
                  Casablanca.decelSafetyFactor,
                  settings.reaction_time_sec,
                  Math.toRadians(settings.angular_stop_margin_degrees));
          if (angle + angularCoastRoom >= Math.toRadians(settings.maximum_angle_degrees))
            return incomplete("Angular braking reserve exhausted");
        }
        if (axis == Axis.TURN
            && angle
                >= Math.toRadians(
                    settings.maximum_angle_degrees - settings.angular_stop_margin_degrees))
          return incomplete("Angular travel guard");
        if (!stopping) {
          held = Math.abs(power - target) <= settings.power_tolerance ? held + dt : 0;
          excited |= held >= settings.peak_hold_sec;
          double reserve =
              SlewTravelBudget.stoppingRoom(
                  Math.max(speed, target * settings.speed_bound_ips),
                  target,
                  tuning.longestTime(),
                  brakeLinear,
                  brakeQuadratic,
                  Casablanca.decelSafetyFactor,
                  settings.reaction_time_sec,
                  settings.stop_margin_inches);
          if (axis == Axis.TURN) {
            var headingBrake = config.pedro.braking.heading;
            double angularReserve =
                SlewTravelBudget.stoppingRoom(
                    Math.max(omega, target * Math.toRadians(settings.angular_speed_bound_dps)),
                    target,
                    tuning.longestTime(),
                    headingBrake.linear,
                    headingBrake.quadratic,
                    Casablanca.decelSafetyFactor,
                    settings.reaction_time_sec,
                    Math.toRadians(settings.angular_stop_margin_degrees));
            stopping =
                angle >= Math.toRadians(settings.drive_angle_degrees)
                    || angle + angularReserve >= Math.toRadians(settings.maximum_angle_degrees);
          } else
            stopping =
                distance >= settings.drive_distance_inches
                    || distance + reserve >= settings.max_travel_inches;
        }

        if (stopping
            && power <= settings.power_tolerance
            && speed <= settings.stopped_speed_ips
            && omega <= Math.toRadians(config.casablanca.heading_lock.settle_rate_dps)) {
          settled += dt;
          if (settled >= settings.settle_sec) {
            result =
                String.format(
                    java.util.Locale.US,
                    "%s: %.1f in, peak %.2f deg / %.1f deg/s",
                    excited ? "Trial completed; contact verdict needed" : "Incomplete excitation",
                    distance,
                    detector.peakTilt(),
                    detector.peakRate());
            return excited ? awaitContactVerdict() : Result.INCOMPLETE;
          }
        } else settled = 0;
        command(stopping ? 0 : direction * requested);
        if (protectionActive()) return incomplete("Collision protection intervened; reposition");
        robot.update();
        telemetry.addData("Phase", stopping ? "Stopping / observing" : "Accelerating / holding");
        telemetry.addData("Travel", "%.1f / %.1f in", distance, settings.max_travel_inches);
        telemetry.addData(
            "Tilt / rate peaks", "%.2f deg / %.1f deg/s", detector.peakTilt(), detector.peakRate());
        telemetry.addLine("KEEP HOLDING A through the stop; release A or B to abort.");
        telemetry.update();
      }
      return incomplete("Operator stopped trial");
    } finally {
      hardStop();
      robot.casablanca.setSlewTuning(null);
    }
  }

  private @Nullable TiltReading baseline(double started) throws InterruptedException {
    hardStop();
    TiltReading previous = imu.read();
    double still = 0;
    while (opModeIsActive() && gamepad1.a && !gamepad1.b) {
      sleep(settings.sample_period_ms);
      robot.update();
      TiltReading sample = readTilt(previous);
      double speed = robot.follower.velocity().toVector2D().magnitude();
      double rate = Math.hypot(sample.pitchRate(previous), sample.rollRate(previous));
      still =
          Double.isFinite(speed)
                  && speed <= settings.stopped_speed_ips
                  && rate <= settings.baseline_rate_dps
              ? still + sample.secondsSince(previous)
              : 0;
      previous = sample;
      telemetry.addLine("Hold A; remain stationary while measuring tilt baseline.");
      telemetry.update();
      if (still >= settings.baseline_sec) return sample;
      if (seconds() - started >= settings.trial_timeout_sec) return null;
    }
    return null;
  }

  private TiltReading readTilt(TiltReading previous) {
    TiltReading sample = imu.read();
    double age = (System.nanoTime() - sample.nanos()) * 1e-9;
    if (!sample.finite()
        || !previous.finite()
        || age < 0
        || age > settings.imu_stale_sec
        || sample.secondsSince(previous) <= 0
        || sample.secondsSince(previous) > settings.imu_stale_sec) {
      throw new IllegalStateException("Invalid or stale tilt samples");
    }
    return sample;
  }

  private double axisPower(DrivePowers p) {
    return switch (axis) {
      case FORWARD -> p.forward();
      case STRAFE -> p.strafe();
      case TURN -> p.turn();
    };
  }

  private void command(double power) {
    Pose pose = robot.follower.pose();
    Vector2D stick =
        Vector2D.cartesian(axis == Axis.FORWARD ? power : 0, axis == Axis.STRAFE ? power : 0);
    if (Casablanca.fieldCentric)
      stick = stick.rotate(pose.heading() - Casablanca.fieldCentricOffsetRad);
    var velocity = robot.follower.velocity();
    double[] output =
        robot.casablanca.adjustDriveInput(
            pose,
            velocity.toVector2D(),
            velocity.omega,
            stick.y(),
            stick.x(),
            axis == Axis.TURN ? power : 0,
            axis == Axis.TURN ? 1 : 0);
    robot.follower.manual(
        ManualDrive.fieldCentric(new DrivePowers(output[1], output[0], output[2]), pose.heading()));
  }

  private Result awaitContactVerdict() throws InterruptedException {
    hardStop();
    boolean released = false;
    while (opModeIsActive()) {
      if (!gamepad1.a && !gamepad1.x && !gamepad1.y) released = true;
      if (gamepad1.b) return incomplete("Contact uncertain; trial discarded");
      if (released && gamepad1.x) {
        result = "Observer reported pod lift";
        return Result.POD_LIFT;
      }
      if (released && gamepad1.y) {
        result = "Observer confirmed pods stayed down";
        return Result.PASS;
      }
      telemetry.addLine(
          "Release buttons. Y = pods stayed down; X = a pod lifted; B = unsure/retry.");
      telemetry.addLine(
          "IMU tilt is diagnostic only; the 4-bar spring can maintain contact while tilted.");
      telemetry.update();
      robot.update();
      sleep(settings.sample_period_ms);
    }
    return incomplete("Stopped before contact verdict");
  }

  private boolean protectionActive() {
    var c = robot.casablanca;
    return c.wasPoseUntrusted()
        || c.getLastDepthRepulsion()
        || c.getLastSideRepulsion()
        || c.getLastDepthScale() < 1
        || c.getLastSideScale() < 1
        || !c.getLastRotationSafe();
  }

  private void hardStop() {
    if (robot == null) return;
    robot.follower.stop();
    robot.update();
    robot.casablanca.reset();
  }

  private Result incomplete(String reason) {
    result = "Incomplete: " + reason;
    return Result.INCOMPLETE;
  }

  private void showSearch(SlewCalibrationSearch search, boolean forward) {
    telemetry.addData("Tilt sensor", imu.source());
    telemetry.addData("Last trial", result);
    telemetry.addData("Search", search.stage());
    telemetry.addData(
        "Candidate",
        "increase_time=%.4f, decrease_time=%.4f",
        search.candidate().increaseTime(),
        search.candidate().decreaseTime());
    if (!search.done()) {
      telemetry.addData(
          "Next trial", axis + (forward ? " positive then stop" : " negative then stop"));
      telemetry.addData("Reposition to", startPose());
      telemetry.addLine("Release A, manually reposition, then HOLD A to run.");
    }
    if (search.best() != null) {
      telemetry.addLine(
          "Copy into casablanca.smoothing." + axis.name().toLowerCase(java.util.Locale.ROOT) + ":");
      telemetry.addData("increase_time", search.best().increaseTime());
      telemetry.addData("decrease_time", search.best().decreaseTime());
      telemetry.addData(
          "Validated input power only",
          (axis == Axis.TURN ? settings.rotation_trial_power : settings.trial_power));
    } else telemetry.addLine("No passing forward/reverse pair yet. No recommendation.");
    telemetry.update();
  }

  /** Reject unsafe overrides before constructing hardware or starting a physical trial. */
  public static void validateSettings(config.Casablanca.JumpTuner settings) {
    // 48 inches is the user's absolute trial ceiling, not a tunable mechanical coefficient.
    double[] positive = {
      settings.trial_power,
      settings.rotation_trial_power,
      settings.drive_angle_degrees,
      settings.maximum_angle_degrees,
      settings.angular_stop_margin_degrees,
      settings.angular_speed_bound_dps,
      settings.max_travel_inches,
      settings.drive_distance_inches,
      settings.stop_margin_inches,
      settings.reaction_time_sec,
      settings.speed_bound_ips,
      settings.baseline_sec,
      settings.peak_hold_sec,
      settings.settle_sec,
      settings.stopped_speed_ips,
      settings.baseline_rate_dps,
      settings.jump_tilt_deg,
      settings.jump_rate_dps,
      settings.jump_hold_sec,
      settings.emergency_tilt_deg,
      settings.imu_stale_sec,
      settings.trial_timeout_sec,
      settings.stall_timeout_sec,
      settings.minimum_progress_inches,
      settings.minimum_progress_degrees,
      settings.heading_drift_deg,
      settings.lateral_drift_inches,
      settings.power_tolerance,
      settings.sample_period_ms,
      config.casablanca.smoothing.max_loop_dt
    };
    for (double value : positive) {
      if (!Double.isFinite(value) || value <= 0)
        throw new IllegalArgumentException("Invalid jump tuner setting");
    }
    if (settings.max_travel_inches > 48
        || settings.trial_power > 1
        || settings.rotation_trial_power > 1
        || settings.drive_angle_degrees
            >= settings.maximum_angle_degrees - settings.angular_stop_margin_degrees
        || settings.stop_margin_inches >= settings.max_travel_inches
        || settings.drive_distance_inches
            >= settings.max_travel_inches - settings.stop_margin_inches
        || settings.emergency_tilt_deg <= settings.jump_tilt_deg
        || settings.power_tolerance >= settings.trial_power
        || settings.sample_period_ms * 1e-3 >= settings.imu_stale_sec
        || settings.reaction_time_sec < config.casablanca.smoothing.max_loop_dt
        || !finite(settings.calibration_pose))
      throw new IllegalArgumentException("Inconsistent jump tuner limits");
  }

  private static boolean finite(Pose pose) {
    return pose != null
        && Double.isFinite(pose.x())
        && Double.isFinite(pose.y())
        && Double.isFinite(pose.heading());
  }

  private static double seconds() {
    return System.nanoTime() * 1e-9;
  }
}
