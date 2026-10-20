package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.config.RobotConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.OdometryMovementSequence;
import org.firstinspires.ftc.teamcode.pedroPathing.OdometryMovementSequence.Step;
import org.firstinspires.ftc.teamcode.telemetry.OdometryTelemetry;

/** Low-power, operator-stepped Pinpoint movement test; loads only drive motors and Pinpoint. */
@TeleOp(name = "Test: Odo 12-inch Movement", group = "Diagnostics")
public final class OdometryMovementTestTeleOp extends LinearOpMode {
  private static final double MAX_POWER = 0.25;
  private static final double TIMEOUT_SECONDS = 10.0;
  private static final double POSITION_TOLERANCE_INCHES = 0.5;
  private static final double HEADING_TOLERANCE_RADIANS = Math.toRadians(3.0);
  private static final double SETTLE_SECONDS = 0.2;

  @Override
  public void runOpMode() {
    telemetry.setMsTransmissionInterval(50);
    telemetry.addLine("Keep still during Pinpoint calibration. Clear space around the robot.");
    OdometryTelemetry.add(telemetry);
    telemetry.update();
    Follower follower = Constants.createFollower(hardwareMap);
    GoBildaPinpointDriver pinpoint =
        ((PinpointLocalizer) follower.getPoseTracker().getLocalizer()).getPinpoint();
    OdometryMovementSequence sequence = new OdometryMovementSequence();
    ElapsedTime legTimer = new ElapsedTime();
    Pose legStart = null;
    Pose target = null;
    int forwardZero = 0;
    int strafeZero = 0;
    double settledSince = -1;
    double lastLegSeconds = 0;
    String status = "Ready: press Y for forward 12 inches";
    follower.setStartingPose(new Pose(0, 0, 0));
    follower.setMaxPower(MAX_POWER);

    try {
      while (opModeInInit()) {
        follower.updatePose();
        // Consume INIT input so a held Y across Play cannot start movement.
        sequence.updateY(gamepad1.y || gamepad2.y, false);
        addOdometryTelemetry(follower, pinpoint);
        telemetry.addLine("Y: forward, left, backward, right (one leg per fresh press)");
        telemetry.addLine("25% max power | B: abort | Measure actual travel with a ruler");
        telemetry.update();
        idle();
      }
      if (isStopRequested()) {
        return;
      }
      // Reset the displayed origin, not the gyro, after any hand movement during INIT.
      follower.setPose(new Pose(0, 0, 0));

      while (opModeIsActive()) {
        boolean wasRunning = sequence.isRunning();
        boolean abort = gamepad1.b || gamepad2.b;
        // Stop before issuing another drive update when the operator holds B.
        if (abort && sequence.isRunning()) {
          follower.breakFollowing();
          sequence.abort();
          lastLegSeconds = legTimer.seconds();
          status = "Aborted: next Y retries the same direction";
        }
        if (sequence.isRunning()) {
          follower.update();
        } else {
          // Keep telemetry live while paused without driving/holding the previous target.
          follower.updatePose();
        }

        Pose pose = follower.getPose();
        boolean ready = pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY
            && finitePose(pose);
        if (sequence.isRunning()) {
          double elapsed = legTimer.seconds();
          double error = Math.hypot(target.getX() - pose.getX(), target.getY() - pose.getY());
          double headingError = Math.abs(Math.atan2(Math.sin(target.getHeading() - pose.getHeading()),
              Math.cos(target.getHeading() - pose.getHeading())));
          boolean atTarget = error <= POSITION_TOLERANCE_INCHES
              && headingError <= HEADING_TOLERANCE_RADIANS;
          if (!atTarget) {
            settledSince = -1;
          } else if (settledSince < 0) {
            settledSince = elapsed;
          }
          if (!ready || elapsed >= TIMEOUT_SECONDS) {
            follower.breakFollowing();
            sequence.abort();
            lastLegSeconds = elapsed;
            status = ready ? "Timed out: next Y retries the same direction"
                : "Pinpoint fault: stopped; check status and pod wiring";
          } else if (atTarget && elapsed - settledSince >= SETTLE_SECONDS) {
            follower.breakFollowing();
            sequence.complete();
            lastLegSeconds = elapsed;
            status = "Completed: paused until a fresh Y press";
          }
        }

        // Presses during movement are consumed but do not queue the next leg.
        // Also prevent a press on the completion loop from immediately launching another leg.
        Step step = sequence.updateY(gamepad1.y || gamepad2.y, ready && !abort && !wasRunning);
        if (step != null) {
          legStart = new Pose(pose.getX(), pose.getY(), pose.getHeading());
          target = step.targetFrom(legStart);
          forwardZero = pinpoint.getEncoderX();
          strafeZero = pinpoint.getEncoderY();
          Path path = new Path(new BezierLine(legStart, target));
          path.setConstantHeadingInterpolation(legStart.getHeading());
          // Hold only until our explicit settle condition; breakFollowing disables hold at pause.
          follower.followPath(path, true);
          legTimer.reset();
          settledSince = -1;
          lastLegSeconds = 0;
          status = "Moving: " + step.getLabel();
        }

        addOdometryTelemetry(follower, pinpoint);
        telemetry.addData("Status", status);
        telemetry.addData("Next Y", "%s 12 in", sequence.getNextStep().getLabel());
        telemetry.addData("Motor Power Limit", "%.0f%%", MAX_POWER * 100);
        telemetry.addLine("Y on either gamepad: next leg | B: abort | Sticks do not drive");
        if (!ready) {
          telemetry.addLine("Movement disabled: Pinpoint must be READY with a finite pose.");
        }
        if (legStart != null) {
          Pose current = follower.getPose();
          double dx = current.getX() - legStart.getX();
          double dy = current.getY() - legStart.getY();
          double forward = dx * Math.cos(legStart.getHeading()) + dy * Math.sin(legStart.getHeading());
          double left = -dx * Math.sin(legStart.getHeading()) + dy * Math.cos(legStart.getHeading());
          Step active = sequence.getActiveStep();
          double along = (forward * active.getForwardInches() + left * active.getLeftInches())
              / OdometryMovementSequence.DISTANCE_INCHES;
          telemetry.addData("Current / Last Leg", active.getLabel());
          telemetry.addData("Leg Time", "%.2f s", sequence.isRunning() ? legTimer.seconds() : lastLegSeconds);
          telemetry.addData("Target", "X %.2f, Y %.2f in", target.getX(), target.getY());
          telemetry.addData("Leg Forward / Left", "%.2f / %.2f in", forward, left);
          telemetry.addData("Odo Distance Along Leg", "%.2f / 12.00 in", along);
          telemetry.addData("Target Position Error", "%.2f in",
              Math.hypot(target.getX() - current.getX(), target.getY() - current.getY()));
          telemetry.addData("Leg Forward / Strafe Tick Change", "%d / %d",
              pinpoint.getEncoderX() - forwardZero, pinpoint.getEncoderY() - strafeZero);
        }
        telemetry.update();
        idle();
      }
    } finally {
      follower.breakFollowing();
    }
  }

  private void addOdometryTelemetry(Follower follower, GoBildaPinpointDriver pinpoint) {
    Pose pose = follower.getPose();
    telemetry.addData("Pose Source", "Pedro: Pinpoint (2 pods + onboard IMU)");
    telemetry.addData("Pinpoint Pose", "X %.2f, Y %.2f in; heading %.1f deg",
        pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
    telemetry.addData("Pinpoint Status", pinpoint.getDeviceStatus());
    telemetry.addData("Connection", pinpoint.getConnectionInfo());
    telemetry.addData("Required Control Hub I2C Bus", RobotConfig.Hardware.PINPOINT_I2C_BUS);
    telemetry.addData("Forward Pod Raw Ticks (X)", pinpoint.getEncoderX());
    telemetry.addData("Strafe Pod Raw Ticks (Y)", pinpoint.getEncoderY());
    OdometryTelemetry.add(telemetry);
  }

  private static boolean finitePose(Pose pose) {
    return Double.isFinite(pose.getX()) && Double.isFinite(pose.getY())
        && Double.isFinite(pose.getHeading());
  }
}
