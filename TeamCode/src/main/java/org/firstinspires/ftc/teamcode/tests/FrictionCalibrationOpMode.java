package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;

@TeleOp(name = "Friction Calibration TeleOp", group = "Calibration")
@Configurable
public class FrictionCalibrationOpMode extends LinearOpMode {

  /**
   * Minimum linear velocity (in/s) sustained over multiple frames to trigger stiction detection.
   */
  public static double velocityThresholdInchesPerSec = 1.0;

  /**
   * Minimum angular velocity (rad/s) sustained over multiple frames to trigger stiction detection.
   */
  public static double angularVelocityThresholdRadPerSec = 0.05;

  /**
   * Number of consecutive loop iterations velocity must exceed threshold to confirm true motion.
   */
  public static int requiredConsecutiveFrames = 5;

  private Follower follower;
  private List<LynxModule> allHubs;

  private double ksX = 0.0;
  private double ksY = 0.0;
  private double ksRotStatic = 0.0;
  private double ksRotMoving = 0.0;
  private double ksRotMaxOmega = 0.0;
  private double suggestedLookaheadTime = 0.0;

  @Override
  public void runOpMode() throws InterruptedException {
    org.firstinspires.ftc.teamcode.robot.config.generated.config.reload();
    org.firstinspires.ftc.teamcode.utilities.OpModeUtil.reportConfig(telemetry);
    allHubs = hardwareMap.getAll(LynxModule.class);
    for (LynxModule module : allHubs) {
      module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    follower = Constants.create(hardwareMap);
    follower.manual(0, 0, 0);

    telemetry.addLine("Friction Auto-Calibration initialized.");
    telemetry.addLine("Place robot on open carpet with plenty of space!");
    telemetry.addLine("---------------------------------------------");
    telemetry.addLine("Buttons:");
    telemetry.addLine("  A (Cross)  -> Tune Forward Friction (kS X)");
    telemetry.addLine("  B (Circle) -> Tune Strafe Friction (kS Y)");
    telemetry.addLine("  X (Square) -> Tune Rotation Friction (kS Static/Moving/MaxOmega)");
    telemetry.update();

    waitForStart();

    while (opModeIsActive() && !isStopRequested()) {
      clearBulkCache();
      follower.update();

      telemetry.addLine("Select a test:");
      telemetry.addLine("  A (Cross)  -> Tune Forward Friction (kS X)");
      telemetry.addLine("  B (Circle) -> Tune Strafe Friction (kS Y)");
      telemetry.addLine("  X (Square) -> Tune Rotation Friction (kS Static/Moving/MaxOmega)");
      telemetry.addLine("---------------------------------------------");
      telemetry.addData(
          "Last Results:",
          "X: %.4f | Y: %.4f | Rot Static: %.4f | Rot Moving: %.4f | Max Omega: %.2f rad/s",
          ksX,
          ksY,
          ksRotStatic,
          ksRotMoving,
          ksRotMaxOmega);
      telemetry.addLine("Update casablanca.friction.rot with Rot Static");
      telemetry.addLine(
          Double.isNaN(ksRotMoving)
              ? "Rot Moving NOT MEASURED (ran out of lane): keep heading_lock.ks_moving"
              : "Update casablanca.heading_lock.ks_moving with Rot Moving");
      telemetry.addData(
          "Suggested rotation_lookahead_time",
          "%.4f s (0.45 / %.2f)",
          suggestedLookaheadTime,
          ksRotMaxOmega);
      telemetry.update();

      if (gamepad1.a) {
        runForwardTest();
      } else if (gamepad1.b) {
        runStrafeTest();
      } else if (gamepad1.x) {
        runRotationTest();
      }

      idle();
    }
  }

  private void settleLocalizer() throws InterruptedException {
    for (int i = 0; i < 15 && opModeIsActive() && !isStopRequested(); i++) {
      clearBulkCache();
      follower.manual(0.0, 0.0, 0.0);
      follower.update();
      sleep(10);
    }
  }

  private void runForwardTest() throws InterruptedException {
    telemetry.addLine("Running Forward (kS X) Test...");
    telemetry.update();
    settleLocalizer();

    double power = 0.0;
    int consecutiveMovingFrames = 0;

    while (opModeIsActive() && !isStopRequested()) {
      clearBulkCache();
      follower.update();

      power += 0.0003;
      follower.manual(power, 0.0, 0.0);

      double currentVelocity = follower.velocity().toVector2D().magnitude();

      if (currentVelocity > velocityThresholdInchesPerSec) {
        consecutiveMovingFrames++;
      } else {
        consecutiveMovingFrames = 0;
      }

      telemetry.addLine("FORWARD TEST (Ramping Power)");
      telemetry.addData("Current Power", "%.4f", power);
      telemetry.addData("Current Velocity", "%.4f in/s", currentVelocity);
      telemetry.addData("Threshold", "%.2f in/s", velocityThresholdInchesPerSec);
      telemetry.addData(
          "Sustained Motion Frames", "%d/%d", consecutiveMovingFrames, requiredConsecutiveFrames);
      telemetry.update();

      if (consecutiveMovingFrames >= requiredConsecutiveFrames) {
        ksX = power;
        break;
      }
      sleep(10);
    }

    stopRobot();
  }

  private void runStrafeTest() throws InterruptedException {
    telemetry.addLine("Running Strafe (kS Y) Test...");
    telemetry.update();
    settleLocalizer();

    double power = 0.0;
    int consecutiveMovingFrames = 0;

    while (opModeIsActive() && !isStopRequested()) {
      clearBulkCache();
      follower.update();

      power += 0.0003;
      follower.manual(0.0, power, 0.0);

      double currentVelocity = follower.velocity().toVector2D().magnitude();

      if (currentVelocity > velocityThresholdInchesPerSec) {
        consecutiveMovingFrames++;
      } else {
        consecutiveMovingFrames = 0;
      }

      telemetry.addLine("STRAFE TEST (Ramping Power)");
      telemetry.addData("Current Power", "%.4f", power);
      telemetry.addData("Current Velocity", "%.4f in/s", currentVelocity);
      telemetry.addData("Threshold", "%.2f in/s", velocityThresholdInchesPerSec);
      telemetry.addData(
          "Sustained Motion Frames", "%d/%d", consecutiveMovingFrames, requiredConsecutiveFrames);
      telemetry.update();

      if (consecutiveMovingFrames >= requiredConsecutiveFrames) {
        ksY = power;
        break;
      }
      sleep(10);
    }

    stopRobot();
  }

  private void runRotationTest() throws InterruptedException {
    telemetry.addLine("Running Rotation Test: Part 1 - Static (at rest)...");
    telemetry.update();
    settleLocalizer();

    double power = 0.0;
    int consecutiveMovingFrames = 0;

    // Phase 1: Static rotation test from a standing start
    while (opModeIsActive() && !isStopRequested()) {
      clearBulkCache();
      follower.update();

      power += 0.0003;
      follower.manual(0.0, 0.0, power);

      double currentVelocity = Math.abs(follower.velocity().omega);

      if (currentVelocity > angularVelocityThresholdRadPerSec) {
        consecutiveMovingFrames++;
      } else {
        consecutiveMovingFrames = 0;
      }

      telemetry.addLine("ROTATION STATIC TEST (Ramping Turn Power from Rest)");
      telemetry.addData("Current Power", "%.4f", power);
      telemetry.addData("Current Angular Velocity", "%.4f rad/s", currentVelocity);
      telemetry.addData("Threshold", "%.2f rad/s", angularVelocityThresholdRadPerSec);
      telemetry.addData(
          "Sustained Motion Frames", "%d/%d", consecutiveMovingFrames, requiredConsecutiveFrames);
      telemetry.update();

      if (consecutiveMovingFrames >= requiredConsecutiveFrames) {
        ksRotStatic = power;
        break;
      }
      sleep(10);
    }

    stopRobot();
    sleep(500);

    // Phase 2: Moving rotation test while translating at 50% forward power
    telemetry.addLine("Running Rotation Test: Part 2 - Moving (translating)...");
    telemetry.update();

    // Command forward power and wait for translation to stabilize
    double steadyForwardPower = 0.5;
    Pose laneStart = follower.pose();
    // The yaw wobble straight-line driving produces on its own: a turn power is only "moving" the
    // robot once it rotates well past this, in the direction commanded. Counting the wobble
    // recorded the very first ramp step (0.0003) as ks_moving.
    double wobble = 0.0;
    for (int i = 0; i < 50 && opModeIsActive() && !isStopRequested(); i++) {
      clearBulkCache();
      follower.manual(steadyForwardPower, 0.0, 0.0);
      follower.update();
      wobble = Math.max(wobble, Math.abs(follower.velocity().omega));
      sleep(10);
    }
    double movingThreshold = Math.max(angularVelocityThresholdRadPerSec, 2.0 * wobble);
    // The same clear lane the drive tuners assume; the robot is still driving forward throughout.
    double maxTravel = config.casablanca.jump_tuner.max_travel_inches;

    power = 0.0;
    consecutiveMovingFrames = 0;
    ksRotMoving = Double.NaN;

    while (opModeIsActive() && !isStopRequested()) {
      clearBulkCache();
      follower.update();
      if (follower.pose().distance(laneStart) > maxTravel) {
        break; // Out of lane before the robot turned: not measured.
      }

      power += 0.0003;
      follower.manual(steadyForwardPower, 0.0, power);

      // Positive turn power turns CCW, which is positive omega.
      double currentVelocity = follower.velocity().omega;

      if (currentVelocity > movingThreshold) {
        consecutiveMovingFrames++;
      } else {
        consecutiveMovingFrames = 0;
      }

      telemetry.addLine("ROTATION MOVING TEST (Ramping Turn Power while Translating)");
      telemetry.addData("Forward Power", "%.2f", steadyForwardPower);
      telemetry.addData("Current Turn Power", "%.4f", power);
      telemetry.addData("Current Angular Velocity", "%.4f rad/s", currentVelocity);
      telemetry.addData("Threshold", "%.2f rad/s (wobble %.2f)", movingThreshold, wobble);
      telemetry.addData("Travel", "%.1f / %.1f in", follower.pose().distance(laneStart), maxTravel);
      telemetry.addData(
          "Sustained Motion Frames", "%d/%d", consecutiveMovingFrames, requiredConsecutiveFrames);
      telemetry.update();

      if (consecutiveMovingFrames >= requiredConsecutiveFrames) {
        ksRotMoving = power;
        break;
      }
      sleep(10);
    }

    stopRobot();
    sleep(500);

    // Phase 3: Measure maximum steady-state angular velocity at 100% turn power
    telemetry.addLine("Running Rotation Test: Part 3 - Max Angular Velocity (100% Turn Power)...");
    telemetry.update();

    double maxObservedOmega = 0.0;
    for (int i = 0; i < 100 && opModeIsActive() && !isStopRequested(); i++) {
      clearBulkCache();
      follower.manual(0.0, 0.0, 1.0);
      follower.update();

      double omega = Math.abs(follower.velocity().omega);
      if (omega > maxObservedOmega) {
        maxObservedOmega = omega;
      }

      telemetry.addLine("ROTATION MAX VELOCITY TEST (Full Power Spin)");
      telemetry.addData("Current Angular Velocity", "%.4f rad/s", omega);
      telemetry.addData("Max Observed", "%.4f rad/s", maxObservedOmega);
      telemetry.update();
      sleep(10);
    }

    ksRotMaxOmega = maxObservedOmega;
    suggestedLookaheadTime = (maxObservedOmega > 0) ? (0.45 / maxObservedOmega) : 0.075;

    stopRobot();
  }

  private void stopRobot() throws InterruptedException {
    follower.manual(0.0, 0.0, 0.0);
    follower.update();

    telemetry.addLine("Test Complete! Stopping robot...");
    telemetry.update();

    while (opModeIsActive()
        && (follower.velocity().toVector2D().magnitude() > 0.05
            || Math.abs(follower.velocity().omega) > 0.005)) {
      clearBulkCache();
      follower.update();
      follower.manual(0.0, 0.0, 0.0);
      sleep(10);
    }

    sleep(500);
  }

  private void clearBulkCache() {
    for (LynxModule module : allHubs) {
      module.clearBulkCache();
    }
  }
}
