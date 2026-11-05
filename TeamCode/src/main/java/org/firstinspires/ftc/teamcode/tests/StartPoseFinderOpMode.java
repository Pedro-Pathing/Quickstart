package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.ConfigLoader;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.records.Alliance;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.OpModeUtil;

/**
 * Odometry never starts the match knowing where it is; this OpMode gives a human the delta needed
 * to physically carry the robot from a TILE seam crossing (two TILES from the audience wall and two
 * from the RED alliance wall, heading 0; the field centre is inside the HIVE frame) to wherever the
 * selected auto's config start pose actually is, then lets that arrival be confirmed as the new
 * origin.
 *
 * <p>Gamepad1: left stick translates and right stick turns (robot-centric, scaled down for fine
 * positioning), dpad left/right selects alliance, dpad up/down selects the CELL A ({@code
 * auto_poses.cell_a}) or CELL B ({@code auto_poses.cell_b}) routine, A confirms the robot is now
 * sitting at the selected start pose and re-zeros the follower there.
 */
@TeleOp(name = "Start Pose Finder", group = "Calibration")
@Configurable
public class StartPoseFinderOpMode extends OpMode {

  /**
   * Ceiling on stick-commanded power. This OpMode exists to walk the robot the last few inches onto
   * a start pose, so it is deliberately slower than match teleop — {@code config.teleop.max_speed}
   * would overshoot the target by more than the delta being nulled out.
   */
  public static double driveSpeed = 0.35;

  private Follower follower;

  private Alliance alliance = Alliance.RED;
  private boolean near = true;
  private Pose configStartPose = new Pose(0, 0, 0);

  private boolean lastDpadLeft, lastDpadRight, lastDpadUp, lastDpadDown, lastA;

  @Override
  public void init() {
    config.reload();
    OpModeUtil.reportConfig(telemetry); // a rejected push or broken tuning, at INIT
    follower = Constants.create(hardwareMap);
    double seam = config.field.size_in / 3.0;
    follower.setPose(new Pose(seam, seam, 0));
    recomputeConfigStartPose();
  }

  @Override
  public void init_loop() {
    handleSelectionInput();
    telemetry.addLine("== Start Pose Finder ==");
    telemetry.addData("Alliance (dpad left/right)", alliance);
    telemetry.addData("Auto (dpad up/down)", near ? "CELL A routine" : "CELL B routine");
    telemetry.addData(
        "Config start pose",
        "(%.1f, %.1f, %.1f deg)",
        configStartPose.x(),
        configStartPose.y(),
        Math.toDegrees(configStartPose.heading()));
    telemetry.update();
  }

  @Override
  public void start() {
    follower.manual(0, 0, 0);
  }

  @Override
  public void loop() {
    handleSelectionInput();

    // Written before update() so the vector lands on the motors this loop rather than next.
    // Deliberately raw and robot-centric, bypassing Casablanca: the whole premise of this OpMode is
    // that the follower's pose is not yet trusted, and Casablanca's zone protections are computed
    // from that pose. Cubed sticks give resolution near center, which is where all the work is.
    follower.manual(
        Math.clamp(-Math.pow(gamepad1.left_stick_y, 3), -driveSpeed, driveSpeed),
        Math.clamp(-Math.pow(gamepad1.left_stick_x, 3), -driveSpeed, driveSpeed),
        Math.clamp(-Math.pow(gamepad1.right_stick_x, 3), -driveSpeed, driveSpeed));

    follower.update();

    Pose currentPose = follower.pose();
    double dx = configStartPose.x() - currentPose.x();
    double dy = configStartPose.y() - currentPose.y();
    double distance = currentPose.distance(configStartPose);
    double bearingDeg = AngleUnit.normalizeDegrees(Math.toDegrees(Math.atan2(dy, dx)));

    telemetry.addLine("== Start Pose Finder ==");
    telemetry.addData("Alliance (dpad left/right)", alliance);
    telemetry.addData("Auto (dpad up/down)", near ? "CELL A routine" : "CELL B routine");
    telemetry.addLine();
    telemetry.addData(
        "Raw follower pose",
        "(%.2f, %.2f, %.1f deg)",
        currentPose.x(),
        currentPose.y(),
        Math.toDegrees(currentPose.heading()));
    telemetry.addData(
        "Raw start pose",
        "(%.2f, %.2f, %.1f deg)",
        configStartPose.x(),
        configStartPose.y(),
        Math.toDegrees(configStartPose.heading()));
    telemetry.addLine();
    telemetry.addData("Distance to start", "%.2f in", distance);
    telemetry.addData("Move", "%+.2f in X, %+.2f in Y", dx, dy);
    telemetry.addData("Bearing (field frame)", "%.1f deg", bearingDeg);
    telemetry.addLine();
    telemetry.addData("A", "confirm robot is at start pose -> re-zero follower here");
    telemetry.update();

    if (gamepad1.a && !lastA) {
      follower.setPose(configStartPose);
    }
    lastA = gamepad1.a;
  }

  private void handleSelectionInput() {
    if (gamepad1.dpad_left && !lastDpadLeft) {
      alliance = Alliance.RED;
      recomputeConfigStartPose();
    }
    if (gamepad1.dpad_right && !lastDpadRight) {
      alliance = Alliance.BLUE;
      recomputeConfigStartPose();
    }
    if (gamepad1.dpad_up && !lastDpadUp) {
      near = true;
      recomputeConfigStartPose();
    }
    if (gamepad1.dpad_down && !lastDpadDown) {
      near = false;
      recomputeConfigStartPose();
    }
    lastDpadLeft = gamepad1.dpad_left;
    lastDpadRight = gamepad1.dpad_right;
    lastDpadUp = gamepad1.dpad_up;
    lastDpadDown = gamepad1.dpad_down;
  }

  private void recomputeConfigStartPose() {
    String allianceStr = alliance == Alliance.RED ? "red" : "blue";
    configStartPose =
        near
            ? ConfigLoader.loadMerged(
                    config.CellAAuto.class, "auto_poses.cell_a." + allianceStr, "auto")
                .start
            : ConfigLoader.loadMerged(
                    config.CellBAuto.class, "auto_poses.cell_b." + allianceStr, "auto")
                .start;
  }
}
