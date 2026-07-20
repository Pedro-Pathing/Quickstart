package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.List;

/**
 * This is the PoseRecorder OpMode. It lets you author field coordinates by moving the robot
 * instead of measuring the field by hand. Set a starting pose below, run the OpMode, then drive
 * the robot with gamepad 1 OR just push it around by hand (odometry tracks it either way). Press
 * a button to capture wherever the robot currently is.
 *
 * <p>Captured poses are shown on the Driver Station telemetry as a numbered, copy-paste-ready list
 * of {@code new Pose(x, y, Math.toRadians(deg))} lines, and each capture is also logged via
 * {@link RobotLog} under the tag "PoseRecorder" so it reaches the coding laptop through
 * {@code adb logcat}.
 *
 * <p>Controls (gamepad 1):
 * <ul>
 *     <li>A - capture the current pose</li>
 *     <li>B - undo (remove the last captured pose)</li>
 *     <li>Back + Y - clear all captured poses</li>
 * </ul>
 *
 * <p>To use, edit {@link #startingPose} below, and if needed the follower-creation line in
 * {@link #init()}. Captures are in the Follower's native Pedro coordinate frame.
 */
@TeleOp(name = "Pose Recorder", group = "Pedro Pathing")
public class PoseRecorder extends OpMode {
    /** Tag used for {@link RobotLog} so captures show up in {@code adb logcat}. */
    private static final String TAG = "PoseRecorder";

    /** Edit this to set where the robot starts on the field. */
    public Pose startingPose = new Pose(72, 72, Math.toRadians(0));

    private Follower follower;

    private final List<Pose> capturedPoses = new ArrayList<>();

    @Override
    public void init() {
        // Edit this line if you build your Follower differently.
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
    }

    @Override
    public void init_loop() {
        telemetry.addLine("Pose Recorder ready.");
        telemetry.addLine("Drive with gamepad 1 or push the robot by hand.");
        telemetry.addLine("A = capture, B = undo, Back + Y = clear.");
        telemetry.update();
        follower.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // Let the robot be driven; it can also just be pushed by hand (odometry tracks it either way).
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();

        if (gamepad1.aWasPressed()) {
            Pose captured = follower.getPose();
            capturedPoses.add(captured);
            RobotLog.ii(TAG, "Captured #%d: %s", capturedPoses.size(), formatPose(captured));
        }

        if (gamepad1.bWasPressed() && !capturedPoses.isEmpty()) {
            Pose removed = capturedPoses.remove(capturedPoses.size() - 1);
            RobotLog.ii(TAG, "Removed last capture: %s", formatPose(removed));
        }

        // Back + Y guard combo so the list isn't cleared by accident.
        if (gamepad1.back && gamepad1.yWasPressed()) {
            capturedPoses.clear();
            RobotLog.ii(TAG, "Cleared all captures.");
        }

        Pose current = follower.getPose();
        telemetry.addLine("A = capture, B = undo, Back + Y = clear.");
        telemetry.addData("Current pose", formatPose(current));
        telemetry.addLine();
        telemetry.addData("Captured poses", capturedPoses.size());
        for (int i = 0; i < capturedPoses.size(); i++) {
            telemetry.addData(String.valueOf(i), formatPose(capturedPoses.get(i)));
        }
        telemetry.update();
    }

    /** Formats a Pose as a copy-paste-ready {@code new Pose(x, y, Math.toRadians(deg))} line. */
    private String formatPose(Pose pose) {
        return String.format(
                "new Pose(%.2f, %.2f, Math.toRadians(%.2f))",
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
    }
}
