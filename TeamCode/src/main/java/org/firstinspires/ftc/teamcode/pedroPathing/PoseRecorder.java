package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

/**
 * This is the PoseRecorder OpMode. It lets you author field coordinates by moving the robot
 * instead of measuring the field by hand. Set a starting pose below, run the OpMode, then drive
 * the robot with gamepad 1 OR just push it around by hand (odometry tracks it either way). Press
 * a button to capture wherever the robot currently is.
 *
 * <p>Captured poses are shown on the Driver Station telemetry as a numbered, copy-paste-ready list
 * of {@code new Pose(x, y, Math.toRadians(deg))} lines. Press X to write the whole list to a text
 * file on the Robot Controller (see {@link #OUTPUT_PATH}) so you can pull it onto your coding
 * laptop and drop the lines straight into your path code.
 *
 * <p>Getting the file onto your computer:
 * <ul>
 *     <li>Android Studio: {@code adb pull /sdcard/FIRST/pose_recorder.txt} (adjust to your PATH).</li>
 *     <li>OnBot Java / no adb: plug the Control Hub or RC phone into your computer over USB and
 *         browse to the FIRST folder, or use the file's on-robot path shown in telemetry.</li>
 * </ul>
 *
 * <p>Controls (gamepad 1):
 * <ul>
 *     <li>A - capture the current pose</li>
 *     <li>B - undo (remove the last captured pose)</li>
 *     <li>X - save all captured poses to the file</li>
 *     <li>Back + Y - clear all captured poses</li>
 * </ul>
 *
 * <p>To use, edit {@link #startingPose} below, and if needed {@link #OUTPUT_PATH} and the
 * follower-creation line in {@link #init()}. Captures are in the Follower's native Pedro
 * coordinate frame.
 */
@TeleOp(name = "Pose Recorder", group = "Pedro Pathing")
public class PoseRecorder extends OpMode {
    /**
     * Where captured poses are written when you press X. Defaults to the FTC "FIRST" folder on the
     * Robot Controller, which every team already has and can reach over adb or USB. Edit this to
     * put the file wherever you like (any absolute path the app can write to).
     */
    public String OUTPUT_PATH = new File(AppUtil.FIRST_FOLDER, "pose_recorder.txt").getPath();

    /** Edit this to set where the robot starts on the field. */
    public Pose startingPose = new Pose(72, 72, Math.toRadians(0));

    private Follower follower;

    private final List<Pose> capturedPoses = new ArrayList<>();

    private String saveStatus = "Nothing saved yet.";

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
        telemetry.addLine("A = capture, B = undo, X = save, Back + Y = clear.");
        telemetry.addData("Save file", OUTPUT_PATH);
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
            capturedPoses.add(follower.getPose());
        }

        if (gamepad1.bWasPressed() && !capturedPoses.isEmpty()) {
            capturedPoses.remove(capturedPoses.size() - 1);
        }

        if (gamepad1.xWasPressed()) {
            saveToFile();
        }

        // Back + Y guard combo so the list isn't cleared by accident.
        if (gamepad1.back && gamepad1.yWasPressed()) {
            capturedPoses.clear();
        }

        Pose current = follower.getPose();
        telemetry.addLine("A = capture, B = undo, X = save, Back + Y = clear.");
        telemetry.addData("Current pose", formatPose(current));
        telemetry.addLine();
        telemetry.addData("Captured poses", capturedPoses.size());
        for (int i = 0; i < capturedPoses.size(); i++) {
            telemetry.addData(String.valueOf(i), formatPose(capturedPoses.get(i)));
        }
        telemetry.addLine();
        telemetry.addData("Save file", OUTPUT_PATH);
        telemetry.addData("Save status", saveStatus);
        telemetry.update();
    }

    /** Writes every captured pose to {@link #OUTPUT_PATH} as copy-paste-ready lines. */
    private void saveToFile() {
        StringBuilder sb = new StringBuilder();
        sb.append("// Pose Recorder capture — ").append(capturedPoses.size()).append(" pose(s)\n");
        for (Pose pose : capturedPoses) {
            sb.append(formatPose(pose)).append("\n");
        }

        File file = new File(OUTPUT_PATH);
        try {
            File parent = file.getParentFile();
            if (parent != null) {
                parent.mkdirs();
            }
            ReadWriteFile.writeFileOrThrow(file, sb.toString());
            saveStatus = "Saved " + capturedPoses.size() + " pose(s) to " + file.getPath();
        } catch (Exception e) {
            saveStatus = "Save FAILED: " + e.getMessage();
        }
    }

    /** Formats a Pose as a copy-paste-ready {@code new Pose(x, y, Math.toRadians(deg))} line. */
    private String formatPose(Pose pose) {
        return String.format(
                "new Pose(%.2f, %.2f, Math.toRadians(%.2f))",
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
    }
}
