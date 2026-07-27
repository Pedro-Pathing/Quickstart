package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Autonomous OpMode that uses Pedro Pathing to align with an AprilTag.
 * Targets Tag ID 586 and maintains a distance of 20 inches.
 */
@Autonomous(name = "AprilTag Align Pedro Auto", group = "Autonomous")
public class AprilTagAlignPedroAuto extends LinearOpMode {
    private Follower follower;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private static final int DESIRED_TAG_ID = 586;
    private static final double DESIRED_DISTANCE = 20.0; // inches

    // PID-like gain constants for alignment (Tuned for smoother approach)
    private static final double DISTANCE_GAIN = 0.02; // Reduced from 0.04
    private static final double BEARING_GAIN = 0.015; // Reduced from 0.03
    private static final double YAW_GAIN = 0.015;     // Reduced from 0.03

    private static final double MAX_AUTO_POWER = 0.15; // Reduced from 0.5
    private static final double MAX_AUTO_TURN = 0.10;  // Reduced from 0.3

    @Override
    public void runOpMode() {
        // Initialize Pedro Pathing Follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        // Initialize AprilTag Processor
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // Initialize Vision Portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        // Send VisionPortal camera stream to FTC Dashboard
        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized. Targeting Tag %d", DESIRED_TAG_ID);
        telemetry.update();

        waitForStart();

        // Start the follower for teleop-style driving control
        follower.startTeleopDrive();

        while (opModeIsActive()) {
            // Update follower state
            follower.update();

            AprilTagDetection targetTag = null;
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            // Search for the desired tag
            for (AprilTagDetection detection : currentDetections) {
                if (detection.id == DESIRED_TAG_ID && detection.metadata != null) {
                    targetTag = detection;
                    break;
                }
            }

            if (targetTag != null) {
                // Calculate errors
                double rangeError = targetTag.ftcPose.range - DESIRED_DISTANCE;
                double bearingError = targetTag.ftcPose.bearing;
                double yawError = targetTag.ftcPose.yaw;

                // Calculate drive powers using proportional control
                double drive = Range.clip(rangeError * DISTANCE_GAIN, -MAX_AUTO_POWER, MAX_AUTO_POWER);
                double strafe = Range.clip(bearingError * BEARING_GAIN, -MAX_AUTO_POWER, MAX_AUTO_POWER);
                double turn = Range.clip(yawError * YAW_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                // Apply powers via Pedro Pathing (robot-centric)
                follower.setTeleOpDrive(drive, strafe, turn, true);

                telemetry.addData("Tag", "ID %d FOUND", DESIRED_TAG_ID);
                telemetry.addData("Range Error", "%.2f in", rangeError);
                telemetry.addData("Bearing Error", "%.2f deg", bearingError);
                telemetry.addData("Yaw Error", "%.2f deg", yawError);
            } else {
                // Tag not found, stop moving
                follower.setTeleOpDrive(0, 0, 0, true);
                telemetry.addData("Tag", "ID %d NOT FOUND", DESIRED_TAG_ID);
            }

            telemetry.addData("Robot Position", "X: %.2f, Y: %.2f", follower.getPose().getX(), follower.getPose().getY());
            telemetry.update();
        }

        // Clean up
        visionPortal.close();
    }
}
