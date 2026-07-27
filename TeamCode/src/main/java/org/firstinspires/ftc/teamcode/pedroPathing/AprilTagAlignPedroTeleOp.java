package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * TeleOp OpMode that uses Pedro Pathing for driving and automatic AprilTag alignment.
 * Includes linear slide control via D-pad.
 * Automatic alignment is triggered by holding the LEFT TRIGGER.
 */
@TeleOp(name = "AprilTag Align Pedro TeleOp", group = "TeleOp")
public class AprilTagAlignPedroTeleOp extends LinearOpMode {
    private Follower follower;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private slideConstants slide;

    private static final int DESIRED_TAG_ID = 586;
    private static final double DESIRED_DISTANCE = 12.5; // inches

    // PID Controllers for higher accuracy
    private SimplePIDController drivePID = new SimplePIDController(0.04, 0.01, 0.002);
    private SimplePIDController strafePID = new SimplePIDController(0.03, 0.01, 0.002);
    private SimplePIDController turnPID = new SimplePIDController(0.03, 0.01, 0.002);
    private ElapsedTime pidTimer = new ElapsedTime();

    private static final double MAX_AUTO_POWER = 0.25; 
    private static final double MAX_AUTO_TURN = 0.15;
    
    // Friction compensation - minimum power required to move the robot
    private static final double MIN_DRIVE_POW = 0.05;

    @Override
    public void runOpMode() {
        // Initialize Pedro Pathing Follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        // Initialize Slide
        slide = new slideConstants(hardwareMap);

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

        // Start subsystems
        follower.startTeleopDrive();
        slide.start();
        pidTimer.reset();

        while (opModeIsActive()) {
            // Update follower and slide state
            follower.update();

            // ------------------------------------
            // SLIDE CONTROL (DPAD UP/DOWN)
            // ------------------------------------
            if (gamepad1.dpad_up) {
                slide.extendToHigh();
            } else if (gamepad1.right_bumper) {
                slide.extendToMiddle();
            } else if (gamepad1.dpad_down) {
                slide.extendToBottom();
            }

            // ------------------------------------
            // DRIVETRAIN CONTROL
            // ------------------------------------
            if (gamepad1.left_trigger > 0.1) {
                // AUTOMATIC ALIGNMENT (Trigger held)
                AprilTagDetection targetTag = null;
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();

                for (AprilTagDetection detection : currentDetections) {
                    if (detection.id == DESIRED_TAG_ID && detection.metadata != null) {
                        targetTag = detection;
                        break;
                    }
                }

                if (targetTag != null) {
                    double rangeError = targetTag.ftcPose.range - DESIRED_DISTANCE;
                    double bearingError = targetTag.ftcPose.bearing;
                    double yawError = targetTag.ftcPose.yaw;

                    double dt = pidTimer.seconds();
                    pidTimer.reset();

                    // Use PID controllers for much higher accuracy
                    // Maximize accuracy and keep tag in sight:
                    // 1. Turn to keep the tag centered in the camera (Bearing -> Turn)
                    // 2. Strafe to square up with the tag (Yaw -> Strafe)
                    // 3. Drive to reach the target distance (Range -> Drive)
                    double drive = drivePID.calculate(rangeError, dt);
                    double strafe = strafePID.calculate(-yawError, dt);
                    double turn = turnPID.calculate(bearingError, dt);

                    // Friction compensation: if the error is significant, ensure we give enough power to move
                    if (Math.abs(rangeError) > 0.5) drive += Math.signum(drive) * MIN_DRIVE_POW;
                    if (Math.abs(yawError) > 1.0) strafe += Math.signum(strafe) * MIN_DRIVE_POW;
                    if (Math.abs(bearingError) > 1.0) turn += Math.signum(turn) * MIN_DRIVE_POW;

                    drive = Range.clip(drive, -MAX_AUTO_POWER, MAX_AUTO_POWER);
                    strafe = Range.clip(strafe, -MAX_AUTO_POWER, MAX_AUTO_POWER);
                    turn = Range.clip(turn, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                    // Apply powers via Pedro Pathing (robot-centric)
                    follower.setTeleOpDrive(drive, strafe, turn, true);
                    telemetry.addData("Mode", "AUTO-ALIGNING");
                    telemetry.addData("Range Error", rangeError);
                } else {
                    // Tag lost, reset PIDs and hold position
                    drivePID.reset();
                    strafePID.reset();
                    turnPID.reset();
                    follower.setTeleOpDrive(0, 0, 0, true);
                    telemetry.addData("Mode", "TAG LOST - STOPPED");
                }
            } else {
                // MANUAL CONTROL (Default)
                drivePID.reset();
                strafePID.reset();
                turnPID.reset();
                // Invert sticks to match common drive orientation if needed
                double y = -gamepad1.left_stick_y * 0.6; // Scaled for TeleOp
                double x = -gamepad1.left_stick_x * 0.6;
                double rx = -gamepad1.right_stick_x * 0.4;
                
                follower.setTeleOpDrive(y, x, rx, false); // false = Field Centric
                telemetry.addData("Mode", "MANUAL");
            }

            telemetry.addData("Slide Pos", slide.getCurrentPosition());
            telemetry.addData("Robot Position", "X: %.2f, Y: %.2f", follower.getPose().getX(), follower.getPose().getY());
            telemetry.update();
        }

        // Clean up
        visionPortal.close();
    }
}
