package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.geometry.BezierLine;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
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

    // CAMERA OFFSET: If your camera is physically rotated, enter the offset here in degrees.
    // If it's turned slightly Counter-Clockwise, this will be a positive number (e.g. 5.0)
    private static final double CAMERA_YAW_OFFSET = 0.0;
    private static final double CAMERA_X_OFFSET = 2.0; // Inches (Robot center is 2" right of camera)

    // --- Heading Hold Constants ---
    private double targetHeading = 0.0;
    private final double headingLock_kP = 0.75;
    private final double headingLock_kD = 0.08;
    private double lastError = 0.0;
    private final ElapsedTime headingTimer = new ElapsedTime();
    private final ElapsedTime servoWiggleTimer = new ElapsedTime();
    private final ElapsedTime slideAnimationTimer = new ElapsedTime();
    private boolean isSlideAnimating = false;
    private boolean isAligning = false;

    // PID Controllers - Smoothed for less jerkiness
    private SimplePIDController drivePID = new SimplePIDController(0.035, 0.005, 0.001);
    private SimplePIDController strafePID = new SimplePIDController(0.025, 0.005, 0.001);
    private SimplePIDController turnPID = new SimplePIDController(0.02, 0.005, 0.001);
    private ElapsedTime pidTimer = new ElapsedTime();

    private static final double MAX_AUTO_POWER = 0.20; 
    
    // Friction compensation - minimum power required to move the robot
    private static final double MIN_DRIVE_POW = 0.04;

    private final double HOME_POSITION = 0.0;
    private final double MAX_POSITION = 1.0;
    private Servo myServo;

    @Override
    public void runOpMode() {
        // Initialize Pedro Pathing Follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        // Initialize Slide
        slide = new slideConstants(hardwareMap);

        myServo = hardwareMap.get(Servo.class, "myServoName");

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
        headingTimer.reset();
        servoWiggleTimer.reset();
        targetHeading = follower.getPose().getHeading();

        while (opModeIsActive()) {
            // Update follower and slide state
            follower.update();

            // ------------------------------------
            // SLIDE CONTROL (DPAD UP/DOWN)
            // ------------------------------------
            if (gamepad1.dpad_up) {
                slide.extendToHigh();
                isSlideAnimating = true;
                slideAnimationTimer.reset();
            } else if (gamepad1.right_bumper) {
                slide.extendToMiddle();
            } else if (gamepad1.dpad_down) {
                slide.extendToBottom();
            }

            if (gamepad1.back) {
                slide.resetEncoder();
            }

            if (gamepad1.dpad_right) {
                if (((int) (servoWiggleTimer.seconds() * 4)) % 2 == 0) {
                    myServo.setPosition(MAX_POSITION);
                } else {
                    myServo.setPosition(HOME_POSITION);
                }
            } else if (gamepad1.dpad_left) {
                myServo.setPosition(HOME_POSITION);
            } else if (isSlideAnimating) {
                if (slideAnimationTimer.seconds() < 3.0) {
                    if (((int) (slideAnimationTimer.seconds() * 4)) % 2 == 0) {
                        myServo.setPosition(MAX_POSITION);
                    } else {
                        myServo.setPosition(HOME_POSITION);
                    }
                } else {
                    isSlideAnimating = false;
                    myServo.setPosition(HOME_POSITION);
                }
            }
            double currentHeading = follower.getPose().getHeading();

            // ------------------------------------
            // DRIVETRAIN CONTROL
            // ------------------------------------
            if (gamepad1.left_trigger > 0.1) {
                // AUTOMATIC ALIGNMENT (Trigger held)
                if (!isAligning) {
                    AprilTagDetection targetTag = null;
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                    for (AprilTagDetection detection : currentDetections) {
                        if (detection.id == DESIRED_TAG_ID && detection.metadata != null) {
                            targetTag = detection;
                            break;
                        }
                    }

                    if (targetTag != null) {
                        Pose currentPose = follower.getPose();
                        double alignmentHeading = currentPose.getHeading();

                        // Target 12.5" in front (Z), and offset by CAMERA_X_OFFSET to center robot
                        double relX = targetTag.ftcPose.x - CAMERA_X_OFFSET;
                        double relZ = targetTag.ftcPose.z - DESIRED_DISTANCE;
                        double relYaw = Math.toRadians(targetTag.ftcPose.yaw);

                        // Corrected conversion from camera Z/X to field X/Y
                        // Robot forward component (relZ) and Robot left component (-relX)
                        double fieldDeltaX = relZ * Math.cos(alignmentHeading) + relX * Math.sin(alignmentHeading);
                        double fieldDeltaY = relZ * Math.sin(alignmentHeading) - relX * Math.cos(alignmentHeading);

                        Pose targetPose = new Pose(currentPose.getX() + fieldDeltaX,
                                                   currentPose.getY() + fieldDeltaY,
                                                   alignmentHeading + relYaw);

                        Path path = new Path(new BezierLine(currentPose, targetPose));
                        path.setLinearHeadingInterpolation(alignmentHeading, targetPose.getHeading());
                        
                        follower.followPath(path, true);
                        isAligning = true;
                    }
                }
                
                if (isAligning) {
                    telemetry.addData("Mode", "AUTO-ALIGNING (PATH)");
                    targetHeading = follower.getPose().getHeading();
                    lastError = 0.0;
                    headingTimer.reset();
                }
            } else {
                if (isAligning) {
                    follower.startTeleopDrive();
                    isAligning = false;
                }
                // MANUAL CONTROL (Default)
                drivePID.reset();
                strafePID.reset();
                turnPID.reset();
                
                // 1. Invert axes for field orientation
                double rawY = -gamepad1.left_stick_y;
                double rawX = -gamepad1.left_stick_x;
                double rawRx = -gamepad1.right_stick_x;

                // 2. Calculate translation vector magnitude
                double translationMagnitude = Math.hypot(rawX, rawY);
                double y = 0.0;
                double x = 0.0;

                if (translationMagnitude > 0.05) {
                    double normalizedMagnitude = (translationMagnitude - 0.05) / (1.0 - 0.05);
                    double scaledPower = Math.pow(normalizedMagnitude, 3) * 0.65;
                    y = (rawY / translationMagnitude) * scaledPower;
                    x = (rawX / translationMagnitude) * scaledPower;
                }

                // 3. Rotation control with Heading Lock
                double rx = 0.0;
                double absRx = Math.abs(rawRx);
                double dt = headingTimer.seconds();
                headingTimer.reset();

                if (absRx > 0.05) {
                    double normalizedRx = (absRx - 0.05) / (1.0 - 0.05);
                    rx = Math.signum(rawRx) * Math.pow(normalizedRx, 3) * 0.65;
                    targetHeading = currentHeading;
                    lastError = 0.0;
                } else {
                    double headingError = targetHeading - currentHeading;
                    headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));

                    if (Math.abs(headingError) < Math.toRadians(0.5)) {
                        rx = 0.0;
                    } else {
                        double derivative = (dt > 0) ? (headingError - lastError) / dt : 0.0;
                        rx = (headingError * headingLock_kP) + (derivative * headingLock_kD);
                        rx = Math.max(-0.65, Math.min(0.65, rx));
                    }
                    lastError = headingError;
                }

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
