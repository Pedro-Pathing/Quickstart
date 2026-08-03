package org.firstinspires.ftc.teamcode.pedroPathing.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.slideConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Base")
public class Base extends LinearOpMode {

    private Follower follower;
    private slideConstants slide;
    private DcMotor intakeMotor;
    private Servo myServo;

    private final double HOME_POSITION = 0.0;
    private final double MAX_POSITION = 1.0;

    private final double DRIVE_SPEED_LIMIT = 0.65;
    private final double DEADZONE = 0.05;

    private double targetHeading = 0.0;
    private final double headingLock_kP = 0.75;
    private final double headingLock_kD = 0.08;

    private double lastError = 0.0;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime servoWiggleTimer = new ElapsedTime();
    private final ElapsedTime slideAnimationTimer = new ElapsedTime();
    private final ElapsedTime pathUpdateTimer = new ElapsedTime();

    private boolean isSlideAnimating = false;
    private boolean lastOptionsState = false;
    private boolean isAligning = false;
    private boolean isCoasting = false;
    private double lastHeading = 0.0;
    private Pose lastTargetPose = null;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // --- ALIGNMENT CONSTANTS ---
    private static final int DESIRED_TAG_ID = 586;
    private static final double DESIRED_DISTANCE = 19.5;
    private static final double ALIGN_UPDATE_SECONDS = 0.25;

    private static final double CAMERA_FORWARD_OFFSET = 0.0;
    private static final double CAMERA_LEFT_OFFSET = -2.0;

    private static final int REQUIRED_STABLE_FRAMES = 4;
    private static final double MAX_TRANSLATION_JUMP = 5.0;
    private static final double MAX_YAW_JUMP_DEG = 10.0;
    private static final double FILTER_ALPHA = 0.3;

    // --- DASHBOARD DRAWING CONSTANTS ---
// FTC Dashboard coordinate max out at 72 and -72 from the center (0,0).
// Tweak these so the green box matches where your 3x5 layout lives on the map!
    private static final double DASHBOARD_TAG_X = 0.0;
    private static final double DASHBOARD_TAG_Y = 40.0;

    private boolean hasFilteredTag = false;
    private double filteredX = 0.0;
    private double filteredY = 0.0;
    private double filteredYaw = 0.0;
    private double lastRawX = 0.0;
    private double lastRawY = 0.0;
    private double lastRawYaw = 0.0;
    private int stableFrames = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        follower = Constants.createFollower(hardwareMap);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        myServo = hardwareMap.get(Servo.class, "myServoName");
        slide = new slideConstants(hardwareMap);

        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new android.util.Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .build();

        FtcDashboard.getInstance().startCameraStream(visionPortal, 15);

        while (!isStopRequested() && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(20);
        }

        if (isStopRequested()) return;

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl != null && exposureControl.isModeSupported(ExposureControl.Mode.Manual)) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            exposureControl.setExposure(15, TimeUnit.MILLISECONDS);
        }

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        if (gainControl != null) {
            gainControl.setGain(200);
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        follower.startTeleopDrive();
        slide.start();

        targetHeading = follower.getPose().getHeading();
        timer.reset();
        servoWiggleTimer.reset();
        pathUpdateTimer.reset();

        while (opModeIsActive()) {
            follower.update();

            boolean currentOptionsState = gamepad1.options || gamepad1.start;
            if (currentOptionsState && !lastOptionsState) {
                Pose currentPose = follower.getPose();
                follower.setPose(new Pose(currentPose.getX(), currentPose.getY(), 0.0));
                targetHeading = 0.0;
                lastError = 0.0;
            }
            lastOptionsState = currentOptionsState;

            double currentHeading = follower.getPose().getHeading();

            if (gamepad1.left_trigger > 0.1) {
                if (!isAligning || pathUpdateTimer.seconds() >= ALIGN_UPDATE_SECONDS) {
                    AprilTagDetection targetTag = null;
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();

                    for (AprilTagDetection detection : currentDetections) {
                        if (detection.id == DESIRED_TAG_ID && detection.metadata != null) {
                            targetTag = detection;
                            break;
                        }
                    }

                    if (targetTag != null) {
                        double rawPedroX = targetTag.ftcPose.y;
                        double rawPedroY = -targetTag.ftcPose.x;
                        double rawYaw = targetTag.ftcPose.yaw;

                        if (!hasFilteredTag) {
                            filteredX = rawPedroX;
                            filteredY = rawPedroY;
                            filteredYaw = rawYaw;
                            hasFilteredTag = true;
                            stableFrames = 1;
                        } else {
                            double xJump = Math.abs(rawPedroX - lastRawX);
                            double yJump = Math.abs(rawPedroY - lastRawY);
                            double yawJump = Math.abs(rawYaw - lastRawYaw);

                            if (xJump > MAX_TRANSLATION_JUMP || yJump > MAX_TRANSLATION_JUMP || yawJump > MAX_YAW_JUMP_DEG) {
                                stableFrames = 0;
                            } else {
                                filteredX = FILTER_ALPHA * rawPedroX + (1.0 - FILTER_ALPHA) * filteredX;
                                filteredY = FILTER_ALPHA * rawPedroY + (1.0 - FILTER_ALPHA) * filteredY;
                                filteredYaw = FILTER_ALPHA * rawYaw + (1.0 - FILTER_ALPHA) * filteredYaw;
                                stableFrames++;
                            }
                        }

                        lastRawX = rawPedroX;
                        lastRawY = rawPedroY;
                        lastRawYaw = rawYaw;

                        if (stableFrames >= REQUIRED_STABLE_FRAMES) {
                            Pose currentPose = follower.getPose();
                            double robotHeading = currentPose.getHeading();

                            double tagLocalX = filteredX + CAMERA_FORWARD_OFFSET;
                            double tagLocalY = filteredY + CAMERA_LEFT_OFFSET;

                            double tagFieldX = currentPose.getX() + (tagLocalX * Math.cos(robotHeading)) - (tagLocalY * Math.sin(robotHeading));
                            double tagFieldY = currentPose.getY() + (tagLocalX * Math.sin(robotHeading)) + (tagLocalY * Math.cos(robotHeading));

                            double tagYawRad = Math.toRadians(filteredYaw);
                            double finalHeading = robotHeading + tagYawRad;

                            double standoffX = tagFieldX - (DESIRED_DISTANCE * Math.cos(finalHeading));
                            double standoffY = tagFieldY - (DESIRED_DISTANCE * Math.sin(finalHeading));

                            double targetX = standoffX - (CAMERA_FORWARD_OFFSET * Math.cos(finalHeading)) + (CAMERA_LEFT_OFFSET * Math.sin(finalHeading));
                            double targetY = standoffY - (CAMERA_FORWARD_OFFSET * Math.sin(finalHeading)) - (CAMERA_LEFT_OFFSET * Math.cos(finalHeading));

                            Pose targetPose = new Pose(targetX, targetY, finalHeading);

                            boolean shouldUpdatePath = false;
                            if (lastTargetPose == null) {
                                shouldUpdatePath = true;
                            } else {
                                double dx = Math.abs(targetPose.getX() - lastTargetPose.getX());
                                double dy = Math.abs(targetPose.getY() - lastTargetPose.getY());
                                double dh = Math.abs(Math.atan2(
                                        Math.sin(targetPose.getHeading() - lastTargetPose.getHeading()),
                                        Math.cos(targetPose.getHeading() - lastTargetPose.getHeading())
                                ));
                                if (dx > 1.5 || dy > 1.5 || dh > Math.toRadians(3.0)) {
                                    shouldUpdatePath = true;
                                }
                            }

                            if (shouldUpdatePath) {
                                BezierLine pathLine = new BezierLine(currentPose, targetPose);
                                Path path = new Path(pathLine);
                                path.setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading());

                                follower.setMaxPower(0.775);
                                follower.followPath(path, true);

                                lastTargetPose = targetPose;
                                telemetry.addData("Align", "Path Coordinates Locked");
                            } else {
                                telemetry.addData("Align", "Holding Stable Path");
                            }

                            isAligning = true;
                            pathUpdateTimer.reset();
                        }
                    } else if (isAligning) {
                        telemetry.addData("Align", "Tag Lost - Continuing Last Path");
                    }
                }

                if (isAligning) {
                    targetHeading = follower.getPose().getHeading();
                    lastError = 0.0;
                    timer.reset();
                }
            } else {
                if (isAligning) {
                    follower.startTeleopDrive();
                    follower.setMaxPower(1.0);
                    isAligning = false;
                    lastTargetPose = null;
                    hasFilteredTag = false;
                }

                double rawY = -gamepad1.left_stick_y;
                double rawX = -gamepad1.left_stick_x;
                double rawRx = -gamepad1.right_stick_x;

                double translationMagnitude = Math.hypot(rawX, rawY);
                double y = 0.0;
                double x = 0.0;

                if (translationMagnitude > DEADZONE) {
                    double normalizedMagnitude = (translationMagnitude - DEADZONE) / (1.0 - DEADZONE);
                    double scaledPower = Math.pow(normalizedMagnitude, 3) * DRIVE_SPEED_LIMIT;
                    y = (rawY / translationMagnitude) * scaledPower;
                    x = (rawX / translationMagnitude) * scaledPower;
                }

                double rx = 0.0;
                double absRx = Math.abs(rawRx);
                double dt = timer.seconds();
                timer.reset();

                // Calculate angular velocity (radians per second)
                double angularVelocity = (dt > 0) ? (currentHeading - lastHeading) / dt : 0.0;
                lastHeading = currentHeading;

                if (absRx > DEADZONE) {
                    // CASE 1: Manual Turn - Driver is actively rotating the robot
                    double normalizedRx = (absRx - DEADZONE) / (1.0 - DEADZONE);
                    rx = Math.signum(rawRx) * Math.pow(normalizedRx, 3) * DRIVE_SPEED_LIMIT;

                    isCoasting = true; // Mark as coasting for when the stick is released
                    targetHeading = currentHeading;
                    lastError = 0.0;
                } else {
                    // CASE 2: No manual rotation commanded
                    if (isCoasting && Math.abs(angularVelocity) < 0.1) {
                        // Robot was spinning but has now physically settled.
                        // ACTIVATE heading lock now.
                        isCoasting = false;
                    }

                    if (isCoasting) {
                        // Still coasting from momentum - No lock power yet
                        rx = 0.0;
                        targetHeading = currentHeading; // Keep target updated until settled
                        lastError = 0.0;
                    } else {
                        // Lock is ACTIVE - Hold the settled heading
                        double headingError = targetHeading - currentHeading;
                        headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));

                        if (Math.abs(headingError) < Math.toRadians(1.0)) {
                            rx = 0.0;
                        } else {
                            double derivative = (dt > 0) ? (headingError - lastError) / dt : 0.0;
                            rx = (headingError * headingLock_kP) + (derivative * headingLock_kD);
                            rx = Math.max(-DRIVE_SPEED_LIMIT, Math.min(DRIVE_SPEED_LIMIT, rx));
                        }
                        lastError = headingError;
                    }
                }

                follower.setTeleOpDrive(y, x, rx, false);

                intakeMotor.setPower(gamepad1.a ? 0.80 : 0.0);

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
                    if (slideAnimationTimer.seconds() < 6.0) {
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
            } // END OF IF/ELSE CONTROLS BLOCK

// --------------------------------------------------------
// --- FTC DASHBOARD DRAWING & TELEMETRY ---
// Moved outside the if/else so it ALWAYS runs!
// --------------------------------------------------------

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();
            Pose currentPose = follower.getPose();

// 1. Draw the AprilTag Target (Green Square)
            fieldOverlay.setStrokeWidth(1);
            fieldOverlay.setStroke("#00FF00");
            fieldOverlay.setFill("#00FF00");
            fieldOverlay.fillRect(DASHBOARD_TAG_X - 2, DASHBOARD_TAG_Y - 2, 4, 4);

// 2. Draw the Robot (Blue Circle)
            fieldOverlay.setStroke("#0000FF");
            fieldOverlay.strokeCircle(currentPose.getX(), currentPose.getY(), 9);

// Draw the Robot Heading (Blue Line protruding from center)
            double headingLineX = currentPose.getX() + 9 * Math.cos(currentHeading);
            double headingLineY = currentPose.getY() + 9 * Math.sin(currentHeading);
            fieldOverlay.strokeLine(currentPose.getX(), currentPose.getY(), headingLineX, headingLineY);

// 3. Draw the Alignment Path (Red Line)
            if (isAligning && lastTargetPose != null) {
                fieldOverlay.setStroke("#FF0000");
                fieldOverlay.strokeLine(currentPose.getX(), currentPose.getY(), lastTargetPose.getX(), lastTargetPose.getY());
                fieldOverlay.strokeCircle(lastTargetPose.getX(), lastTargetPose.getY(), 2);
            }

// Send the drawing to FTC Dashboard
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

// Send standard text telemetry
            com.pedropathing.math.Vector velocity = follower.getVelocity();
            double velMag = velocity.getMagnitude();

            // Estimates based on measured deceleration constants in Constants.java
            // Acceleration is ~32 in/s^2 forward, ~68 in/s^2 lateral. Using average for estimate.
            double avgDecel = 50.0;
            double stopTime = (velMag > 0) ? velMag / avgDecel : 0;
            double stopDist = 0.5 * velMag * stopTime;

            telemetry.addData("X Position", currentPose.getX());
            telemetry.addData("Y Position", currentPose.getY());
            telemetry.addData("Heading (Deg)", Math.toDegrees(currentHeading));
            telemetry.addData("Target Heading (Deg)", Math.toDegrees(targetHeading));
            telemetry.addData("Slide Position", slide.getCurrentPosition());
            telemetry.addData("Est. Stop Time", "%.2f s", stopTime);
            telemetry.addData("Est. Stop Dist", "%.2f in", stopDist);
            telemetry.update();

        } // END OF WHILE LOOP
        visionPortal.close();
    }
}