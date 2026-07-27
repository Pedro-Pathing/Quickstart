package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "TeleopOffseason")
public class TeleopOffseason extends LinearOpMode {

    private Follower follower;
    private slideConstants slide;
    private DcMotor intakeMotor;
    private Servo myServo;

    // Define preset positions (0.0 to 1.0)
    private final double HOME_POSITION = 0.0;
    private final double MAX_POSITION = 1.0;

    private final double DRIVE_SPEED_LIMIT = 0.65;
    private final double DEADZONE = 0.05;

    // --- Heading Hold Constants ---
    private double targetHeading = 0.0;
    private final double headingLock_kP = 0.75;
    private final double headingLock_kD = 0.08;

    // State variables
    private double lastError = 0.0;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime servoWiggleTimer = new ElapsedTime();

    // Edge detection for field-centric reset button
    private boolean lastOptionsState = false;

    // --- AprilTag Alignment Features ---
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private static final int DESIRED_TAG_ID = 586;
    private static final double DESIRED_DISTANCE = 12.5; // inches

    // PID Controllers for AprilTag Alignment
    private SimplePIDController drivePID = new SimplePIDController(0.04, 0.01, 0.002);
    private SimplePIDController strafePID = new SimplePIDController(0.03, 0.01, 0.002);
    private SimplePIDController turnPID = new SimplePIDController(0.03, 0.01, 0.002);
    private ElapsedTime pidTimer = new ElapsedTime();

    private static final double MAX_AUTO_POWER = 0.25;
    private static final double MAX_AUTO_TURN = 0.15;
    private static final double MIN_DRIVE_POW = 0.05;

    @Override
    public void runOpMode() throws InterruptedException {

        // Enable bulk reading on REV Hubs to minimize loop times and maximize odometry accuracy
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

        // --- AprilTag Initialization ---
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized. Bulk reading active.");
        telemetry.update();

        waitForStart();

        follower.startTeleopDrive();
        slide.start();

        // Initialize target heading and reset timer
        targetHeading = follower.getPose().getHeading();
        timer.reset();
        pidTimer.reset();
        servoWiggleTimer.reset();

        while (opModeIsActive()) {
            // Update localization algorithms
            follower.update();

            // ------------------------------------
            // FIELD-CENTRIC HEADING RESET (START/OPTIONS BUTTON)
            // ------------------------------------
            boolean currentOptionsState = gamepad1.options || gamepad1.start;
            if (currentOptionsState && !lastOptionsState) {
                // Keep X/Y position, reset heading to 0 (making current orientation "Forward")
                Pose currentPose = follower.getPose();
                follower.setPose(new Pose(currentPose.getX(), currentPose.getY(), 0.0));

                // Reset target heading for the lock algorithm
                targetHeading = 0.0;
                lastError = 0.0;
            }
            lastOptionsState = currentOptionsState;

            // Current robot heading from Pedro Pathing
            double currentHeading = follower.getPose().getHeading();

            // ------------------------------------
            // DRIVETRAIN CONTROL VIA ODOMETRY
            // ------------------------------------
            if (gamepad1.left_trigger > 0.1) {
                // AUTOMATIC APRILTAG ALIGNMENT
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
                    double pidDt = pidTimer.seconds();
                    pidTimer.reset();

                    // Maximize accuracy and keep tag in sight:
                    // 1. Turn to keep the tag centered in the camera (Bearing -> Turn)
                    // 2. Strafe to square up with the tag (Yaw -> Strafe)
                    // 3. Drive to reach the target distance (Range -> Drive)
                    double drive = drivePID.calculate(rangeError, pidDt);
                    double strafe = strafePID.calculate(-yawError, pidDt);
                    double turn = turnPID.calculate(bearingError, pidDt);

                    if (Math.abs(rangeError) > 0.5) drive += Math.signum(drive) * MIN_DRIVE_POW;
                    if (Math.abs(yawError) > 1.0) strafe += Math.signum(strafe) * MIN_DRIVE_POW;
                    if (Math.abs(bearingError) > 1.0) turn += Math.signum(turn) * MIN_DRIVE_POW;

                    drive = Range.clip(drive, -MAX_AUTO_POWER, MAX_AUTO_POWER);
                    strafe = Range.clip(strafe, -MAX_AUTO_POWER, MAX_AUTO_POWER);
                    turn = Range.clip(turn, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                    follower.setTeleOpDrive(drive, strafe, turn, true);
                    telemetry.addData("Mode", "AUTO-ALIGNING");
                } else {
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

                // 1. Invert axes for field orientation
                double rawY = -gamepad1.left_stick_y;
                double rawX = -gamepad1.left_stick_x;
                double rawRx = -gamepad1.right_stick_x;

                // 2. Calculate translation vector magnitude
                double translationMagnitude = Math.hypot(rawX, rawY);
                double y = 0.0;
                double x = 0.0;

                if (translationMagnitude > DEADZONE) {
                    // Continuous scaling: zero out power right at the deadzone edge
                    double normalizedMagnitude = (translationMagnitude - DEADZONE) / (1.0 - DEADZONE);

                    // Apply cubic curve for micro-adjustments near center + scale by overall limit
                    double scaledPower = Math.pow(normalizedMagnitude, 3) * DRIVE_SPEED_LIMIT;

                    // Maintain directional angle
                    y = (rawY / translationMagnitude) * scaledPower;
                    x = (rawX / translationMagnitude) * scaledPower;
                }

                // 3. Rotation control with Heading Lock
                double rx = 0.0;
                double absRx = Math.abs(rawRx);
                double dt = timer.seconds();
                timer.reset();

                if (absRx > DEADZONE) {
                    // CASE 1: Driver IS turning manually (Right Stick)
                    double normalizedRx = (absRx - DEADZONE) / (1.0 - DEADZONE);
                    rx = Math.signum(rawRx) * Math.pow(normalizedRx, 3) * DRIVE_SPEED_LIMIT;

                    targetHeading = currentHeading;
                    lastError = 0.0;

                } else {
                    // CASE 2: Lock target heading using PD Controller
                    double headingError = targetHeading - currentHeading;

                    // Normalize error to stay within [-pi, pi] radians (angle wrapping)
                    headingError = Math.atan2(Math.sin(headingError), Math.cos(headingError));

                    if (Math.abs(headingError) < Math.toRadians(0.5)) {
                        rx = 0.0;
                    } else {
                        double derivative = (dt > 0) ? (headingError - lastError) / dt : 0.0;
                        rx = (headingError * headingLock_kP) + (derivative * headingLock_kD);
                        rx = Math.max(-DRIVE_SPEED_LIMIT, Math.min(DRIVE_SPEED_LIMIT, rx));
                    }

                    lastError = headingError;
                }

                // Output to Pedro Pathing (false = field-centric mode)
                follower.setTeleOpDrive(y, x, rx, false);

                // ------------------------------------
                // SUBSYSTEM CONTROL
                // ------------------------------------
                intakeMotor.setPower(gamepad1.a ? 0.80 : 0.0);

                if (gamepad1.dpad_up) {
                    slide.extendToHigh();
                } else if (gamepad1.dpad_down) {
                    slide.extendToBottom();
                }

                if (gamepad1.dpad_right) {
                    if (((int) (servoWiggleTimer.seconds() * 4)) % 2 == 0) {
                        myServo.setPosition(MAX_POSITION);
                    } else {
                        myServo.setPosition(HOME_POSITION);
                    }
                } else if (gamepad1.dpad_left) {
                    myServo.setPosition(HOME_POSITION);
                }

                // Telemetry Output
                telemetry.addData("X Position", follower.getPose().getX());
                telemetry.addData("Y Position", follower.getPose().getY());
                telemetry.addData("Heading (Deg)", Math.toDegrees(currentHeading));
                telemetry.addData("Target Heading (Deg)", Math.toDegrees(targetHeading));
                telemetry.addData("Slide Position", slide.getCurrentPosition());
                telemetry.update();
            }
        }
        visionPortal.close();
    }
}