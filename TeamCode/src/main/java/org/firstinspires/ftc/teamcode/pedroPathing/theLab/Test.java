package org.firstinspires.ftc.teamcode.pedroPathing.theLab;

import static org.firstinspires.ftc.teamcode.pedroPathing.constants.TeleOpConstants.*;

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
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.slideConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.subSystems.AprilTagAligner;
import org.firstinspires.ftc.teamcode.pedroPathing.subSystems.HeadingLockHandler;
import org.firstinspires.ftc.teamcode.pedroPathing.subSystems.ServoAnimationHandler;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * test - Optimized and structured TeleOp OpMode.
 * Uses shared components for Heading Lock, AprilTag Alignment, and Servo Animations.
 */
@TeleOp(name = "Test")
public class Test extends LinearOpMode {

    private Follower follower;
    private slideConstants slide;
    private DcMotor intakeMotor;
    
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // --- Helper Components ---
    private AprilTagAligner aligner;
    private HeadingLockHandler headingLock;
    private ServoAnimationHandler servoAnim;

    private boolean lastOptionsState = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // 1. Hardware Initialization
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        follower = Constants.createFollower(hardwareMap);
        slide = new slideConstants(hardwareMap);
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 2. Vision Initialization
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new android.util.Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .build();

        FtcDashboard.getInstance().startCameraStream(visionPortal, 15);
        setupCameraControls();

        // 3. Component Initialization
        aligner = new AprilTagAligner(aprilTag, follower);
        headingLock = new HeadingLockHandler(follower);
        servoAnim = new ServoAnimationHandler(hardwareMap.get(Servo.class, "myServoName"));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        follower.startTeleopDrive();
        slide.start();
        headingLock.resetHeading(follower.getPose().getHeading());

        while (opModeIsActive()) {
            follower.update();

            // --- Reset Heading (Field Centric Sync) ---
            boolean currentOptionsState = gamepad1.options || gamepad1.start;
            if (currentOptionsState && !lastOptionsState) {
                Pose cp = follower.getPose();
                follower.setPose(new Pose(cp.getX(), cp.getY(), 0.0));
                headingLock.resetHeading(0.0);
            }
            lastOptionsState = currentOptionsState;

            // --- Alignment & Drive Logic ---
            if (gamepad1.left_trigger > 0.1) {
                if (!aligner.isAligning()) aligner.startAlignment();
                aligner.update();
                headingLock.resetHeading(follower.getPose().getHeading());
            } else {
                aligner.stopAlignment();
                handleManualDrive();
            }

            // --- Mechanism Logic ---
            handleMechanisms();

            // --- Feedback & Visualization ---
            updateDashboardAndTelemetry();
        }
        visionPortal.close();
    }

    private void handleManualDrive() {
        double rawY = -gamepad1.left_stick_y;
        double rawX = -gamepad1.left_stick_x;
        double magnitude = Math.hypot(rawX, rawY);

        double y = 0.0, x = 0.0;
        if (magnitude > DEADZONE) {
            double scalar = Math.pow((magnitude - DEADZONE) / (1.0 - DEADZONE), 3) * DRIVE_SPEED_LIMIT;
            y = (rawY / magnitude) * scalar;
            x = (rawX / magnitude) * scalar;
        }

        double rx = headingLock.calculateRotationPower(-gamepad1.right_stick_x, magnitude);
        follower.setTeleOpDrive(y, x, rx, false);
    }

    private void handleMechanisms() {
        intakeMotor.setPower(gamepad1.a ? 0.80 : 0.0);

        if (gamepad1.dpad_up) {
            slide.extendToHigh();
            servoAnim.startAnimation();
        } else if (gamepad1.right_bumper) {
            slide.extendToMiddle();
        } else if (gamepad1.dpad_down) {
            slide.extendToBottom();
        }

        if (gamepad1.back) slide.resetEncoder();

        servoAnim.update(gamepad1.dpad_right, gamepad1.dpad_left);
    }

    private void updateDashboardAndTelemetry() {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay();
        Pose p = follower.getPose();

        // Draw Map
        canvas.setStrokeWidth(1);
        canvas.setStroke("#00FF00").setFill("#00FF00").fillRect(DASHBOARD_TAG_X - 2, DASHBOARD_TAG_Y - 2, 4, 4);
        canvas.setStroke("#0000FF").strokeCircle(p.getX(), p.getY(), 9);
        canvas.strokeLine(p.getX(), p.getY(), p.getX() + 9 * Math.cos(p.getHeading()), p.getY() + 9 * Math.sin(p.getHeading()));

        if (aligner.isAligning() && aligner.getLastTargetPose() != null) {
            Pose target = aligner.getLastTargetPose();
            canvas.setStroke("#FF0000").strokeLine(p.getX(), p.getY(), target.getX(), target.getY());
            canvas.strokeCircle(target.getX(), target.getY(), 2);
        }

        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        telemetry.addData("X / Y", "%.1f, %.1f", p.getX(), p.getY());
        telemetry.addData("Heading", "%.1f deg", Math.toDegrees(p.getHeading()));
        telemetry.addData("Slide", slide.getCurrentPosition());
        
        com.pedropathing.math.Vector v = follower.getVelocity();
        double stopDist = 0.5 * v.getMagnitude() * (v.getMagnitude() / 50.0);
        telemetry.addData("Est. Stop Dist", "%.1f in", stopDist);
        telemetry.update();
    }

    private void setupCameraControls() {
        while (!isStopRequested() && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) sleep(20);
        if (isStopRequested()) return;

        ExposureControl ec = visionPortal.getCameraControl(ExposureControl.class);
        if (ec != null && ec.isModeSupported(ExposureControl.Mode.Manual)) {
            ec.setMode(ExposureControl.Mode.Manual);
            ec.setExposure(15, TimeUnit.MILLISECONDS);
        }
        GainControl gc = visionPortal.getCameraControl(GainControl.class);
        if (gc != null) gc.setGain(200);
    }
}
