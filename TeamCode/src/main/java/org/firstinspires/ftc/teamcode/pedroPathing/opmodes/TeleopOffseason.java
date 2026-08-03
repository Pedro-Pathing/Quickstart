package org.firstinspires.ftc.teamcode.pedroPathing.opmodes;

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
import org.firstinspires.ftc.teamcode.pedroPathing.subSystems.MechanismStateMachine;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/** Structured competition TeleOp OpMode ensuring full functional integrity with modular logic. */
@TeleOp(name = "TeleopOffseason")
public class TeleopOffseason extends LinearOpMode {

    private Follower follower;
    private slideConstants slide;
    private DcMotor intakeMotor;
    
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private AprilTagAligner aligner;
    private HeadingLockHandler headingLock;
    private MechanismStateMachine mechanisms;

    private boolean lastOptionsState = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // 1. Hardware & Pedro Pathing Initialization
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

        // 3. Modular Subsystem Components
        aligner = new AprilTagAligner(aprilTag, follower);
        headingLock = new HeadingLockHandler(follower);
        mechanisms = new MechanismStateMachine(slide, new ServoAnimationHandler(hardwareMap.get(Servo.class, "myServoName")));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized & Stream Active");
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
                follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), 0.0));
                headingLock.resetHeading(0.0);
            }
            lastOptionsState = currentOptionsState;

            // --- Driving & Alignment Selection ---
            if (gamepad1.left_trigger > 0.1) {
                if (!aligner.isAligning()) aligner.startAlignment();
                aligner.update();
                headingLock.resetHeading(follower.getPose().getHeading());
            } else {
                aligner.stopAlignment();
                handleManualDrive();
            }

            // --- Mechanism & Intake Logic ---
            intakeMotor.setPower(gamepad1.a ? 0.80 : 0.0);
            mechanisms.update(gamepad1);

            // --- Telemetry & Dashboard Map Rendering ---
            updateDashboardAndTelemetry();
        }
        visionPortal.close();
    }

    /** Translates joystick inputs into robot movement using the Settled Heading Lock logic. */
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

    /** Visualizes the robot and target state on the Dashboard field map with diagnostic data. */
    private void updateDashboardAndTelemetry() {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay();
        Pose p = follower.getPose();

        // 1. Render Map Graphics
        canvas.setStrokeWidth(1);
        canvas.setStroke("#00FF00").setFill("#00FF00").fillRect(DASHBOARD_TAG_X - 2, DASHBOARD_TAG_Y - 2, 4, 4); // Target
        canvas.setStroke("#0000FF").strokeCircle(p.getX(), p.getY(), 9); // Robot
        canvas.strokeLine(p.getX(), p.getY(), p.getX() + 9 * Math.cos(p.getHeading()), p.getY() + 9 * Math.sin(p.getHeading())); // Heading

        if (aligner.isAligning() && aligner.getLastTargetPose() != null) {
            Pose target = aligner.getLastTargetPose();
            canvas.setStroke("#FF0000").strokeLine(p.getX(), p.getY(), target.getX(), target.getY()); // Path
            canvas.strokeCircle(target.getX(), target.getY(), 2);
        }
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        // 2. Textual Diagnostic Data
        telemetry.addData("Pose", "X: %.1f, Y: %.1f, H: %.1f deg", p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
        telemetry.addData("Target Heading", "%.1f deg", Math.toDegrees(headingLock.getTargetHeading()));
        telemetry.addData("Slide", slide.getCurrentPosition());
        telemetry.addData("Mode", aligner.isAligning() ? "AUTO-ALIGN" : "MANUAL");

        com.pedropathing.math.Vector v = follower.getVelocity();
        double stopTime = (v.getMagnitude() > 0) ? v.getMagnitude() / 50.0 : 0;
        telemetry.addData("Est. Stop Dist", "%.1f in (%.2f s)", 0.5 * v.getMagnitude() * stopTime, stopTime);
        telemetry.update();
    }

    /** Configures camera controls for optimized frame processing. */
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
