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
import org.firstinspires.ftc.teamcode.pedroPathing.subSystems.MechanismStateMachine;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/** Optimized TeleOp OpMode using a refactored Finite State Machine (FSM) architecture. */
@TeleOp(name = "Corrupted")
public class Corrupted extends LinearOpMode {

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

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        follower = Constants.createFollower(hardwareMap);
        slide = new slideConstants(hardwareMap);
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new android.util.Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .build();

        FtcDashboard.getInstance().startCameraStream(visionPortal, 15);
        setupCameraControls();

        aligner = new AprilTagAligner(aprilTag, follower);
        headingLock = new HeadingLockHandler(follower);
        mechanisms = new MechanismStateMachine(slide, new ServoAnimationHandler(hardwareMap.get(Servo.class, "myServoName")));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        follower.startTeleopDrive();
        slide.start();
        headingLock.resetHeading(follower.getPose().getHeading());

        while (opModeIsActive()) {
            follower.update();

            boolean currentOptionsState = gamepad1.options || gamepad1.start;
            if (currentOptionsState && !lastOptionsState) {
                follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), 0.0));
                headingLock.resetHeading(0.0);
            }
            lastOptionsState = currentOptionsState;

            if (gamepad1.left_trigger > 0.1) {
                if (!aligner.isAligning()) aligner.startAlignment();
                aligner.update();
                headingLock.resetHeading(follower.getPose().getHeading());
            } else {
                aligner.stopAlignment();
                handleManualDrive();
            }

            intakeMotor.setPower(gamepad1.a ? 0.80 : 0.0);
            mechanisms.update(gamepad1);

            updateDashboardAndTelemetry();
        }
        visionPortal.close();
    }

    /** Translates joystick inputs into robot movement with automated heading correction. */
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

    /** Visualizes the robot and target state on the FTC Dashboard map. */
    private void updateDashboardAndTelemetry() {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay();
        Pose p = follower.getPose();

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

        telemetry.addData("Pose", "%.1f, %.1f, %.1f deg", p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
        telemetry.addData("Slide", slide.getCurrentPosition());
        telemetry.update();
    }

    /** Forces high-performance camera settings for low-latency tag detection. */
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
