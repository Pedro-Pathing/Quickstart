package org.firstinspires.ftc.teamcode.pedroPathing.theLab;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.TeleOpConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.slideConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.subSystems.AprilTagAligner;
import org.firstinspires.ftc.teamcode.pedroPathing.subSystems.HeadingLockHandler;
import org.firstinspires.ftc.teamcode.pedroPathing.subSystems.ServoAnimationHandler;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * AprilTagAlignPedroTeleOp - Structured version using shared helper components.
 */
@TeleOp(name = "AprilTag Align Pedro TeleOp", group = "TeleOp")
public class AprilTagAlignPedroTeleOp extends LinearOpMode {

    private Follower follower;
    private slideConstants slide;
    private DcMotor intakeMotor;
    
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private AprilTagAligner aligner;
    private HeadingLockHandler headingLock;
    private ServoAnimationHandler servoAnim;

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
                .addProcessor(aprilTag)
                .build();

        aligner = new AprilTagAligner(aprilTag, follower);
        headingLock = new HeadingLockHandler(follower);
        servoAnim = new ServoAnimationHandler(hardwareMap.get(Servo.class, "myServoName"));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        follower.startTeleopDrive();
        slide.start();
        headingLock.resetHeading(follower.getPose().getHeading());

        while (opModeIsActive()) {
            follower.update();

            // Field Centric Reset
            boolean currentOptionsState = gamepad1.options || gamepad1.start;
            if (currentOptionsState && !lastOptionsState) {
                follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), 0.0));
                headingLock.resetHeading(0.0);
            }
            lastOptionsState = currentOptionsState;

            // Drive & Align
            if (gamepad1.left_trigger > 0.1) {
                if (!aligner.isAligning()) aligner.startAlignment();
                aligner.update();
                headingLock.resetHeading(follower.getPose().getHeading());
            } else {
                aligner.stopAlignment();
                handleManualDrive();
            }

            handleMechanisms();

            telemetry.addData("X / Y", "%.1f, %.1f", follower.getPose().getX(), follower.getPose().getY());
            telemetry.addData("Heading", "%.1f deg", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("Mode", aligner.isAligning() ? "AUTO-ALIGN" : "MANUAL");
            telemetry.update();
        }
        visionPortal.close();
    }

    private void handleManualDrive() {
        double rawY = -gamepad1.left_stick_y;
        double rawX = -gamepad1.left_stick_x;
        double magnitude = Math.hypot(rawX, rawY);

        double y = 0.0, x = 0.0;
        if (magnitude > TeleOpConstants.DEADZONE) {
            double scalar = Math.pow((magnitude - TeleOpConstants.DEADZONE) / (1.0 - TeleOpConstants.DEADZONE), 3) * TeleOpConstants.DRIVE_SPEED_LIMIT;
            y = (rawY / magnitude) * scalar;
            x = (rawX / magnitude) * scalar;
        }

        double rx = headingLock.calculateRotationPower(-gamepad1.right_stick_x, magnitude);
        follower.setTeleOpDrive(y, x, rx, false);
    }

    private void handleMechanisms() {
        if (intakeMotor != null) intakeMotor.setPower(gamepad1.a ? 0.80 : 0.0);

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
}
