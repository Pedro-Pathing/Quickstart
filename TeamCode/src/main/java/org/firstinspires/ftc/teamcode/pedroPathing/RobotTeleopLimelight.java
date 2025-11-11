package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.List;

/**
 * Standard Robot TeleOp with Limelight turret tracking (pipeline 8, AprilTag ID 24)
 * Uses Limelight FTC library via HTTP JSON.
 */
@TeleOp(name = "RobotTeleop_Limelight", group = "Examples")
public class RobotTeleopLimelight extends OpMode {

    private Follower follower;
    private static final double DEAD_ZONE = 0.1;

    private DcMotorEx shooterMotor;
    private DcMotorEx intakeMotor;
    private DcMotorEx transferMotor;
    private CRServo turretCR;

    // Turret control constants
    private static final double TURRET_MANUAL_POWER = 0.45;
    private static final double TURRET_KP = 0.03;  // Proportional gain for auto-aim
    private static final double TURRET_DEADZONE = 1.0; // degrees

    // Limelight
    private Limelight3A limelight;
    private static final int TARGET_TAG_ID = 24;
    private static final int PIPELINE_ID = 2;

    private final Pose startPose = new Pose(0, 0, 0);

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        transferMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        turretCR = hardwareMap.get(CRServo.class, "turretServo");
        turretCR.setPower(0.0);

        // Limelight setup - get from hardware map
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(PIPELINE_ID);
        limelight.start();

        telemetry.addLine("RobotTeleop Initialized (Limelight + CRServo turret)");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // -------------------- DRIVE --------------------
        double xInput = Math.abs(gamepad1.left_stick_x) > DEAD_ZONE ? gamepad1.left_stick_x : 0;
        double yInput = Math.abs(gamepad1.left_stick_y) > DEAD_ZONE ? -gamepad1.left_stick_y : 0;
        double turnInput = Math.abs(gamepad1.right_stick_x) > DEAD_ZONE ? gamepad1.right_stick_x : 0;
        double powerScale = gamepad1.right_trigger > 0.5 ? 0.25 : 1.0;

        follower.updateErrors();
        follower.updateVectors();
        follower.setTeleOpDrive(yInput * powerScale, xInput * powerScale, turnInput * powerScale, true);
        follower.update();

        // -------------------- SHOOTER --------------------
        if (gamepad1.a) {
            shooterMotor.setVelocity(1420);
        } else if (gamepad1.b) {
            shooterMotor.setVelocity(0);
        } else if (gamepad1.x) {
            shooterMotor.setVelocity(1035);
        } else if (gamepad1.y) {
            shooterMotor.setVelocity(1200);
        }

        // -------------------- TURRET CONTROL --------------------
        LLResult result = limelight.getLatestResult();
        boolean trackingTag = false;
        double tx = 0.0;
        int detectedTagID = -1;

        if (result != null && result.isValid()) {
            // Get fiducial (AprilTag) results
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            if (fiducials != null && !fiducials.isEmpty()) {
                // Look for our target tag
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    if (fiducial.getFiducialId() == TARGET_TAG_ID) {
                        tx = fiducial.getTargetXDegrees(); // degrees, positive = target to right
                        detectedTagID = fiducial.getFiducialId();
                        trackingTag = true;
                        break;
                    }
                }

                // If we didn't find target tag, use the first detected tag for telemetry
                if (!trackingTag && !fiducials.isEmpty()) {
                    detectedTagID = fiducials.get(0).getFiducialId();
                }
            }
        }

        if (trackingTag) {
            double turretPower = TURRET_KP * tx;

            // Apply deadzone
            if (Math.abs(tx) < TURRET_DEADZONE) {
                turretCR.setPower(0);
            } else {
                turretCR.setPower(-turretPower); // sign may need to flip depending on wiring
            }

            telemetry.addData("Turret Mode", "AUTO (Tracking Tag 24)");
            telemetry.addData("tx", "%.2f", tx);
        } else {
            // Manual turret control if not seeing tag
            if (gamepad1.dpad_right && !gamepad1.dpad_left) {
                turretCR.setPower(TURRET_MANUAL_POWER);
            } else if (gamepad1.dpad_left && !gamepad1.dpad_right) {
                turretCR.setPower(-TURRET_MANUAL_POWER);
            } else {
                turretCR.setPower(0.0);
            }

            telemetry.addData("Turret Mode", "MANUAL / No Tag");
        }

        // -------------------- INTAKE + TRANSFER --------------------
        intakeMotor.setPower(gamepad1.right_bumper ? 0.5 : 0.0);
        transferMotor.setPower(gamepad1.left_bumper ? 0.5 : 0.0);

        // -------------------- TELEMETRY --------------------
        telemetry.addData("Shooter RPM", "%.0f", shooterMotor.getVelocity());
        telemetry.addData("Pose X", "%.2f", follower.getPose().getX());
        telemetry.addData("Pose Y", "%.2f", follower.getPose().getY());
        telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Detected Tag ID", detectedTagID);
        telemetry.update();
    }

    @Override
    public void stop() {
        shooterMotor.setPower(0);
        intakeMotor.setPower(0);
        transferMotor.setPower(0);
        turretCR.setPower(0.0);
        if (limelight != null) limelight.stop();
    }
}