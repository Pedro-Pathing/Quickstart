package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import org.firstinspires.ftc.teamcode.TurretPIDController;
import org.firstinspires.ftc.teamcode.LimelightTracker;

/**
 * Standard Robot TeleOp with Smooth Limelight turret tracking
 * Logic moved to TurretPIDController and LimelightTracker subsystems.
 */
@TeleOp(name = "RobotTeleop_Refactored_Smooth", group = "Examples")
public class RobotTeleopRefactored extends OpMode {

    // --- Subsystems ---
    private Follower follower;
    private LimelightTracker limelightTracker;
    private TurretPIDController turretController;

    // --- Drivetrain/Mechanism Motors ---
    private DcMotorEx shooterMotor;
    private DcMotorEx intakeMotor;
    private DcMotorEx transferMotor;

    // --- Constants (Moved from original TeleOp) ---
    private static final double DEAD_ZONE = 0.1;
    private static final int TARGET_TAG_ID = 24;
    private static final int PIPELINE_ID = 2;
    private final Pose startPose = new Pose(0, 0, 0);

    // --- Turret PID Constants (Tuned Values) ---
    private static final double TURRET_MANUAL_POWER = 0.45;
    private static final double TURRET_KP = 0.045;
    private static final double TURRET_KI = 0.002;
    private static final double TURRET_KD = 0.015;
    private static final double TURRET_DEADZONE = 0.3;
    private static final double TURRET_MAX_POWER = 0.7;
    private static final double TURRET_MIN_POWER = 0.05;
    private static final double TURRET_MAX_ACCELERATION = 1.5; // Max power change per second
    private static final double FILTER_ALPHA = 0.7;
    private static final double INTEGRAL_LIMIT = 0.3;

    // --- State Variable ---
    private boolean autoAimEnabled = true; // Start in auto-aim mode

    @Override
    public void init() {
        // -------------------- 1. Initialize Follower/Drive --------------------
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // -------------------- 2. Initialize Mechanisms --------------------
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        transferMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // -------------------- 3. Initialize Subsystems --------------------
        limelightTracker = new LimelightTracker(hardwareMap, "limelight", PIPELINE_ID, TARGET_TAG_ID);
        turretController = new TurretPIDController(
                hardwareMap, "turretServo",
                TURRET_KP, TURRET_KI, TURRET_KD,
                TURRET_MANUAL_POWER, TURRET_DEADZONE, TURRET_MAX_POWER,
                TURRET_MIN_POWER, TURRET_MAX_ACCELERATION, FILTER_ALPHA,
                INTEGRAL_LIMIT);

        telemetry.addLine("RobotTeleop Refactored Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // -------------------- 1. READ SENSORS (Limelight) --------------------
        limelightTracker.update();
        boolean trackingTag = limelightTracker.isTargetFound();
        double tx = limelightTracker.getTx();

        // -------------------- 2. DRIVER INPUT & Toggles --------------------
        // Example: Toggle Auto Aim using a button (e.g., gamepad2.y)
        if (gamepad2.y && !autoAimEnabled) {
            autoAimEnabled = true;
        } else if (gamepad2.x && autoAimEnabled) {
            autoAimEnabled = false;
        }

        // -------------------- 3. TURRET CONTROL (Clean Subsystem Call) --------------------
        double turretPower = 0.0;
        if (autoAimEnabled && trackingTag) {
            turretPower = turretController.updateAutoAim(tx);
            telemetry.addData("Turret Mode", "AUTO (Tag " + TARGET_TAG_ID + ")");
            telemetry.addData("Turret Status", turretController.isAimed() ? "🎯 LOCKED ON TARGET" : "🔄 TRACKING");

        } else if (autoAimEnabled && !trackingTag) {
            turretController.stop(); // Stop if in auto mode but lost target
            telemetry.addData("Turret Mode", "AUTO (Searching)");
            telemetry.addData("Turret Status", "⚠️ NO TARGET");
        } else {
            // Manual control override (using gamepad1 d-pad for example)
            double manualInput = (gamepad1.dpad_right ? 1.0 : 0.0) + (gamepad1.dpad_left ? -1.0 : 0.0);
            turretController.updateManual(manualInput);
            turretPower = turretController.getLastTurretPower(); // Get the actual power set
            telemetry.addData("Turret Mode", "MANUAL");
        }


        // -------------------- 4. DRIVE --------------------
        double xInput = Math.abs(gamepad1.left_stick_x) > DEAD_ZONE ? gamepad1.left_stick_x : 0;
        double yInput = Math.abs(gamepad1.left_stick_y) > DEAD_ZONE ? -gamepad1.left_stick_y : 0;
        double turnInput = Math.abs(gamepad1.right_stick_x) > DEAD_ZONE ? gamepad1.right_stick_x : 0;
        double powerScale = gamepad1.right_trigger > 0.5 ? 0.25 : 1.0;

        follower.updateErrors();
        follower.updateVectors();
        follower.setTeleOpDrive(yInput * powerScale, xInput * powerScale, turnInput * powerScale, true);
        follower.update();

        // -------------------- 5. SHOOTER & INTAKE --------------------
        // Shooter control logic remains here
        if (gamepad1.a) {
            shooterMotor.setVelocity(1420);
        } else if (gamepad1.b) {
            shooterMotor.setVelocity(0);
        } else if (gamepad1.x) {
            shooterMotor.setVelocity(1000);
        } else if (gamepad1.y) {
            shooterMotor.setVelocity(1200);
        }

        intakeMotor.setPower(gamepad1.right_bumper ? 0.5 : 0.0);
        transferMotor.setPower(gamepad1.left_bumper ? -0.75 : 0.0);


        // -------------------- 6. TELEMETRY --------------------
        telemetry.addData("--- Limelight/Turret Data ---", "---");
        telemetry.addData("Target Found (TV)", trackingTag);
        if (trackingTag || autoAimEnabled) {
            telemetry.addData("Raw Error (tx)", "%.2f°", tx);
            telemetry.addData("Filtered Error", "%.2f°", turretController.getFilteredTx());
            telemetry.addData("Turret Power", "%.3f", turretPower);
        }
        telemetry.addData("Detected Tag ID", limelightTracker.getDetectedTagID());
        telemetry.addData("Auto Aim Enabled (X/Y)", autoAimEnabled);

        telemetry.addData("--- Drive/Mechanism ---", "---");
        telemetry.addData("Shooter RPM", "%.0f", shooterMotor.getVelocity());
        telemetry.addData("Pose X", "%.2f", follower.getPose().getX());
        telemetry.addData("Pose Y", "%.2f", follower.getPose().getY());
        telemetry.update();
    }

    @Override
    public void stop() {
        shooterMotor.setPower(0);
        intakeMotor.setPower(0);
        transferMotor.setPower(0);
        turretController.stop();
        limelightTracker.stop();
    }
}
