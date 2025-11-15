package org.firstinspires.ftc.teamcode.experimental;
import org.firstinspires.ftc.teamcode.Robot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Standard Robot TeleOp with Smooth Limelight turret tracking, now with
 * "coast to last-seen" logic.
 */
@TeleOp(name = "RobotTeleop_Refactored_Coast", group = "Examples")
public class RobotTeleopRefactored extends OpMode {

    // --- Subsystems ---
    private Follower follower;
    private LimelightTracker limelightTracker;
    private TurretPIDController turretController;

    // --- Constants ---
    private static final double DEAD_ZONE = 0.1;
    private final Pose startPose = new Pose(0, 0, 0);


    // --- State Variable ---
    private boolean autoAimEnabled = true;
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);

        // -------------------- 1. Initialize Follower/Drive --------------------
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // -------------------- 2. Initialize Mechanisms --------------------
        limelightTracker = new LimelightTracker(robot);
        turretController = new TurretPIDController(robot);

        telemetry.addLine("RobotTeleop Refactored Initialized with COASTING");
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
        // Toggle Auto Aim using a button (e.g., gamepad2.y for ON, gamepad2.x for OFF)
        if (gamepad2.y && !autoAimEnabled) {
            autoAimEnabled = true;
        } else if (gamepad2.x && autoAimEnabled) {
            autoAimEnabled = false;
        }

        // -------------------- 3. TURRET CONTROL --------------------
        double turretPower = 0.0;
        double lastValidTx = turretController.getLastValidTx();

        // Condition to engage Auto Aim: Must be enabled AND (Target is visible OR we are currently coasting)
        boolean shouldAutoAim = autoAimEnabled && (trackingTag || lastValidTx != 0.0);

        if (shouldAutoAim) {
            // Use the subsystem's update method, passing both the raw error and visibility
            turretPower = turretController.updateAutoAim(tx, trackingTag);

            telemetry.addData("Turret Mode", "AUTO (Tag " + Robot.current_tag_id + ")");

            if (turretController.isAimed()) {
                telemetry.addData("Turret Status", "🎯 LOCKED ON TARGET");
            } else if (trackingTag) {
                telemetry.addData("Turret Status", "🔄 TRACKING");
            } else {
                telemetry.addData("Turret Status", "👻 COASTING BACK");
            }

        } else if (autoAimEnabled && !shouldAutoAim) {
            // Auto mode enabled, but target lost AND turret is centered/stopped
            turretController.stop();
            telemetry.addData("Turret Mode", "AUTO (Search Stopped)");
            telemetry.addData("Turret Status", "⚠️ NO TARGET");

        } else {
            // Manual control
            double manualInput = (gamepad1.dpad_right ? 1.0 : 0.0) + (gamepad1.dpad_left ? -1.0 : 0.0);
            turretController.updateManual(manualInput);
            turretPower = turretController.getLastTurretPower();
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



        // -------------------- 6. TELEMETRY --------------------
        telemetry.addData("--- Limelight/Turret Data ---", "---");
        telemetry.addData("Target Found (TV)", trackingTag);
        telemetry.addData("Raw Error (tx)", "%.2f°", tx);
        telemetry.addData("Last Valid Tx", "%.2f°", lastValidTx);
        telemetry.addData("Filtered Error", "%.2f°", turretController.getFilteredTx());
        telemetry.addData("Turret Power", "%.3f", turretPower);
        telemetry.addData("Auto Aim Enabled (X/Y)", autoAimEnabled);

        telemetry.addData("--- Drive/Mechanism ---", "---");
        telemetry.addData("Pose X", "%.2f", follower.getPose().getX());
        telemetry.addData("Pose Y", "%.2f", follower.getPose().getY());
        telemetry.update();
    }

    @Override
    public void stop() {
        turretController.stop();
        limelightTracker.stop();
    }
}
