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
 * Standard Robot TeleOp with Smooth Limelight turret tracking
 * Features: PID control, velocity limiting, low-pass filtering
 */
@TeleOp(name = "RobotTeleop_Limelight_Smooth", group = "Examples")
public class RobotTeleopLimelight extends OpMode {

    private Follower follower;
    private static final double DEAD_ZONE = 0.1;

    private DcMotorEx shooterMotor;
    private DcMotorEx intakeMotor;
    private DcMotorEx transferMotor;
    private CRServo turretCR;

    // Turret PID constants - TUNED for smooth tracking
    private static final double TURRET_MANUAL_POWER = 0.45;
    private static final double TURRET_KP = 0.045;  // Proportional - reduced for less aggression
    private static final double TURRET_KI = 0.002;  // Integral - for steady-state accuracy
    private static final double TURRET_KD = 0.015;  // Derivative - dampens oscillation
    private static final double TURRET_DEADZONE = 0.3; // Tighter alignment threshold
    private static final double TURRET_MAX_POWER = 0.7; // Increased max for fast response
    private static final double TURRET_MIN_POWER = 0.05; // Minimum power to overcome friction

    // Velocity limiting - prevents servo from quitting on fast turns
    private static final double TURRET_MAX_ACCELERATION = 1.5; // Max power change per loop

    // Low-pass filter for smoothing
    private static final double FILTER_ALPHA = 0.7; // 0=all history, 1=no filtering

    // PID state variables
    private double lastError = 0.0;
    private double integralSum = 0.0;
    private double lastTx = 0.0;
    private double filteredTx = 0.0;
    private double lastTurretPower = 0.0;
    private long lastLoopTime = 0;
    private static final double INTEGRAL_LIMIT = 0.3; // Prevent integral windup

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

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(PIPELINE_ID);
        limelight.start();

        lastLoopTime = System.nanoTime();

        telemetry.addLine("RobotTeleop Initialized - SMOOTH TURRET v2.0");
        telemetry.addLine("Features: PID + Velocity Limiting + Filtering");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // Calculate loop time for derivative
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastLoopTime) / 1e9; // seconds
        lastLoopTime = currentTime;
        dt = Math.max(dt, 0.001); // Prevent division by zero

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
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            if (fiducials != null && !fiducials.isEmpty()) {
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    if (fiducial.getFiducialId() == TARGET_TAG_ID) {
                        tx = fiducial.getTargetXDegrees();
                        detectedTagID = fiducial.getFiducialId();
                        trackingTag = true;
                        break;
                    }
                }

                if (!trackingTag && !fiducials.isEmpty()) {
                    detectedTagID = fiducials.get(0).getFiducialId();
                }
            }
        }

        if (trackingTag) {
            // Low-pass filter to smooth noisy measurements
            filteredTx = FILTER_ALPHA * tx + (1 - FILTER_ALPHA) * filteredTx;

            // PID calculation
            double error = filteredTx;

            // Proportional term
            double pTerm = TURRET_KP * error;

            // Integral term with anti-windup
            integralSum += error * dt;
            integralSum = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, integralSum));
            double iTerm = TURRET_KI * integralSum;

            // Derivative term (rate of change of error)
            double derivative = (error - lastError) / dt;
            double dTerm = TURRET_KD * derivative;

            // Combine PID terms
            double turretPower = pTerm + iTerm + dTerm;

            // Apply minimum power threshold to overcome friction
            if (Math.abs(turretPower) > 0.01 && Math.abs(turretPower) < TURRET_MIN_POWER) {
                turretPower = Math.signum(turretPower) * TURRET_MIN_POWER;
            }

            // Clamp to maximum power
            turretPower = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, turretPower));

            // Velocity limiting - prevent sudden power changes
            double powerChange = turretPower - lastTurretPower;
            if (Math.abs(powerChange) > TURRET_MAX_ACCELERATION * dt) {
                turretPower = lastTurretPower + Math.signum(powerChange) * TURRET_MAX_ACCELERATION * dt;
            }

            // Apply deadzone for lock-on
            if (Math.abs(error) < TURRET_DEADZONE) {
                turretCR.setPower(0);
                integralSum = 0; // Reset integral when locked
                telemetry.addData("Turret Status", "🎯 LOCKED ON TARGET");
            } else {
                turretCR.setPower(turretPower);
                telemetry.addData("Turret Status", "🔄 TRACKING");
            }

            // Update state for next loop
            lastError = error;
            lastTurretPower = turretPower;
            lastTx = tx;

            telemetry.addData("Turret Mode", "AUTO (Tag 24)");
            telemetry.addData("Raw Error", "%.2f°", tx);
            telemetry.addData("Filtered Error", "%.2f°", filteredTx);
            telemetry.addData("P | I | D", "%.3f | %.3f | %.3f", pTerm, iTerm, dTerm);
            telemetry.addData("Turret Power", "%.3f", turretPower);

        } else {
            // Reset PID when target lost
            integralSum = 0;
            lastError = 0;
            filteredTx = 0;

            // Manual turret control
            if (gamepad1.dpad_right && !gamepad1.dpad_left) {
                lastTurretPower = TURRET_MANUAL_POWER;
                turretCR.setPower(TURRET_MANUAL_POWER);
            } else if (gamepad1.dpad_left && !gamepad1.dpad_right) {
                lastTurretPower = -TURRET_MANUAL_POWER;
                turretCR.setPower(-TURRET_MANUAL_POWER);
            } else {
                lastTurretPower = 0.0;
                turretCR.setPower(0.0);
            }

            telemetry.addData("Turret Mode", "⚠️ MANUAL / No Tag");
        }

        // -------------------- INTAKE + TRANSFER --------------------
        intakeMotor.setPower(gamepad1.right_bumper ? 0.5 : 0.0);
        transferMotor.setPower(gamepad1.left_bumper ? 0.5 : 0.0);

        // -------------------- TELEMETRY --------------------
        telemetry.addData("Shooter RPM", "%.0f", shooterMotor.getVelocity());
        telemetry.addData("Pose X", "%.2f", follower.getPose().getX());
        telemetry.addData("Pose Y", "%.2f", follower.getPose().getY());
        telemetry.addData("Heading", "%.2f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Detected Tag", detectedTagID);
        telemetry.addData("Loop Time", "%.1f ms", dt * 1000);
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