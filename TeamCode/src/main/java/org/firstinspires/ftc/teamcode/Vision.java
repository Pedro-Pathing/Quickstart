package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;


import java.util.List;


public class Vision {
    private Robot robot;
    private Limelight3A limelight;
    private Turret turret;
    private long lastLoopTime = 0;

    private double lastError = 0.0;
    private double integralSum = 0.0;
    private double lastTx = 0.0;
    private double tx = 0.0;
    double dt = 0; // seconds

    private double filteredTx = 0.0;
    private double lastTurretPower = 0.0;
    private static final double INTEGRAL_LIMIT = 0.3; // Prevent integral windup
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

    public Vision (HardwareMap hardwareMap, Robot robot) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(Robot.current_pipeline_id);
        limelight.start();
        lastLoopTime = System.nanoTime();
        this.turret = robot.turret;
        this.robot = robot;
    }

    public void update() {
        // Calculate loop time for derivative
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastLoopTime) / 1e9; // seconds
        lastLoopTime = currentTime;
        dt = Math.max(dt, 0.001); // Prevent division by zero
        // -------------------- TURRET CONTROL --------------------
        LLResult result = limelight.getLatestResult();
        boolean trackingTag = false;
        double tx = 0.0;
        int detectedTagID = -1;

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            if (fiducials != null && !fiducials.isEmpty()) {
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    if (fiducial.getFiducialId() == Robot.current_tag_id) {
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
                turret.setTurretPower(0);
                integralSum = 0; // Reset integral when locked
                //telemetry.addData("Turret Status", "🎯 LOCKED ON TARGET");
            } else {
                turret.setTurretPower(turretPower);
                //telemetry.addData("Turret Status", "🔄 TRACKING");
            }

            // Update state for next loop
            lastError = error;
            lastTurretPower = turretPower;
            lastTx = tx;

//            telemetry.addData("Turret Mode", "AUTO (Tag 24)");
//            telemetry.addData("Raw Error", "%.2f°", tx);
//            telemetry.addData("Filtered Error", "%.2f°", filteredTx);
//            telemetry.addData("P | I | D", "%.3f | %.3f | %.3f", pTerm, iTerm, dTerm);
//            telemetry.addData("Turret Power", "%.3f", turretPower);
        } else {
        // Reset PID when target lost
        integralSum = 0;
        lastError = 0;
        filteredTx = 0;
     }
    }

}
