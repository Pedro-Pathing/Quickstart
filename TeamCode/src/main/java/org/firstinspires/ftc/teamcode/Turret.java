package org.firstinspires.ftc.teamcode;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    public CRServo turretCR;
    private static final double TURRET_POWER = 0.45;

    private static final double TURRET_STOP = 0.0;

    // Turret Control Constants (Copied from your code)
    private static final double TURRET_KP = 0.045;
    private static final double TURRET_KI = 0.002;
    private static final double TURRET_KD = 0.015;
    private static final double TURRET_DEADZONE = 0.3;
    private static final double TURRET_MAX_POWER = 0.7;
    private static final double TURRET_MIN_POWER = 0.05;
    private static final double TURRET_MAX_ACCELERATION = 1.5;
    private static final double FILTER_ALPHA = 0.7;
    private static final double INTEGRAL_LIMIT = 0.3;

    // PID State Variables
    private double lastError = 0.0;
    private double integralSum = 0.0;
    private double lastTurretPower = 0.0;
    private double filteredTx = 0.0;
    private double lastKnownTurretWorldAngle = 0.0; // The target's angle in field coordinates
    private Pose lastKnownRobotPose = new Pose(0, 0, 0); // Last pose when target was visible

    // Prediction/State Variables
    private boolean trackingLost = true;
    private double lastTx = 0.0;

    public Turret (HardwareMap hardwareMap) {
        turretCR = hardwareMap.get(CRServo.class, "turretServo");
        turretCR.setPower(0.0); // start stopped
    }

    public void goLeft() {
        turretCR.setPower(-TURRET_POWER); // rotate left
    }

    public void goRight() {
        turretCR.setPower(TURRET_POWER); // rotate right
    }

    public void stopTurret() {
        turretCR.setPower(TURRET_STOP); // stop turret
    }

    public void setTurretPower(double power) {
        turretCR.setPower(power);
    }


    public double calculatePower(double currentTx, boolean isTargetVisible, Pose currentRobotPose, double dt) {

        // 1. Update State and World Angle when target is visible
        if (isTargetVisible) {
            // Low-pass filter for smoothing vision data
            filteredTx = FILTER_ALPHA * currentTx + (1 - FILTER_ALPHA) * filteredTx;

            // Calculate and store the target's angle in FIELD coordinates (World Angle)
            // World Angle = Robot Heading + Turret Angle (filteredTx is the angle relative to robot)
            lastKnownTurretWorldAngle = currentRobotPose.getHeading() + Math.toRadians(filteredTx);
            lastKnownRobotPose = currentRobotPose;
            trackingLost = false;
        } else {
            // Target not visible
            trackingLost = true;
            // Decay the integral to prevent windup if target is lost for a while
            integralSum *= 0.95;
        }

        double error;

        // 2. Determine Error Source (Vision vs. Prediction)
        if (!trackingLost) {
            // A. VISION MODE: Use filtered vision error
            error = filteredTx;
        } else {
            // B. PREDICTION MODE (Outside FOV Tracking)
            // Predicted Error = World Target Angle - Current Robot Heading
            double predictedTurretAngle = lastKnownTurretWorldAngle - currentRobotPose.getHeading();

            // The error is the difference between the predicted turret angle (in degrees) and 0
            error = Math.toDegrees(predictedTurretAngle);
        }

        // 3. PID Calculation (Same as your original code, now using 'error')

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

        // Apply minimum power threshold
        if (Math.abs(turretPower) > 0.01 && Math.abs(turretPower) < TURRET_MIN_POWER) {
            turretPower = Math.signum(turretPower) * TURRET_MIN_POWER;
        }

        // Clamp to maximum power
        turretPower = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, turretPower));

        // Velocity limiting
        double powerChange = turretPower - lastTurretPower;
        if (Math.abs(powerChange) > TURRET_MAX_ACCELERATION * dt) {
            turretPower = lastTurretPower + Math.signum(powerChange) * TURRET_MAX_ACCELERATION * dt;
        }

        // Apply deadzone for lock-on (only stop if within the deadzone)
        if (Math.abs(error) < TURRET_DEADZONE) {
            turretPower = 0;
            // Reset integral when locked only in VISION mode, or let it accumulate slowly in PREDICTION
            if (!trackingLost) integralSum = 0;
        }

        // Update state for next loop
        lastError = error;
        lastTurretPower = turretPower;

        return turretPower;
    }

    /**
     * Calculates the required power for the turret motor based on a given error
     * (e.g., from Limelight or Odometry).
     * This method is a simplified version that relies solely on the provided error,
     * using the class's internal PID state (integralSum, lastError, etc.).
     *
     * @param error The angle error in degrees (input to the PID).
     * @param dt The time elapsed since the last loop in seconds.
     * @return The calculated motor power (-TURRET_MAX_POWER to TURRET_MAX_POWER).
     */
    public double calculateSimplePIDPower(double error, double dt) {
        // 1. PID Calculation

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

        // 2. Output Limiting and Constraints

        // Apply minimum power threshold to overcome friction
        if (Math.abs(turretPower) > 0.01 && Math.abs(turretPower) < TURRET_MIN_POWER) {
            turretPower = Math.signum(turretPower) * TURRET_MIN_POWER;
        }

        // Clamp to maximum power
        turretPower = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, turretPower));

        // Velocity limiting
        double powerChange = turretPower - lastTurretPower;
        if (Math.abs(powerChange) > TURRET_MAX_ACCELERATION * dt) {
            turretPower = lastTurretPower + Math.signum(powerChange) * TURRET_MAX_ACCELERATION * dt;
        }

        // Apply deadzone for lock-on
        if (Math.abs(error) < TURRET_DEADZONE) {
            turretPower = 0;
            integralSum = 0; // Reset integral when locked
        }

        // 3. Update State for next loop
        lastError = error;
        lastTurretPower = turretPower;

        return turretPower;
    }

    public boolean isTrackingLost() {
        return trackingLost;
    }

    public double getFilteredTx() {
        return filteredTx;
    }

    public double getLastError() {
        return lastError;
    }

}
