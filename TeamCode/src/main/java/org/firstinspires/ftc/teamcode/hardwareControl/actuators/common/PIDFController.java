package org.firstinspires.ftc.teamcode.hardwareControl.actuators.common;

import com.qualcomm.robotcore.util.Range;

public class PIDFController {
    private double kP, kI, kD, kF; // PIDF coefficients
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private double maxIntegralSum = Double.MAX_VALUE; // Anti-windup limit
    private double maxOutput = 1.0; // Typical max power for FTC motors
    private double minOutput = -1.0; // Typical min power for FTC motors
    private long lastUpdateTime = 0;

    /**
     * Constructor for PIDFController
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     * @param kF Feedforward gain
     */
    public PIDFController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    /**
     * Sets the output limits for the controller
     * @param min Minimum output (e.g., -1.0 for motor power)
     * @param max Maximum output (e.g., 1.0 for motor power)
     */
    public void setOutputLimits(double min, double max) {
        this.minOutput = min;
        this.maxOutput = max;
    }

    /**
     * Sets the maximum integral sum to prevent windup
     * @param maxIntegralSum Maximum allowed integral sum
     */
    public void setMaxIntegralSum(double maxIntegralSum) {
        this.maxIntegralSum = maxIntegralSum;
    }

    /**
     * Calculates the control output based on setpoint and current position
     * @param setpoint Desired target (e.g., encoder ticks or velocity)
     * @param current Current position or state (e.g., encoder ticks or velocity)
     * @return Control output (e.g., motor power between -1.0 and 1.0)
     */
    public double calculate(double setpoint, double current) {
        long currentTime = System.nanoTime();
        double deltaTime = (lastUpdateTime == 0) ? 0 : (currentTime - lastUpdateTime) / 1_000_000_000.0; // Seconds
        lastUpdateTime = currentTime;

        // Calculate error
        double error = setpoint - current;

        // Proportional term
        double pTerm = kP * error;

        // Integral term (only accumulate if deltaTime is valid)
        if (deltaTime > 0) {
            integralSum += error * deltaTime;
            // Anti-windup: limit integral sum
            integralSum = Range.clip(integralSum, -maxIntegralSum, maxIntegralSum);

        }
        double iTerm = kI * integralSum;

        // Derivative term
        double dTerm = (deltaTime > 0) ? kD * (error - lastError) / deltaTime : 0;
        lastError = error;

        // Feedforward term
        double fTerm = kF * Math.signum(error);

        // Calculate total output
        double output = pTerm + iTerm + dTerm + fTerm;

        // Clamp output to motor power limits
        return Math.max(Math.min(output, maxOutput), minOutput);
    }

    /**
     * Resets the controller's integral and last error
     */
    public void reset() {
        integralSum = 0.0;
        lastError = 0.0;
        lastUpdateTime = 0;
    }

    /**
     * Getters and setters for PIDF coefficients
     */
    public void setPIDF(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public double getKP() { return kP; }
    public double getKI() { return kI; }
    public double getKD() { return kD; }
    public double getKF() { return kF; }
}