package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Manages the CR Servo turret using PID control, filtering, and velocity limiting.
 */
public class TurretPIDController {
    private final CRServo turretServo;

    // Turret PID constants
    private final double kP, kI, kD;
    private final double manualPower;
    private final double deadzone;
    private final double maxPower;
    private final double minPower;
    private final double maxAcceleration;
    private final double filterAlpha;
    private final double integralLimit;

    // PID state variables
    private double lastError = 0.0;
    private double integralSum = 0.0;
    private double filteredTx = 0.0;
    private double lastTurretPower = 0.0;
    private final ElapsedTime timer = new ElapsedTime();

    /**
     * @param hardwareMap The OpMode's hardware map.
     * @param servoName The name of the CR Servo.
     * @param kP Proportional gain.
     * @param kI Integral gain.
     * @param kD Derivative gain.
     * @param manualPower Power for manual control.
     * @param deadzone Error threshold for stopping.
     * @param maxPower Maximum power limit for the servo.
     * @param minPower Minimum power to overcome friction.
     * @param maxAcceleration Max power change per second.
     * @param filterAlpha Low-pass filter coefficient (0-1).
     * @param integralLimit Anti-windup limit for integral sum.
     */
    public TurretPIDController(
            HardwareMap hardwareMap, String servoName,
            double kP, double kI, double kD,
            double manualPower, double deadzone, double maxPower,
            double minPower, double maxAcceleration, double filterAlpha,
            double integralLimit) {

        turretServo = hardwareMap.get(CRServo.class, servoName);

        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.manualPower = manualPower;
        this.deadzone = deadzone;
        this.maxPower = maxPower;
        this.minPower = minPower;
        this.maxAcceleration = maxAcceleration;
        this.filterAlpha = filterAlpha;
        this.integralLimit = integralLimit;

        timer.reset(); // Start the timer for delta time calculation
    }

    /**
     * Executes the full PID control loop for automatic aiming.
     * @param rawTx The horizontal angle error (tx) from the Limelight.
     * @return The calculated servo power.
     */
    public double updateAutoAim(double rawTx) {
        // Calculate delta time (dt)
        double dt = timer.seconds();
        timer.reset();
        dt = Math.max(dt, 0.001); // Prevent division by zero

        // 1. Low-pass filter to smooth noisy measurements
        filteredTx = filterAlpha * rawTx + (1 - filterAlpha) * filteredTx;

        double error = filteredTx;

        // 2. PID calculation
        double pTerm = kP * error;

        // Integral term with anti-windup
        integralSum += error * dt;
        integralSum = Range.clip(integralSum, -integralLimit, integralLimit);
        double iTerm = kI * integralSum;

        // Derivative term (rate of change of error)
        double derivative = (error - lastError) / dt;
        double dTerm = kD * derivative;

        // Combine PID terms
        double turretPower = pTerm + iTerm + dTerm;

        // 3. Apply minimum power threshold to overcome friction
        if (Math.abs(turretPower) > 0.01 && Math.abs(turretPower) < minPower) {
            turretPower = Math.signum(turretPower) * minPower;
        }

        // 4. Clamp to maximum power
        turretPower = Range.clip(turretPower, -maxPower, maxPower);

        // 5. Velocity limiting - prevent sudden power changes
        double powerChange = turretPower - lastTurretPower;
        double maxChangeAllowed = maxAcceleration * dt;

        if (Math.abs(powerChange) > maxChangeAllowed) {
            turretPower = lastTurretPower + Math.signum(powerChange) * maxChangeAllowed;
        }

        // 6. Apply deadzone for lock-on
        if (Math.abs(error) < deadzone) {
            turretServo.setPower(0);
            integralSum = 0; // Reset integral when locked
            turretPower = 0.0;
        } else {
            turretServo.setPower(turretPower);
        }

        // 7. Update state for next loop
        lastError = error;
        lastTurretPower = turretPower;

        return turretPower;
    }

    /**
     * Runs the turret based on manual stick input.
     * @param input The raw input from the gamepad stick (e.g., gamepad.left_stick_x).
     */
    public void updateManual(double input) {
        // Stop PID logic and reset state
        resetPIDState();

        double power = input * manualPower;
        lastTurretPower = power;
        turretServo.setPower(power);
    }

    /**
     * Resets the PID state variables. Should be called when switching from auto to manual
     * or when the target is lost.
     */
    public void resetPIDState() {
        integralSum = 0.0;
        lastError = 0.0;
        filteredTx = 0.0;
        // lastTurretPower is kept to allow a smooth transition if needed,
        // but resetting it to 0 is also often fine.
    }

    public void stop() {
        turretServo.setPower(0.0);
        resetPIDState();
    }

    // --- Telemetry Getters ---
    public double getFilteredTx() { return filteredTx; }
    public double getLastError() { return lastError; }
    public double getIntegralSum() { return integralSum; }
    public double getLastTurretPower() { return lastTurretPower; }
    public boolean isAimed() { return Math.abs(filteredTx) < deadzone; }
}
