package org.firstinspires.ftc.teamcode.experimental;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Robot;

/**
 * Manages the CR Servo turret using PID control, filtering, velocity limiting,
 * and "coast to last-seen position" logic.
 */
public class TurretPIDController {
    private Robot robot;


    // --- Constants ---
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
    private static final double TURRET_MAX_ACCELERATION = 1.5;
    private static final double FILTER_ALPHA = 0.7;
    private static final double INTEGRAL_LIMIT = 0.3;


    // PID state variables
    private double lastError = 0.0;
    private double integralSum = 0.0;
    private double filteredTx = 0.0;
    private double lastTurretPower = 0.0;
    private final ElapsedTime timer = new ElapsedTime();

    // Coasting State Variable
    private double lastValidTx = 0.0; // Stores the last known error when target was seen

    public TurretPIDController(Robot robot) {
        timer.reset(); // Start the timer for delta time calculation
        this.robot = robot;
    }

    /**
     * Executes the full PID control loop for automatic aiming, with "coast" ability.
     * @param rawTx The horizontal angle error (tx) from the Limelight.
     * @param targetIsVisible Boolean indicating if the Limelight has a valid target.
     * @return The calculated servo power.
     */
    public double updateAutoAim(double rawTx, boolean targetIsVisible) {
        // Calculate delta time (dt)
        double dt = timer.seconds();
        timer.reset();
        dt = Math.max(dt, 0.001); // Prevent division by zero

        double targetTx;

        // --- CORE MODIFICATION: COASTING LOGIC ---
        if (targetIsVisible) {
            // Target is visible: Use current measurement and update last seen position.
            lastValidTx = rawTx;
            targetTx = rawTx;
        } else {
            // Target is lost: Use the last known position to guide the turret back.
            targetTx = lastValidTx;
        }

        // 1. Low-pass filter (applied to the chosen error source)
        filteredTx = FILTER_ALPHA * targetTx + (1 - FILTER_ALPHA) * filteredTx;

        double error = filteredTx;

        // 2. PID calculation
        double pTerm = TURRET_KP * error;

        // Integral term with anti-windup. Only accumulate integral if the target is visible
        // OR we are currently coasting back (i.e., lastValidTx is not zero).
        if (targetIsVisible || Math.abs(lastValidTx) > 0.1) {
            integralSum += error * dt;
        }

        // Clamp integral sum
        integralSum = Range.clip(integralSum, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        double iTerm = TURRET_KI * integralSum;

        // Derivative term
        double derivative = (error - lastError) / dt;
        double dTerm = TURRET_KD * derivative;

        // Combine PID terms
        double turretPower = pTerm + iTerm + dTerm;

        // 3. Apply minimum power threshold
        if (Math.abs(turretPower) > 0.01 && Math.abs(turretPower) < TURRET_MIN_POWER) {
            turretPower = Math.signum(turretPower) * TURRET_MIN_POWER;
        }

        // 4. Clamp to maximum power
        turretPower = Range.clip(turretPower, -TURRET_MAX_POWER, TURRET_MAX_POWER);

        // 5. Velocity limiting
        double powerChange = turretPower - lastTurretPower;
        double maxChangeAllowed = TURRET_MAX_ACCELERATION * dt;

        if (Math.abs(powerChange) > maxChangeAllowed) {
            turretPower = lastTurretPower + Math.signum(powerChange) * maxChangeAllowed;
        }

        // 6. Apply deadzone for lock-on
        if (Math.abs(error) < TURRET_DEADZONE) {
            robot.turret.setTurretPower(0);
            integralSum = 0; // Reset integral when locked
            turretPower = 0.0;

            // If the turret successfully centers on the last known position, clear the coasting setpoint
            if (!targetIsVisible) {
                lastValidTx = 0.0;
            }
        } else {
            robot.turret.setTurretPower(turretPower);
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

        double power = input * TURRET_MANUAL_POWER;
        lastTurretPower = power;
        robot.turret.setTurretPower(power);
    }

    /**
     * Resets the PID state variables.
     */
    public void resetPIDState() {
        integralSum = 0.0;
        lastError = 0.0;
        filteredTx = 0.0;
        lastValidTx = 0.0; // Clear the coasting target
    }

    public void stop() {
        robot.turret.setTurretPower(0.0);
        resetPIDState();
    }

    // --- Telemetry Getters ---
    public double getFilteredTx() { return filteredTx; }
    public double getLastValidTx() { return lastValidTx; }
    public boolean isAimed() { return Math.abs(filteredTx) < TURRET_DEADZONE; }
    public double getLastTurretPower() { return lastTurretPower; }
}