package org.firstinspires.ftc.teamcode.intake;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.helpers.Devices;

/**
 * The intake: one motor (or a CR servo on a motor port) that pulls balls in. PLACEHOLDER until the
 * intake is built: the motor name, direction, power and pickup settings all need setting.
 *
 * Pickup detection from motor current: a ball being pulled through loads the rollers, so the current
 * jumps above what the motor draws spinning freely. While running, update() keeps a slow average of
 * the free-running current (the baseline) and reports a pickup when the current stays more than
 * pickupCurrentRiseAmps above it for pickupHoldMs. The surge as the motor starts is ignored.
 * A colour or distance sensor in the intake would be more reliable; hasPickedUp() is where it would go.
 *
 * Motor current isn't in the bulk read, so each update() while running costs a slow hardware read.
 * If the motor isn't in the configuration, the intake is disabled and never reports a pickup.
 */
@Configurable
public class Intake {
    public static String motorName = "intake";
    public static DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
    public static double power = 1.0;

    /** A pickup is the current this far above the free-running baseline... */
    public static double pickupCurrentRiseAmps = 1.0;
    /** ...for at least this long. */
    public static double pickupHoldMs = 60;
    /** Current is ignored for this long after starting, while the motor spins up. */
    public static double startupIgnoreMs = 250;
    /** Time constant of the baseline's average. */
    public static double baselineTimeConstantSeconds = 0.3;

    private final DcMotorEx motor;  // null when the intake is disabled
    private final ElapsedTime runTimer = new ElapsedTime();
    private final ElapsedTime riseTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();

    private boolean running = false;
    private boolean reversing = false;
    private boolean rising = false;
    private boolean pickedUp = false;
    private double baselineAmps = Double.NaN;
    private double currentAmps = 0;

    public Intake(HardwareMap hardwareMap) {
        motor = Devices.get(hardwareMap, DcMotorEx.class, motorName, m -> {
            m.setDirection(direction);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        });
    }

    public boolean isAvailable() {
        return motor != null;
    }

    /** The motor's name in quotes if it's missing, otherwise "". */
    public String missingDevices() {
        return Devices.missing(motor, motorName);
    }

    /** Start pulling balls in. Clears any earlier pickup. */
    public void run() {
        if (!running && motor != null) motor.setPower(power);
        reversing = false;
        if (!running) {
            runTimer.reset();
            loopTimer.reset();
            baselineAmps = Double.NaN;
        }
        running = true;
        rising = false;
        pickedUp = false;
    }

    public void stop() {
        if ((running || reversing) && motor != null) motor.setPower(0);
        running = false;
        reversing = false;
    }

    /** Run the rollers outwards, to spit a ball out or clear a jam. run() or stop() ends it. */
    public void reverse() {
        if (!reversing && motor != null) motor.setPower(-power);
        running = false;
        reversing = true;
    }

    public boolean isReversing() {
        return reversing;
    }

    public boolean isRunning() {
        return running;
    }

    /** Call every loop while running: watches the current for a pickup. */
    public void update() {
        if (!running || motor == null) return;
        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (runTimer.milliseconds() < startupIgnoreMs) return;

        currentAmps = motor.getCurrent(CurrentUnit.AMPS);
        if (Double.isNaN(baselineAmps)) {
            baselineAmps = currentAmps;
            return;
        }
        if (currentAmps > baselineAmps + pickupCurrentRiseAmps) {
            if (!rising) {
                rising = true;
                riseTimer.reset();
            } else if (riseTimer.milliseconds() >= pickupHoldMs) {
                pickedUp = true;
            }
        } else {
            rising = false;
            // Only follow the current while it's free-running, so a pickup doesn't raise the baseline
            double alpha = dt / (baselineTimeConstantSeconds + dt);
            baselineAmps += alpha * (currentAmps - baselineAmps);
        }
    }

    /** Whether a ball has been picked up since run(). */
    public boolean hasPickedUp() {
        return pickedUp;
    }

    public double getCurrentAmps() {
        return currentAmps;
    }

    public double getBaselineAmps() {
        return baselineAmps;
    }
}
