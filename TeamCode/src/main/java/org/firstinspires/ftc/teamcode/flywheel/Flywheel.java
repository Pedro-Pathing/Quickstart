package org.firstinspires.ftc.teamcode.flywheel;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.controllers.Controller;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.helpers.Devices;

import java.util.Locale;
import java.util.function.LongSupplier;

/**
 * Flywheel velocity control using Pedro Pathing 3 controllers: PID feedback on the filtered RPM error
 * plus feedforward on the target RPM, with a per-second limit on how fast the power can change.
 * Call one of the run methods once per loop.
 *
 * Ready (healthy flywheel): the filtered RPM must stay within readyTolerancePercent of the target
 * for readyHoldMs without a break. Merely passing through the window, e.g. overshooting on the way
 * up, doesn't count, because the wheel isn't settled and a ball fed then would leave at the wrong
 * speed. Leaving the window, or changing the target, restarts the hold.
 *
 * Each flywheel starts with the DEFAULT_ gains below; use the setters to give it its own tuned values.
 *
 * If the motor isn't in the robot configuration, the flywheel is disabled: it reads 0 RPM, never
 * reports ready, and the run methods do nothing (see missingDevices()).
 *
 * Safety (runToRPM only): if the wheel is getting at least faultMinPower but the encoder reads
 * almost nothing for faultSeconds, the motor current tells why:
 *  - High current (near what the motor draws stalled at that power): the wheel is JAMMED. Power is
 *    cut until reset(), to protect the motor.
 *  - Low current: the wheel is spinning but its ENCODER_FAULT means the speed can't be measured.
 *    It runs open loop on feedforward alone (kS + kV * target, battery-compensated), so it can still
 *    score where the RPM window is wide, without the controller winding up to full power. It counts
 *    as ready once the power has been steady for openLoopSettleSeconds. The current is checked again
 *    every jamCheckSeconds in case the wheel jams later.
 * Open loop needs tuned kV and kS: with the defaults (0) the wheel just stops.
 * The safety settings below are shared by all flywheels and live-tunable in Panels.
 */
@Configurable
public class Flywheel {
    public enum Health { OK, ENCODER_FAULT, JAMMED }

    // Fault detection. TODO: check these on the robot (unplug an encoder; hold a wheel at low power)
    /** Only suspect a fault while the power is at least this. */
    public static double faultMinPower = 0.3;
    /** The encoder counts as reading nothing below this many RPM. */
    public static double faultMaxRPM = 100;
    /** How long it must read nothing at faultMinPower or more before the current is checked. */
    public static double faultSeconds = 1.0;
    /** Current the motor draws stalled at full power (the HD Hex motor's stall current is 8.5 A). */
    public static double stallCurrentAmps = 8.5;
    /** Jammed when the current is over this fraction of the stall current at the present power. */
    public static double jamCurrentFraction = 0.6;
    /** While running open loop, how often the current is checked for a jam. */
    public static double jamCheckSeconds = 1.0;
    /** Open loop: ready once the power has been steady (not slew-limited) for this long, and after each shot. */
    public static double openLoopSettleSeconds = 1.0;
    /** Battery voltage the feedforward gains were measured at. */
    public static double nominalVoltage = 12.0;

    // Default settings every new flywheel starts with
    /** Encoder counts per motor revolution (the HD Hex motor's built-in encoder). */
    public static final double DEFAULT_COUNTS_PER_REV = 28.0;
    // Feedback gains (power per RPM of error)
    public static final double DEFAULT_KP = 0.0005;
    public static final double DEFAULT_KI = 0.0;
    public static final double DEFAULT_KD = 0.0000064;
    // Feedforward gains: kV is power per RPM of target, kS is the constant power to overcome friction
    public static final double DEFAULT_KV = 0.00015018455632909438;
    public static final double DEFAULT_KS = 0.001970753886511566;
    /** Largest change in motor power allowed per second (2.0 = zero to full power in 0.5 s). */
    public static final double DEFAULT_MAX_POWER_RATE = 2.0;
    /** Low-pass filter time constant for the measured RPM in seconds (0 = no filtering). */
    public static final double DEFAULT_RPM_FILTER_TIME_CONSTANT = 0.03;
    /** The flywheel is ready to launch when within this percentage of its target speed. */
    public static final double DEFAULT_READY_TOLERANCE_PERCENT = 2.0;
    /** How long (in milliseconds) the speed must stay within the tolerance before the flywheel is ready. */
    public static final double DEFAULT_READY_HOLD_MS = 80;

    /** Longest loop time used for the slew limit, so a stalled loop can't allow a large jump. */
    private static final double MAX_LOOP_SECONDS = 0.05;
    /** Power changes smaller than this aren't sent to the motor, as each write takes loop time. */
    private static final double POWER_WRITE_THRESHOLD = 0.002;
    /** Open loop: the power counts as steady while within this of the feedforward power. */
    private static final double SETTLED_POWER_ERROR = 0.02;
    /** The battery voltage is read at most this often in open loop, as each read takes loop time. */
    private static final double VOLTAGE_READ_INTERVAL_SECONDS = 0.1;

    private final String motorName;
    private final DcMotorEx motor;  // null when the flywheel is disabled
    private final VoltageSensor voltageSensor;  // null if there's no hub to read it from
    private final Controller controller;
    private final LongSupplier clock;  // nanoseconds; injectable so tests don't have to sleep
    private final Stopwatch loopTimer, readyTimer, faultTimer, jamCheckTimer, settleTimer, voltageTimer;

    private double countsPerRev = DEFAULT_COUNTS_PER_REV;
    private double kP = DEFAULT_KP;
    private double kI = DEFAULT_KI;
    private double kD = DEFAULT_KD;
    private double kV = DEFAULT_KV;
    private double kS = DEFAULT_KS;
    private double maxPowerRate = DEFAULT_MAX_POWER_RATE;
    private double rpmFilterTimeConstant = DEFAULT_RPM_FILTER_TIME_CONSTANT;
    private double readyTolerancePercent = DEFAULT_READY_TOLERANCE_PERCENT;
    private double readyHoldMs = DEFAULT_READY_HOLD_MS;

    private double rawRPM = 0;
    private double filteredRPM = 0;
    private double rpmRate = 0;
    private double power = 0;
    private double targetRPM = 0;

    private Health health = Health.OK;
    private boolean faultSuspected = false;
    private boolean inReadyWindow = false;  // within tolerance since readyTimer was last reset
    private double readyWindowTarget = 0;   // the target the hold is being timed against
    private double voltage = Double.NaN;
    private double faultCurrent = 0;

    public Flywheel(HardwareMap hardwareMap, String motorName, DcMotorSimple.Direction direction) {
        this(hardwareMap, motorName, direction, System::nanoTime);
    }

    /** For tests: takes the clock (in nanoseconds) the timers run from. */
    Flywheel(HardwareMap hardwareMap, String motorName, DcMotorSimple.Direction direction, LongSupplier clock) {
        this.clock = clock;
        loopTimer = new Stopwatch();
        readyTimer = new Stopwatch();
        faultTimer = new Stopwatch();
        jamCheckTimer = new Stopwatch();
        settleTimer = new Stopwatch();
        voltageTimer = new Stopwatch();
        this.motorName = motorName;
        motor = Devices.get(hardwareMap, DcMotorEx.class, motorName, m -> {
            m.setDirection(direction);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        });
        voltageSensor = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next() : null;

        // The controllers read this flywheel's gains through suppliers every loop, so the setters
        // take effect immediately without rebuilding the controllers
        controller = Controller.sum(
                Controller.proportional(() -> kP),
                Controller.integral(() -> kI),
                Controller.derivative(() -> kD),
                Controller.proportionalFeedforward(() -> kV),
                Controller.staticTargetFeedforward(() -> kS)
        );
    }

    /** Whether the motor was found. */
    public boolean isAvailable() {
        return motor != null;
    }

    /** The motor's name in quotes if it's missing, otherwise "". */
    public String missingDevices() {
        return Devices.missing(motor, motorName);
    }

    /**
     * Call before starting closed-loop control, e.g. after waitForStart or when switching modes.
     * Also clears any fault, so it's the way to try a jammed or faulty flywheel again.
     */
    public void reset() {
        loopTimer.reset();
        controller.reset();
        rawRPM = readRPM();
        filteredRPM = rawRPM;
        rpmRate = 0;
        health = Health.OK;
        faultSuspected = false;
        inReadyWindow = false;
    }

    /** Drives the flywheel towards targetRPM: closed loop normally, open loop with an encoder fault. */
    public void runToRPM(double targetRPM) {
        this.targetRPM = targetRPM;
        double dt = measure();
        if (health != Health.OK) inReadyWindow = false;
        switch (health) {
            case OK:
                double error = targetRPM - filteredRPM;
                // Passing the filtered RPM's rate of change makes the D term act on the measurement
                applyPower(controller.calculate(targetRPM, error, rpmRate), dt);
                updateReadyHold();
                checkEncoder();
                break;
            case ENCODER_FAULT:
                double feedforward = openLoopPower(targetRPM);
                applyPower(feedforward, dt);
                if (Math.abs(feedforward - power) > SETTLED_POWER_ERROR) settleTimer.reset();
                checkForJam();
                break;
            case JAMMED:
                cutPower();
                break;
        }
    }

    /**
     * Call when a ball is fed. In open loop the wheel's speed after a shot can't be measured, so it
     * isn't ready again until it has had openLoopSettleSeconds to recover.
     */
    public void shotFired() {
        settleTimer.reset();
    }

    public Health getHealth() {
        return health;
    }

    /** What's wrong, for telemetry, or "" when the flywheel is healthy. */
    public String faultMessage() {
        switch (health) {
            case ENCODER_FAULT:
                return String.format(Locale.US, "encoder reads 0 RPM at %.1f A: running open loop", faultCurrent);
            case JAMMED:
                return String.format(Locale.US, "jammed (%.1f A with the wheel not turning): power cut", faultCurrent);
            default:
                return "";
        }
    }

    /** Sends a fixed power (still rate-limited) with no feedback, for feedforward testing. */
    public void runOpenLoop(double requestedPower) {
        targetRPM = 0;
        inReadyWindow = false;
        applyPower(requestedPower, measure());
    }

    /** Cuts power; with FLOAT the wheel coasts to a stop. */
    public void stop() {
        targetRPM = 0;
        inReadyWindow = false;
        cutPower();
        measure();
    }

    public double rawRPM() {
        return rawRPM;
    }

    public double filteredRPM() {
        return filteredRPM;
    }

    public double power() {
        return power;
    }

    public double targetRPM() {
        return targetRPM;
    }

    /**
     * True when the flywheel is running to a target and its filtered speed has stayed within
     * readyTolerancePercent of it for readyHoldMs, so a ball fed now leaves at the intended speed.
     * The hold is timed in runToRPM, so call that first each loop. In open loop (an
     * encoder fault) the speed can't be measured, so it's ready once the power has been steady for
     * openLoopSettleSeconds. Never ready when jammed or missing.
     */
    public boolean isReady() {
        if (motor == null || targetRPM <= 0) return false;
        switch (health) {
            case OK:
                return inReadyWindow && readyTimer.milliseconds() >= readyHoldMs;
            case ENCODER_FAULT:
                return settleTimer.seconds() >= openLoopSettleSeconds;
            default:
                return false;
        }
    }

    // ---------------- Settings ----------------

    /** Sets all three feedback gains at once. */
    public void setPID(double kP, double kI, double kD) {
        setKP(kP);
        setKI(kI);
        setKD(kD);
    }

    /** Sets both feedforward gains at once. */
    public void setFeedforward(double kV, double kS) {
        setKV(kV);
        setKS(kS);
    }

    public double getKP() {
        return kP;
    }

    public void setKP(double kP) {
        this.kP = kP;
    }

    public double getKI() {
        return kI;
    }

    public void setKI(double kI) {
        this.kI = kI;
    }

    public double getKD() {
        return kD;
    }

    public void setKD(double kD) {
        this.kD = kD;
    }

    public double getKV() {
        return kV;
    }

    public void setKV(double kV) {
        this.kV = kV;
    }

    public double getKS() {
        return kS;
    }

    public void setKS(double kS) {
        this.kS = kS;
    }

    public double getMaxPowerRate() {
        return maxPowerRate;
    }

    public void setMaxPowerRate(double maxPowerRate) {
        this.maxPowerRate = maxPowerRate;
    }

    public double getRpmFilterTimeConstant() {
        return rpmFilterTimeConstant;
    }

    public void setRpmFilterTimeConstant(double rpmFilterTimeConstant) {
        this.rpmFilterTimeConstant = rpmFilterTimeConstant;
    }

    public double getReadyTolerancePercent() {
        return readyTolerancePercent;
    }

    public void setReadyTolerancePercent(double readyTolerancePercent) {
        this.readyTolerancePercent = readyTolerancePercent;
    }

    public double getReadyHoldMs() {
        return readyHoldMs;
    }

    public void setReadyHoldMs(double readyHoldMs) {
        this.readyHoldMs = readyHoldMs;
    }

    public double getCountsPerRev() {
        return countsPerRev;
    }

    public void setCountsPerRev(double countsPerRev) {
        this.countsPerRev = countsPerRev;
    }

    /** The motor's direction, or null if it's missing. */
    public DcMotorSimple.Direction getDirection() {
        return motor != null ? motor.getDirection() : null;
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        if (motor != null) motor.setDirection(direction);
    }

    // ---------------- Internals ----------------

    /** Reads the RPM and updates the low-pass filter. Returns this loop's duration in seconds. */
    private double measure() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        rawRPM = readRPM();
        double alpha = rpmFilterTimeConstant > 0 ? dt / (rpmFilterTimeConstant + dt) : 1.0;
        double lastFilteredRPM = filteredRPM;
        filteredRPM += alpha * (rawRPM - filteredRPM);
        if (dt > 1e-3) {
            rpmRate = (filteredRPM - lastFilteredRPM) / dt;
        }
        return dt;
    }

    /**
     * Times how long the filtered RPM has been continuously within tolerance of the target. A new
     * target restarts the hold, as the wheel has to settle on that one.
     */
    private void updateReadyHold() {
        boolean inWindow = targetRPM > 0
                && Math.abs(targetRPM - filteredRPM) <= targetRPM * readyTolerancePercent / 100;
        if (!inWindow) {
            inReadyWindow = false;
        } else if (!inReadyWindow || targetRPM != readyWindowTarget) {
            inReadyWindow = true;
            readyWindowTarget = targetRPM;
            readyTimer.reset();
        }
    }

    private void applyPower(double requested, double dt) {
        if (motor == null) return;
        double out = Range.clip(requested, 0.0, 1.0);
        // Limit how quickly the power can change, scaled by this loop's duration
        double maxStep = maxPowerRate * Math.min(dt, MAX_LOOP_SECONDS);
        out = Range.clip(out, power - maxStep, power + maxStep);
        double change = Math.abs(out - power);
        if (change >= POWER_WRITE_THRESHOLD || (change > 0 && (out == 0.0 || out == 1.0))) {
            motor.setPower(out);
            power = out;
        }
    }

    private void cutPower() {
        if (power != 0) {
            power = 0;
            if (motor != null) motor.setPower(0);
        }
    }

    private double readRPM() {
        return motor != null ? motor.getVelocity() / countsPerRev * 60.0 : 0.0;
    }

    /** Closed loop: is the wheel getting power but the encoder reading nothing? If so for long enough, find out why. */
    private void checkEncoder() {
        if (motor == null || power < faultMinPower || Math.abs(rawRPM) >= faultMaxRPM) {
            faultSuspected = false;
            return;
        }
        if (!faultSuspected) {
            faultSuspected = true;
            faultTimer.reset();
        } else if (faultTimer.seconds() >= faultSeconds) {
            faultSuspected = false;
            if (isJammed()) {
                setHealth(Health.JAMMED);
                cutPower();
            } else {
                setHealth(Health.ENCODER_FAULT);
                settleTimer.reset();
                jamCheckTimer.reset();
            }
        }
    }

    /** Open loop: every jamCheckSeconds, check the current in case the wheel has jammed. */
    private void checkForJam() {
        if (jamCheckTimer.seconds() < jamCheckSeconds) return;
        jamCheckTimer.reset();
        if (power >= faultMinPower && isJammed()) {
            setHealth(Health.JAMMED);
            cutPower();
        }
    }

    /**
     * A stalled motor draws about stallCurrentAmps * power; one turning freely draws much less, as
     * its back EMF opposes the supply. Motor current isn't in the bulk read, so this is a slow read:
     * it's only made while a fault is suspected.
     */
    private boolean isJammed() {
        faultCurrent = motor.getCurrent(CurrentUnit.AMPS);
        return faultCurrent > jamCurrentFraction * stallCurrentAmps * power;
    }

    private void setHealth(Health newHealth) {
        health = newHealth;
        RobotLog.ww("Flywheel", "\"%s\" %s", motorName, faultMessage());
    }

    /** Elapsed time since reset(), read from the (injectable) clock. */
    private final class Stopwatch {
        private long startNanos = clock.getAsLong();

        void reset() {
            startNanos = clock.getAsLong();
        }

        double seconds() {
            return (clock.getAsLong() - startNanos) / 1e9;
        }

        double milliseconds() {
            return seconds() * 1000.0;
        }
    }

    /** Feedforward power for a target, scaled up as the battery drops below nominalVoltage. */
    private double openLoopPower(double target) {
        if (voltageSensor != null && (Double.isNaN(voltage) || voltageTimer.seconds() > VOLTAGE_READ_INTERVAL_SECONDS)) {
            voltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }
        double compensation = voltage > 1 ? nominalVoltage / voltage : 1.0;
        return (kS + kV * target) * compensation;
    }
}
