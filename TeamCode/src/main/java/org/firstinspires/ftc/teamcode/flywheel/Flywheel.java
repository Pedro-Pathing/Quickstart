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
 * speed. Leaving the window, or the target moving by more than the tolerance, restarts the hold.
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
 *
 * kV and kS are the feedforward at nominalVoltage (FlywheelTuner fits them that way). Closed and
 * open loop both scale them by nominalVoltage / battery voltage, so a fresh battery doesn't
 * overshoot the target and a flat one doesn't fall short.
 *
 * Encoder: by default the speed comes from the motor's built-in encoder. useEncoder() switches to
 * another one, e.g. a REV Through Bore on the flywheel shaft (set countsPerRev to
 * THROUGH_BORE_COUNTS_PER_REV), plugged into this motor's encoder port or another motor's. The hub
 * reports velocity as a 16-bit number, which wraps above 32767 counts per second (only 240 RPM for
 * a Through Bore), so the reading is corrected using the change in position, which doesn't wrap.
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
    /** Encoder counts per revolution of a REV Through Bore encoder. */
    public static final double THROUGH_BORE_COUNTS_PER_REV = 8192.0;
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
    /** The hub's velocity is a signed 16-bit count, so it wraps by this many counts per second. */
    private static final double VELOCITY_WRAP = 65536.0;
    /**
     * How often the unwrapped speed is checked against the change in position. Long enough that
     * a few milliseconds' timing slip at each end of the window stays well under half a wrap
     * (a third at 6000 RPM with 4 ms each way).
     */
    private static final double WRAP_CHECK_SECONDS = 0.3;
    /**
     * For this long after the first read, the wrap is chosen by the change in position since then
     * rather than by the last reading, as there's no reading yet to go by (the wheel may still be
     * coasting from the last OpMode).
     */
    private static final double SYNC_SECONDS = 0.1;

    private final String motorName;
    private final DcMotorEx motor;  // null when the flywheel is disabled
    private String encoderName = "";  // "" when the encoder is the motor's own
    private DcMotorEx encoder;  // the motor whose encoder port it's plugged into; null if missing
    private boolean encoderReversed = false;
    private boolean hasRead = false;  // whether the encoder has been read yet
    private boolean synced = false;  // whether the last reading can be trusted to unwrap the next
    private int lastPosition = 0;
    private double lastVelocity = 0;  // counts per second, unwrapped
    // Wrap check: the position at the start of the window, its length, and the counts the
    // unwrapped speeds add up to over it
    private int checkStartPosition = 0;
    private double checkSeconds = 0;
    private double checkCounts = 0;
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
    private double voltageCompensation = 1.0;  // nominalVoltage / battery voltage
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
        encoder = motor;
        voltageSensor = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next() : null;

        // The controllers read this flywheel's gains through suppliers every loop, so the setters
        // take effect immediately without rebuilding the controllers. The feedforward gains are
        // for nominalVoltage, so they're scaled for the battery, as in open loop.
        controller = Controller.sum(
                Controller.proportional(() -> kP),
                Controller.integral(() -> kI),
                Controller.derivative(() -> kD),
                Controller.proportionalFeedforward(() -> kV * voltageCompensation),
                Controller.staticTargetFeedforward(() -> kS * voltageCompensation)
        );
    }

    /** Whether the motor was found. */
    public boolean isAvailable() {
        return motor != null;
    }

    /** The motor's name (and a separate encoder's) in quotes if it's missing, otherwise "". */
    public String missingDevices() {
        return Devices.join(Devices.missing(motor, motorName),
                encoderName.isEmpty() ? "" : Devices.missing(encoder, encoderName));
    }

    /**
     * Reads the speed from the encoder plugged into the encoder port of the motor called encoderName,
     * or from this flywheel's own motor if encoderName is "". Set countsPerRev to match the encoder.
     * That motor's direction (set by whatever drives it) also sets which way the encoder counts, so
     * use setEncoderReversed() rather than changing it. Call before reset().
     */
    public void useEncoder(HardwareMap hardwareMap, String encoderName) {
        this.encoderName = encoderName;
        encoder = encoderName.isEmpty() ? motor : Devices.get(hardwareMap, DcMotorEx.class, encoderName);
        resync();
    }

    /**
     * Call before starting closed-loop control, e.g. after waitForStart or when switching modes.
     * Also clears any fault, so it's the way to try a jammed or faulty flywheel again.
     */
    public void reset() {
        loopTimer.reset();
        controller.reset();
        rawRPM = readRPM(0);
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
                updateVoltageCompensation();
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

    public boolean isEncoderReversed() {
        return encoderReversed;
    }

    /** Set when the encoder counts backwards (negative RPM while the wheel spins forwards). */
    public void setEncoderReversed(boolean encoderReversed) {
        if (encoderReversed != this.encoderReversed) resync();
        this.encoderReversed = encoderReversed;
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

        rawRPM = readRPM(dt);
        double alpha = rpmFilterTimeConstant > 0 ? dt / (rpmFilterTimeConstant + dt) : 1.0;
        double lastFilteredRPM = filteredRPM;
        filteredRPM += alpha * (rawRPM - filteredRPM);
        if (dt > 1e-3) {
            rpmRate = (filteredRPM - lastFilteredRPM) / dt;
        }
        return dt;
    }

    /**
     * Times how long the filtered RPM has been continuously within tolerance of the target. A target
     * that moves further than the tolerance from the one the hold started on restarts the hold, as
     * the wheel has to settle on the new speed. Smaller moves don't: in TeleOp and the autos the
     * target follows the camera's distance and changes a little on almost every loop, and
     * restarting on each of those would keep the flywheel from ever being ready.
     */
    private void updateReadyHold() {
        double band = targetRPM * readyTolerancePercent / 100;
        boolean inWindow = targetRPM > 0 && Math.abs(targetRPM - filteredRPM) <= band;
        if (!inWindow) {
            inReadyWindow = false;
        } else if (!inReadyWindow || Math.abs(targetRPM - readyWindowTarget) > band) {
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

    /**
     * The encoder's speed in RPM. dt is the time in seconds since the last read, or 0 if unknown.
     *
     * The hub's velocity has 16-bit wraps added back: as many as bring it closest to the last
     * speed read. The wheel's speed changes by much less than half a wrap (240 RPM for a Through
     * Bore) between reads, and unlike an estimate from the change in position over one loop, this
     * doesn't depend on timing. The loop time is measured here, but the position is sampled during
     * the bulk read, a variable few milliseconds later; in a 20 ms loop at 3000 RPM, a 2 ms slip
     * puts that estimate out by more than half a wrap, which showed up as 480 RPM spikes.
     *
     * The change in position is the guide instead for the first SYNC_SECONDS (there's no reading
     * yet; the wheel may still be coasting from the last OpMode, so readings in that time can be
     * a wrap out) and after a long gap between reads. Every WRAP_CHECK_SECONDS it's compared with
     * the counts the unwrapped speeds add up to over the same time. Over that longer window, timing
     * slips are small, so a wrap's worth of difference means one was missed (e.g. a sudden jolt
     * of more than half a wrap in one loop) and is put right.
     */
    private double readRPM(double dt) {
        if (motor == null || encoder == null) return 0.0;
        if (!hasRead) dt = 0;  // e.g. the first read after INIT: the time since construction means nothing
        int sign = encoderReversed ? -1 : 1;
        int position = sign * encoder.getCurrentPosition();
        // int subtraction stays right even when the position wraps past Integer.MAX_VALUE
        int moved = position - lastPosition;
        double sinceSync = checkSeconds + dt;
        double reference;
        if (dt >= WRAP_CHECK_SECONDS) {
            reference = moved / dt;
        } else if (!synced && sinceSync > 0) {
            reference = (position - checkStartPosition) / sinceSync;
        } else {
            reference = lastVelocity;
        }
        double velocity = unwrap(sign * encoder.getVelocity(), reference);

        if (dt > 0) {
            checkSeconds += dt;
            checkCounts += (lastVelocity + velocity) / 2 * dt;
            if (checkSeconds >= SYNC_SECONDS) synced = true;
            if (checkSeconds >= WRAP_CHECK_SECONDS) {
                double missed = ((position - checkStartPosition) - checkCounts) / checkSeconds;
                velocity += VELOCITY_WRAP * Math.round(missed / VELOCITY_WRAP);
                startWrapCheck(position);
            }
        } else {
            startWrapCheck(position);
        }

        hasRead = true;
        lastPosition = position;
        lastVelocity = velocity;
        return velocity / countsPerRev * 60.0;
    }

    /** Forgets the previous reading, e.g. after a change of encoder, so the next reads sync afresh. */
    private void resync() {
        hasRead = false;
        synced = false;
        lastVelocity = 0;
    }

    /** velocity with as many 16-bit wraps added as bring it closest to reference. */
    private static double unwrap(double velocity, double reference) {
        return velocity + VELOCITY_WRAP * Math.round((reference - velocity) / VELOCITY_WRAP);
    }

    private void startWrapCheck(int position) {
        checkStartPosition = position;
        checkSeconds = 0;
        checkCounts = 0;
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

    /** Feedforward power for a target, scaled for the battery (see updateVoltageCompensation()). */
    private double openLoopPower(double target) {
        updateVoltageCompensation();
        return (kS + kV * target) * voltageCompensation;
    }

    /**
     * Sets voltageCompensation to nominalVoltage / battery voltage, so the feedforward gives the
     * same motor voltage whatever the battery: less power on a fresh 13.8 V battery, more as it
     * sags. The battery isn't in the bulk read, so it's read at most every
     * VOLTAGE_READ_INTERVAL_SECONDS. With no battery reading, the gains are used as they are.
     */
    private void updateVoltageCompensation() {
        if (voltageSensor != null && (Double.isNaN(voltage) || voltageTimer.seconds() > VOLTAGE_READ_INTERVAL_SECONDS)) {
            voltage = voltageSensor.getVoltage();
            voltageTimer.reset();
        }
        voltageCompensation = voltage > 1 ? nominalVoltage / voltage : 1.0;
    }
}
