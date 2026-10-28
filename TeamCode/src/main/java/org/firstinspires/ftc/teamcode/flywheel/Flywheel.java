package org.firstinspires.ftc.teamcode.flywheel;

import com.pedropathing.controllers.Controller;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.helpers.Devices;

/**
 * Flywheel velocity control using Pedro Pathing 3 controllers: PID feedback on the filtered RPM error
 * plus feedforward on the target RPM, with a per-second limit on how fast the power can change.
 * Call one of the run methods once per loop.
 *
 * Each flywheel starts with the DEFAULT_ gains below; use the setters to give it its own tuned values.
 *
 * If the motor isn't in the robot configuration, the flywheel is disabled: it reads 0 RPM, never
 * reports ready, and the run methods do nothing (see missingDevices()).
 */
public class Flywheel {
    // Default settings every new flywheel starts with
    /** Encoder counts per motor revolution (the HD Hex motor's built-in encoder). */
    public static final double DEFAULT_COUNTS_PER_REV = 28.0;
    // Feedback gains (power per RPM of error)
    public static final double DEFAULT_KP = 0.005;
    public static final double DEFAULT_KI = 0.000002;
    public static final double DEFAULT_KD = 0.000004;
    // Feedforward gains: kV is power per RPM of target, kS is the constant power to overcome friction
    public static final double DEFAULT_KV = 0.0;
    public static final double DEFAULT_KS = 0.0;
    /** Largest change in motor power allowed per second (2.0 = zero to full power in 0.5 s). */
    public static final double DEFAULT_MAX_POWER_RATE = 2.0;
    /** Low-pass filter time constant for the measured RPM in seconds (0 = no filtering). */
    public static final double DEFAULT_RPM_FILTER_TIME_CONSTANT = 0.03;
    /** The flywheel is ready to launch when within this percentage of its target speed. */
    public static final double DEFAULT_READY_TOLERANCE_PERCENT = 2.0;

    /** Longest loop time used for the slew limit, so a stalled loop can't allow a large jump. */
    private static final double MAX_LOOP_SECONDS = 0.05;
    /** Power changes smaller than this aren't sent to the motor, as each write takes loop time. */
    private static final double POWER_WRITE_THRESHOLD = 0.002;

    private final String motorName;
    private final DcMotorEx motor;  // null when the flywheel is disabled
    private final Controller controller;
    private final ElapsedTime loopTimer = new ElapsedTime();

    private double countsPerRev = DEFAULT_COUNTS_PER_REV;
    private double kP = DEFAULT_KP;
    private double kI = DEFAULT_KI;
    private double kD = DEFAULT_KD;
    private double kV = DEFAULT_KV;
    private double kS = DEFAULT_KS;
    private double maxPowerRate = DEFAULT_MAX_POWER_RATE;
    private double rpmFilterTimeConstant = DEFAULT_RPM_FILTER_TIME_CONSTANT;
    private double readyTolerancePercent = DEFAULT_READY_TOLERANCE_PERCENT;

    private double rawRPM = 0;
    private double filteredRPM = 0;
    private double rpmRate = 0;
    private double power = 0;
    private double targetRPM = 0;

    public Flywheel(HardwareMap hardwareMap, String motorName, DcMotorSimple.Direction direction) {
        this.motorName = motorName;
        motor = Devices.get(hardwareMap, DcMotorEx.class, motorName, m -> {
            m.setDirection(direction);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        });

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

    /** Call before starting closed-loop control, e.g. after waitForStart or when switching modes. */
    public void reset() {
        loopTimer.reset();
        controller.reset();
        rawRPM = readRPM();
        filteredRPM = rawRPM;
        rpmRate = 0;
    }

    /** Drives the flywheel towards targetRPM using the controllers. */
    public void runToRPM(double targetRPM) {
        this.targetRPM = targetRPM;
        double dt = measure();
        double error = targetRPM - filteredRPM;
        // Passing the filtered RPM's rate of change makes the D term act on the measurement
        applyPower(controller.calculate(targetRPM, error, rpmRate), dt);
    }

    /** Sends a fixed power (still rate-limited) with no feedback, for feedforward testing. */
    public void runOpenLoop(double requestedPower) {
        targetRPM = 0;
        applyPower(requestedPower, measure());
    }

    /** Cuts power; with FLOAT the wheel coasts to a stop. */
    public void stop() {
        targetRPM = 0;
        if (power != 0) {
            power = 0;
            if (motor != null) motor.setPower(0);
        }
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
     * True when the flywheel is running to a target and its filtered speed is within
     * readyTolerancePercent of it, so a ball fed now leaves at the intended speed.
     */
    public boolean isReady() {
        return motor != null && targetRPM > 0 && Math.abs(targetRPM - filteredRPM) <= targetRPM * readyTolerancePercent / 100;
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

    private double readRPM() {
        return motor != null ? motor.getVelocity() / countsPerRev * 60.0 : 0.0;
    }
}
