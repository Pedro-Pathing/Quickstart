package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.flywheel.Flywheel;
import org.firstinspires.ftc.teamcode.helpers.BulkCache;
import org.firstinspires.ftc.teamcode.helpers.SystemStatus;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Tuning OpMode for a flywheel's gains. Set motorName (and direction) in Panels before INIT to
 * choose which flywheel to tune, e.g. the nectar or pollen launcher.
 *
 * A / Cross     Feedforward test: steps through fixed powers and fits kV and kS
 * D-pad up      Apply the fitted kV and kS (after the feedforward test)
 * B / Circle    Step test: alternates between two target speeds, for tuning kP, kD and kI
 * Y / Triangle  Shot test: holds shotRPM and measures the speed drop and recovery after each shot
 * X / Square    Stop the flywheel
 *
 * Watch targetRPM, filteredRPM and power on the Panels graph and tune the gains below in Panels.
 * They are copied into the flywheel every loop. Copy the final values into that flywheel's setup code.
 */
@TeleOp(name = "Flywheel Tuner", group = "Tuning")
@Configurable
public class FlywheelTuner extends OpMode {

    // Which flywheel to tune (read at INIT)
    public static String motorName = "Launcher";
    public static DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;

    // Gains being tuned, starting from the Flywheel defaults
    public static double kP = Flywheel.DEFAULT_KP;
    public static double kI = Flywheel.DEFAULT_KI;
    public static double kD = Flywheel.DEFAULT_KD;
    public static double kV = Flywheel.DEFAULT_KV;
    public static double kS = Flywheel.DEFAULT_KS;
    public static double maxPowerRate = Flywheel.DEFAULT_MAX_POWER_RATE;
    public static double rpmFilterTimeConstant = Flywheel.DEFAULT_RPM_FILTER_TIME_CONSTANT;

    // Feedforward test settings
    public static double ffStartPower = 0.2;
    public static double ffEndPower = 1.0;
    public static double ffPowerStep = 0.1;
    /** Time at each power before measuring, so the wheel reaches a steady speed. */
    public static double ffSettleSeconds = 2.0;
    /** Time spent averaging the speed at each power. */
    public static double ffMeasureSeconds = 1.0;
    /** Speeds below this are ignored in the fit, because the wheel is barely turning. */
    public static double ffMinRPM = 100;

    // Step test settings
    public static double stepLowRPM = 2300;
    public static double stepHighRPM = 3300;
    public static double stepSeconds = 3.0;

    // Shot test settings
    public static double shotRPM = 2800;
    /** A drop of more than this percentage below the target counts as a shot. */
    public static double shotDropPercent = 3.0;
    /** The wheel counts as recovered (and ready) when within this percentage of the target. */
    public static double readyPercent = 2.0;

    /** How long at the end of each step the steady error is averaged over. */
    private static final double STEADY_WINDOW_SECONDS = 0.5;
    /** How long the wheel must stay within readyPercent before it shows as ready. */
    private static final double READY_HOLD_SECONDS = 0.3;

    private enum Mode { STOPPED, FEEDFORWARD, STEP, SHOT }

    private final SystemStatus status = new SystemStatus();
    private BulkCache bulkCache;
    private Flywheel flywheel;
    private VoltageSensor voltageSensor;  // null if there's no hub to read it from
    private TelemetryManager telemetryM;
    private final ElapsedTime modeTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();

    private Mode mode = Mode.STOPPED;

    // Feedforward test state
    private final List<Double> ffPowers = new ArrayList<>();
    private final List<Double> ffSpeeds = new ArrayList<>();
    private int ffIndex = 0;
    private double ffRPMSum = 0;
    private int ffRPMCount = 0;
    private boolean ffHasResult = false;
    private double fitKV = 0, fitKS = 0, fitR2 = 0;
    private String ffMessage = "";

    // Step test state
    private boolean stepHigh = false;
    private double stepStartRPM = 0;
    private double stepTarget = 0;
    private double riseStartTime = -1, riseEndTime = -1;
    private double peakBeyond = 0;
    private double steadyErrorSum = 0;
    private int steadyErrorCount = 0;
    private String stepUpResult = "none yet";
    private String stepDownResult = "none yet";

    // Shot test state
    private boolean ready = false;
    private double readySince = -1;
    private boolean inDip = false;
    private double dipStartTime = 0;
    private double dipMinRPM = 0;
    private int shots = 0;
    private double lastDropPercent = 0;
    private double lastRecoverySeconds = 0;
    private double totalRecoverySeconds = 0;

    @Override
    public void init() {
        bulkCache = new BulkCache(hardwareMap);
        flywheel = new Flywheel(hardwareMap, motorName, direction);
        applySettings();
        voltageSensor = hardwareMap.voltageSensor.iterator().hasNext() ? hardwareMap.voltageSensor.iterator().next() : null;
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        status.check("Flywheel", flywheel.missingDevices());

        telemetry.addLine("Flywheel Tuner ready for " + motorName + ". Press START, then:");
        telemetry.addLine("A/Cross: feedforward test   B/Circle: step test");
        telemetry.addLine("Y/Triangle: shot test   X/Square: stop");
        status.addTelemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void start() {
        bulkCache.clear();
        loopTimer.reset();
    }

    @Override
    public void loop() {
        bulkCache.clear();
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        if (gamepad1.xWasPressed()) {
            mode = Mode.STOPPED;
        } else if (gamepad1.aWasPressed()) {
            startFeedforward();
        } else if (gamepad1.bWasPressed()) {
            if (mode == Mode.STEP) {
                mode = Mode.STOPPED;
            } else {
                startStep();
            }
        } else if (gamepad1.yWasPressed()) {
            startShot();
        }

        if (gamepad1.dpadUpWasPressed() && ffHasResult) {
            kV = fitKV;
            kS = fitKS;
            ffMessage = "Applied kV and kS. Copy them into the flywheel's setup code to keep them.";
        }

        applySettings();

        switch (mode) {
            case FEEDFORWARD:
                updateFeedforward();
                break;
            case STEP:
                updateStep();
                break;
            case SHOT:
                updateShot();
                break;
            default:
                flywheel.stop();
                break;
        }

        // Telemetry is only built when Panels is due to send it, to keep the loop fast
        if (telemetryM.getShouldUpdateLines()) {
            showTelemetry(loopMs);
        }
    }

    @Override
    public void stop() {
        flywheel.stop();
    }

    private void showTelemetry(double loopMs) {
        // Graph lines for Panels
        telemetryM.addData("targetRPM", flywheel.targetRPM());
        telemetryM.addData("filteredRPM", flywheel.filteredRPM());
        telemetryM.addData("rawRPM", flywheel.rawRPM());
        telemetryM.addData("power", flywheel.power());

        status.addTelemetry(telemetry);
        telemetry.addData("Flywheel", motorName);
        if (flywheel.getHealth() != Flywheel.Health.OK) {
            telemetry.addData("FAULT", flywheel.faultMessage() + " (start a test again to retry)");
        }
        telemetry.addData("Mode", mode);
        if (voltageSensor != null) telemetry.addData("Battery", "%.2f V", voltageSensor.getVoltage());
        telemetry.addData("Loop time", "%.0f ms", loopMs);
        telemetry.addData("Gains", "kP %.6f  kI %.7f  kD %.7f", flywheel.getKP(), flywheel.getKI(), flywheel.getKD());
        telemetry.addData("Feedforward", "kV %.10f  kS %.6f", flywheel.getKV(), flywheel.getKS());

        switch (mode) {
            case FEEDFORWARD:
                showFeedforward();
                break;
            case STEP:
                showStep();
                break;
            case SHOT:
                showShot();
                break;
            default:
                if (ffHasResult || !ffMessage.isEmpty()) {
                    showFeedforward();
                } else {
                    telemetry.addLine("A/Cross: feedforward test   B/Circle: step test");
                    telemetry.addLine("Y/Triangle: shot test   X/Square: stop");
                }
                break;
        }

        telemetryM.update(telemetry);
    }

    /** Copies the Panels values into the flywheel, so changes take effect straight away. */
    private void applySettings() {
        flywheel.setPID(kP, kI, kD);
        flywheel.setFeedforward(kV, kS);
        flywheel.setMaxPowerRate(maxPowerRate);
        flywheel.setRpmFilterTimeConstant(rpmFilterTimeConstant);
    }

    // ---------------- Feedforward test ----------------

    private void startFeedforward() {
        mode = Mode.FEEDFORWARD;
        ffPowers.clear();
        ffSpeeds.clear();
        for (double p = ffStartPower; p <= ffEndPower + 1e-6; p += ffPowerStep) {
            ffPowers.add(Math.min(p, 1.0));
        }
        ffIndex = 0;
        ffRPMSum = 0;
        ffRPMCount = 0;
        ffHasResult = false;
        ffMessage = "";
        flywheel.reset();
        modeTimer.reset();
    }

    private void updateFeedforward() {
        if (ffIndex >= ffPowers.size()) {
            flywheel.stop();
            return;
        }

        double power = ffPowers.get(ffIndex);
        flywheel.runOpenLoop(power);

        double t = modeTimer.seconds();
        if (t > ffSettleSeconds) {
            ffRPMSum += flywheel.rawRPM();
            ffRPMCount++;
        }

        if (t > ffSettleSeconds + ffMeasureSeconds) {
            double averageRPM = ffRPMCount > 0 ? ffRPMSum / ffRPMCount : 0;
            ffSpeeds.add(averageRPM);

            ffIndex++;
            ffRPMSum = 0;
            ffRPMCount = 0;
            modeTimer.reset();

            if (ffIndex >= ffPowers.size()) {
                fitFeedforward();
                mode = Mode.STOPPED;
            }
        }
    }

    /** Least-squares fit of power = kS + kV * RPM over the points where the wheel was turning. */
    private void fitFeedforward() {
        int n = 0;
        double sx = 0, sy = 0, sxx = 0, sxy = 0, syy = 0;
        for (int i = 0; i < ffSpeeds.size(); i++) {
            double rpm = ffSpeeds.get(i);
            if (rpm < ffMinRPM) continue;
            double p = ffPowers.get(i);
            n++;
            sx += rpm;
            sy += p;
            sxx += rpm * rpm;
            sxy += rpm * p;
            syy += p * p;
        }

        double denominator = n * sxx - sx * sx;
        if (n < 3 || Math.abs(denominator) < 1e-9) {
            ffHasResult = false;
            ffMessage = "Not enough readings to fit. Check the flywheel spins, then try again.";
            return;
        }

        fitKV = (n * sxy - sx * sy) / denominator;
        fitKS = (sy - fitKV * sx) / n;

        double ssTotal = syy - sy * sy / n;
        double ssResidual = 0;
        for (int i = 0; i < ffSpeeds.size(); i++) {
            double rpm = ffSpeeds.get(i);
            if (rpm < ffMinRPM) continue;
            double predicted = fitKS + fitKV * rpm;
            double residual = ffPowers.get(i) - predicted;
            ssResidual += residual * residual;
        }
        fitR2 = ssTotal > 0 ? 1 - ssResidual / ssTotal : 0;

        ffHasResult = true;
        ffMessage = "Press D-pad up to apply these values.";
    }

    private void showFeedforward() {
        if (mode == Mode.FEEDFORWARD && ffIndex < ffPowers.size()) {
            telemetry.addData("Feedforward test", "step %d of %d at power %.2f",
                    ffIndex + 1, ffPowers.size(), ffPowers.get(ffIndex));
        }
        for (int i = 0; i < ffSpeeds.size(); i++) {
            telemetry.addData(String.format(Locale.US, "Power %.2f", ffPowers.get(i)), "%.0f RPM", ffSpeeds.get(i));
        }
        if (ffHasResult) {
            telemetry.addData("Fitted kV", "%.10f", fitKV);
            telemetry.addData("Fitted kS", "%.6f", fitKS);
            telemetry.addData("Fit quality (R²)", "%.3f (aim for 0.98 or more)", fitR2);
            telemetryM.addData("fittedKV", fitKV);
            telemetryM.addData("fittedKS", fitKS);
        }
        if (!ffMessage.isEmpty()) {
            telemetry.addLine(ffMessage);
        }
    }

    // ---------------- Step test ----------------

    private void startStep() {
        mode = Mode.STEP;
        flywheel.reset();
        stepHigh = false;
        stepUpResult = "none yet";
        stepDownResult = "none yet";
        beginStep(stepLowRPM);
    }

    private void beginStep(double target) {
        stepStartRPM = flywheel.filteredRPM();
        stepTarget = target;
        riseStartTime = -1;
        riseEndTime = -1;
        peakBeyond = 0;
        steadyErrorSum = 0;
        steadyErrorCount = 0;
        modeTimer.reset();
    }

    private void updateStep() {
        flywheel.runToRPM(stepTarget);

        double t = modeTimer.seconds();
        double rpm = flywheel.filteredRPM();
        double change = stepTarget - stepStartRPM;

        if (Math.abs(change) > 1) {
            // Progress from 0 (start speed) to 1 (target speed), in the direction of the step
            double progress = (rpm - stepStartRPM) / change;
            if (riseStartTime < 0 && progress >= 0.1) riseStartTime = t;
            if (riseEndTime < 0 && progress >= 0.9) riseEndTime = t;
            peakBeyond = Math.max(peakBeyond, (progress - 1) * Math.abs(change));
        }

        if (t > stepSeconds - STEADY_WINDOW_SECONDS) {
            steadyErrorSum += Math.abs(stepTarget - rpm);
            steadyErrorCount++;
        }

        if (t >= stepSeconds) {
            recordStep(change);
            stepHigh = !stepHigh;
            beginStep(stepHigh ? stepHighRPM : stepLowRPM);
        }
    }

    private void recordStep(double change) {
        // The first step starts from rest, so it isn't a fair test
        if (Math.abs(change) < 1 || stepStartRPM < 1) return;

        String rise = riseStartTime >= 0 && riseEndTime >= 0
                ? String.format(Locale.US, "%.2f s", riseEndTime - riseStartTime)
                : "didn't reach 90%";
        double overshootPercent = 100 * peakBeyond / Math.abs(change);
        double steadyError = steadyErrorCount > 0 ? steadyErrorSum / steadyErrorCount : 0;
        String result = String.format(Locale.US, "rise %s, overshoot %.0f%%, steady error %.0f RPM",
                rise, overshootPercent, steadyError);

        if (change > 0) {
            stepUpResult = result;
        } else {
            stepDownResult = result;
        }
    }

    private void showStep() {
        telemetry.addData("Step test", "target %.0f RPM, next change in %.1f s",
                stepTarget, Math.max(0, stepSeconds - modeTimer.seconds()));
        telemetry.addData("Error now", "%.0f RPM", stepTarget - flywheel.filteredRPM());
        telemetry.addData("Last step up", stepUpResult);
        telemetry.addData("Last step down", stepDownResult);
        telemetry.addLine("Press B/Circle again to stop.");
    }

    // ---------------- Shot test ----------------

    private void startShot() {
        mode = Mode.SHOT;
        flywheel.reset();
        ready = false;
        readySince = -1;
        inDip = false;
        shots = 0;
        lastDropPercent = 0;
        lastRecoverySeconds = 0;
        totalRecoverySeconds = 0;
        modeTimer.reset();
    }

    private void updateShot() {
        flywheel.runToRPM(shotRPM);

        double t = modeTimer.seconds();
        double rpm = flywheel.filteredRPM();
        boolean withinReady = Math.abs(shotRPM - rpm) <= shotRPM * readyPercent / 100;

        if (inDip) {
            dipMinRPM = Math.min(dipMinRPM, rpm);
            if (rpm >= shotRPM * (1 - readyPercent / 100)) {
                inDip = false;
                shots++;
                lastDropPercent = 100 * (shotRPM - dipMinRPM) / shotRPM;
                lastRecoverySeconds = t - dipStartTime;
                totalRecoverySeconds += lastRecoverySeconds;
                readySince = t;
            }
        } else if (ready && rpm < shotRPM * (1 - shotDropPercent / 100)) {
            // A sudden drop while ready means a ball went through
            inDip = true;
            ready = false;
            dipStartTime = t;
            dipMinRPM = rpm;
        }

        if (!inDip) {
            if (withinReady) {
                if (readySince < 0) readySince = t;
                if (t - readySince >= READY_HOLD_SECONDS) ready = true;
            } else {
                readySince = -1;
                ready = false;
            }
        }
    }

    private void showShot() {
        telemetryM.addData("ready", ready ? 1 : 0);
        telemetry.addData("Shot test", "target %.0f RPM", shotRPM);
        telemetry.addData("Flywheel.isReady()", "%s (needs %.0f ms in the window)", flywheel.isReady() ? "YES" : "no", flywheel.getReadyHoldMs());
        telemetry.addData("Ready to shoot", inDip ? "NO - recovering" : (ready ? "YES" : "not yet"));
        telemetry.addData("Shots detected", shots);
        if (shots > 0) {
            telemetry.addData("Last speed drop", "%.1f%%", lastDropPercent);
            telemetry.addData("Last recovery time", "%.2f s", lastRecoverySeconds);
            telemetry.addData("Average recovery time", "%.2f s", totalRecoverySeconds / shots);
        }
    }
}
