package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.flywheel.Flywheel;
import org.firstinspires.ftc.teamcode.helpers.BulkCache;
import org.firstinspires.ftc.teamcode.helpers.SystemStatus;
import org.firstinspires.ftc.teamcode.leds.Leds;
import org.firstinspires.ftc.teamcode.leds.RobotState;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Runs one launcher at a chosen speed under its normal closed-loop control and logs every launch it
 * detects, to line up with slow-motion video of the shots.
 *
 * INIT          D-pad left: Nectar launcher   D-pad right: Pollen launcher
 * D-pad up/down Target speed up or down by rpmStep (startRPM to begin with, 0 to maxRPM; 0 stops)
 *
 * A launch is a sudden drop in the filtered RPM while the wheel is holding its target: the ball
 * takes energy from the wheel as it is pinched and accelerated. It counts once the speed has dropped
 * dropPercent below where it was, and ends when the wheel is back within recoveredPercent of the
 * target (or after maxLaunchSeconds). A second ball arriving while the wheel is still recovering
 * starts a new launch. RPM is used rather than motor current because the velocity comes in the bulk
 * read, so it's sampled every loop, while each current read is a slow extra hub transaction.
 *
 * LEDs, for lining the video up with the log: yellow when ready for a ball, red while not (spinning
 * up, recovering or stopped), and a green flash of flashSeconds the moment each launch is detected.
 * The flash comes a little after the drop starts (once it passes dropPercent) and the Blinkin takes
 * a few tens of ms to change, so the log records the detection time too.
 *
 * Each launch is written as a CSV line to FIRST/data/FlywheelCharacterization/ on the robot:
 *  launch       launch number, from 1
 *  time_ms      when the speed started to drop, in ms since START was pressed
 *  flash_ms     when the launch was detected and the LEDs flashed green, in ms since START
 *  clock        the same moment as a time of day (the robot's clock), to help find it on video
 *  battery_v    battery voltage just before the launch
 *  target_rpm   the target speed selected
 *  before_rpm   filtered RPM at the instant before the drop started
 *  lowest_rpm   lowest filtered RPM during the launch
 *  recovery_ms  from the start of the drop until back within recoveredPercent (blank if it never was)
 * The file's header records the launcher, its gains and the detection settings used.
 */
@TeleOp(name = "Flywheel Characterization", group = "Tuning")
@Configurable
public class FlywheelCharacterization extends OpMode {
    public static double startRPM = 2000;
    public static double rpmStep = 200;
    public static double maxRPM = 5400;

    // Launch detection
    /** The wheel must hold within this percentage of its target... */
    public static double armedPercent = 2.0;
    /** ...for this long before a drop counts as a launch (so spin-up and target changes don't). */
    public static double armedSeconds = 0.3;
    /** A drop of more than this percentage of the target, below the speed before it, is a launch. */
    public static double dropPercent = 3.0;
    /** A launch ends when the wheel is back within this percentage of its target... */
    public static double recoveredPercent = 2.0;
    /** ...or after this long, whichever comes first. */
    public static double maxLaunchSeconds = 2.0;
    /** How long the LEDs flash green when a launch is detected. */
    public static double flashSeconds = 0.3;

    private enum Launcher {
        NECTAR("Nectar", "NectarFlywheel", DcMotorSimple.Direction.FORWARD),
        POLLEN("Pollen", "PollenFlywheel", DcMotorSimple.Direction.REVERSE);

        final String label;
        final String motorName;
        final DcMotorSimple.Direction direction;

        Launcher(String label, String motorName, DcMotorSimple.Direction direction) {
            this.label = label;
            this.motorName = motorName;
            this.direction = direction;
        }
    }

    /** The battery is read this often between launches; each read is a slow hub transaction. */
    private static final double VOLTAGE_READ_INTERVAL_SECONDS = 0.25;
    /** Recent filtered-RPM samples kept for finding where a drop started (a few hundred ms of loops). */
    private static final int HISTORY_SIZE = 128;
    /** A drop is measured from the highest speed in this long before it. */
    private static final double PEAK_WINDOW_MS = 150;
    private static final File LOG_FOLDER = new File(AppUtil.ROBOT_DATA_DIR, "FlywheelCharacterization");

    private final SystemStatus status = new SystemStatus();
    private BulkCache bulkCache;
    private VoltageSensor voltageSensor;  // null if there's no hub to read it from
    private TelemetryManager telemetryM;
    private Leds leds;

    private Launcher launcher = Launcher.POLLEN;
    private Flywheel flywheel;
    private double targetRPM;

    private final ElapsedTime runTimer = new ElapsedTime();
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();
    private long startClockMs;
    private double voltage = Double.NaN;

    // Filtered RPM history, as a ring buffer
    private final double[] historyMs = new double[HISTORY_SIZE];
    private final double[] historyRPM = new double[HISTORY_SIZE];
    private int historyCount = 0;
    private int historyNext = 0;

    // Launch detection state
    private double armedSinceMs = -1;
    private boolean armed = false;
    private boolean inLaunch = false;
    private double launchStartMs, launchDetectedMs, launchBeforeRPM, launchVoltage, launchTargetRPM, launchLowestRPM;
    /** Highest speed since the lowest point of this launch, for spotting a second ball. */
    private double peakSinceLowestRPM;
    private int launches = 0;
    private String lastLaunch = "none yet";

    private PrintWriter log;  // null if the file couldn't be opened
    private String logName = "";
    private String logError = "";

    @Override
    public void init() {
        bulkCache = new BulkCache(hardwareMap);
        voltageSensor = hardwareMap.voltageSensor.iterator().hasNext() ? hardwareMap.voltageSensor.iterator().next() : null;
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        leds = new Leds(hardwareMap);
        leds.setState(RobotState.NO_SHOT);
        leds.update();
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpadLeftWasPressed()) launcher = Launcher.NECTAR;
        if (gamepad1.dpadRightWasPressed()) launcher = Launcher.POLLEN;

        telemetry.addLine("Flywheel Characterization");
        telemetry.addData("Launcher", "%s (\"%s\")", launcher.label, launcher.motorName);
        telemetry.addLine("D-pad left: Nectar   D-pad right: Pollen");
        telemetry.addLine("Press START, then D-pad up/down to change the target speed.");
        telemetry.update();
    }

    @Override
    public void start() {
        bulkCache.clear();
        flywheel = new Flywheel(hardwareMap, launcher.motorName, launcher.direction);
        status.check(launcher.label + " launcher", flywheel.missingDevices());
        status.check("LEDs", leds.missingDevices());
        flywheel.reset();
        targetRPM = clampRPM(startRPM);
        openLog();

        runTimer.reset();
        startClockMs = System.currentTimeMillis();
        loopTimer.reset();
        readVoltage();
    }

    @Override
    public void loop() {
        bulkCache.clear();
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        if (gamepad1.dpadUpWasPressed()) setTarget(targetRPM + rpmStep);
        if (gamepad1.dpadDownWasPressed()) setTarget(targetRPM - rpmStep);

        if (targetRPM > 0) {
            flywheel.runToRPM(targetRPM);
        } else {
            flywheel.stop();
        }

        double nowMs = runTimer.milliseconds();
        double rpm = flywheel.filteredRPM();
        record(nowMs, rpm);
        detectLaunch(nowMs, rpm);
        leds.setState(armed && !inLaunch ? RobotState.READY : RobotState.NO_SHOT);
        leds.update();

        // Between launches only, so the reading logged is the voltage before the ball, not the sag during it
        if (!inLaunch && voltageTimer.seconds() >= VOLTAGE_READ_INTERVAL_SECONDS) readVoltage();

        // Telemetry is only built when Panels is due to send it, to keep the loop fast
        if (telemetryM.getShouldUpdateLines()) {
            showTelemetry(loopMs);
        }
    }

    @Override
    public void stop() {
        if (flywheel != null) flywheel.stop();
        // A launch still recovering when STOP is pressed is still worth keeping
        if (inLaunch) endLaunch(false);
        if (log != null) log.close();
    }

    private void setTarget(double rpm) {
        targetRPM = clampRPM(rpm);
        // The wheel has to settle at the new speed before a drop counts as a launch
        armed = false;
        armedSinceMs = -1;
        if (inLaunch) endLaunch(false);
    }

    private double clampRPM(double rpm) {
        return Range.clip(rpm, 0, maxRPM);
    }

    private void readVoltage() {
        if (voltageSensor != null) voltage = voltageSensor.getVoltage();
        voltageTimer.reset();
    }

    // ---------------- Launch detection ----------------

    private void record(double ms, double rpm) {
        historyMs[historyNext] = ms;
        historyRPM[historyNext] = rpm;
        historyNext = (historyNext + 1) % HISTORY_SIZE;
        historyCount = Math.min(historyCount + 1, HISTORY_SIZE);
    }

    private void detectLaunch(double nowMs, double rpm) {
        if (targetRPM <= 0 || flywheel.getHealth() != Flywheel.Health.OK) {
            armed = false;
            armedSinceMs = -1;
            if (inLaunch) endLaunch(false);
            return;
        }
        double drop = targetRPM * dropPercent / 100;

        if (inLaunch) {
            if (rpm < launchLowestRPM) {
                launchLowestRPM = rpm;
                peakSinceLowestRPM = rpm;
            } else {
                peakSinceLowestRPM = Math.max(peakSinceLowestRPM, rpm);
            }

            if (Math.abs(targetRPM - rpm) <= targetRPM * recoveredPercent / 100) {
                endLaunch(true);
                // Back at speed: ready for the next ball straight away
                armed = true;
                armedSinceMs = nowMs;
            } else if (peakSinceLowestRPM > launchLowestRPM + drop && rpm < peakSinceLowestRPM - drop) {
                // Recovering, then a fresh drop: another ball arrived before the wheel was back at speed
                endLaunch(false);
                beginLaunch(rpm);
            } else if (nowMs - launchStartMs > maxLaunchSeconds * 1000) {
                endLaunch(false);
            }
            return;
        }

        if (!armed) {
            if (Math.abs(targetRPM - rpm) <= targetRPM * armedPercent / 100) {
                if (armedSinceMs < 0) armedSinceMs = nowMs;
                if (nowMs - armedSinceMs >= armedSeconds * 1000) armed = true;
            } else {
                armedSinceMs = -1;
            }
            return;
        }

        // Armed: compare with the highest speed in the last moments, so a slow drift isn't a launch
        if (rpm < recentPeakRPM(Math.max(armedSinceMs, nowMs - PEAK_WINDOW_MS)) - drop) beginLaunch(rpm);
    }

    /** Highest filtered RPM since sinceMs (only since arming, so spin-up overshoot doesn't count). */
    private double recentPeakRPM(double sinceMs) {
        double peak = 0;
        for (int i = 0; i < historyCount; i++) {
            if (historyMs[i] >= sinceMs) peak = Math.max(peak, historyRPM[i]);
        }
        return peak;
    }

    /**
     * Starts a launch. The drop is only noticed once it's past dropPercent, so walk back through the
     * history to the last sample before the speed started falling: that's when the ball arrived.
     */
    private void beginLaunch(double rpm) {
        int index = (historyNext - 1 + HISTORY_SIZE) % HISTORY_SIZE;  // this loop's sample
        for (int steps = 1; steps < historyCount; steps++) {
            int previous = (index - 1 + HISTORY_SIZE) % HISTORY_SIZE;
            if (historyRPM[previous] <= historyRPM[index]) break;
            index = previous;
        }
        inLaunch = true;
        launchDetectedMs = runTimer.milliseconds();
        leds.flash(RobotState.GOOD_SHOT, flashSeconds);
        launchStartMs = historyMs[index];
        launchBeforeRPM = historyRPM[index];
        launchLowestRPM = rpm;
        peakSinceLowestRPM = rpm;
        launchTargetRPM = targetRPM;
        launchVoltage = voltage;
    }

    private void endLaunch(boolean recovered) {
        inLaunch = false;
        launches++;
        double recoveryMs = runTimer.milliseconds() - launchStartMs;
        String clock = new SimpleDateFormat("HH:mm:ss.SSS", Locale.US)
                .format(new Date(startClockMs + Math.round(launchStartMs)));

        lastLaunch = String.format(Locale.US, "#%d at %.0f ms: %.0f -> %.0f RPM (%.1f%%)%s",
                launches, launchStartMs, launchBeforeRPM, launchLowestRPM,
                100 * (launchBeforeRPM - launchLowestRPM) / launchBeforeRPM,
                recovered ? String.format(Locale.US, ", recovered in %.0f ms", recoveryMs) : "");

        if (log != null) {
            log.printf(Locale.US, "%d,%.0f,%.0f,%s,%.2f,%.0f,%.0f,%.0f,%s%n",
                    launches, launchStartMs, launchDetectedMs, clock, launchVoltage, launchTargetRPM,
                    launchBeforeRPM, launchLowestRPM,
                    recovered ? String.format(Locale.US, "%.0f", recoveryMs) : "");
            // Flushed every launch, so nothing is lost if the robot is switched off mid-run
            log.flush();
        }
    }

    // ---------------- Log file ----------------

    private void openLog() {
        String stamp = new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss", Locale.US).format(new Date());
        File file = new File(LOG_FOLDER, launcher.label + "_" + stamp + ".csv");
        try {
            if (!LOG_FOLDER.isDirectory() && !LOG_FOLDER.mkdirs()) throw new IOException("can't create " + LOG_FOLDER);
            log = new PrintWriter(new FileWriter(file));
            logName = file.getName();
            log.printf(Locale.US, "# %s launcher (\"%s\"), started %s%n", launcher.label, launcher.motorName,
                    new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS", Locale.US).format(new Date()));
            log.printf(Locale.US, "# Gains: kP %.6f kI %.7f kD %.7f kV %.10f kS %.6f, RPM filter %.3f s%n",
                    flywheel.getKP(), flywheel.getKI(), flywheel.getKD(), flywheel.getKV(), flywheel.getKS(),
                    flywheel.getRpmFilterTimeConstant());
            log.printf(Locale.US, "# Detection: drop %.1f%%, armed within %.1f%% for %.2f s, recovered within %.1f%%%n",
                    dropPercent, armedPercent, armedSeconds, recoveredPercent);
            log.println("launch,time_ms,flash_ms,clock,battery_v,target_rpm,before_rpm,lowest_rpm,recovery_ms");
            log.flush();
        } catch (IOException e) {
            log = null;
            logError = e.getMessage();
            RobotLog.ee("FlywheelCharacterization", e, "Couldn't open the launch log");
        }
    }

    // ---------------- Telemetry ----------------

    private void showTelemetry(double loopMs) {
        // Graph lines for Panels
        telemetryM.addData("targetRPM", flywheel.targetRPM());
        telemetryM.addData("filteredRPM", flywheel.filteredRPM());
        telemetryM.addData("power", flywheel.power());
        telemetryM.addData("inLaunch", inLaunch ? 1 : 0);

        status.addTelemetry(telemetry);
        telemetry.addData("Launcher", launcher.label);
        if (flywheel.getHealth() != Flywheel.Health.OK) {
            telemetry.addData("FAULT", flywheel.faultMessage() + " (restart the OpMode to retry)");
        }
        telemetry.addData("Target", "%.0f RPM (D-pad up/down)", targetRPM);
        telemetry.addData("Speed", "%.0f RPM", flywheel.filteredRPM());
        telemetry.addData("Ready for a ball", inLaunch ? "NO - recovering" : (armed ? "YES" : "not yet"));
        if (!Double.isNaN(voltage)) telemetry.addData("Battery", "%.2f V", voltage);
        telemetry.addData("Loop time", "%.0f ms", loopMs);
        telemetry.addData("Launches logged", launches);
        telemetry.addData("Last launch", lastLaunch);
        if (log != null) {
            telemetry.addData("Log", "FIRST/data/FlywheelCharacterization/" + logName);
        } else {
            telemetry.addData("Log", "NOT RECORDING: " + logError);
        }

        telemetryM.update(telemetry);
    }
}
