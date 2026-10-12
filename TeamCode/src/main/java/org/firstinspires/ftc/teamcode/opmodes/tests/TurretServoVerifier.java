package org.firstinspires.ftc.teamcode.opmodes.tests;

import static org.firstinspires.ftc.teamcode.opmodes.tests.TurretServoAssemblyAligner.applyPwmRange;
import static org.firstinspires.ftc.teamcode.opmodes.tests.TurretServoAssemblyAligner.centerPos;
import static org.firstinspires.ftc.teamcode.opmodes.tests.TurretServoAssemblyAligner.clamp;
import static org.firstinspires.ftc.teamcode.opmodes.tests.TurretServoAssemblyAligner.clampToSharedRange;
import static org.firstinspires.ftc.teamcode.opmodes.tests.TurretServoAssemblyAligner.edge;
import static org.firstinspires.ftc.teamcode.opmodes.tests.TurretServoAssemblyAligner.floodgateAmps;
import static org.firstinspires.ftc.teamcode.opmodes.tests.TurretServoAssemblyAligner.largeStep;
import static org.firstinspires.ftc.teamcode.opmodes.tests.TurretServoAssemblyAligner.leftCmdFor;
import static org.firstinspires.ftc.teamcode.opmodes.tests.TurretServoAssemblyAligner.rightCmdFor;
import static org.firstinspires.ftc.teamcode.opmodes.tests.TurretServoAssemblyAligner.rightOffset;
import static org.firstinspires.ftc.teamcode.opmodes.tests.TurretServoAssemblyAligner.rightOffsetLargeStep;
import static org.firstinspires.ftc.teamcode.opmodes.tests.TurretServoAssemblyAligner.rightOffsetMax;
import static org.firstinspires.ftc.teamcode.opmodes.tests.TurretServoAssemblyAligner.rightOffsetMin;
import static org.firstinspires.ftc.teamcode.opmodes.tests.TurretServoAssemblyAligner.rightOffsetSmallStep;
import static org.firstinspires.ftc.teamcode.opmodes.tests.TurretServoAssemblyAligner.smallStep;
import static org.firstinspires.ftc.teamcode.opmodes.tests.TurretServoAssemblyAligner.stickRatePerSec;
import static org.firstinspires.ftc.teamcode.opmodes.tests.TurretServoAssemblyAligner.writeServo;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.AbsoluteAnalogEncoder;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

/**
 * Checks the paired turret servos after assembly (VERIFY) and compares their encoder response (CHARACTERIZE).
 * Initial alignment is done in {@link TurretServoAssemblyAligner}, whose centerPos/rightOffset/invertRight
 * settings this op mode shares.
 */
@Configurable
@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name = "TurretServoVerifier", group = "Test")
public class TurretServoVerifier extends OpMode {

    public enum Mode {
        VERIFY,
        CHARACTERIZE
    }

    public enum CharacterizeServo {
        LEFT,
        RIGHT
    }

    public static double verifySweepMin = 0.35;
    public static double verifySweepMax = 0.65;
    public static double verifySweepRatePerSec = 0.20;

    public static int maxCharacterizeSamples = 20;
    public static double compareCommandTolerance = 0.01;

    private Servo turretL;
    private Servo turretR;
    private AbsoluteAnalogEncoder turretEnc;
    private AnalogInput floodgateAnalog;
    private double floodgateCurrent = Double.NaN;

    private final Gamepad current = new Gamepad();
    private final Gamepad previous = new Gamepad();

    private double targetPos = 0.50;
    private Mode selectedMode = Mode.VERIFY;
    private boolean modeRunning = false;
    private boolean leftEnabled = false;
    private boolean rightEnabled = false;
    private boolean verifyAutoSweep = false;
    private double verifySweepDirection = 1.0;
    private CharacterizeServo characterizeServo = CharacterizeServo.LEFT;

    private final double[] leftCmdSamples = new double[64];
    private final double[] leftEncSamples = new double[64];
    private int leftSampleCount = 0;
    private final double[] rightCmdSamples = new double[64];
    private final double[] rightEncSamples = new double[64];
    private int rightSampleCount = 0;

    private int compareMatchedCount = 0;
    private double compareAvgSignedEncErr = Double.NaN;
    private double compareAvgAbsEncErr = Double.NaN;
    private double compareMaxAbsEncErr = Double.NaN;
    private double suggestedRightOffsetDelta = Double.NaN;
    private double leftDegPerCommand = Double.NaN;
    private double rightDegPerCommand = Double.NaN;

    private long lastLoopNs;

    @Override
    public void init() {
        turretL = hardwareMap.get(Servo.class, "turretL");
        turretR = hardwareMap.get(Servo.class, "turretR");
        applyPwmRange(turretL);
        applyPwmRange(turretR);
        writeServo(turretL, false, 0.5);
        writeServo(turretR, false, 0.5);

        try {
            AnalogInput analog = hardwareMap.get(AnalogInput.class, "turretAnalog");
            turretEnc = new AbsoluteAnalogEncoder(analog, 3.3, 0, 1);
        } catch (Exception ignored) {
            turretEnc = null;
        }
        floodgateAnalog = hardwareMap.get(AnalogInput.class, "floodgate");
        floodgateCurrent = floodgateAmps(floodgateAnalog);

        lastLoopNs = System.nanoTime();
        targetPos = clampToSharedRange(centerPos);
    }

    @Override
    public void loop() {
        previous.copy(current);
        current.copy(gamepad1);
        floodgateCurrent = floodgateAmps(floodgateAnalog);

        long nowNs = System.nanoTime();
        double dtSec = Math.max(0.0, (nowNs - lastLoopNs) / 1e9);
        lastLoopNs = nowNs;

        if (!modeRunning && edge(current.start, previous.start)) {
            selectedMode = (selectedMode == Mode.VERIFY) ? Mode.CHARACTERIZE : Mode.VERIFY;
            verifyAutoSweep = false;
        }

        boolean startedThisLoop = false;
        if (!modeRunning && edge(current.a, previous.a)) {
            modeRunning = true;
            startedThisLoop = true;
        }

        if (edge(current.right_stick_button, previous.right_stick_button)) {
            modeRunning = false;
        }

        if (modeRunning) {
            boolean rtNow = current.right_trigger > 0.6;
            boolean rtPrev = previous.right_trigger > 0.6;
            boolean ltNow = current.left_trigger > 0.6;
            boolean ltPrev = previous.left_trigger > 0.6;

            if (edge(current.back, previous.back)) targetPos = centerPos;
            if (edge(current.dpad_up, previous.dpad_up)) rightOffset += rightOffsetSmallStep;
            if (edge(current.dpad_down, previous.dpad_down)) rightOffset -= rightOffsetSmallStep;
            if (edge(rtNow, rtPrev)) rightOffset += rightOffsetLargeStep;
            if (edge(ltNow, ltPrev)) rightOffset -= rightOffsetLargeStep;

            rightOffset = clamp(rightOffset, rightOffsetMin(), rightOffsetMax());
            targetPos = clampToSharedRange(targetPos);
        }

        if (modeRunning && !startedThisLoop) {
            if (selectedMode == Mode.VERIFY) {
                runVerifyMode(dtSec);
            } else {
                runCharacterizeMode(dtSec);
            }
        } else {
            leftEnabled = false;
            rightEnabled = false;
            verifyAutoSweep = false;
        }

        writeServo(turretL, leftEnabled, leftCmdFor(targetPos));
        writeServo(turretR, rightEnabled, rightCmdFor(targetPos));

        showTelemetry();
    }

    @Override
    public void stop() {
        writeServo(turretL, false, leftCmdFor(targetPos));
        writeServo(turretR, false, rightCmdFor(targetPos));
    }

    private void adjustTargetManually(double dtSec) {
        targetPos += (-current.left_stick_y) * stickRatePerSec * dtSec;
        if (edge(current.dpad_right, previous.dpad_right)) targetPos += smallStep;
        if (edge(current.dpad_left, previous.dpad_left)) targetPos -= smallStep;
        if (edge(current.right_bumper, previous.right_bumper)) targetPos += largeStep;
        if (edge(current.left_bumper, previous.left_bumper)) targetPos -= largeStep;
    }

    private void runVerifyMode(double dtSec) {
        leftEnabled = true;
        rightEnabled = true;

        if (edge(current.x, previous.x)) verifyAutoSweep = !verifyAutoSweep;
        if (edge(current.y, previous.y)) verifySweepDirection *= -1.0;
        if (edge(current.a, previous.a)) {
            verifyAutoSweep = false;
            targetPos = centerPos;
        }

        if (verifyAutoSweep) {
            targetPos += verifySweepDirection * verifySweepRatePerSec * dtSec;
            double lo = Math.min(verifySweepMin, verifySweepMax);
            double hi = Math.max(verifySweepMin, verifySweepMax);

            if (targetPos >= hi) {
                targetPos = hi;
                verifySweepDirection = -1.0;
            } else if (targetPos <= lo) {
                targetPos = lo;
                verifySweepDirection = 1.0;
            }
        } else {
            adjustTargetManually(dtSec);
        }

        targetPos = clampToSharedRange(targetPos);
    }

    private void runCharacterizeMode(double dtSec) {
        if (edge(current.y, previous.y)) {
            characterizeServo = (characterizeServo == CharacterizeServo.LEFT)
                    ? CharacterizeServo.RIGHT
                    : CharacterizeServo.LEFT;
        }

        leftEnabled = characterizeServo == CharacterizeServo.LEFT;
        rightEnabled = characterizeServo == CharacterizeServo.RIGHT;

        adjustTargetManually(dtSec);
        targetPos = clampToSharedRange(targetPos);

        if (edge(current.a, previous.a)) captureSample();

        if (edge(current.b, previous.b)) {
            clearActiveSamples();
            resetCompareStats();
        }

        if (edge(current.x, previous.x)) computeCompareStats();
    }

    private void captureSample() {
        if (turretEnc == null) return;

        int cap = Math.max(1, Math.min(maxCharacterizeSamples, leftCmdSamples.length));
        maxCharacterizeSamples = cap;
        double encoderDeg = turretEnc.getCurrentPosition();
        if (characterizeServo == CharacterizeServo.LEFT) {
            if (leftSampleCount >= cap) return;
            leftCmdSamples[leftSampleCount] = leftCmdFor(targetPos);
            leftEncSamples[leftSampleCount] = encoderDeg;
            leftSampleCount++;
        } else {
            if (rightSampleCount >= cap) return;
            rightCmdSamples[rightSampleCount] = rightCmdFor(targetPos);
            rightEncSamples[rightSampleCount] = encoderDeg;
            rightSampleCount++;
        }
    }

    private void clearActiveSamples() {
        if (characterizeServo == CharacterizeServo.LEFT) {
            leftSampleCount = 0;
        } else {
            rightSampleCount = 0;
        }
    }

    private void resetCompareStats() {
        compareMatchedCount = 0;
        compareAvgSignedEncErr = Double.NaN;
        compareAvgAbsEncErr = Double.NaN;
        compareMaxAbsEncErr = Double.NaN;
        suggestedRightOffsetDelta = Double.NaN;
        leftDegPerCommand = Double.NaN;
        rightDegPerCommand = Double.NaN;
    }

    private void computeCompareStats() {
        resetCompareStats();
        if (leftSampleCount == 0 || rightSampleCount == 0) return;

        double tolerance = Math.max(0.0, compareCommandTolerance);
        double sumSigned = 0.0;
        double sumAbs = 0.0;
        double maxAbs = 0.0;

        for (int li = 0; li < leftSampleCount; li++) {
            int bestRi = -1;
            double bestCmdDelta = Double.POSITIVE_INFINITY;

            for (int ri = 0; ri < rightSampleCount; ri++) {
                double cmdDelta = Math.abs(leftCmdSamples[li] - rightCmdSamples[ri]);
                if (cmdDelta <= tolerance && cmdDelta < bestCmdDelta) {
                    bestCmdDelta = cmdDelta;
                    bestRi = ri;
                }
            }

            if (bestRi >= 0) {
                double encErr = wrapSignedDeg(leftEncSamples[li] - rightEncSamples[bestRi]);
                double absErr = Math.abs(encErr);
                sumSigned += encErr;
                sumAbs += absErr;
                maxAbs = Math.max(maxAbs, absErr);
                compareMatchedCount++;
            }
        }

        if (compareMatchedCount > 0) {
            compareAvgSignedEncErr = sumSigned / compareMatchedCount;
            compareAvgAbsEncErr = sumAbs / compareMatchedCount;
            compareMaxAbsEncErr = maxAbs;

            leftDegPerCommand = estimateSlopeDegPerPos(leftCmdSamples, leftEncSamples, leftSampleCount);
            rightDegPerCommand = estimateSlopeDegPerPos(rightCmdSamples, rightEncSamples, rightSampleCount);

            if (!Double.isNaN(rightDegPerCommand) && Math.abs(rightDegPerCommand) > 1e-6) {
                suggestedRightOffsetDelta = compareAvgSignedEncErr / rightDegPerCommand;
            }
        }
    }

    private static double estimateSlopeDegPerPos(double[] cmdSamples, double[] encSamples, int count) {
        if (count < 2) return Double.NaN;

        double sumX = 0.0;
        double sumY = 0.0;
        for (int i = 0; i < count; i++) {
            sumX += cmdSamples[i];
            sumY += encSamples[i];
        }
        double meanX = sumX / count;
        double meanY = sumY / count;

        double sxx = 0.0;
        double sxy = 0.0;
        for (int i = 0; i < count; i++) {
            double dx = cmdSamples[i] - meanX;
            double dy = encSamples[i] - meanY;
            sxx += dx * dx;
            sxy += dx * dy;
        }

        if (Math.abs(sxx) < 1e-9) return Double.NaN;
        return sxy / sxx;
    }

    private void showTelemetry() {
        double leftCmd = leftCmdFor(targetPos);
        double rightCmd = rightCmdFor(targetPos);
        double encoderDeg = (turretEnc != null) ? turretEnc.getCurrentPosition() : Double.NaN;

        Logger.recordOutput("FloodgateAmps", floodgateCurrent);
        Logger.recordOutput("TurretVerify/FloodgateAmps", floodgateCurrent);
        Logger.recordOutput("TurretVerify/ModeSelected", selectedMode.ordinal());
        Logger.recordOutput("TurretVerify/ModeRunning", modeRunning ? 1.0 : 0.0);
        Logger.recordOutput("TurretVerify/TargetPos", targetPos);
        Logger.recordOutput("TurretVerify/LeftCmd", leftCmd);
        Logger.recordOutput("TurretVerify/RightCmd", rightCmd);
        Logger.recordOutput("TurretVerify/RightOffset", rightOffset);
        Logger.recordOutput("TurretVerify/EncoderDeg", encoderDeg);
        Logger.recordOutput("TurretVerify/CompareAvgSignedEncErr", compareAvgSignedEncErr);
        Logger.recordOutput("TurretVerify/SuggestedRightOffsetDelta", suggestedRightOffsetDelta);

        telemetry.addData("Floodgate Current (A)", "%.2f", floodgateCurrent);
        telemetry.addData("Selected Mode", selectedMode);
        telemetry.addData("Mode Running", modeRunning);
        telemetry.addData("TargetPos", "%.4f", targetPos);
        telemetry.addData("LeftCmd / RightCmd", "%.4f / %.4f", leftCmd, rightCmd);
        telemetry.addData("Left / Right Enabled", "%s / %s", leftEnabled, rightEnabled);
        telemetry.addData("Right Offset", "%.4f", rightOffset);
        telemetry.addData("Turret Enc Deg", turretEnc != null ? String.format("%.2f", encoderDeg) : "not available");

        telemetry.addLine("START: switch mode (idle only), A: start mode, RS: stop mode");

        if (!modeRunning) {
            telemetry.addLine("IDLE: outputs disabled so turret will not move.");
        } else if (selectedMode == Mode.VERIFY) {
            telemetry.addData("AutoSweep", verifyAutoSweep);
            telemetry.addLine("VERIFY: both servos enabled and moving together.");
            telemetry.addLine("Stick/Dpad L-R/bumpers move target, BACK center");
            telemetry.addLine("X toggle autosweep, Y reverse sweep, A center/stop sweep");
            telemetry.addLine("Dpad Up/Down: rightOffset +/- small, RT/LT: +/- large");
            telemetry.addLine("If gears chatter, stop and adjust invertRight/offsets/mechanical mesh.");
        } else {
            telemetry.addData("Characterize Servo", characterizeServo);
            telemetry.addData("Left / Right Samples", "%d / %d", leftSampleCount, rightSampleCount);
            telemetry.addData("Matched Points", compareMatchedCount);
            telemetry.addData("Avg Enc Delta Signed (L-R)", fmt(compareAvgSignedEncErr));
            telemetry.addData("Avg |Enc Delta| Deg", fmt(compareAvgAbsEncErr));
            telemetry.addData("Max |Enc Delta| Deg", fmt(compareMaxAbsEncErr));
            telemetry.addData("Suggested rightOffset Δ", fmt(suggestedRightOffsetDelta));
            if (!Double.isNaN(suggestedRightOffsetDelta)) {
                telemetry.addData("Suggested rightOffset", fmt(rightOffset + suggestedRightOffsetDelta));
            }
            telemetry.addData("Left / Right deg per cmd", "%s / %s", fmt(leftDegPerCommand), fmt(rightDegPerCommand));

            telemetry.addLine("CHAR: Y swap active servo (other PWM disabled)");
            telemetry.addLine("Stick/Dpad L-R/bumpers move target, A capture point, B clear active set");
            telemetry.addLine("Dpad Up/Down: rightOffset +/- small, RT/LT: +/- large");
            telemetry.addLine("X compare left-vs-right datasets, BACK center target");
        }

        telemetry.update();
    }

    private static double wrapSignedDeg(double degrees) {
        return ((degrees + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    }

    private static String fmt(double value) {
        if (Double.isNaN(value)) return "n/a";
        return String.format("%.3f", value);
    }
}
