package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLog;

import java.util.ArrayList;
import java.util.List;

/**
 * Guided, step-by-step setup for the paired turret servos. Instructions are shown on the Driver Station.
 * Paired-motion checks and encoder comparison live in {@link TurretServoVerifier}.
 */
@Configurable
@PsiKitAutoLog(rlogPort = 5802)
@TeleOp(name = "TurretServoAssemblyAligner", group = "Test")
public class TurretServoAssemblyAligner extends OpMode {

    private enum Step {
        INTRO,
        TEST_LEFT,
        TEST_RIGHT,
        PWM_PROMPT,
        PWM_TEST,
        SHOW_VALUES,
        ALIGN,
        DONE,
        FAULT
    }

    // Shared with TurretServoVerifier.
    public static double centerPos = 0.50;
    public static double targetPos = 0.50;
    public static double minPos = 0.00;
    public static double maxPos = 1.00;
    public static double stickRatePerSec = 0.35;
    public static double smallStep = 0.0025;
    public static double largeStep = 0.01;
    public static boolean invertRight = false;
    public static double rightOffset = 0.0;
    public static double rightOffsetSmallStep = 0.001;
    public static double rightOffsetLargeStep = 0.005;
    public static double pwmMinUs = 500.0;
    public static double pwmMaxUs = 2500.0;

    public static double pwmTestStepUs = 50.0;
    public static double pwmTestLowStartUs = 850.0;
    public static double pwmTestHighStartUs = 2150.0;

    public static double currentLimitAmps = 8.0;
    public static double currentTripMs = 500.0;
    public static double currentStopMs = 1500.0;

    private static final double FLOODGATE_MAX_AMPS = 80.0;
    private static final double ADC_REF_VOLTS = 3.3;

    private Servo turretL;
    private Servo turretR;
    private AnalogInput floodgateAnalog;

    private final Gamepad current = new Gamepad();
    private final Gamepad previous = new Gamepad();

    private Step step = Step.INTRO;
    private boolean leftEnabled = false;
    private boolean rightEnabled = false;
    private double leftCmd = 0.5;
    private double rightCmd = 0.5;
    private double testPos = 0.5;

    private List<Double> pwmSequence = new ArrayList<>();
    private int pwmIndex = 0;
    private boolean pwmTestingLeft = true;

    private double floodgateCurrent = Double.NaN;
    private long overSinceNs = 0;
    private long lastLoopNs;

    @Override
    public void init() {
        turretL = hardwareMap.get(Servo.class, "turretL");
        turretR = hardwareMap.get(Servo.class, "turretR");
        applyPwmRange(turretL);
        applyPwmRange(turretR);

        turretL.setDirection(Servo.Direction.REVERSE);

        writeServo(turretL, false, 0.5);
        writeServo(turretR, false, 0.5);

        floodgateAnalog = hardwareMap.get(AnalogInput.class, "floodgate");
        floodgateCurrent = floodgateAmps(floodgateAnalog);
        lastLoopNs = System.nanoTime();
        targetPos = clampToSharedRange(targetPos);
    }

    @Override
    public void loop() {
        previous.copy(current);
        current.copy(gamepad1);
        floodgateCurrent = floodgateAmps(floodgateAnalog);

        long nowNs = System.nanoTime();
        double dtSec = Math.max(0.0, (nowNs - lastLoopNs) / 1e9);
        lastLoopNs = nowNs;

        updateCurrentProtection(nowNs);

        if (step != Step.FAULT && step != Step.DONE
                && edge(current.right_stick_button, previous.right_stick_button)) {
            enterStep(Step.DONE);
        }

        leftEnabled = false;
        rightEnabled = false;

        switch (step) {
            case INTRO:
                if (edge(current.a, previous.a)) enterStep(Step.TEST_LEFT);
                break;

            case TEST_LEFT:
                testPos = stickAdjusted(testPos, dtSec);
                leftEnabled = true;
                leftCmd = testPos;
                if (edge(current.a, previous.a)) enterStep(Step.TEST_RIGHT);
                break;

            case TEST_RIGHT:
                testPos = stickAdjusted(testPos, dtSec);
                rightEnabled = true;
                rightCmd = testPos;
                if (edge(current.a, previous.a)) enterStep(Step.PWM_PROMPT);
                break;

            case PWM_PROMPT:
                if (edge(current.a, previous.a)) {
                    startPwmTest(true);
                } else if (edge(current.b, previous.b)) {
                    enterStep(Step.SHOW_VALUES);
                }
                break;

            case PWM_TEST:
                runPwmTest();
                break;

            case SHOW_VALUES:
                if (edge(current.a, previous.a)) {
                    targetPos = clampToSharedRange(centerPos);
                    enterStep(Step.ALIGN);
                }
                break;

            case ALIGN:
                runAlign(dtSec);
                break;

            case DONE:
            case FAULT:
                break;
        }

        writeServo(turretL, leftEnabled, leftCmd);
        writeServo(turretR, rightEnabled, rightCmd);

        showTelemetry();
    }

    @Override
    public void stop() {
        writeServo(turretL, false, leftCmd);
        writeServo(turretR, false, rightCmd);
    }

    private void enterStep(Step next) {
        step = next;
        testPos = clamp(centerPos, minPos, maxPos);
    }

    private double stickAdjusted(double pos, double dtSec) {
        return clamp(pos + (-current.left_stick_y) * stickRatePerSec * dtSec, minPos, maxPos);
    }

    private void startPwmTest(boolean left) {
        enterStep(Step.PWM_TEST);
        pwmTestingLeft = left;
        pwmSequence = buildPwmSequence();
        pwmIndex = 0;
    }

    // Sequence per servo: center, low end stepping down to pwmMinUs, center, high end stepping up to pwmMaxUs, center.
    private List<Double> buildPwmSequence() {
        double stepUs = Math.max(1.0, pwmTestStepUs);
        double center = (pwmMinUs + pwmMaxUs) / 2.0;
        List<Double> seq = new ArrayList<>();
        seq.add(center);
        for (double us = pwmTestLowStartUs; us > pwmMinUs; us -= stepUs) seq.add(us);
        seq.add(pwmMinUs);
        seq.add(center);
        for (double us = pwmTestHighStartUs; us < pwmMaxUs; us += stepUs) seq.add(us);
        seq.add(pwmMaxUs);
        seq.add(center);
        return seq;
    }

    private void runPwmTest() {
        if (edge(current.b, previous.b)) {
            enterStep(Step.SHOW_VALUES);
            return;
        }
        if (edge(current.a, previous.a)) {
            pwmIndex++;
            if (pwmIndex >= pwmSequence.size()) {
                if (pwmTestingLeft) {
                    startPwmTest(false);
                } else {
                    enterStep(Step.SHOW_VALUES);
                }
                return;
            }
        }

        double pos = usToPos(pwmSequence.get(pwmIndex));
        if (pwmTestingLeft) {
            leftEnabled = true;
            leftCmd = pos;
        } else {
            rightEnabled = true;
            rightCmd = pos;
        }
    }

    private void runAlign(double dtSec) {
        boolean rtNow = current.right_trigger > 0.6;
        boolean rtPrev = previous.right_trigger > 0.6;
        boolean ltNow = current.left_trigger > 0.6;
        boolean ltPrev = previous.left_trigger > 0.6;

        if (edge(current.b, previous.b)) {
            enterStep(Step.DONE);
            return;
        }

        targetPos += (-current.left_stick_y) * stickRatePerSec * dtSec;
        if (edge(current.dpad_right, previous.dpad_right)) targetPos += smallStep;
        if (edge(current.dpad_left, previous.dpad_left)) targetPos -= smallStep;
        if (edge(current.right_bumper, previous.right_bumper)) targetPos += largeStep;
        if (edge(current.left_bumper, previous.left_bumper)) targetPos -= largeStep;
        if (edge(current.a, previous.a)) targetPos = centerPos;

        if (edge(current.dpad_up, previous.dpad_up)) rightOffset += rightOffsetSmallStep;
        if (edge(current.dpad_down, previous.dpad_down)) rightOffset -= rightOffsetSmallStep;
        if (edge(rtNow, rtPrev)) rightOffset += rightOffsetLargeStep;
        if (edge(ltNow, ltPrev)) rightOffset -= rightOffsetLargeStep;

        rightOffset = clamp(rightOffset, rightOffsetMin(), rightOffsetMax());
        targetPos = clampToSharedRange(targetPos);

        leftEnabled = true;
        rightEnabled = true;
        leftCmd = leftCmdFor(targetPos);
        rightCmd = rightCmdFor(targetPos);
    }

    // Disables the servos on sustained high current, then stops the OpMode if the current stays high.
    private void updateCurrentProtection(long nowNs) {
        if (!(floodgateCurrent > currentLimitAmps)) {
            overSinceNs = 0;
            return;
        }
        if (overSinceNs == 0) overSinceNs = nowNs;

        double overMs = (nowNs - overSinceNs) / 1e6;
        if (step != Step.FAULT && overMs >= currentTripMs) {
            enterStep(Step.FAULT);
        } else if (step == Step.FAULT && overMs >= currentTripMs + currentStopMs) {
            requestOpModeStop();
        }
    }

    private void showTelemetry() {
        boolean overLimit = floodgateCurrent > currentLimitAmps;
        telemetry.addData("Floodgate Current (A)", "%.2f (limit %.1f)%s",
                floodgateCurrent, currentLimitAmps, overLimit ? "  ** HIGH **" : "");

        Logger.recordOutput("FloodgateAmps", floodgateCurrent);
        Logger.recordOutput("TurretAligner/FloodgateAmps", floodgateCurrent);
        Logger.recordOutput("TurretAligner/Step", step.ordinal());
        Logger.recordOutput("TurretAligner/LeftEnabled", leftEnabled ? 1.0 : 0.0);
        Logger.recordOutput("TurretAligner/RightEnabled", rightEnabled ? 1.0 : 0.0);
        Logger.recordOutput("TurretAligner/TargetPos", targetPos);
        Logger.recordOutput("TurretAligner/LeftCmd", leftCmd);
        Logger.recordOutput("TurretAligner/RightCmd", rightCmd);
        Logger.recordOutput("TurretAligner/RightOffset", rightOffset);

        switch (step) {
            case INTRO:
                telemetry.addLine("STEP 1/6: Start");
                telemetry.addLine("Both servos are UNPOWERED. Support the turret and keep hands clear.");
                telemetry.addLine("Press A to begin.");
                break;

            case TEST_LEFT:
                telemetry.addLine("STEP 2/6: Test turretL");
                telemetry.addLine("turretL is now POWERED. Push the left stick up/down.");
                telemetry.addLine("Verify turretL moves smoothly in both directions.");
                telemetry.addData("turretL cmd", "%.4f  (%s)", leftCmd, pwmText(leftCmd));
                telemetry.addLine("Press A when turretL works (it will be unpowered).");
                break;

            case TEST_RIGHT:
                telemetry.addLine("STEP 3/6: Test turretR");
                telemetry.addLine("turretR is now POWERED. Push the left stick up/down.");
                telemetry.addLine("Verify turretR moves smoothly in both directions.");
                telemetry.addData("turretR cmd", "%.4f  (%s)", rightCmd, pwmText(rightCmd));
                telemetry.addLine("Press A when turretR works (it will be unpowered).");
                break;

            case PWM_PROMPT:
                telemetry.addLine("STEP 4/6: PWM limits (optional)");
                telemetry.addLine("Test/verify the limits of each servo's PWM range?");
                telemetry.addLine("The servo will be driven near its extremes: unload it first.");
                telemetry.addLine("A = yes, B = skip");
                break;

            case PWM_TEST:
                showPwmTestTelemetry();
                break;

            case SHOW_VALUES:
                telemetry.addLine("STEP 5/6: Review values");
                showValues();
                telemetry.addLine("Change values in Panels/configurables or in the source if needed.");
                telemetry.addLine("Press A to POWER BOTH servos at centerPos.");
                break;

            case ALIGN:
                telemetry.addLine("STEP 6/6: Align and mesh");
                showValues();
                telemetry.addLine("1. A moves both servos to centerPos (they are powered now).");
                telemetry.addLine("2. Fit/mesh the gear while the servos hold this position.");
                telemetry.addLine("3. Gear won't mesh? Nudge rightOffset: Dpad Up/Down = fine, triggers = coarse.");
                telemetry.addLine("   (Stick, Dpad L/R, bumpers move targetPos.)");
                telemetry.addLine("4. When done, note rightOffset and press B to unpower.");
                break;

            case DONE:
                telemetry.addLine("DONE: servos are UNPOWERED.");
                showValues();
                telemetry.addLine("Copy rightOffset into the source if it should persist, then stop the OpMode.");
                break;

            case FAULT:
                telemetry.addLine("HIGH CURRENT FAULT: servos disabled.");
                telemetry.addLine("Find the cause (binding gears?), then restart the OpMode.");
                telemetry.addLine("OpMode stops automatically if current stays high.");
                break;
        }

        telemetry.update();
    }

    private void showPwmTestTelemetry() {
        String servoName = pwmTestingLeft ? "turretL" : "turretR";
        double us = pwmSequence.get(Math.min(pwmIndex, pwmSequence.size() - 1));
        telemetry.addLine("STEP 4/6: PWM limits - " + servoName);
        telemetry.addData("Pulse", "%.0f us  (%d/%d)", us, pwmIndex + 1, pwmSequence.size());
        telemetry.addLine("Watch for buzzing, straining, or high current.");
        telemetry.addLine("A = next pulse, B = skip remaining PWM tests");
    }

    private void showValues() {
        telemetry.addData("centerPos", "%.4f", centerPos);
        telemetry.addData("targetPos", "%.4f", targetPos);
        telemetry.addData("rightOffset", "%.4f  (range %.4f .. %.4f)", rightOffset, rightOffsetMin(), rightOffsetMax());
        telemetry.addData("invertRight", invertRight);
        telemetry.addData("minPos / maxPos", "%.2f / %.2f", minPos, maxPos);
        telemetry.addData("LeftCmd", "%.4f  (%s)", leftCmdFor(targetPos), pwmText(leftCmdFor(targetPos)));
        telemetry.addData("RightCmd", "%.4f  (%s)", rightCmdFor(targetPos), pwmText(rightCmdFor(targetPos)));
    }

    private static String pwmText(double pos) {
        double us = pwmMinUs + clamp(pos, 0.0, 1.0) * (pwmMaxUs - pwmMinUs);
        return String.format("%.0f us", us);
    }

    private static double usToPos(double us) {
        return clamp((us - pwmMinUs) / Math.max(1.0, pwmMaxUs - pwmMinUs), 0.0, 1.0);
    }

    static double floodgateAmps(AnalogInput input) {
        return input.getVoltage() / ADC_REF_VOLTS * FLOODGATE_MAX_AMPS;
    }

    static double leftCmdFor(double target) {
        return clamp(target, minPos, maxPos);
    }

    static double rightCmdFor(double target) {
        double base = invertRight ? (1.0 - target) : target;
        return clamp(base + rightOffset, minPos, maxPos);
    }

    static double clampToSharedRange(double requested) {
        double sharedMin = Math.max(minPos, invertRight ? (1.0 - maxPos) + rightOffset : minPos - rightOffset);
        double sharedMax = Math.min(maxPos, invertRight ? (1.0 - minPos) + rightOffset : maxPos - rightOffset);
        return clamp(requested, sharedMin, sharedMax);
    }

    static double rightOffsetMin() {
        double lo = Math.min(minPos, maxPos);
        double hi = Math.max(minPos, maxPos);
        return invertRight ? (2.0 * lo) - 1.0 : -(hi - lo);
    }

    static double rightOffsetMax() {
        double lo = Math.min(minPos, maxPos);
        double hi = Math.max(minPos, maxPos);
        return invertRight ? (2.0 * hi) - 1.0 : (hi - lo);
    }

    static void writeServo(Servo servo, boolean enabled, double position) {
        if (servo == null) return;

        if (servo instanceof PwmControl) {
            if (enabled) {
                ((PwmControl) servo).setPwmEnable();
            } else {
                ((PwmControl) servo).setPwmDisable();
                return;
            }
        } else if (!enabled) {
            return;
        }

        servo.setPosition(position);
    }

    static void applyPwmRange(Servo servo) {
        if (!(servo instanceof PwmControl)) return;
        double lo = Math.min(pwmMinUs, pwmMaxUs);
        double hi = Math.max(pwmMinUs, pwmMaxUs);
        ((PwmControl) servo).setPwmRange(new PwmControl.PwmRange(lo, hi));
    }

    static boolean edge(boolean now, boolean prevVal) {
        return now && !prevVal;
    }

    static double clamp(double value, double lo, double hi) {
        return Math.max(lo, Math.min(value, hi));
    }
}
