package org.firstinspires.ftc.teamcode.opmodes.TeleOP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.turretmotor.TurretMotor;
import org.firstinspires.ftc.teamcode.subsystems.turretmotor.TurretMotorConstants;
import org.firstinspires.ftc.teamcode.util.PosePersistency;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@Config
@TeleOp(name = "Turret Motor Auto Tuner", group = "Tuning")
public class TurretMotorAutoTuner extends LinearOpMode {
    public static double KS_RAMP_POWER_PER_S = 0.1;
    public static double KS_MOVE_VEL_DEG_S = 10;
    public static double KS_MAX_POWER = 0.4;

    public static double[] KV_POWERS = {0.15, 0.25, 0.35, 0.45};
    public static double KV_SWEEP_DEG = 120;
    public static double KV_SAMPLE_START_FRACTION = 0.4;
    public static double KV_SAMPLE_END_FRACTION = 0.9;

    public static double RELAY_POWER_ABOVE_KS = 0.1;
    public static double RELAY_HYSTERESIS_DEG = 0.5;
    public static int RELAY_CYCLES = 8;
    public static int RELAY_SKIP_CYCLES = 3;
    public static double RELAY_ABORT_DEG = 60;
    public static double RELAY_TIMEOUT_S = 15;

    public static double MOVE_KP = 1.5;
    public static double MOVE_MIN_POWER = 0.15;
    public static double MOVE_MAX_POWER = 0.4;
    public static double MOVE_TOLERANCE_DEG = 2;
    public static double MOVE_TIMEOUT_S = 4;

    public static boolean APPLY_RESULTS = true;

    private DcMotorEx motor;
    private List<LynxModule> hubs;
    private double zeroTicks;
    private String phase = "init";
    private String error = null;

    private double staticKs = Double.NaN;
    private double fitKs = Double.NaN;
    private double kV = Double.NaN;
    private double ku = Double.NaN;
    private double tu = Double.NaN;
    private double kP = Double.NaN;
    private double kI = Double.NaN;
    private double kD = Double.NaN;
    private final List<double[]> kvSamples = new ArrayList<>();

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        motor = hardwareMap.get(DcMotorEx.class, TurretMotorConstants.MOTOR_NAME);
        motor.setDirection(TurretMotorConstants.MOTOR_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        zeroTicks = PosePersistency.getTurretMotorZero(motor.getCurrentPosition());

        while (opModeInInit()) {
            read();
            if (gamepad1.y) {
                zeroTicks = motor.getCurrentPosition();
                PosePersistency.saveTurretMotorZero(zeroTicks);
            }
            telemetry.addLine("Point the turret forward and press Y to zero it here.");
            telemetry.addLine("The turret will sweep about +/-" + (int) KV_SWEEP_DEG + " deg. Keep cables and hands clear.");
            telemetry.addData("Turret deg", "%.1f", Math.toDegrees(angle()));
            telemetry.update();
        }

        try {
            tuneKs();
            tuneKv();
            tunePid();
            if (APPLY_RESULTS && error == null) apply();
            phase = error == null ? "done" : "failed";
        } finally {
            motor.setPower(0);
        }

        while (opModeIsActive()) {
            read();
            showResults();
        }
    }

    private void tuneKs() {
        double positive = rampUntilMoving(1);
        double negative = rampUntilMoving(-1);
        if (Double.isNaN(positive) || Double.isNaN(negative)) {
            fail("turret never moved during kS ramp; check MOTOR_NAME, wiring and KS_MAX_POWER");
            return;
        }
        staticKs = (positive + negative) / 2;
    }

    private double rampUntilMoving(int direction) {
        if (!moveTo(0)) return Double.NaN;
        phase = "kS ramp " + (direction > 0 ? "+" : "-");
        long start = System.nanoTime();
        while (opModeIsActive()) {
            read();
            double power = KS_RAMP_POWER_PER_S * seconds(start);
            if (power > KS_MAX_POWER || Math.abs(Math.toDegrees(angle())) > KV_SWEEP_DEG) break;
            motor.setPower(direction * power);
            showLive(direction * power);
            if (Math.abs(Math.toDegrees(velocity())) > KS_MOVE_VEL_DEG_S) {
                motor.setPower(0);
                return power;
            }
        }
        motor.setPower(0);
        return Double.NaN;
    }

    private void tuneKv() {
        if (error != null) return;
        kvSamples.clear();
        for (double power : KV_POWERS) {
            if (power <= staticKs) continue;
            for (int direction = 1; direction >= -1; direction -= 2) {
                double speed = sweep(direction, power);
                if (!opModeIsActive()) return;
                if (!Double.isNaN(speed)) kvSamples.add(new double[]{speed, power});
            }
        }
        if (kvSamples.size() < 2) {
            fail("not enough kV samples; raise KV_POWERS above kS (" + fmt(staticKs) + ")");
            return;
        }
        double n = kvSamples.size(), sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (double[] s : kvSamples) {
            sx += s[0];
            sy += s[1];
            sxx += s[0] * s[0];
            sxy += s[0] * s[1];
        }
        double denominator = n * sxx - sx * sx;
        if (Math.abs(denominator) < 1e-9) {
            fail("kV samples all had the same speed");
            return;
        }
        kV = (n * sxy - sx * sy) / denominator;
        fitKs = Math.max(0, (sy - kV * sx) / n);
    }

    private double sweep(int direction, double power) {
        double sweep = Math.toRadians(KV_SWEEP_DEG);
        if (!moveTo(-direction * sweep)) return Double.NaN;
        phase = String.format(Locale.US, "kV sweep %s%.2f", direction > 0 ? "+" : "-", power);
        double total = 2 * sweep;
        double sum = 0;
        int count = 0;
        long start = System.nanoTime();
        while (opModeIsActive() && seconds(start) < MOVE_TIMEOUT_S * 2) {
            read();
            double travelled = direction * angle() + sweep;
            if (travelled >= total) break;
            motor.setPower(direction * power);
            showLive(direction * power);
            if (travelled >= KV_SAMPLE_START_FRACTION * total && travelled <= KV_SAMPLE_END_FRACTION * total) {
                sum += Math.abs(velocity());
                count++;
            }
        }
        motor.setPower(0);
        return count > 3 ? sum / count : Double.NaN;
    }

    private void tunePid() {
        if (error != null) return;
        if (!moveTo(0)) return;
        phase = "relay";
        double ks = Double.isNaN(fitKs) ? staticKs : fitKs;
        double relay = ks + RELAY_POWER_ABOVE_KS;
        double hysteresis = Math.toRadians(RELAY_HYSTERESIS_DEG);
        int output = 1;
        List<Double> risingTimes = new ArrayList<>();
        double max = Double.NEGATIVE_INFINITY;
        double min = Double.POSITIVE_INFINITY;
        List<Double> amplitudes = new ArrayList<>();
        long start = System.nanoTime();
        while (opModeIsActive() && risingTimes.size() <= RELAY_CYCLES) {
            read();
            double t = seconds(start);
            double a = angle();
            if (Math.abs(Math.toDegrees(a)) > RELAY_ABORT_DEG || t > RELAY_TIMEOUT_S) {
                motor.setPower(0);
                fail("relay test did not settle into a steady oscillation; lower RELAY_POWER_ABOVE_KS");
                return;
            }
            if (output > 0 && a > hysteresis) {
                output = -1;
            } else if (output < 0 && a < -hysteresis) {
                output = 1;
                risingTimes.add(t);
                if (risingTimes.size() > RELAY_SKIP_CYCLES + 1) amplitudes.add((max - min) / 2);
                max = Double.NEGATIVE_INFINITY;
                min = Double.POSITIVE_INFINITY;
            }
            max = Math.max(max, a);
            min = Math.min(min, a);
            motor.setPower(output * relay);
            showLive(output * relay);
        }
        motor.setPower(0);
        if (!opModeIsActive()) return;
        int first = RELAY_SKIP_CYCLES;
        int last = risingTimes.size() - 1;
        if (last - first < 2 || amplitudes.isEmpty()) {
            fail("not enough relay cycles; raise RELAY_CYCLES");
            return;
        }
        tu = (risingTimes.get(last) - risingTimes.get(first)) / (last - first);
        double amplitude = 0;
        for (double value : amplitudes) amplitude += value;
        amplitude /= amplitudes.size();
        if (amplitude <= 0) {
            fail("relay amplitude was zero");
            return;
        }
        ku = 4 * RELAY_POWER_ABOVE_KS / (Math.PI * amplitude);
        kP = ku / 2.2;
        double ti = 2.2 * tu;
        double td = tu / 6.3;
        kI = kP / ti;
        kD = kP * td;
        moveTo(0);
    }

    private boolean moveTo(double target) {
        phase = String.format(Locale.US, "moving to %.0f deg", Math.toDegrees(target));
        long start = System.nanoTime();
        double minPower = Double.isNaN(staticKs) ? MOVE_MIN_POWER : Math.max(MOVE_MIN_POWER, staticKs);
        while (opModeIsActive()) {
            read();
            double err = target - angle();
            if (Math.abs(Math.toDegrees(err)) < MOVE_TOLERANCE_DEG && Math.abs(Math.toDegrees(velocity())) < 10) {
                motor.setPower(0);
                return true;
            }
            if (seconds(start) > MOVE_TIMEOUT_S) {
                motor.setPower(0);
                fail(String.format(Locale.US, "could not reach %.0f deg; the turret may be blocked or MOTOR_REVERSED is wrong", Math.toDegrees(target)));
                return false;
            }
            double power = Range.clip(MOVE_KP * err + Math.signum(err) * minPower, -MOVE_MAX_POWER, MOVE_MAX_POWER);
            motor.setPower(power);
            showLive(power);
        }
        motor.setPower(0);
        return false;
    }

    private void apply() {
        TurretMotorConstants.MOTOR_kS = fitKs;
        TurretMotorConstants.MOTOR_kV = kV;
        TurretMotorConstants.MOTOR_kP = kP;
        TurretMotorConstants.MOTOR_kI = kI;
        TurretMotorConstants.MOTOR_kD = kD;
    }

    private void fail(String message) {
        if (error == null) error = message;
    }

    private void read() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
    }

    private double angle() {
        return TurretMotor.ticksToAngle(motor.getCurrentPosition() - zeroTicks);
    }

    private double velocity() {
        return TurretMotor.ticksToAngle(motor.getVelocity());
    }

    private static double seconds(long startNanos) {
        return (System.nanoTime() - startNanos) / 1e9;
    }

    private void showLive(double power) {
        telemetry.addData("Phase", phase);
        telemetry.addData("Turret deg", Math.toDegrees(angle()));
        telemetry.addData("Turret vel deg/s", Math.toDegrees(velocity()));
        telemetry.addData("Power", power);
        telemetry.update();
    }

    private void showResults() {
        telemetry.addData("Phase", phase);
        if (error != null) telemetry.addData("Error", error);
        telemetry.addData("Static kS (breakaway)", fmt(staticKs));
        telemetry.addData("Kinetic kS (fit)", fmt(fitKs));
        telemetry.addData("kV (power per rad/s)", fmt(kV));
        for (double[] s : kvSamples) {
            telemetry.addData(String.format(Locale.US, "  power %.2f", s[1]), "%.0f deg/s", Math.toDegrees(s[0]));
        }
        telemetry.addData("Ku / Tu s", "%s / %s", fmt(ku), fmt(tu));
        telemetry.addData("kP", fmt(kP));
        telemetry.addData("kI", fmt(kI));
        telemetry.addData("kD", fmt(kD));
        telemetry.addLine(APPLY_RESULTS && error == null
                ? "Applied to TurretMotorConstants for this session. Copy them into the code to keep them."
                : "Not applied.");
        telemetry.update();
    }

    private static String fmt(double value) {
        return Double.isNaN(value) ? "-" : String.format(Locale.US, "%.4f", value);
    }
}
