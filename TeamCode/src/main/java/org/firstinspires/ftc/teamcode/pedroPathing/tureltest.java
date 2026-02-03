package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Turret AprilTag PD + FireLock", group = "Test")
public class tureltest extends LinearOpMode {

    /* ================= HARDWARE ================= */

    private HuskyLens huskyLens;
    private DcMotorEx turret;

    /* ================= CAMERA ================= */

    private static final int IMAGE_WIDTH = 320;
    private static final int CENTER_X = IMAGE_WIDTH / 2;

    /* ================= PD TUNING ================= */

    private static final double KP = 0.003;
    private static final double KD = 0.0015;

    private static final double MAX_POWER = 0.4;
    private static final double MIN_POWER = 0.12;

    private static final int DEADZONE = 6;

    /* ================= LOCK ================= */

    private static final int LOCK_CYCLES = 5;
    private boolean targetLocked = false;
    private int lockCounter = 0;

    /* ================= SAFETY ================= */

    private static final int TURRET_MIN = -900;
    private static final int TURRET_MAX = 900;

    /* ================= PD STATE ================= */

    private int lastError = 0;
    private ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        /* ---------- INIT ---------- */

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        turret = hardwareMap.get(DcMotorEx.class, "turret");


        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setPower(0);


        if (!huskyLens.knock()) {
            telemetry.addLine("❌ HuskyLens NOT connected");
            telemetry.update();
            sleep(2000);
            return;
        }

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        telemetry.addLine("✅ PD tracking ready");
        telemetry.addLine("🔫 Shooter only when LOCKED");
        telemetry.update();

        waitForStart();
        loopTimer.reset();

        /* ---------- LOOP ---------- */

        while (opModeIsActive()) {

            idle();

            HuskyLens.Block[] tags = huskyLens.blocks();

            if (tags.length > 0) {
                trackTagPD(tags[0]);
            } else {
                resetLock();
                turret.setPower(0);
            }

            telemetry.addData("Locked", targetLocked);
            telemetry.addData("Turret Pos", turret.getCurrentPosition());
            telemetry.update();
        }
    }

    /* ================= PD TRACKING ================= */

    private void trackTagPD(HuskyLens.Block tag) {

        int error = tag.x - CENTER_X;
        int derivative = error - lastError;
        lastError = error;

        telemetry.addData("Tag X", tag.x);
        telemetry.addData("Error", error);
        telemetry.addData("Derivative", derivative);

        /* ----- LOCK LOGIC ----- */

        if (Math.abs(error) <= DEADZONE) {
            lockCounter++;

            if (lockCounter >= LOCK_CYCLES) {
                targetLocked = true;
                turret.setPower(0);
                return;
            }
        } else {
            resetLock();
        }

        /* ----- PD CONTROL ----- */

        double power = -(KP * error + KD * derivative);
        power = clip(power, MIN_POWER, MAX_POWER);

        applySafePower(power);

        /* ----- SHOOTER LOCK ----- */

        if (!targetLocked) {
        }
    }

    /* ================= SAFETY ================= */

    private void applySafePower(double power) {

        int pos = turret.getCurrentPosition();

        if ((pos <= TURRET_MIN && power < 0) ||
                (pos >= TURRET_MAX && power > 0)) {
            turret.setPower(0);
            return;
        }

        turret.setPower(power);
    }

    /* ================= LOCK RESET ================= */

    private void resetLock() {
        targetLocked = false;
        lockCounter = 0;
        lastError = 0;
    }

    /* ================= UTILS ================= */

    private double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, Math.abs(value))) * Math.signum(value);
    }
}