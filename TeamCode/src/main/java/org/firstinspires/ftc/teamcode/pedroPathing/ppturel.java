package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Turret Pinpoint Basic", group = "Test")
public class ppturel extends LinearOpMode {

    /* ================= HARDWARE ================= */

    private DcMotorEx turret;

    /* ================= PINPOINT ================= */

    // Заглушка: сюда ты подставишь реальные данные Pinpoint
    private double robotX = 72;
    private double robotY = 36;
    private double robotHeading = 0; // radians

    /* ================= TARGET (FIELD COORDS) ================= */

    // Пример: центр бэкдропа
    private static final double TARGET_X = 72;
    private static final double TARGET_Y = 36;

    /* ================= PID ================= */

    private static final double KP = 4.0;
    private static final double KD = 0.3;

    private double lastError = 0;

    /* ================= TURRET ================= */

    private static final double TICKS_PER_RAD = 450; // НАСТРОИТЬ
    private static final int TURRET_MIN = -900;
    private static final int TURRET_MAX = 900;

    @Override
    public void runOpMode() {

        turret = hardwareMap.get(DcMotorEx.class, "turret");

        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            // 🔴 1. Получаем позу от Pinpoint
            updatePinpointPose();

            // 🔵 2. Вычисляем угол на цель
            double targetAngle = Math.atan2(
                    TARGET_Y - robotY,
                    TARGET_X - robotX
            );

            // 🟢 3. Ошибка турели (нормализуем)
            double turretError = normalizeAngle(targetAngle - robotHeading);

            // 🧠 4. PD
            double derivative = turretError - lastError;
            lastError = turretError;

            double power = KP * turretError + KD * derivative;

            // ⚙️ 5. Safety
            int pos = turret.getCurrentPosition();
            if ((pos <= TURRET_MIN && power < 0) ||
                    (pos >= TURRET_MAX && power > 0)) {
                power = 0;
            }

            turret.setPower(clamp(power, -0.5, 0.5));

            telemetry.addData("Robot (x,y)", "%.1f , %.1f", robotX, robotY);
            telemetry.addData("Heading (deg)", Math.toDegrees(robotHeading));
            telemetry.addData("Turret error (deg)", Math.toDegrees(turretError));
            telemetry.update();

            idle();
        }
    }

    /* ================= PINPOINT UPDATE ================= */

    private void updatePinpointPose() {
        // ❗ ВАЖНО:
        // Здесь ты просто читаешь Pinpoint:
        //
        // Pose2d pose = pinpoint.getPose();
        // robotX = pose.getX();
        // robotY = pose.getY();
        // robotHeading = pose.getHeading();

        // Заглушка:
        robotX += 0;
    }

    /* ================= UTILS ================= */

    private double normalizeAngle(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(v, max));
    }
}