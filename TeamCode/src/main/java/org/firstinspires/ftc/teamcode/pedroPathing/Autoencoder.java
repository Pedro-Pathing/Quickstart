package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Autoblue", group="Autonomous")
public class Autoencoder extends LinearOpMode {


    private static final double FAR_SHOOTER_P = 22;
    private static final double FAR_SHOOTER_F = 7.5;
    private static final double FAR_SHOOTER_VELOCITY = 1450; // Целевая скорость для дальнего шутера

    // Типы шутеров
    public enum ShooterType {
        FAR,      // Дальний (1150 тиков/сек)

    }

    private ShooterType currentShooterType = ShooterType.FAR; // По умолчанию дальний

    // Статические объекты для удобного доступа к каждому типу шутера
    public final ShooterController FAR = new ShooterController(ShooterType.FAR);


    // Внутренний класс для управления шутером
    public class ShooterController {
        private final ShooterType type;

        public ShooterController(ShooterType type) {
            this.type = type;
        }

        public void setPower(double power) {
            startShooter(type, power);
        }

        public void setVelocity(double velocity) {
            setShooterType(type);
            shut1.setVelocity(-velocity);
            shut2.setVelocity(velocity);
        }

        public void stop() {
            if (currentShooterType == type) {
                stopShooter();
            }
        }

        public ShooterType getType() {
            return type;
        }
    }

    /*
     * Примеры использования разных типов шутеров:
     *
     * // Использование статических объектов (рекомендуется):
     * FAR.setPower(1.0);        // Дальний шутер на полную мощность
     * SREDNIY.setPower(0.8);    // Средний шутер на 80%
     * SMALL.setPower(0.6);      // Ближний шутер на 60%
     *
     * // Или через методы:
     * setShooterType(ShooterType.FAR);
     * startShooter(1.0);
     *
     * startShooter(ShooterType.SREDNIY, 1.0);
     */

    // Константы для энкодеров
    private static final double COUNTS_PER_CM = 28.0; // Примерное значение, настройте под ваш робот
    private static final double COUNTS_PER_DEGREE = 700.0 / 180.0; // Примерное значение для поворотов
    private static final double DRIVE_SPEED_GAIN = 1.0; // Коэффициент масштабирования мощности

    private DcMotor left1, left2, right1, right2;
    private DcMotor intake;
    private DcMotorEx shut1, shut2;
    private Servo pod;

    // PID контроллеры (только для шутера)
    private ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {

        left1 = hardwareMap.get(DcMotor.class, "left1");
        left2 = hardwareMap.get(DcMotor.class, "left2");
        right1 = hardwareMap.get(DcMotor.class, "right2");
        right2 = hardwareMap.get(DcMotor.class, "right1");

        intake = hardwareMap.get(DcMotor.class, "intake");
        shut1 = hardwareMap.get(DcMotorEx.class, "shut1");
        shut2 = hardwareMap.get(DcMotorEx.class, "shut2");

        pod = hardwareMap.get(Servo.class, "pod");

        right1.setDirection(DcMotor.Direction.REVERSE);
        right2.setDirection(DcMotor.Direction.REVERSE);

        // Настройка энкодеров для приводных моторов
        left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Настройка шутеров с PIDF
        shut1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shut2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shut1.setDirection(DcMotorSimple.Direction.REVERSE);
        shut2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Устанавливаем PIDF коэффициенты по умолчанию для дальнего шутера
        setShooterType(ShooterType.FAR);

        // Инициализация таймера
        timer = new ElapsedTime();

        waitForStart();

        if (!opModeIsActive()) return;
        pod.setPosition(0.15);
        moveBackwardSimple(-0.5, 80);
        setShooterType(ShooterType.FAR);
        pod.setPosition(0.4);
        intake.setPower(0.67);
        waitMs(750);
        pod.setPosition(0.15);
        intake.setPower(0.0);
        turnAngleSimple(-35);
        strafeLeftSimple(-0.5, 85);





        for (int i = 0; i < 3 && opModeIsActive(); i++) {
            feedAndShoot(500, 0);
        }



        stopAll();
    }



    void fireShot() {
        pod.setPosition(1.0);
        waitMs(400);
        pod.setPosition(0.7);
        waitMs(300);
    }

    void intakeShot() {
        runIntake(-1, 100);
    }

    void moveForwardSimple(double p, double cm) {
        int targetCounts = (int)(cm * COUNTS_PER_CM);
        int startPosition = (left1.getCurrentPosition() + left2.getCurrentPosition() +
                right1.getCurrentPosition() + right2.getCurrentPosition()) / 4;
        int targetPosition = startPosition + targetCounts;

        double scaledPower = scalePower(p);
        double error = targetPosition - startPosition;

        while (opModeIsActive() && Math.abs(error) > 30) {
            int currentPosition = (left1.getCurrentPosition() + left2.getCurrentPosition() +
                    right1.getCurrentPosition() + right2.getCurrentPosition()) / 4;
            error = targetPosition - currentPosition;

            double finalPower = Math.max(-1.0, Math.min(1.0, scaledPower));

            setAll(finalPower, finalPower, finalPower, finalPower);
            idle();
        }
        stopDrive();
    }

    void moveBackwardSimple(double p, double cm) {
        int targetCounts = (int)(cm * COUNTS_PER_CM);
        int startPosition = (left1.getCurrentPosition() + left2.getCurrentPosition() +
                right1.getCurrentPosition() + right2.getCurrentPosition()) / 4;
        int targetPosition = startPosition - targetCounts;

        double scaledPower = scalePower(-p); // Отрицательное для движения назад
        double error = targetPosition - startPosition;

        while (opModeIsActive() && Math.abs(error) > 30) {
            int currentPosition = (left1.getCurrentPosition() + left2.getCurrentPosition() +
                    right1.getCurrentPosition() + right2.getCurrentPosition()) / 4;
            error = targetPosition - currentPosition;

            double finalPower = Math.max(-1.0, Math.min(1.0, scaledPower));

            setAll(finalPower, finalPower, finalPower, finalPower);
            idle();
        }
        stopDrive();
    }

    void moveForwardWithSpin(double drivePower, double cm, double shutPower1, double shutPower2, long spinupMs) {
        // Запускаем шутеры с velocity control для текущего типа шутера
        double velocity = getShooterVelocity();
        shut1.setVelocity(-shutPower1 * velocity);
        shut2.setVelocity(shutPower2 * velocity);
        ElapsedTime spinTimer = new ElapsedTime();
        spinTimer.reset();

        // Простое движение без PID
        int targetCounts = (int)(cm * COUNTS_PER_CM);
        int startPosition = (left1.getCurrentPosition() + left2.getCurrentPosition() +
                right1.getCurrentPosition() + right2.getCurrentPosition()) / 4;
        int targetPosition = startPosition + targetCounts;

        double scaledPower = scalePower(drivePower);
        double error = targetPosition - startPosition;

        while (opModeIsActive() && Math.abs(error) > 30) {
            int currentPosition = (left1.getCurrentPosition() + left2.getCurrentPosition() +
                    right1.getCurrentPosition() + right2.getCurrentPosition()) / 4;
            error = targetPosition - currentPosition;

            double finalPower = Math.max(-1.0, Math.min(1.0, scaledPower));

            setAll(finalPower, finalPower, finalPower, finalPower);
            idle();
        }
        stopDrive();

        long remaining = spinupMs - (long) spinTimer.milliseconds();
        if (remaining > 0) {
            waitMs(remaining);
        }
    }

    void strafeLeftSimple(double p, double cm) {
        int targetCounts = (int)(cm * COUNTS_PER_CM);
        // Для strafe используем разницу между левыми и правыми моторами
        int startLeft = (left1.getCurrentPosition() + left2.getCurrentPosition()) / 2;
        int startRight = (right1.getCurrentPosition() + right2.getCurrentPosition()) / 2;
        int startPosition = (startLeft - startRight) / 2;
        int targetPosition = startPosition - targetCounts;

        double scaledPower = scalePower(p);
        double error = targetPosition - startPosition;

        while (opModeIsActive() && Math.abs(error) > 30) {
            int currentLeft = (left1.getCurrentPosition() + left2.getCurrentPosition()) / 2;
            int currentRight = (right1.getCurrentPosition() + right2.getCurrentPosition()) / 2;
            int currentPosition = (currentLeft - currentRight) / 2;
            error = targetPosition - currentPosition;

            double finalPower = Math.max(-1.0, Math.min(1.0, scaledPower));

            setAll(-finalPower, finalPower, finalPower, -finalPower);
            idle();
        }
        stopDrive();
    }

    void strafeRightSimple(double p, double cm) {
        int targetCounts = (int)(cm * COUNTS_PER_CM);
        // Для strafe используем разницу между левыми и правыми моторами
        int startLeft = (left1.getCurrentPosition() + left2.getCurrentPosition()) / 2;
        int startRight = (right1.getCurrentPosition() + right2.getCurrentPosition()) / 2;
        int startPosition = (startLeft - startRight) / 2;
        int targetPosition = startPosition + targetCounts;

        double scaledPower = scalePower(p);
        double error = targetPosition - startPosition;

        while (opModeIsActive() && Math.abs(error) > 30) {
            int currentLeft = (left1.getCurrentPosition() + left2.getCurrentPosition()) / 2;
            int currentRight = (right1.getCurrentPosition() + right2.getCurrentPosition()) / 2;
            int currentPosition = (currentLeft - currentRight) / 2;
            error = targetPosition - currentPosition;

            double finalPower = Math.max(-1.0, Math.min(1.0, scaledPower));

            setAll(finalPower, -finalPower, -finalPower, finalPower);
            idle();
        }
        stopDrive();
    }

    void turnAngleSimple(double deg) {
        int targetCounts = (int)(Math.abs(deg) * COUNTS_PER_DEGREE);
        int startLeft = (left1.getCurrentPosition() + left2.getCurrentPosition()) / 2;
        int startRight = (right1.getCurrentPosition() + right2.getCurrentPosition()) / 2;

        int targetLeft, targetRight;
        double turnPower = 0.5; // Базовая мощность для поворота
        if (deg > 0) {
            targetLeft = startLeft + targetCounts;
            targetRight = startRight - targetCounts;
        } else {
            targetLeft = startLeft - targetCounts;
            targetRight = startRight + targetCounts;
        }

        double errorLeft = Math.abs(targetLeft - startLeft);
        double errorRight = Math.abs(targetRight - startRight);

        while (opModeIsActive() && (errorLeft > 5 || errorRight > 5)) {
            int currentLeft = (left1.getCurrentPosition() + left2.getCurrentPosition()) / 2;
            int currentRight = (right1.getCurrentPosition() + right2.getCurrentPosition()) / 2;

            errorLeft = Math.abs(targetLeft - currentLeft);
            errorRight = Math.abs(targetRight - currentRight);

            double powerLeft = deg > 0 ? turnPower : -turnPower;
            double powerRight = deg > 0 ? -turnPower : turnPower;

            setAll(powerLeft, powerLeft, powerRight, powerRight);
            idle();
        }
        stopDrive();
    }

    void setAll(double l1, double l2, double r1, double r2) {
        left1.setPower(l1);
        left2.setPower(l2);
        right1.setPower(r1);
        right2.setPower(r2);
    }

    double scalePower(double power) {
        double scaled = power * DRIVE_SPEED_GAIN;
        if (scaled > 1.0) return 1.0;
        if (scaled < -1.0) return -1.0;
        return scaled;
    }

    void setShooterType(ShooterType type) {
        currentShooterType = type;
        PIDFCoefficients pidfCoefficients;

        // PID только для шутера
        switch (type) {
            case FAR:
                pidfCoefficients = new PIDFCoefficients(FAR_SHOOTER_P, 0, 0, FAR_SHOOTER_F);
                break;
            default:
                pidfCoefficients = new PIDFCoefficients(FAR_SHOOTER_P, 0, 0, FAR_SHOOTER_F);
        }

        shut1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shut2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    double getShooterVelocity() {
        switch (currentShooterType) {
            case FAR:
                return FAR_SHOOTER_VELOCITY;
            default:
                return FAR_SHOOTER_VELOCITY;
        }
    }

    void startShooter(double power) {
        // Используем velocity control с PIDF для текущего типа шутера
        double targetVelocity = getShooterVelocity() * power;
        shut1.setVelocity(-targetVelocity);
        shut2.setVelocity(targetVelocity);
    }

    void startShooter(ShooterType type, double power) {
        // Устанавливаем тип и запускаем шутер
        setShooterType(type);
        startShooter(power);
    }

    void stopShooter() {
        shut1.setVelocity(0);
        shut2.setVelocity(0);
    }

    void waitShooterMs(long durationMs) {
        waitMs(durationMs);
    }

    void runIntake(double power, long durationMs) {
        intake.setPower(power);
        waitMs(durationMs);
        intake.setPower(0);
    }

    void feedAndShoot(long feedMs, long settleMs) {
        if (feedMs > 0) {
            runIntake(-1, feedMs);
        }
        if (settleMs > 0) {
            waitMs(settleMs);
        }
        fireShot();
    }

    void shootVolley(int shots, long gapMs) {
        for (int i = 0; i < shots && opModeIsActive(); i++) {
            fireShot();
            if (i < shots - 1 && gapMs > 0) {
                waitMs(gapMs);
            }
        }
    }

    void waitMs(long durationMs) {
        if (durationMs <= 0) return;
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < durationMs) {
            idle();
        }
    }

    void stopDrive() {
        setAll(0,0,0,0);
    }

    void stopAll() {
        stopDrive();
        stopShooter();
        intake.setPower(0);
        pod.setPosition(0.7);
    }
}
