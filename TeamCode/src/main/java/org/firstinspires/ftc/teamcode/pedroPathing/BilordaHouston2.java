package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="BilordaHouston2", group="FTC")
public class BilordaHouston2 extends OpMode {

    private DcMotor intake, sort;
    private DcMotorEx shut1, shut2;
    private DcMotor left1, left2, right1, right2;
    private Servo pod, shoot1, turel;

    String last;

    private boolean lastYButtonState = false;
    private boolean lastXButtonState = false;

    // Состояния для toggle-логики шутеров
    private boolean lastDpadUpState = false;
    private boolean lastDpadRightState = false;
    private boolean lastDpadDownState = false;
    private boolean farShooterActive = false;
    private boolean sredniyShooterActive = false;
    private boolean smallShooterActive = false;
    
    private static final int ENCODER_COUNTS_PER_REVOLUTION = 560; // Neo мотор с редуктором 1:20: 28 * 20 = 560 тиков на оборот
    private static final int SORT_ENCODER_COUNTS_PER_120_DEGREES = (int) Math.round(ENCODER_COUNTS_PER_REVOLUTION * 120.0 / 360.0); // 120 градусов = 1/3 оборота (187 тиков)

    // PID параметры для FarShooter (dpad_up)
    private static final double FAR_SHOOTER_P = 5;
    private static final double FAR_SHOOTER_F = 20;

    // PID параметры для sredniyshooter (dpad_right)
    private static final double SREDNIY_SHOOTER_P = 5;
    private static final double SREDNIY_SHOOTER_F = 13;

    // PID параметры для smallshooter (dpad_down)
    private static final double SMALL_SHOOTER_P = 5;
    private static final double SMALL_SHOOTER_F = 8;

    // Целевые скорости для разных шутеров (в тиках в секунду)
    private static final double FAR_SHOOTER_VELOCITY = 1500;
    private static final double SREDNIY_SHOOTER_VELOCITY = 1500;
    private static final double SMALL_SHOOTER_VELOCITY = 1500;

    // Ограничение поворота турели на 200 градусов
    private static final double TURRET_MIN_POSITION = 0.0; // 0 градусов
    private static final double TURRET_MAX_POSITION = 0.667; // 200 градусов (200/300)
    private double turretPosition = 0.333; // Начальная позиция турели

    @Override
    public void init() {
        last = "";

        // Инициализация моторов меканум-драйва
        left1 = hardwareMap.get(DcMotor.class, "left1");
        left2 = hardwareMap.get(DcMotor.class, "left2");
        right1 = hardwareMap.get(DcMotor.class, "right1");
        right2 = hardwareMap.get(DcMotor.class, "right2");

        // Установка направлений моторов
        left1.setDirection(DcMotor.Direction.REVERSE);
        left2.setDirection(DcMotor.Direction.FORWARD);
        right1.setDirection(DcMotor.Direction.REVERSE);
        right2.setDirection(DcMotor.Direction.FORWARD);

        intake = hardwareMap.get(DcMotor.class, "intake");
        sort = hardwareMap.get(DcMotor.class, "sort");
        shut1 = hardwareMap.get(DcMotorEx.class, "shut1");
        shut2 = hardwareMap.get(DcMotorEx.class, "shut2");

        pod = hardwareMap.get(Servo.class, "pod");
        shoot1 = hardwareMap.get(Servo.class, "shoot1");
        turel = hardwareMap.get(Servo.class, "turel");

        // Начальная позиция турели с ограничением на 200 градусов
        turretPosition = Range.clip(turretPosition, TURRET_MIN_POSITION, TURRET_MAX_POSITION);
        turel.setPosition(turretPosition);

        // Настройка энкодера для sort
        sort.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sort.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sort.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Настройка моторов дальнего шута с PIDF
        shut1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shut2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Настройка направления
        shut1.setDirection(DcMotorSimple.Direction.REVERSE);
        shut2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Применение PIDF коэффициентов к обоим моторам (по умолчанию для FarShooter)
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(FAR_SHOOTER_P, 0, 0, FAR_SHOOTER_F);
        shut1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shut2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    @Override
    public void loop() {
        
        // Прямое управление движением через механику меканум-драйва (как в оригинале)
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rotation = gamepad1.right_stick_x;  // Без минуса, как в оригинале

        // Формула меканум-драйва
        double leftFrontPower = y + x + rotation;
        double leftBackPower = y - x + rotation;
        double rightFrontPower = y - x - rotation;
        double rightBackPower = y + x - rotation;

        // Применяем мощность к моторам
        left1.setPower(Math.max(-1, Math.min(1, leftFrontPower)));
        left2.setPower(Math.max(-1, Math.min(1, leftBackPower)));
        right1.setPower(Math.max(-1, Math.min(1, rightFrontPower)));
        right2.setPower(Math.max(-1, Math.min(1, rightBackPower)));
        
        // Управление intake и sort
        if (gamepad1.left_bumper) {
            intake.setPower(0.-9);
            sort.setPower(-0.2);
        } else if (gamepad2.left_trigger > 0) {
            intake.setPower(-0.9);
        } else {
            intake.setPower(0.0);
            sort.setPower(0.0);
        }

        // Проверка завершения поворота sort
        if (sort.getMode() == DcMotor.RunMode.RUN_TO_POSITION && !sort.isBusy()) {
            sort.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sort.setPower(0.0);
        }

        // Логика поворота sort на -120 градусов (кнопка Y) - только при нажатии
        boolean currentYButtonState = gamepad2.y;
        if (currentYButtonState && !lastYButtonState) {
            // Поворачиваем только если мотор не занят
            if (sort.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                int currentPosition = sort.getCurrentPosition();
                int targetPosition = currentPosition - SORT_ENCODER_COUNTS_PER_120_DEGREES;

                sort.setTargetPosition(targetPosition);
                sort.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sort.setPower(0.55);
            }
        }
        lastYButtonState = currentYButtonState;

        // Логика поворота sort на +120 градусов (кнопка X) - только при нажатии
        boolean currentXButtonState = gamepad2.x;
        if (currentXButtonState && !lastXButtonState) {
            // Поворачиваем только если мотор не занят
            if (sort.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                int currentPosition = sort.getCurrentPosition();
                int targetPosition = currentPosition + SORT_ENCODER_COUNTS_PER_120_DEGREES;

                sort.setTargetPosition(targetPosition);
                sort.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sort.setPower(-0.55);
            }
        }
        lastXButtonState = currentXButtonState;

        // Управление pod
        if (gamepad2.a) {
            pod.setPosition(0.7);
            sort.setPower(0.3);
        } else {
            pod.setPosition(0.0);
        }

        // Toggle-логика для FarShooter (dpad_up)
        boolean currentDpadUpState = gamepad2.dpad_up;
        if (currentDpadUpState && !lastDpadUpState) {
            farShooterActive = !farShooterActive;
            if (farShooterActive) {
                sredniyShooterActive = false;
                smallShooterActive = false;
                PIDFCoefficients pidfCoefficients = new PIDFCoefficients(FAR_SHOOTER_P, 0, 0, FAR_SHOOTER_F);
                shut1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
                shut2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
                // Поднимаем shoot1 на 0.3 при включении среднего шута
                shoot1.setPosition(0.1);
            } else {
                // Опускаем shoot1 на 0.0 при выключении среднего шута
                shoot1.setPosition(0.5);
            }
        }
        lastDpadUpState = currentDpadUpState;

        // Toggle-логика для sredniyshooter (dpad_right)
        boolean currentDpadRightState = gamepad2.dpad_right;
        if (currentDpadRightState && !lastDpadRightState) {
            sredniyShooterActive = !sredniyShooterActive;
            if (sredniyShooterActive) {
                farShooterActive = false;
                smallShooterActive = false;
                PIDFCoefficients pidfCoefficients = new PIDFCoefficients(SREDNIY_SHOOTER_P, 0, 0, SREDNIY_SHOOTER_F);
                shut1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
                shut2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            }
        }
        lastDpadRightState = currentDpadRightState;

        // Toggle-логика для smallshooter (dpad_down)
        boolean currentDpadDownState = gamepad2.dpad_down;
        if (currentDpadDownState && !lastDpadDownState) {
            smallShooterActive = !smallShooterActive;
            if (smallShooterActive) {
                farShooterActive = false;
                sredniyShooterActive = false;
                PIDFCoefficients pidfCoefficients = new PIDFCoefficients(SMALL_SHOOTER_P, 0, 0, SMALL_SHOOTER_F);
                shut1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
                shut2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            }
        }
        lastDpadDownState = currentDpadDownState;

        // Управление моторами в зависимости от активного шутера
        if (farShooterActive) {
            shut1.setVelocity(FAR_SHOOTER_VELOCITY);
            shut2.setVelocity(FAR_SHOOTER_VELOCITY);
        } else if (sredniyShooterActive) {
            shut1.setVelocity(SREDNIY_SHOOTER_VELOCITY);
            shut2.setVelocity(SREDNIY_SHOOTER_VELOCITY);
        } else if (smallShooterActive) {
            shut1.setVelocity(SMALL_SHOOTER_VELOCITY);
            shut2.setVelocity(SMALL_SHOOTER_VELOCITY);
        } else {
            shut1.setVelocity(0);
            shut2.setVelocity(0);
        }

        // Управление турелью (можно добавить управление через gamepad при необходимости)
        turretPosition = Range.clip(turretPosition, TURRET_MIN_POSITION, TURRET_MAX_POSITION);
        turel.setPosition(turretPosition);

        // Телеметрия
        telemetry.addData("Left1 Power", left1.getPower());
        telemetry.addData("Left2 Power", left2.getPower());
        telemetry.addData("Right1 Power", right1.getPower());
        telemetry.addData("Right2 Power", right2.getPower());
        telemetry.addData("Shut1 Velocity", shut1.getVelocity());
        telemetry.addData("Shut2 Velocity", shut2.getVelocity());
        
        if (farShooterActive) {
            telemetry.addData("Target Velocity", "FarShooter: " + FAR_SHOOTER_VELOCITY);
        } else if (sredniyShooterActive) {
            telemetry.addData("Target Velocity", "SredniyShooter: " + SREDNIY_SHOOTER_VELOCITY);
        } else if (smallShooterActive) {
            telemetry.addData("Target Velocity", "SmallShooter: " + SMALL_SHOOTER_VELOCITY);
        } else {
            telemetry.addData("Target Velocity", "None (0)");
        }
        
        telemetry.addData("Sort Position", sort.getCurrentPosition());
        telemetry.addData("Sort Mode", sort.getMode());
        telemetry.addData("Turret Position", turretPosition);

        telemetry.update();
    }
}
