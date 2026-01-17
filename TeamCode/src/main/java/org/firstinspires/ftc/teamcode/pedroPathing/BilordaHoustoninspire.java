package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;


@TeleOp(name="BilordaHoustoninspire", group="FTC")
public class BilordaHoustoninspire extends OpMode {

    //    private CRServo l1, l2;
    private DcMotor intake;
    private DcMotorEx shut1,shut2,sort;
    private Servo pod,shoot1,turel;
    private Follower follower;

    String last;

    private double liftPower = 1.0;
    private boolean lastYButtonState = false;

    private boolean lastXButtonState = false;

    // Состояния для toggle-логики шутеров
    private boolean lastDpadUpState = false;
    private boolean lastDpadRightState = false;
    private boolean lastDpadDownState = false;
    private boolean farShooterActive = false;
    private boolean sredniyShooterActive = false;
    private boolean smallShooterActive = false;
    private static final int ENCODER_COUNTS_PER_REVOLUTION = 560; // Neo мотор с редуктором 1:20: 28 * 20 = 560 тиков на оборот (больше точность)
    private static final int SORT_ENCODER_COUNTS_PER_120_DEGREES = ENCODER_COUNTS_PER_REVOLUTION / 4; // 120 градусов = 1/3 оборота (примерно 187 тиков)

    // PID параметры для FarShooter (dpad_up)
    private static final double FAR_SHOOTER_P = 5;  // Вставьте значение P для FarShooter
    private static final double FAR_SHOOTER_F = 20;  // Вставьте значение F для FarShooter

    // PID параметры для sredniyshooter (dpad_right)
    private static final double SREDNIY_SHOOTER_P = 5;  // Вставьте значение P для sredniyshooter
    private static final double SREDNIY_SHOOTER_F = 13;  // Вставьте значение F для sredniyshooter

    // PID параметры для smallshooter (dpad_down)
    private static final double SMALL_SHOOTER_P = 5;  // Вставьте значение P для smallshooter
    private static final double SMALL_SHOOTER_F = 8;  // Вставьте значение F для smallshooter

    // PID параметры для sort
    private static final double SORT_P = 100;  // Значение P для sort
    private static final double SORT_F = 40;  // Значение F для sort

    // Скорость для sort (в тиках в секунду)
    private static final double SORT_VELOCITY = 500; // Скорость для sort при использовании velocity control

    // Целевые скорости для разных шутеров (в тиках в секунду)
    private static final double FAR_SHOOTER_VELOCITY = 1500; // Настройте скорость для FarShooter (dpad_up)
    private static final double SREDNIY_SHOOTER_VELOCITY = 1500; // Настройте скорость для sredniyshooter (dpad_right)
    private static final double SMALL_SHOOTER_VELOCITY = 1500; // Настройте скорость для smallshooter (dpad_down)

    // Webcam1 c AprilTag (Logitech)




    // Ограничение поворота турели на 200 градусов
    // Для стандартного серво: 0.0-1.0 = 0-300 градусов, значит 200 градусов = 200/300 = 0.667
    private static final double TURRET_MIN_POSITION = 0.0; // 0 градусов
    private static final double TURRET_MAX_POSITION = 0.667; // 200 градусов (200/300)



    @Override
    public void init() {
        last = "";

        // Инициализация Follower для field-centric управления
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, 0)); // Начальная позиция по центру поля
        follower.update();

        intake = hardwareMap.get(DcMotor.class, "intake");
        sort = hardwareMap.get(DcMotorEx.class, "sort");
        shut1 = hardwareMap.get(DcMotorEx.class, "shut1");
        shut2 = hardwareMap.get(DcMotorEx.class, "shut2");

        pod = hardwareMap.get(Servo.class, "pod");
        shoot1 = hardwareMap.get(Servo.class, "shoot1");
        turel = hardwareMap.get(Servo.class, "turel");

        // Начальная позиция shoot1 (опущено)
        shoot1.setPosition(0.5);


        // Настройка энкодера для sort
        sort.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sort.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sort.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Применение PIDF коэффициентов к sort
        PIDFCoefficients sortPidfCoefficients = new PIDFCoefficients(SORT_P, 0, 0, SORT_F);
        sort.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, sortPidfCoefficients);

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
    public void start() {
        // Запуск field-centric управления
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // Обновление Follower (необходимо для field-centric)
        follower.update();

        // Field-centric управление (последний параметр false = field-centric)
        follower.setTeleOpDrive(
            -gamepad1.left_stick_y,
            -gamepad1.left_stick_x,
            -gamepad1.right_stick_x,
            false // false = field-centric, true = robot-centric
        );


        if (gamepad1.left_bumper) {
            intake.setPower(0.9);

        } else if (gamepad1.left_trigger > 0) {
            intake.setPower(-0.9);

        } else {
            intake.setPower(0.0);

        }


        // Логика переключения позиций sort (3 позиции по 120 градусов)
        boolean currentYButtonState = gamepad2.y;
        if (currentYButtonState && !lastYButtonState) {
            // При нажатии кнопки двигаемся на 120 градусов (относительно текущей позиции)
            // Проверяем, не находится ли мотор уже в режиме позиционирования
            if (sort.getMode() != DcMotor.RunMode.RUN_TO_POSITION || !sort.isBusy()) {
                int currentPosition = sort.getCurrentPosition();
                int targetPosition = currentPosition - SORT_ENCODER_COUNTS_PER_120_DEGREES; // Минус для вращения в другую сторону

                sort.setTargetPosition(targetPosition);
                sort.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sort.setPower(0.55); // Положительная мощность для движения к позиции
            }
        }
        lastYButtonState = currentYButtonState;
        // Если мотор достиг позиции, переключаем в режим удержания
        if (sort.getMode() == DcMotor.RunMode.RUN_TO_POSITION && !sort.isBusy()) {
            sort.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sort.setPower(0.0);
        }

        // Логика переключения позиций sort (3 позиции по 120 градусов)
        boolean currentXButtonState = gamepad2.x;
        if (currentXButtonState && !lastXButtonState) {
            // При нажатии кнопки двигаемся на 120 градусов (относительно текущей позиции)
            // Проверяем, не находится ли мотор уже в режиме позиционирования
            if (sort.getMode() != DcMotor.RunMode.RUN_TO_POSITION || !sort.isBusy()) {
                int currentPosition = sort.getCurrentPosition();
                int targetPosition = currentPosition - SORT_ENCODER_COUNTS_PER_120_DEGREES; // Минус для вращения в другую сторону

                sort.setTargetPosition(targetPosition);
                sort.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sort.setPower(0.55); // Положительная мощность для движения к позиции
            }
        }
        lastYButtonState = currentYButtonState;
        // Если мотор достиг позиции, переключаем в режим удержания
        if (sort.getMode() == DcMotor.RunMode.RUN_TO_POSITION && !sort.isBusy()) {
            sort.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sort.setPower(0.0);
        }






        if (gamepad2.a) {

            if (sort.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                sort.setVelocity(SORT_VELOCITY * 0.3);
            } else {
                sort.setPower(0.3);
            }
        } else {
            if (sort.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                sort.setVelocity(0);
            } else {
                sort.setPower(0);
            }
        }

        // Toggle-логика для FarShooter (dpad_up)
        boolean currentDpadUpState = gamepad2.dpad_up;
        if (currentDpadUpState && !lastDpadUpState) {
            // Переключаем состояние при нажатии кнопки
            farShooterActive = !farShooterActive;
            // Выключаем другие шутеры при включении этого
            if (farShooterActive) {
                sredniyShooterActive = false;
                smallShooterActive = false;
                // Применяем PID параметры для FarShooter
                PIDFCoefficients pidfCoefficients = new PIDFCoefficients(FAR_SHOOTER_P, 0, 0, FAR_SHOOTER_F);
                shut1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
                shut2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
                // Поднимаем shoot1 на 0.5 при включении дальнего шута
                shoot1.setPosition(0.5);
            } else {
                // Опускаем shoot1 на 0.0 при выключении дальнего шута
                shoot1.setPosition(0.0);
            }
        }
        lastDpadUpState = currentDpadUpState;

        // Toggle-логика для sredniyshooter (dpad_right)
        boolean currentDpadRightState = gamepad2.dpad_right;
        if (currentDpadRightState && !lastDpadRightState) {
            // Переключаем состояние при нажатии кнопки
            sredniyShooterActive = !sredniyShooterActive;
            // Выключаем другие шутеры при включении этого
            if (sredniyShooterActive) {
                farShooterActive = false;
                smallShooterActive = false;
                // Применяем PID параметры для sredniyshooter
                PIDFCoefficients pidfCoefficients = new PIDFCoefficients(SREDNIY_SHOOTER_P, 0, 0, SREDNIY_SHOOTER_F);
                shut1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
                shut2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
                // Поднимаем shoot1 на 0.3 при включении среднего шута
                shoot1.setPosition(0.3);
            } else {
                // Опускаем shoot1 на 0.0 при выключении среднего шута
                shoot1.setPosition(0.0);
            }
        }
        lastDpadRightState = currentDpadRightState;

        // Toggle-логика для smallshooter (dpad_down)
        boolean currentDpadDownState = gamepad2.dpad_down;
        if (currentDpadDownState && !lastDpadDownState) {
            // Переключаем состояние при нажатии кнопки
            smallShooterActive = !smallShooterActive;
            // Выключаем другие шутеры при включении этого
            if (smallShooterActive) {
                farShooterActive = false;
                sredniyShooterActive = false;
                // Применяем PID параметры для smallshooter
                PIDFCoefficients pidfCoefficients = new PIDFCoefficients(SMALL_SHOOTER_P, 0, 0, SMALL_SHOOTER_F);
                shut1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
                shut2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
                // Опускаем shoot1 на 0.0, так как дальний шут выключен
                shoot1.setPosition(0.0);
            }
        }
        lastDpadDownState = currentDpadDownState;

        // Управление моторами в зависимости от активного шутера
        if (farShooterActive) {
            // Используем FarShooter с PIDF управлением
            shut1.setVelocity(FAR_SHOOTER_VELOCITY);
            shut2.setVelocity(FAR_SHOOTER_VELOCITY);
        } else if (sredniyShooterActive) {
            // Использование sredniyshooter с управлением через velocity
            shut1.setVelocity(SREDNIY_SHOOTER_VELOCITY);
            shut2.setVelocity(SREDNIY_SHOOTER_VELOCITY);
        } else if (smallShooterActive) {
            // Использование smallshooter с управлением через velocity
            shut1.setVelocity(SMALL_SHOOTER_VELOCITY);
            shut2.setVelocity(SMALL_SHOOTER_VELOCITY);
        } else {
            // Остановка моторов, если ни один шутер не активен
            shut1.setVelocity(0);
            shut2.setVelocity(0);
        }







        telemetry.addData("Robot Position", follower.getPose());
        telemetry.addData("Robot Heading", Math.toDegrees(follower.getHeading()));
        telemetry.addData("Shut1 Velocity", shut1.getVelocity());
        telemetry.addData("Shut2 Velocity", shut2.getVelocity());
        // Показываем текущую целевую скорость в зависимости от активного шутера
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

        telemetry.update();
    }}
