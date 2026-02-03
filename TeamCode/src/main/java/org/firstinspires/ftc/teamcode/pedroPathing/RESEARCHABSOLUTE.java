package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "ResearchABSOLUTE", group = "Autonomous")
@Configurable // Panels
public class RESEARCHABSOLUTE extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    // Устройства для управления
    private static DcMotor intake;
    private static DcMotorEx shut1;
    private static DcMotorEx shut2;
    private static Servo pod;
    private static ElapsedTime timer;
    private ElapsedTime path6Timer; // Таймер для ожидания после Path6
    private ElapsedTime path9Timer; // Таймер для ожидания после Path9
    private ElapsedTime path14Timer; // Таймер для ожидания после Path14

    // PID параметры для шутера
    private static final double FAR_SHOOTER_P = 100;
    private static final double FAR_SHOOTER_F = 20;
    private static final double FAR_SHOOTER_VELOCITY = 1400;

    private static final double MID_SHOOTER_P = 100;
    private static final double MID_SHOOTER_F = 20;
    private static final double MID_SHOOTER_VELOCITY = 1150;

    private static final double POD_CLOSED = 0.1;
    private static final double POD_OPENED = 0.3;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Инициализация устройств
        intake = hardwareMap.get(DcMotor.class, "intake");
        shut1 = hardwareMap.get(DcMotorEx.class, "shut1");
        shut2 = hardwareMap.get(DcMotorEx.class, "shut2");
        pod = hardwareMap.get(Servo.class, "pod");

        // ===== SHOOTER CONFIG =====
        shut1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shut2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shut1.setDirection(DcMotorSimple.Direction.FORWARD);
        shut2.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidf = new PIDFCoefficients(FAR_SHOOTER_P, 0, 0, FAR_SHOOTER_F);
        shut1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shut2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        intake.setPower(0);
        shut1.setVelocity(0);
        shut2.setVelocity(0);
        pod.setPosition(POD_CLOSED);

        // Инициализация таймеров
        timer = new ElapsedTime();
        path6Timer = new ElapsedTime();
        path9Timer = new ElapsedTime();
        path14Timer = new ElapsedTime();

        follower = Constants.createFollower(hardwareMap);
        // Начальная позиция должна совпадать с началом Path1
        follower.setStartingPose(new Pose(25.269, 129.543, Math.toRadians(142)));

        paths = new Paths(follower); // Build paths
        pathState = 0; // Initialize state machine

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        // Start the first path when autonomous starts
        pathState = 1;
        follower.followPath(paths.Path1);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

































    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;
        public PathChain Path12;
        public PathChain Path13;
        public PathChain Path14;
        public PathChain Path15;
        public PathChain Path16;
        public PathChain Path17;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(25.269, 129.543),

                                    new Pose(55.409, 102.977)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(134))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(55.409, 102.977),
                                    new Pose(66.027, 79.629),
                                    new Pose(13.275, 84.315)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(13.275, 84.315),

                                    new Pose(58.770, 84.417)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(134))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(58.770, 84.417),
                                    new Pose(54.024, 53.640),
                                    new Pose(15.986, 58.685)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(180))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.986, 58.685),

                                    new Pose(61.030, 86.193)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(134))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(61.030, 86.193),

                                    new Pose(12.193, 65.821)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(180))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(12.193, 65.821),

                                    new Pose(6.893, 52.364)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(6.893, 52.364),

                                    new Pose(61.103, 85.781)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(134))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(61.103, 85.781),

                                    new Pose(12.643, 66.446)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(180))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(12.643, 66.446),

                                    new Pose(6.520, 52.320)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))

                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(6.520, 52.320),

                                    new Pose(61.118, 85.700)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(134))

                    .build();

            Path12 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(61.118, 85.700),
                                    new Pose(69.534, 27.047),
                                    new Pose(16.005, 35.431)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(180))

                    .build();

            Path13 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(16.005, 35.431),
                                    new Pose(45.317, 57.137),
                                    new Pose(60.899, 85.480)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(134))

                    .build();

            Path14 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(60.899, 85.480),

                                    new Pose(12.332, 66.120)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(180))

                    .build();

            Path15 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(12.332, 66.120),

                                    new Pose(6.953, 52.401)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))

                    .build();

            Path16 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(6.953, 52.401),

                                    new Pose(61.264, 86.252)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(134))

                    .build();

            Path17 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(61.264, 86.252),

                                    new Pose(37.260, 64.384)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(180))

                    .build();
        }
    }


    public int autonomousPathUpdate() {
        // State machine for autonomous path following
        switch (pathState) {

            case 0:
                // Waiting to start
                break;
            case 1:
                // Following Path1
                if (!follower.isBusy()) {
                    // Дальний шутер
                    // Включаем интейк перед Path2
                    intake.setPower(0.67);
                    pathState = 2;
                    follower.followPath(paths.Path2);
                }
                break;
            case 2:
                // Following Path2
                if (!follower.isBusy()) {
                    // Выключаем интейк после Path2
                    intake.setPower(0);
                    pathState = 3;
                    follower.followPath(paths.Path3);
                }
                break;
            case 3:
                // Following Path3
                if (!follower.isBusy()) {
                    // Дальний шутер
                    // Включаем интейк перед Path4
                    intake.setPower(0.67);
                    pathState = 4;
                    follower.followPath(paths.Path4);
                }
                break;
            case 4:
                // Following Path4
                if (!follower.isBusy()) {
                    // Выключаем интейк после Path4
                    intake.setPower(0);
                    pathState = 5;
                    follower.followPath(paths.Path5);
                }
                break;
            case 5:
                // Following Path5
                if (!follower.isBusy()) {
                    // Включаем интейк перед Path6
                    intake.setPower(0.67);
                    pathState = 6;
                    follower.followPath(paths.Path6);
                }
                break;
            case 6:
                // Following Path6
                if (!follower.isBusy()) {
                    // Выключаем интейк после Path6
                    intake.setPower(0);
                    // Запускаем таймер ожидания при первом завершении пути
                    if (path6Timer.seconds() == 0 || path6Timer.seconds() >= 1.0) {
                        path6Timer.reset();
                    }
                    // Ждем 1 секунду перед переходом к Path7
                    if (path6Timer.seconds() >= 1.0) {
                        // Включаем интейк с мощностью 0.8 для Path7
                        intake.setPower(0.8);
                        pathState = 7;
                        follower.followPath(paths.Path7);
                    }
                }
                break;
            case 7:
                // Following Path7
                if (!follower.isBusy()) {
                    // Выключаем интейк после Path7
                    intake.setPower(0);
                    pathState = 8;
                    follower.followPath(paths.Path8);
                }
                break;
            case 8:
                // Following Path8
                if (!follower.isBusy()) {
                    // Выключаем интейк после Path8
                    intake.setPower(0);
                    pathState = 9;
                    follower.followPath(paths.Path9);
                }
                break;
            case 9:
                // Following Path9
                if (!follower.isBusy()) {
                    // Запускаем таймер ожидания при первом завершении пути
                    if (path9Timer.seconds() == 0 || path9Timer.seconds() >= 1.0) {
                        path9Timer.reset();
                    }
                    // Ждем 1 секунду перед переходом к Path10
                    if (path9Timer.seconds() >= 1.0) {
                        // Включаем интейк с мощностью 0.8 для Path10
                        intake.setPower(0.8);
                        pathState = 10;
                        follower.followPath(paths.Path10);
                    }
                }
                break;
            case 10:
                // Following Path10
                if (!follower.isBusy()) {
                    // Выключаем интейк после Path10
                    intake.setPower(0);
                    pathState = 11;
                    follower.followPath(paths.Path11);
                }
                break;
            case 11:
                // Following Path11
                if (!follower.isBusy()) {
                    // Включаем интейк перед Path12
                    intake.setPower(0.67);
                    pathState = 12;
                    follower.followPath(paths.Path12);
                }
                break;
            case 12:
                // Following Path12
                if (!follower.isBusy()) {
                    // Выключаем интейк после Path12
                    intake.setPower(0);
                    pathState = 13;
                    follower.followPath(paths.Path13);
                }
                break;
            case 13:
                // Following Path13
                if (!follower.isBusy()) {
                    // Включаем интейк перед Path14
                    intake.setPower(0.67);
                    pathState = 14;
                    follower.followPath(paths.Path14);
                }
                break;
            case 14:
                // Following Path14
                if (!follower.isBusy()) {
                    // Выключаем интейк после Path14
                    intake.setPower(0);
                    // Запускаем таймер ожидания при первом завершении пути
                    if (path14Timer.seconds() == 0 || path14Timer.seconds() >= 1.0) {
                        path14Timer.reset();
                    }
                    // Ждем 1 секунду перед переходом к Path15
                    if (path14Timer.seconds() >= 1.0) {
                        // Включаем интейк с мощностью 0.8 для Path15
                        intake.setPower(0.8);
                        pathState = 15;
                        follower.followPath(paths.Path15);
                    }
                }
                break;
            case 15:
                // Following Path15
                if (!follower.isBusy()) {
                    // Выключаем интейк после Path15
                    intake.setPower(0);
                    pathState = 16;
                    follower.followPath(paths.Path16);
                }
                break;
            case 16:
                // Following Path16
                if (!follower.isBusy()) {
                    // Выключаем интейк после Path16
                    intake.setPower(0);
                    pathState = 17;
                    follower.followPath(paths.Path17);
                }
                break;
            case 17:
                // Following Path17
                if (!follower.isBusy()) {
                    // Path17 завершен, автономный режим закончен
                    intake.setPower(0);
                    pathState = 18;
                    stopAll();
                }
                break;
            case 18:
                // Autonomous finished
                break;



        }
        return pathState;
    }

    // Метод для последовательности стрельбы
    static void shootSequence(boolean useMidShooter) {
        double shooterVelocity;
        double shooterP;
        double shooterF;

        if (useMidShooter) {
            // Средний шутер
            shooterVelocity = MID_SHOOTER_VELOCITY;
            shooterP = MID_SHOOTER_P;
            shooterF = MID_SHOOTER_F;
        } else {
            // Дальний шутер
            shooterVelocity = FAR_SHOOTER_VELOCITY;
            shooterP = FAR_SHOOTER_P;
            shooterF = FAR_SHOOTER_F;
        }

        // Устанавливаем PIDF коэффициенты
        PIDFCoefficients pidf = new PIDFCoefficients(shooterP, 0, 0, shooterF);
        shut1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shut2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        // Разгоняем шутер
        shut1.setVelocity(shooterVelocity);
        shut2.setVelocity(shooterVelocity);

        // Ждем 1 секунду для разгона шутера
        timer.reset();
        while (timer.milliseconds() < 1000) {
            // Ждем 1000мс для разгона шутера
        }

        // Открываем серву
        pod.setPosition(POD_OPENED);

        timer.reset();
        while (timer.milliseconds() < 500) {
            // Ждем 1000мс для разгона шутера
        }
        // Запускаем intake
        intake.setPower(0.67);

        // Ждем немного для работы интейка
        timer.reset();
        while (timer.milliseconds() < 2000) {
            // Ждем 500мс
        }

        // Закрываем серву
        pod.setPosition(POD_CLOSED);

        // Останавливаем intake
        intake.setPower(0);
    }

    // Метод для остановки всех устройств
    void stopAll() {
        shut1.setVelocity(0);
        shut2.setVelocity(0);
        intake.setPower(0);
        pod.setPosition(POD_CLOSED);
    }


}
