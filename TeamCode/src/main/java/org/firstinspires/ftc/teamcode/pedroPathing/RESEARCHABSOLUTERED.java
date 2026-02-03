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


@Autonomous(name = "ResearchABSOLUTERED", group = "Autonomous")
@Configurable // Panels
public class RESEARCHABSOLUTERED extends OpMode {
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
    private ElapsedTime path2Timer; // Таймер для Path2

    // PID параметры для шутера
    private static final double FAR_SHOOTER_P = 100;
    private static final double FAR_SHOOTER_F = 20;
    private static final double FAR_SHOOTER_VELOCITY = 1400;

    private static final double MID_SHOOTER_P = 100;
    private static final double MID_SHOOTER_F = 20;
    private static final double MID_SHOOTER_VELOCITY = 1150;

    private static final double POD_CLOSED = 0.15;
    private static final double POD_OPENED = 0.4;

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
        pod.setPosition(POD_OPENED);

        // Инициализация таймеров
        timer = new ElapsedTime();
        path2Timer = new ElapsedTime();

        follower = Constants.createFollower(hardwareMap);
        // Красная сторона: начальная позиция (начальная точка Path1)
        follower.setStartingPose(new Pose(123.225, 126.018, Math.toRadians(45)));

        paths = new Paths(follower); // Build paths
        pathState = 0; // Initialize state machine

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        // Start the first path when autonomous starts
        pod.setPosition(POD_CLOSED);
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

                                    new Pose(16.921, 65.821)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(180))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(16.921, 65.821),

                                    new Pose(10.833, 52.167)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(10.833, 52.167),

                                    new Pose(61.103, 85.781)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(134))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(61.103, 85.781),

                                    new Pose(16.977, 65.855)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(180))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(16.977, 65.855),

                                    new Pose(10.854, 52.320)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))

                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(10.854, 52.320),

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

                                    new Pose(16.469, 65.923)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(134), Math.toRadians(180))

                    .build();

            Path15 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(16.469, 65.923),

                                    new Pose(10.893, 52.204)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))

                    .build();

            Path16 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(10.893, 52.204),

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
                    // Включаем интейк перед Path2
                    intake.setPower(0.8);
                    pathState = 2;
                    path2Timer.reset(); // Сбрасываем таймер для Path2
                    follower.followPath(paths.Path2);
                }
                break;
            case 2:
                // Following Path2 - переходит на Path3 через 6 секунд или когда путь завершен
                if (path2Timer.seconds() >= 6.0 || !follower.isBusy()) {
                    // Выключаем интейк после Path2
                    intake.setPower(0);
                    pathState = 3;
                }
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
