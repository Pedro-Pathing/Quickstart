package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Teleoperado IMU", group = "TeleOp")
public class Teleop extends LinearOpMode {

    // =========================================================
    // MOTORES DO CHASSI
    // =========================================================

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    // =========================================================
    // MECANISMOS
    // =========================================================

    private DcMotor shooter;
    private DcMotor feeder;
    private DcMotor spindexer;

    // =========================================================
    // IMU
    // =========================================================

    private IMU imu;

    // Ângulo que o robô deve manter durante o strafe
    private double targetHeading = 0.0;

    // Indica se o Heading Lock está ativo
    private boolean headingLockActive = false;

    // =========================================================
    // CONTROLE DA CORREÇÃO DO STRAFE
    // =========================================================

    /*
     * P = força principal da correção.
     *
     * Antes estava em 0.020.
     * Agora está mais forte para corrigir rapidamente.
     */
    private static final double HEADING_KP = 0.15;

    /*
     * D = reage quando o robô começa a girar rapidamente.
     *
     * Ajuda a corrigir antes de o erro ficar muito grande.
     */
    private static final double HEADING_KD = 0.02;

    // Limite máximo da correção de giro
    private static final double MAX_HEADING_CORRECTION = 10;

    // Erro anterior usado pelo D
    private double lastHeadingError = 0.0;

    // Cronômetro para calcular o D
    private final ElapsedTime headingTimer = new ElapsedTime();

    // =========================================================
    // SHOOTER
    // =========================================================

    private double shooterPower = 0.0;

    private boolean lastRightTrigger = false;
    private boolean lastLeftTrigger = false;

    @Override
    public void runOpMode() {

        // =====================================================
        // MAPEAMENTO DOS MOTORES
        // =====================================================

        leftFront =
                hardwareMap.get(DcMotor.class, "leftFront");

        leftBack =
                hardwareMap.get(DcMotor.class, "leftBack");

        rightFront =
                hardwareMap.get(DcMotor.class, "rightFront");

        rightBack =
                hardwareMap.get(DcMotor.class, "rightBack");


        shooter =
                hardwareMap.get(DcMotor.class, "shooter");

        feeder =
                hardwareMap.get(DcMotor.class, "feeder");

        spindexer =
                hardwareMap.get(DcMotor.class, "Spindexer");

        // =====================================================
        // CONFIGURAÇÃO DA IMU
        //
        // Logo REV = LEFT
        // USB = UP
        // =====================================================

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot hubOrientation =
                new RevHubOrientationOnRobot(

                        RevHubOrientationOnRobot
                                .LogoFacingDirection.LEFT,

                        RevHubOrientationOnRobot
                                .UsbFacingDirection.UP
                );

        imu.initialize(
                new IMU.Parameters(hubOrientation)
        );

        // =====================================================
        // DIREÇÃO DOS MOTORES
        // =====================================================

        leftFront.setDirection(
                DcMotor.Direction.REVERSE
        );

        leftBack.setDirection(
                DcMotor.Direction.REVERSE
        );

        rightFront.setDirection(
                DcMotor.Direction.FORWARD
        );

        rightBack.setDirection(
                DcMotor.Direction.FORWARD
        );

        // =====================================================
        // ZERO POWER BEHAVIOR
        // =====================================================

        leftFront.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        leftBack.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        rightFront.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        rightBack.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        shooter.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        feeder.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        spindexer.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        // Tudo começa parado
        stopAllMotors();

        telemetry.addLine("TeleOp pronto");
        telemetry.addLine("Pressione START");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        // Zera o ângulo atual da IMU
        imu.resetYaw();

        while (opModeIsActive()) {

            controlMecanum();

            controlShooterAndSpindexer();

            controlFeeder();

            telemetry.addData(
                    "Heading",
                    "%.1f graus",
                    getHeading()
            );

            telemetry.addData(
                    "IMU no strafe",
                    headingLockActive
                            ? "ATIVA"
                            : "DESATIVADA"
            );

            telemetry.addData(
                    "Angulo alvo",
                    "%.1f graus",
                    targetHeading
            );

            telemetry.addData(
                    "Shooter",
                    "%.1f",
                    shooterPower
            );

            telemetry.update();
        }

        stopAllMotors();
    }

    // =========================================================
    // CONTROLE MECANUM
    // =========================================================

    private void controlMecanum() {

        // =====================================================
        // LEITURA DOS ANALÓGICOS
        // =====================================================

        // Frente / trás
        double y =
                -gamepad1.left_stick_y;

        /*
         * STRAFE
         *
         * O sinal foi invertido porque no seu robô:
         *
         * Analógico esquerda -> robô ia para direita
         * Analógico direita -> robô ia para esquerda
         */
        double x =
                -gamepad1.left_stick_x;

        // Giro manual
        double manualRotation =
                gamepad1.right_stick_x;

        // =====================================================
        // DEADZONE
        // =====================================================

        y = applyDeadzone(y, 0.08);

        x = applyDeadzone(x, 0.08);

        manualRotation =
                applyDeadzone(
                        manualRotation,
                        0.08
                );

        // =====================================================
        // DETECTA STRAFE PURO
        // =====================================================

        boolean pureStrafe =

                Math.abs(x) > 0.15

                        && Math.abs(y) < 0.15

                        && Math.abs(manualRotation) < 0.15;

        double rotation;

        // =====================================================
        // STRAFE COM HEADING LOCK
        // =====================================================

        if (pureStrafe) {

            /*
             * Essa parte acontece apenas UMA VEZ
             * quando o strafe começa.
             */
            if (!headingLockActive) {

                // Salva o ângulo inicial
                targetHeading =
                        getHeading();

                headingLockActive =
                        true;

                // Reinicia o controlador
                lastHeadingError =
                        0.0;

                headingTimer.reset();
            }

            // Ângulo atual
            double currentHeading =
                    getHeading();

            // =================================================
            // ERRO DE ÂNGULO
            // =================================================

            double headingError =
                    normalizeAngle(
                            currentHeading
                                    - targetHeading
                    );

            // =================================================
            // TEMPO ENTRE OS LOOPS
            // =================================================

            double deltaTime =
                    headingTimer.seconds();

            headingTimer.reset();

            // Evita divisão por zero
            if (deltaTime < 0.001) {
                deltaTime = 0.001;
            }

            // =================================================
            // DERIVADA
            // =================================================

            double derivative =
                    (
                            headingError
                                    - lastHeadingError
                    )
                            / deltaTime;

            // =================================================
            // CONTROLE PD
            // =================================================

            rotation =

                    (headingError * HEADING_KP)

                            +

                            (derivative * HEADING_KD);

            // Salva o erro atual
            lastHeadingError =
                    headingError;

            // Limita a correção
            rotation =
                    clamp(

                            rotation,

                            -MAX_HEADING_CORRECTION,

                            MAX_HEADING_CORRECTION
                    );

        } else {

            // =================================================
            // FORA DO STRAFE
            // =================================================

            headingLockActive =
                    false;

            // Piloto controla o giro normalmente
            rotation =
                    manualRotation;
        }

        // =====================================================
        // CÁLCULO MECANUM
        // =====================================================

        double frontLeftPower =

                y
                        + x
                        + rotation;


        double backLeftPower =

                y
                        - x
                        + rotation;


        double frontRightPower =

                y
                        - x
                        - rotation;


        double backRightPower =

                y
                        + x
                        - rotation;

        // =====================================================
        // NORMALIZAÇÃO
        // =====================================================

        double denominator =

                Math.max(

                        Math.abs(frontLeftPower),

                        Math.max(

                                Math.abs(backLeftPower),

                                Math.max(

                                        Math.abs(frontRightPower),

                                        Math.abs(backRightPower)

                                )
                        )
                );

        denominator =
                Math.max(
                        denominator,
                        1.0
                );

        frontLeftPower /=
                denominator;

        backLeftPower /=
                denominator;

        frontRightPower /=
                denominator;

        backRightPower /=
                denominator;

        // =====================================================
        // ENVIA POTÊNCIA PARA OS MOTORES
        // =====================================================

        leftFront.setPower(
                frontLeftPower
        );

        leftBack.setPower(
                backLeftPower
        );

        rightFront.setPower(
                frontRightPower
        );

        rightBack.setPower(
                backRightPower
        );
    }

    // =========================================================
    // SHOOTER E SPINDEXER
    // =========================================================

    private void controlShooterAndSpindexer() {

        boolean currentRightTrigger =

                gamepad2.right_trigger
                        > 0.5;


        boolean currentLeftTrigger =

                gamepad2.left_trigger
                        > 0.5;


        // =====================================================
        // RIGHT TRIGGER
        // =====================================================

        if (
                currentRightTrigger

                        &&

                        !lastRightTrigger
        ) {

            if (shooterPower == 1.0) {

                shooterPower =
                        0.0;

            } else {

                shooterPower =
                        1.0;
            }
        }

        // =====================================================
        // LEFT TRIGGER
        // =====================================================

        if (
                currentLeftTrigger

                        &&

                        !lastLeftTrigger
        ) {

            if (shooterPower == -1.0) {

                shooterPower =
                        0.0;

            } else {

                shooterPower =
                        -1.0;
            }
        }

        // Atualiza estado dos triggers

        lastRightTrigger =
                currentRightTrigger;

        lastLeftTrigger =
                currentLeftTrigger;


        // Shooter

        shooter.setPower(
                shooterPower
        );


        // Spindexer

        spindexer.setPower(
                shooterPower * 0.8
        );
    }

    // =========================================================
    // FEEDER
    // =========================================================

    private void controlFeeder() {

        if (gamepad2.right_bumper) {

            feeder.setPower(
                    1.0
            );

        } else if (gamepad2.left_bumper) {

            feeder.setPower(
                    -1.0
            );

        } else {

            feeder.setPower(
                    0.0
            );
        }
    }

    // =========================================================
    // LEITURA DA IMU
    // =========================================================

    private double getHeading() {

        return imu

                .getRobotYawPitchRollAngles()

                .getYaw(
                        AngleUnit.DEGREES
                );
    }

    // =========================================================
    // NORMALIZAÇÃO DO ÂNGULO
    // =========================================================

    private double normalizeAngle(
            double angle
    ) {

        while (angle > 180.0) {

            angle -=
                    360.0;
        }

        while (angle < -180.0) {

            angle +=
                    360.0;
        }

        return angle;
    }

    // =========================================================
    // DEADZONE
    // =========================================================

    private double applyDeadzone(
            double value,
            double deadzone
    ) {

        if (
                Math.abs(value)
                        < deadzone
        ) {

            return 0.0;
        }

        return value;
    }

    // =========================================================
    // LIMITADOR
    // =========================================================

    private double clamp(
            double value,
            double minimum,
            double maximum
    ) {

        return Math.max(

                minimum,

                Math.min(
                        maximum,
                        value
                )
        );
    }

    // =========================================================
    // PARA TODOS OS MOTORES
    // =========================================================

    private void stopAllMotors() {

        if (leftFront != null) {

            leftFront.setPower(
                    0.0
            );
        }

        if (leftBack != null) {

            leftBack.setPower(
                    0.0
            );
        }

        if (rightFront != null) {

            rightFront.setPower(
                    0.0
            );
        }

        if (rightBack != null) {

            rightBack.setPower(
                    0.0
            );
        }

        if (shooter != null) {

            shooter.setPower(
                    0.0
            );
        }

        if (feeder != null) {

            feeder.setPower(
                    0.0
            );
        }

        if (spindexer != null) {

            spindexer.setPower(
                    0.0
            );
        }
    }
}