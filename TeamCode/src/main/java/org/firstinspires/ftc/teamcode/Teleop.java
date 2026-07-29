package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Código de Teleoperado para a equipe MEGA.
 * Chassi Mecanum sem Sensores (Sem IMU).
 * Inclui Compensação de Strafe (Bias), Shooter, Spindexer, Feeder e Servos com Delay.
 */
@TeleOp(name = "Teleoperado", group = "TeleOp")
public class Teleop extends LinearOpMode {

    // Chassi (nomes de motores mantidos em inglês)
    private DcMotor leftFront, leftBack, rightBack, rightFront;

    // Mecanismos (nomes de motores mantidos em inglês)
    private DcMotor spindexer, feeder, shooter;
    private Servo servoEsquerdo, servoDireito;

    // --- CONFIGURAÇÃO DE COMPENSAÇÃO MANUAL (BIAS) ---
    private double FATOR_COMPENSACAO_STRAFE = 0.8;

    // --- CONFIGURAÇÃO DOS SERVOS ---
    // Posições (0.0 a 1.0). Ajuste conforme a montagem do seu robô.
    private double SERVO_INATIVO = 0.0;     // Posição inicial (recolhido)
    private double SERVO_ATIVO = 0.55;      // Posição de 100 graus (aproximadamente 0.55 na maioria dos servos)
    private double TEMPO_ACELERACAO = 1.0;  // Tempo em segundos para o motor atingir o RPM máximo

    @Override
    public void runOpMode() {

        // Mapeamento de Hardware
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        spindexer = hardwareMap.get(DcMotor.class, "Spindexer");
        feeder    = hardwareMap.get(DcMotor.class, "feeder");
        shooter   = hardwareMap.get(DcMotor.class, "shooter");

        servoEsquerdo = hardwareMap.get(Servo.class, "servoLeft");
        servoDireito  = hardwareMap.get(Servo.class, "servoRight");

        // Direção dos motores
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Comportamento Zero Power
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Variáveis de Estado
        boolean ultimoGatilhoDireito = false;
        boolean ultimoGatilhoEsquerdo = false;
        double shooterActivePower = 0;

        // Timer para o delay dos servos
        ElapsedTime shooterTimer = new ElapsedTime();
        boolean aguardandoAceleracao = false;

        telemetry.addLine("Pronto! (Sem IMU)");
        telemetry.update();

        // Inicia servos na posição idle
        servoEsquerdo.setPosition(SERVO_INATIVO);
        servoDireito.setPosition(SERVO_INATIVO);

        waitForStart();

        while (opModeIsActive()) {

            // --- CONTROLE DE MOVIMENTAÇÃO (Gamepad 1) ---
            double eixoY = -gamepad1.left_stick_y;
            double eixoX = gamepad1.left_stick_x;
            double rotacao = gamepad1.right_stick_x;

            double multiplicadorEsquerdo = 1.0;
            double multiplicadorDireito = 1.0;

            if (eixoX < -0.1) {
                multiplicadorDireito = 1.0 + (Math.abs(eixoX) * FATOR_COMPENSACAO_STRAFE);
            } else if (eixoX > 0.1) {
                multiplicadorEsquerdo = 1.0 + (Math.abs(eixoX) * FATOR_COMPENSACAO_STRAFE);
            }

            double fl = (eixoY + eixoX + rotacao) * multiplicadorEsquerdo;
            double bl = (eixoY - eixoX + rotacao) * multiplicadorEsquerdo;
            double fr = (eixoY - eixoX - rotacao) * multiplicadorDireito;
            double br = (eixoY + eixoX - rotacao) * multiplicadorDireito;

            double potenciaMaxima = Math.max(Math.abs(fl), Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br))));
            if (potenciaMaxima > 1.0) {
                fl /= potenciaMaxima; bl /= potenciaMaxima; fr /= potenciaMaxima; br /= potenciaMaxima;
            }

            leftFront.setPower(fl);
            leftBack.setPower(bl);
            rightFront.setPower(fr);
            rightBack.setPower(br);


            // --- CONTROLE DO SHOOTER, SPINDEXER E SERVOS (Gamepad 2) ---
            boolean gatilhoDireitoAtual = gamepad2.right_trigger > 0.5;
            boolean gatilhoEsquerdoAtual = gamepad2.left_trigger > 0.5;

            // Lógica de Toggle
            if (gatilhoDireitoAtual && !ultimoGatilhoDireito) {
                if (shooterActivePower == -1.0) {
                    shooterActivePower = 0;
                    aguardandoAceleracao = false;
                } else {
                    shooterActivePower = -1.0;
                    shooterTimer.reset(); // Inicia contagem para spin-up
                    aguardandoAceleracao = true;
                }
            }
            if (gatilhoEsquerdoAtual && !ultimoGatilhoEsquerdo) {
                if (shooterActivePower == 1.0) {
                    shooterActivePower = 0;
                    aguardandoAceleracao = false;
                } else {
                    shooterActivePower = 1.0;
                    shooterTimer.reset(); // Inicia contagem para spin-up
                    aguardandoAceleracao = true;
                }
            }

            ultimoGatilhoDireito = gatilhoDireitoAtual;
            ultimoGatilhoEsquerdo = gatilhoEsquerdoAtual;

            shooter.setPower(shooterActivePower);
            spindexer.setPower(shooterActivePower * 0.8);

            // Controle dos Servos com Delay
            if (shooterActivePower == 1.0) { // Ativado pelo gatilho direito
                servoEsquerdo.setPosition(SERVO_INATIVO);

                if (aguardandoAceleracao && shooterTimer.seconds() >= TEMPO_ACELERACAO)
                {
                    servoDireito.setPosition(SERVO_ATIVO); // Servo direito gira após delay
                }
                else if (!aguardandoAceleracao)
                {
                    servoDireito.setPosition(SERVO_ATIVO);
                }
            } else if (shooterActivePower == -1.0) { // Ativado pelo gatilho esquerdo
                servoDireito.setPosition(SERVO_INATIVO);
                if (aguardandoAceleracao && shooterTimer.seconds() >= TEMPO_ACELERACAO) {
                    servoEsquerdo.setPosition(SERVO_ATIVO); // Servo esquerdo gira após delay
                } else if (!aguardandoAceleracao) {
                    servoEsquerdo.setPosition(SERVO_ATIVO);
                }
            } else {
                // Desligado: servos voltam para idle
                servoEsquerdo.setPosition(SERVO_INATIVO);
                servoDireito.setPosition(SERVO_INATIVO);
                aguardandoAceleracao = false;
            }

            // --- CONTROLE DO FEEDER (Gamepad 2) ---
            if (gamepad2.right_bumper) feeder.setPower(1.0);
            else if (gamepad2.left_bumper) feeder.setPower(-1.0);
            else feeder.setPower(0);

            // Telemetria
            telemetry.addData("Shooter", shooterActivePower);
            telemetry.addData("Timer", "%.2f", shooterTimer.seconds());
            telemetry.update();
        }
    }
}