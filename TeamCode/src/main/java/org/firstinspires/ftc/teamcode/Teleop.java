package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Código de Teleoperado para a equipe MEGA.
 * Chassi Mecanum sem Sensores (Sem IMU).
 * Inclui Compensação de Strafe (Bias), Shooter, Spindexer, Feeder e Servos com Delay de 2.5s.
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
    private double SERVO_RESET = 0.0;       // Posição de RESET (0 graus)
    private double SERVO_ATIVO = 0.5;       // Posição de 90 graus (aproximadamente 0.5 na maioria dos servos)
    private double TEMPO_ESPERA = 2.5;      // Tempo em segundos para o motor atingir o RPM máximo

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

        // Timer para o delay sincronizado
        ElapsedTime shooterTimer = new ElapsedTime();
        boolean aguardandoAceleracao = false;

        telemetry.addLine("Pronto! (Sem IMU)");
        telemetry.update();

        // Inicia servos na posição de reset (0)
        servoEsquerdo.setPosition(SERVO_RESET);
        servoDireito.setPosition(SERVO_RESET);

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

            // Lógica de Toggle RT (Lado Direito / Trás)
            if (gatilhoDireitoAtual && !ultimoGatilhoDireito) {
                if (shooterActivePower == -1.0) {
                    shooterActivePower = 0;
                    aguardandoAceleracao = false;
                } else {
                    shooterActivePower = -1.0;
                    shooterTimer.reset(); // Inicia contagem de 2.5s
                    aguardandoAceleracao = true;
                }
            }
            
            // Lógica de Toggle LT (Lado Esquerdo / Frente)
            if (gatilhoEsquerdoAtual && !ultimoGatilhoEsquerdo) {
                if (shooterActivePower == 1.0) {
                    shooterActivePower = 0;
                    aguardandoAceleracao = false;
                } else {
                    shooterActivePower = 1.0;
                    shooterTimer.reset(); // Inicia contagem de 2.5s
                    aguardandoAceleracao = true;
                }
            }

            ultimoGatilhoDireito = gatilhoDireitoAtual;
            ultimoGatilhoEsquerdo = gatilhoEsquerdoAtual;

            // Liga o Shooter imediatamente
            shooter.setPower(shooterActivePower);

            // --- Lógica Sincronizada (Spindexer + Servos) ---
            if (shooterActivePower != 0) {
                if (aguardandoAceleracao) {
                    if (shooterTimer.seconds() >= TEMPO_ESPERA) {
                        // Passou o tempo de espera: liga Spindexer e move o Servo correto
                        spindexer.setPower(shooterActivePower * 0.8);
                        
                        if (shooterActivePower == 1.0) {
                            servoEsquerdo.setPosition(SERVO_ATIVO);
                            servoDireito.setPosition(SERVO_RESET);
                        } else {
                            servoDireito.setPosition(SERVO_ATIVO);
                            servoEsquerdo.setPosition(SERVO_RESET);
                        }
                    } else {
                        // Ainda acelerando: mantém Spindexer e Servos parados
                        spindexer.setPower(0);
                        servoEsquerdo.setPosition(SERVO_RESET);
                        servoDireito.setPosition(SERVO_RESET);
                    }
                } else {
                    // Estado de manutenção (se não for o toggle inicial)
                    spindexer.setPower(shooterActivePower * 0.8);
                    if (shooterActivePower == 1.0) servoEsquerdo.setPosition(SERVO_ATIVO);
                    else servoDireito.setPosition(SERVO_ATIVO);
                }
            } else {
                // TUDO DESLIGADO -> RESET IMEDIATO
                spindexer.setPower(0);
                servoEsquerdo.setPosition(SERVO_RESET);
                servoDireito.setPosition(SERVO_RESET);
                aguardandoAceleracao = false;
            }

            // --- CONTROLE DO FEEDER (Gamepad 2) ---
            if (gamepad2.right_bumper) feeder.setPower(1.0);
            else if (gamepad2.left_bumper) feeder.setPower(-1.0);
            else feeder.setPower(0);

            // Telemetria
            telemetry.addData("Shooter", shooterActivePower);
            telemetry.addData("Spindexer", spindexer.getPower());
            telemetry.addData("Timer", "%.2f", shooterTimer.seconds());
            telemetry.addData("Servo Esq", servoEsquerdo.getPosition());
            telemetry.addData("Servo Dir", servoDireito.getPosition());
            telemetry.update();
        }
    }
}
