package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Código de Teleoperado Completo para a equipe MEGA.
 * Chassi Mecanum, Shooter, Spindexer, Feeder e Servos.
 * Sequência: Shooter -> Wait 2s -> Servo -> Wait 0.5s -> Spindexer.
 */
@TeleOp(name = "Teleoperado MEGA v3", group = "TeleOp")
public class Teleop extends LinearOpMode {

    // Chassi
    private DcMotor leftFront, leftBack, rightBack, rightFront;

    // Mecanismos
    private DcMotor spindexer, feeder, shooter;
    private Servo servoLeft, servoRight;

    // --- CONFIGURAÇÃO DE COMPENSAÇÃO MANUAL (BIAS) ---
    private double FATOR_COMPENSACAO_STRAFE = 0.8;

    // --- CONFIGURAÇÃO DOS SERVOS ---
    private double posZeroEsquerda = 0.0;
    private double posZeroDireita = 0.0;
    private double SERVO_ATIVO = 0.48; // Aproximadamente 70 graus
    
    // --- TEMPOS (TIMER) ---
    private double TEMPO_ESPERA_SHOOTER = 2.0;    // Tempo para o shooter acelerar
    private double TEMPO_MOVIMENTO_SERVO = 0.5;  // Tempo para o servo chegar na posição final

    @Override
    public void runOpMode() {

        // Hardware Map
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        spindexer  = hardwareMap.get(DcMotor.class, "Spindexer");
        feeder     = hardwareMap.get(DcMotor.class, "feeder");
        shooter    = hardwareMap.get(DcMotor.class, "shooter");
        servoLeft  = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");

        // Direção dos Motores
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Direção dos Servos
        servoLeft.setDirection(Servo.Direction.FORWARD);
        servoRight.setDirection(Servo.Direction.FORWARD);

        // Zero Power Behavior
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("--- AJUSTE MANUAL DOS SERVOS ---");
        telemetry.addLine("Ajuste os servos manualmente agora!");
        telemetry.addLine("START salvará a posição como ZERO.");
        telemetry.update();

        waitForStart();
        
        // Salvamos a posição atual como o "Zero"
        posZeroEsquerda = servoLeft.getPosition();
        posZeroDireita = servoRight.getPosition();

        // Variáveis de Estado
        boolean lastRT = false;
        boolean lastLT = false;
        double shooterPower = 0;
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {

            // --- MOVIMENTAÇÃO MECANUM ---
            double eixoY = -gamepad1.left_stick_y;
            double eixoX = gamepad1.left_stick_x;
            double rotacao = gamepad1.right_stick_x;

            double multEsq = 1.0;
            double multDir = 1.0;
            if (eixoX < -0.1) multDir = 1.0 + (Math.abs(eixoX) * FATOR_COMPENSACAO_STRAFE);
            else if (eixoX > 0.1) multEsq = 1.0 + (Math.abs(eixoX) * FATOR_COMPENSACAO_STRAFE);

            double fl = (eixoY + eixoX + rotacao) * multEsq;
            double bl = (eixoY - eixoX + rotacao) * multEsq;
            double fr = (eixoY - eixoX - rotacao) * multDir;
            double br = (eixoY + eixoX - rotacao) * multDir;

            double max = Math.max(Math.abs(fl), Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br))));
            if (max > 1.0) {
                fl /= max; bl /= max; fr /= max; br /= max;
            }

            leftFront.setPower(fl);
            leftBack.setPower(bl);
            rightFront.setPower(fr);
            rightBack.setPower(br);


            // --- MECANISMOS (GAMEPAD 2) ---
            boolean currentRT = gamepad2.right_trigger > 0.5;
            boolean currentLT = gamepad2.left_trigger > 0.5;

            // Toggle RT (Lado Direito / Negativo)
            if (currentRT && !lastRT) {
                if (shooterPower == -1.0) shooterPower = 0;
                else {
                    shooterPower = -1.0;
                    timer.reset(); 
                }
            }
            // Toggle LT (Lado Esquerdo / Positivo)
            if (currentLT && !lastLT) {
                if (shooterPower == 1.0) shooterPower = 0;
                else {
                    shooterPower = 1.0;
                    timer.reset(); 
                }
            }
            lastRT = currentRT;
            lastLT = currentLT;

            // Liga o Shooter imediatamente
            shooter.setPower(shooterPower);

            // --- LÓGICA DE SINCRONIZAÇÃO (Shooter -> Servo -> Spindexer) ---
            if (shooterPower != 0) {
                double tempoDecorrido = timer.seconds();

                // ETAPA 1: Shooter acelerando (0s até 2.0s)
                if (tempoDecorrido < TEMPO_ESPERA_SHOOTER) {
                    spindexer.setPower(0);
                    servoLeft.setPosition(posZeroEsquerda);
                    servoRight.setPosition(posZeroDireita);
                } 
                // ETAPA 2: Servo se movendo (2.0s até 2.5s)
                else if (tempoDecorrido < (TEMPO_ESPERA_SHOOTER + TEMPO_MOVIMENTO_SERVO)) {
                    spindexer.setPower(0); // Spindexer CONTINUA PARADO
                    
                    if (shooterPower == 1.0) { // LT
                        servoLeft.setPosition(Range.clip(posZeroEsquerda + SERVO_ATIVO, 0.0, 1.0));
                        servoRight.setPosition(posZeroDireita);
                    } else { // RT
                        servoRight.setPosition(Range.clip(posZeroDireita + SERVO_ATIVO, 0.0, 1.0));
                        servoLeft.setPosition(posZeroEsquerda);
                    }
                } 
                // ETAPA 3: Tudo liberado (Após 2.5s)
                else {
                    spindexer.setPower(shooterPower * 0.8); // SÓ LIGA AGORA
                    
                    if (shooterPower == 1.0) {
                        servoLeft.setPosition(Range.clip(posZeroEsquerda + SERVO_ATIVO, 0.0, 1.0));
                    } else {
                        servoRight.setPosition(Range.clip(posZeroDireita + SERVO_ATIVO, 0.0, 1.0));
                    }
                }
            } else {
                // RESET TOTAL IMEDIATO
                spindexer.setPower(0);
                servoLeft.setPosition(posZeroEsquerda);
                servoRight.setPosition(posZeroDireita);
            }

            // FEEDER
            if (gamepad2.right_bumper) feeder.setPower(1.0);
            else if (gamepad2.left_bumper) feeder.setPower(-1.0);
            else feeder.setPower(0);

            // TELEMETRIA
            telemetry.addData("Timer", "%.2f s", timer.seconds());
            if (shooterPower != 0) {
                if (timer.seconds() < TEMPO_ESPERA_SHOOTER) telemetry.addData("Fase", "Acelerando Shooter...");
                else if (timer.seconds() < (TEMPO_ESPERA_SHOOTER + TEMPO_MOVIMENTO_SERVO)) telemetry.addData("Fase", "Movendo Servo...");
                else telemetry.addData("Fase", "Lançamento Pronto!");
            }
            telemetry.update();
        }
    }
}
