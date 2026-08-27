package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TELEOPERADO COMPLETO", group = "LinearOpMode")
public class teleoperadoCompleto extends LinearOpMode {

    // Variáveis para a lógica do Toggle (liga/desliga) do Shooter
    private boolean shooterLigado = false;
    private boolean rtPressionado = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- MOTORES DA TRAÇÃO (PLAYER 1) ---
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");

        // --- MOTORES DO PLAYER 2 ---
        DcMotor feederMotor = hardwareMap.dcMotor.get("feeder");
        DcMotor shooterMotor = hardwareMap.dcMotor.get("shooter");

        // Inverter os motores do lado esquerdo para manter a tração alinhada
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Inicializado");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ==========================================
            // PLAYER 1: MOVIMENTAÇÃO MECANUM
            // ==========================================
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

            frontLeftMotor.setPower((y + x + rx) / denominator);
            backLeftMotor.setPower((y - x + rx) / denominator);
            frontRightMotor.setPower((y - x - rx) / denominator);
            backRightMotor.setPower((y + x - rx) / denominator);

            // ==========================================
            // PLAYER 2: CONTROLE DO FEEDER (LB / RB)
            // ==========================================
            double feederPower = 0.8;

            if (gamepad2.left_bumper) {
                feederMotor.setPower(-feederPower);
            } else if (gamepad2.right_bumper) {
                feederMotor.setPower(feederPower);
            } else {
                feederMotor.setPower(0.0);
            }
            
            // PLAYER 2: CONTROLE DO SHOOTER (RT - TOGGLE)
            // Considera o gatilho pressionado se passar de 50% do curso
            boolean rtAtual = gamepad2.right_trigger > 0.5;

            // Detecta a transição: Pressionou AGORA e NÃO estava pressionado antes
            if (rtAtual && !rtPressionado) {
                shooterLigado = !shooterLigado; // Inverte o estado (true vira false, false vira true)
            }
            rtPressionado = rtAtual; // Atualiza o estado anterior do botão

            // Aplica a potência no Shooter (sentido anti-horário: potência negativa)
            double shooterPower = 1.0; // Ajuste a velocidade do shooter aqui (0.0 a 1.0)
            if (shooterLigado) {
                shooterMotor.setPower(-shooterPower);
            } else {
                shooterMotor.setPower(0.0);
            }

            // TELEMETRIA
            telemetry.addData("P1 - Frente/Trás (Y)", y);
            telemetry.addData("P1 - Lateral (X)", x);
            telemetry.addData("P1 - Giro (RX)", rx);
            telemetry.addData("P2 - Feeder Potência", feederMotor.getPower());
            telemetry.addData("P2 - Shooter Status", shooterLigado ? "LIGADO" : "DESLIGADO");
            telemetry.update();
        }
    }
}