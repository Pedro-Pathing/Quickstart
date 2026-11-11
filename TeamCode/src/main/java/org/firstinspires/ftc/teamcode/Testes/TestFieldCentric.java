package org.firstinspires.ftc.teamcode.Testes;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Autonomo.pedro.Constants;
import org.firstinspires.ftc.teamcode.Mech.FieldCentric;
import org.firstinspires.ftc.teamcode.Mech.TurnOn_TurnOff;

/*
 * Teste de drive FIELD-CENTRIC (perspectiva do piloto) usando o Follower do Pedro.
 *
 * Botoes (gamepad1):
 *   A        = alterna FIELD <-> ROBOT-centric (borda de subida)
 *   options  = reseta o heading do piloto para a direcao atual do robo
 *
 *   stick esquerdo x = direita do piloto, y = frente do piloto
 *   stick direito  x = giro (rx, nao rotacionado)
 */
@TeleOp(name = "Teste Ultimate Moviment", group = "Testes")
public class TestFieldCentric extends LinearOpMode {

    private final TurnOn_TurnOff btnA = new TurnOn_TurnOff();

    private boolean fieldCentric = true;
    private boolean lastA = false;
    private boolean lastOptions = false;

    @Override
    public void runOpMode() {

        Follower follower = Constants.create(hardwareMap);
        follower.update();
        FieldCentric fc = new FieldCentric(follower);
        // Frente do piloto = para onde o robo aponta no init
        fc.resetHeading();

        telemetry.addData("Status", "Pronto");
        telemetry.addData("Botoes", "A: alterna modo | options: reseta heading");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {


            double x  = gamepad1.left_stick_x;
            double y  = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;


            boolean options = gamepad1.options;
            if (options && !lastOptions) fc.resetHeading();
            lastOptions = options;


            boolean a = gamepad1.a;
            fieldCentric = btnA.IsActivate(a, lastA, fieldCentric);
            lastA = a;

            // robot-centric: mesma convencao do Pedro, sem rotacao
            DrivePowers p = fieldCentric ? fc.powers(-x, y, -rx) : new DrivePowers(y, -x, -rx);
            follower.manual(p);
            follower.update();

            double headingRad = follower.pose().heading();

            telemetry.addData("Modo", fieldCentric ? "FIELD" : "ROBOT");
            telemetry.addData("Heading (graus)", "%.1f", Math.toDegrees(headingRad));
            telemetry.addData("Heading rel piloto (graus)", "%.1f", norm180(headingRad - fc.getOffsetRad()));
            telemetry.addData("Offset (graus)", "%.1f", Math.toDegrees(fc.getOffsetRad()));
            telemetry.addData("x / y piloto", "%.2f / %.2f", x, y);
            telemetry.addData("forward / strafe(+esq) robo", "%.2f / %.2f", p.forward(), p.strafe());
            telemetry.addData("turn", "%.2f", p.turn());
            telemetry.addData("Botoes", "A: alterna modo | options: reseta heading");
            telemetry.update();
        }
    }

    private double norm180(double rad) {
        double d = Math.toDegrees(rad);
        while (d > 180.0) d -= 360.0;
        while (d < -180.0) d += 360.0;
        return d;
    }
}
