package org.firstinspires.ftc.teamcode.PIDS;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Mech.MidTake;
import org.firstinspires.ftc.teamcode.Mech.Shooter;
import org.firstinspires.ftc.teamcode.Mech.TurnOn_TurnOff;

@TeleOp(name = "Shooter PIDF Tuning", group = "Tuning")
public class TunningShooters extends LinearOpMode {


    // =========================
    // VALORES INICIAIS
    // =========================

    double targetVelocity = 1455;


    private double Pshooter = 50;
    private double Fshooter = 15;
    private double PmidTake = 50;
    private double Fmidtake = 15;


    @Override
    public void runOpMode() {

        // =========================
        //      HARDWARE
        // =========================

        Shooter shooter = new Shooter();
        shooter.init(hardwareMap);
        MidTake midTake = new MidTake();
        midTake.init(hardwareMap);
        TurnOn_TurnOff button = new TurnOn_TurnOff();

            //P
        boolean lastDpadUp    = false;
        boolean lastDpadDown  = false;
        boolean lastDpadUp2   = false;
        boolean lastDpadDown2 = false;

            //F
        boolean lastRight  = false;
        boolean lastRight2 = false;
        boolean lastLeft   = false;
        boolean lastLeft2  = false;

            //TargetVelocity
        boolean lastA  = false, lastY = false;
        boolean B_ON = false, lastB = false;




        waitForStart();

        while(opModeIsActive()){

            boolean B = gamepad1.b;
            B_ON = button.IsActivate(B, lastB, B_ON);
            double velocityShooter = shooter.getVelocity();
            double velocityMidtake = midTake.getVelocity();


            //P
            if (gamepad1.dpad_up && !lastDpadUp ) Pshooter += 0.5;
            if (gamepad2.dpad_up && !lastDpadUp2) PmidTake += 0.5;


            lastDpadUp  = gamepad1.dpad_up;
            lastDpadUp2 = gamepad2.dpad_up;


            if (gamepad1.dpad_down && !lastDpadDown ) Pshooter -= 0.5;
            if (gamepad2.dpad_down && !lastDpadDown2) PmidTake -= 0.5;

            lastDpadDown  = gamepad1.dpad_down;
            lastDpadDown2 = gamepad2.dpad_down;


            //F

            if (gamepad1.dpad_right && !lastRight ) Fshooter += 0.5;
            if (gamepad2.dpad_right && !lastRight2) Fmidtake += 0.5;


            lastRight  = gamepad1.dpad_right;
            lastRight2 = gamepad2.dpad_right;


            if (gamepad1.dpad_left  && !lastLeft ) Fshooter -= 0.5;
            if (gamepad2.dpad_left&& !lastLeft2) Fmidtake -= 0.5;

            lastLeft  = gamepad1.dpad_left;
            lastLeft2 = gamepad2.dpad_left;


            if (B_ON){
                shooter.Tune(Pshooter, 0, 0, Fshooter, targetVelocity);
                midTake.Tune(PmidTake, 0, 0, Fmidtake, targetVelocity);
            }
            else{
                shooter.Tune(Pshooter, 0, 0, Fshooter, 0);
                midTake.Tune(PmidTake, 0, 0, Fmidtake, 0);
            }


            if (gamepad1.a && !lastA) targetVelocity += 20;
            if (gamepad1.y && !lastY) targetVelocity -= 20;

            lastB = B;
            lastA = gamepad1.a;
            lastY = gamepad1.y;




            //velocidades para debug

            telemetry.addData("B", B_ON);
            telemetry.addLine("===========================================================");
            telemetry.addData("P do shooter", Pshooter);
            telemetry.addData("F do shooter", Fshooter);
            telemetry.addLine("===========================================================");
            telemetry.addData("P do midtake", PmidTake);
            telemetry.addData("F do midtake", Fmidtake);
            telemetry.addLine("===========================================================");
            telemetry.addData("Target", targetVelocity);
            telemetry.addData("Vel. Shooter", velocityShooter);
            telemetry.addData("Vel. MidTake", velocityMidtake);
            telemetry.addLine("===========================================================");
            telemetry.addData("Erro Shooter", targetVelocity - velocityShooter);
            telemetry.addData("Erro MidTake", targetVelocity - velocityMidtake);
            telemetry.update();
            //==============================================
            //==============================================
        }
    }
}