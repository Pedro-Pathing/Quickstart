package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Debug;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class PIDFMoteurShooter1 extends OpMode {

    private static final int TICKS_PER_REV_6000 = 28; // GoBilda 5203 ratio 1:1

    private static final double MAX_RPM = 6000.0;
    private DcMotorEx Shooter, Shooter2;

    private ElapsedTime timeretat = new ElapsedTime();
    private double currentTargetVel = 0.0;
    public double highVelocity = 5000;
    public double lowVelocity = 3500;
    double curTargetVelocity = highVelocity;
    double F = 0;
    double P = 0;

    double[] stepSizes = {10.0, 1.0, 0.2, 0.001, 0.001};
    int stepIndex = 1;


    @Override
    public void init() {

        double maxVelRPM = 4700.0;
        double maxVel = (maxVelRPM * TICKS_PER_REV_6000) / 60.0;
        double kF = 13;
        double kP = 1; // Agressif car systeme rapide
        double kI = 0.0; // nuisible voir inutile avec shooter
        double kD = 0.0002; // amortissement raisonnable

        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

    }

    //Test avec deux moteurs
    //Shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
    //Shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //Shooter2.setDirection(DcMotor.Direction.REVERSE);
    //Shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    //Shooter2.setVelocityPIDFCoefficients(kP, kI, kD, kF);}
    //
    public void setshooterFullPower(double Shootpower) {
        // mettre le moteur a fond pour lancer de balle sans controle
        Shooter.setPower(Shootpower);
        //Shooter2.setPower(Shootpower);
    }

    public void setShooterTargetRPM(double targetRPM) {

        //Conversion RPM -> ticks/sec
        double targetTicksPerSec = (targetRPM * TICKS_PER_REV_6000) / 60.0;
        Shooter.setVelocity(targetTicksPerSec);

        /*if (targetRPM > 0) {
            double ramp = 10.0; //un peu plus nerveux qu'un ascenseur à dubai

            double tolerancemoteur = 25; // ticks/s  50PMx28/60

            double error = targetTicksPerSec - currentTargetVel;
            if (Math.abs(error) > tolerancemoteur) {
                double step = Math.copySign(Math.min(Math.abs(error), ramp), error);
                currentTargetVel += step;
            } else {
                currentTargetVel = targetTicksPerSec; // nous sommes assez proche, on reste sur cette zone de vitesse)
            }

            Shooter.setVelocity(currentTargetVel);}
            //Shooter2.setVelocity(currentTargetVel);}
        else{
                currentTargetVel = 0;
                Shooter.setVelocity(0.0);
            }
            */


        //setshooterFullPower(1);

    }

    //public double getShooter2VelocityRPM () {
    //double ticksPerSec = Shooter2.getVelocity();
    //return (ticksPerSec * 60) / TICKS_PER_REV_6000;

    // Méthode lire la vitesse du moteur de lancement Balle shooter et affichage
    public double getShooterVelocityRPM() {
        double ticksPerSec = Shooter.getVelocity();
        return (ticksPerSec * 60) / TICKS_PER_REV_6000;
    }

    @Override
    public void loop() {
        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }

            if (gamepad1.bWasPressed()) {
                stepIndex = (stepIndex + 1) % stepSizes.length;
            }

            if (gamepad1.dpadLeftWasPressed()) {
                F -= stepSizes[stepIndex];
            }
            if (gamepad1.dpadRightWasPressed()) {
                F += stepSizes[stepIndex];
            }

            if (gamepad1.dpadUpWasPressed()) {
                P += stepSizes[stepIndex];
            }
            if (gamepad1.dpadDownWasPressed()) {
                P -= stepSizes[stepIndex];
            }
            //setnew PIDF coefficient
            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
            Shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

            Shooter.setVelocity((curTargetVelocity));

            double curVelocity = Shooter.getVelocity();
            double error = curTargetVelocity - curVelocity;

            telemetry.addData("Shooter RPM", getShooterVelocityRPM());
            telemetry.addData("Target Velocity", curVelocity);
            telemetry.addData("Error", error);
            telemetry.addData("tuning P", P);
            telemetry.addData("Tuning F", F);
            telemetry.addData("Step Size", stepSizes[stepIndex]);

        }
    }
}


