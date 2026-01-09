package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Debug;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class Test2MoteurShooter extends OpMode {

        private static final int TICKS_PER_REV_6000 = 28; // GoBilda 5203 ratio 1:1

        private static final double MAX_RPM = 6000.0;
        private DcMotorEx Shooter, Shooter2;

        private ElapsedTime timeretat = new ElapsedTime();
        private double currentTargetVel = 0.0;

        @Override
        public void init () {

            double maxVelRPM = 4700.0;
            double maxVel = (maxVelRPM * TICKS_PER_REV_6000) / 60.0;
            double kF = 1;
            double kP = 6.0; // Agressif car systeme rapide
            double kI = 0.0; // nuisible voir inutile avec shooter
            double kD = 0.4; // amortissement raisonnable

            Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
            Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            //Shooter.setVelocityPIDFCoefficients(kP, kI, kD, kF);

            //Test avec deux moteurs
            Shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
            Shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Shooter2.setDirection(DcMotor.Direction.REVERSE);
            Shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            //Shooter2.setVelocityPIDFCoefficients(kP, kI, kD, kF);
            }


            public void setshooterFullPower ( double Shootpower){
                // mettre le moteur a fond pour lancer de balle sans controle
                Shooter.setPower(Shootpower);
                Shooter2.setPower(Shootpower);
            }

            public void setShooterTargetRPM ( double targetRPM){

                //Conversion RPM -> ticks/sec
                double targetTicksPerSec = (targetRPM * TICKS_PER_REV_6000) / 60.0;
                Shooter.setVelocity(targetTicksPerSec);
                Shooter2.setVelocity(targetTicksPerSec);

        }


        public double getShooter2VelocityRPM () {
        double ticksPerSec = Shooter2.getVelocity();
        return (ticksPerSec * 60) / TICKS_PER_REV_6000;
        }

        // Méthode lire la vitesse du moteur de lancement Balle shooter et affichage
        public double getShooterVelocityRPM () {
        double ticksPerSec = Shooter.getVelocity();
        return (ticksPerSec * 60) / TICKS_PER_REV_6000;
        }
        @Override
        public void loop () {

        setShooterTargetRPM(4000);

        telemetry.addData("vitesse", getShooterVelocityRPM());
        telemetry.addData("vitesseshoot2",getShooter2VelocityRPM());

        }
    }

