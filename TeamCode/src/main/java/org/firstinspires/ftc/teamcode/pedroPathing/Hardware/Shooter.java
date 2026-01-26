package org.firstinspires.ftc.teamcode.pedroPathing.Hardware;
import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {
    private static final int TICKS_PER_REV_6000 = 28; // GoBilda 5203 ratio 1:1
    private int Shootpower;
    private static final double MAX_RPM = 6000.0;
    private DcMotorEx Shooter,Shooter2;
    private Indexeur indexeur;

    private enum Shooteretat {
        IDLE,  //Repos
        TirProche,
        TirMoyen,
        TirLoin,
        TirMax,

    }
    private Shooteretat shooteretat = Shooteretat.IDLE;
    private ElapsedTime timeretat = new ElapsedTime();
    private int shooterlowspeed= 4000;
    private int shootermediumspeed = 4500;
    private int shootermaxspeed = 5000;

    private double currentTargetVel = 0.0;

    public void init(@NonNull HardwareMap hwMap) {

        double maxVelRPM = 4700.0;
        double maxVel = (maxVelRPM * TICKS_PER_REV_6000) / 60.0;
        double kF = 1;
        double kP = 6.0; // Agressif car systeme rapide
        double kI = 0.0 ; // nuisible voir inutile avec shooter
        double kD = 0.4 ; // amortissement raisonnable

        this.indexeur = indexeur;
        Shooter = hwMap.get(DcMotorEx.class, "Shooter");
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //Shooter.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        //Test avec deux moteurs
        Shooter2 = hwMap.get(DcMotorEx.class, "Shooter2");
        Shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooter2.setDirection(DcMotor.Direction.REVERSE);
        Shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //Shooter2.setVelocityPIDFCoefficients(kP, kI, kD, kF);

    }
    public void update() {
        int ballcomptage = indexeur.getBalles();

        switch (shooteretat) {
            case IDLE:
                setShooterTargetRPM(0);
                if (ballcomptage > 2) {
                    timeretat.reset();
                    shooteretat = shooteretat.TirProche;
                }

                break;
            case TirProche:
                setShooterTargetRPM(shooterlowspeed);

                break;
            case TirMoyen:
                setShooterTargetRPM(shootermediumspeed);

                break;
            case TirLoin:
                setShooterTargetRPM(shootermaxspeed);

                break;
        }
    }

        public void setshooterFullPower(double Shootpower){
            // mettre le moteur a fond pour lancer de balle sans controle
            Shooter.setPower(Shootpower);
            Shooter2.setPower(Shootpower);

        }

        public void setShooterTargetRPM(double targetRPM) {

            //Conversion RPM -> ticks/sec
            double targetTicksPerSec = (targetRPM * TICKS_PER_REV_6000) / 60.0;
            Shooter.setVelocity(targetTicksPerSec);
            Shooter2.setVelocity(targetTicksPerSec);
        }
        public void displayShooterVelocity(Telemetry telemetry) {
            telemetry.addData("Shooter Speed (RPM)", getShooterVelocityRPM());
        }

        public double getShooter2VelocityRPM() {
            double ticksPerSec = Shooter2.getVelocity();
            return (ticksPerSec * 60) / TICKS_PER_REV_6000;
        }

        // Méthode lire la vitesse du moteur de lancement Balle shooter et affichage
        public double getShooterVelocityRPM() {
            double ticksPerSec = Shooter.getVelocity();
            return (ticksPerSec * 60) / TICKS_PER_REV_6000;
        }
    public void setIndexeur(Indexeur indexeur) {
        this.indexeur = indexeur;
    }


    }

