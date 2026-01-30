package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp.Debug;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.AfficheurLeft;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.AfficheurRight;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Indexeur;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.SpinTurret;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.AngleShooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.ServoTireur;
import org.firstinspires.ftc.teamcode.pedroPathing.logique.TireurManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name="Test Tir Auto", group="debug")

public class TestTirAuto extends OpMode {

        // --- Modules ---
        private Shooter shooter;
        private SpinTurret tourelle;
        private AngleShooter angleShooter;
        private ServoTireur servoTireur;
        private Indexeur indexeur;
        private Intake intake;

        private AfficheurLeft afficheurLeft;
        private AfficheurRight afficheurRight;

        // --- Manager ---
        private TireurManager tireurManager;

        // Pour détecter un appui unique
        private boolean lastX = false;

        @Override

        public void init() {

            tourelle = new SpinTurret();
            tourelle.init(hardwareMap);

            angleShooter = new AngleShooter();
            angleShooter.init(hardwareMap);
            indexeur = new Indexeur();
            indexeur.init(hardwareMap);

            afficheurLeft = new AfficheurLeft();
            afficheurLeft.init(hardwareMap);

            afficheurRight = new AfficheurRight();
            afficheurRight.init(hardwareMap);


            intake = new Intake(indexeur, afficheurLeft);
            intake.init(hardwareMap);
            indexeur.setIntake(intake);

            shooter = new Shooter();
            shooter.setIndexeur(indexeur);   // ✔️ on passe l’indexeur
            shooter.init(hardwareMap);       // ✔️ on initialise le hardware


            servoTireur = new ServoTireur(indexeur);  // ✔️ constructeur correct
            servoTireur.init(hardwareMap);            // ✔️ initialisation du servo
            tireurManager = new TireurManager(shooter, tourelle, angleShooter, servoTireur, indexeur, intake, afficheurRight);
            ;
            telemetry.addLine(">>> Test Tir Auto prêt <<<");
        }



    @Override
        public void loop() {

            // --- Déclenchement tir auto ---
            if (gamepad2.x && !lastX) {

                // Exemple : tir droit devant
                double angleTourelle = 0;      // à adapter
                double angleShooter = 15;      // à adapter
                double vitesseShooter = 4800;  // à adapter

                tireurManager.startTirAuto(angleTourelle, angleShooter, vitesseShooter);
            }


            lastX = gamepad2.x;
            // --- Mise à jour du manager ---

            intake.update();
            indexeur.update();
            tireurManager.update();


            // --- Telemetry utile ---
            telemetry.addLine("=== TIR AUTO ===");
            telemetry.addData("État tireur manager", tireurManager.getState());
            telemetry.addData("État indexeur", indexeur.getEtat());
            telemetry.addData("Etat de l'intake", intake.getEtat());
            telemetry.addData("Shooter RPM", shooter.getShooterVelocityRPM());
            telemetry.addData("Servo pos", servoTireur.getPosition());
            telemetry.addData("Index rotation finie", indexeur.isRotationTerminee());
            telemetry.addData("Nombre de balles", indexeur.getBalles());
            telemetry.update();
        }
    }


