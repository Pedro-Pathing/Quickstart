package org.firstinspires.ftc.teamcode.pedroPathing.TeleOp;

import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Indexeur;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.SpinTurret;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.AngleShooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.ServoTireur;
import org.firstinspires.ftc.teamcode.pedroPathing.logique.TireurManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import java.util.function.Supplier;
@TeleOp(name="Test Tir Auto Teleop", group="Test")

public class TestTirAuto extends OpMode {

        // --- Modules ---
        private Shooter shooter;
        private SpinTurret tourelle;
        private AngleShooter angleShooter;
        private ServoTireur servoTireur;
        private Indexeur indexeur;

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

            shooter = new Shooter();
            shooter.setIndexeur(indexeur);   // ✔️ on passe l’indexeur
            shooter.init(hardwareMap);       // ✔️ on initialise le hardware


            servoTireur = new ServoTireur(indexeur);  // ✔️ constructeur correct
            servoTireur.init(hardwareMap);            // ✔️ initialisation du servo


            tireurManager = new TireurManager(shooter, tourelle, angleShooter, servoTireur, indexeur);

            telemetry.addLine(">>> Test Tir Auto prêt <<<");
        }



    @Override
        public void loop() {

            // --- Déclenchement tir auto ---
            if (gamepad2.x && !lastX) {

                // Exemple : tir droit devant
                double angleTourelle = -30;      // à adapter
                double angleShooter = 15;      // à adapter
                double vitesseShooter = 500;  // à adapter

                tireurManager.startTirAuto(angleTourelle, angleShooter, vitesseShooter);
            }

            lastX = gamepad2.x;

            // --- Mise à jour du manager ---
            tireurManager.update();

            // --- Telemetry utile ---
            telemetry.addLine("=== TIR AUTO ===");
            telemetry.addData("État", tireurManager.getState());
            telemetry.addData("Shooter RPM", shooter.getShooterVelocityRPM());
            telemetry.addData("Servo pos", servoTireur.getPosition());
            telemetry.addData("Index rotation finie", indexeur.isRotationTerminee());
            telemetry.update();
        }
    }


