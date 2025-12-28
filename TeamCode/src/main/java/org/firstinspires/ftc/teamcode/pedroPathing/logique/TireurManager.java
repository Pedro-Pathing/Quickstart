package org.firstinspires.ftc.teamcode.pedroPathing.logique;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.SpinTurret;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.AngleShooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Indexeur;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.ServoTireur;

public class TireurManager {

    // --- Modules contrôlés ---
    private final Shooter shooter;
    private final SpinTurret tourelle;
    private final AngleShooter angleShooter;
    private final ServoTireur servoTireur;
    private final Indexeur indexeur;

    // --- Machine à états ---
    public enum TirState {
        IDLE,
        SHOOTER_SPINUP,
        TURRET_POSITION,
        ANGLE_POSITION,
        SERVO_PUSH,
        SERVO_RETRACT,
        INDEX_ADVANCE,
        WAIT_AFTER_INDEX
    }

    private TirState state = TirState.IDLE;
    private final ElapsedTime timer = new ElapsedTime();
    private int tirsEffectues = 0;

    // --- Cibles dynamiques ---
    private double angleCibleTourelle = 0;
    private double angleCibleShooter = 0;
    private double vitesseCibleShooter = 0;

    public TireurManager(Shooter shooter,
                         SpinTurret tourelle,
                         AngleShooter angleShooter,
                         ServoTireur servoTireur,
                         Indexeur indexeur) {

        this.shooter = shooter;
        this.tourelle = tourelle;
        this.angleShooter = angleShooter;
        this.servoTireur = servoTireur;
        this.indexeur = indexeur;
    }

    public void update() {
        switch (state) {

            case IDLE:
                break;

            // --- 1) Shooter spin-up ---
            case SHOOTER_SPINUP:
                tourelle.allerVersAngle(angleCibleTourelle);

                if (shooter.getShooterVelocityRPM() >= vitesseCibleShooter) {
                    timer.reset();
                    state = TirState.TURRET_POSITION;
                }
                break;

            // --- 2) Positionnement tourelle ---
            case TURRET_POSITION:
                tourelle.allerVersAngle(angleCibleTourelle);

                if (tourelle.isAtAngle(angleCibleTourelle)) {
                    timer.reset();
                    state = TirState.ANGLE_POSITION;
                }
                break;

            // --- 3) Positionnement angle shooter ---
            case ANGLE_POSITION:
                angleShooter.setAngle(angleCibleShooter);

                if (angleShooter.isAtAngle(angleCibleShooter)) {
                    timer.reset();
                    state = TirState.SERVO_PUSH;
                }
                break;

            // --- 4) Pousser la balle ---
            case SERVO_PUSH:
                servoTireur.push();

                if (timer.milliseconds() > 200) {
                    timer.reset();
                    state = TirState.SERVO_RETRACT;
                }
                break;

            // --- 5) Rétracter le servo ---
            case SERVO_RETRACT:
                servoTireur.retract();

                if (timer.milliseconds() > 200) {

                    tirsEffectues++;   // Tir réellement terminé ici

                    if (tirsEffectues >= 3) {
                        shooter.setShooterTargetRPM(0);
                        state = TirState.IDLE;
                    } else {
                        indexeur.avancerPourTir();
                        timer.reset();
                        state = TirState.INDEX_ADVANCE;
                    }
                }
                break;

            // --- 6) Attendre fin rotation indexeur ---
            case INDEX_ADVANCE:
                if (indexeur.isRotationTerminee()) {
                    timer.reset();
                    state = TirState.WAIT_AFTER_INDEX;
                }
                break;

            // --- 7) Petite pause avant tir suivant ---
            case WAIT_AFTER_INDEX:
                if (timer.milliseconds() > 150) {
                    timer.reset();
                    state = TirState.SERVO_PUSH;
                }
                break;
        }
    }

    // --- Lancer un tir automatique ---
    public void startTirAuto(double angleTourelle, double angleShooter, double vitesseShooter) {
        this.angleCibleTourelle = angleTourelle;
        this.angleCibleShooter = angleShooter;
        this.vitesseCibleShooter = vitesseShooter;

        tirsEffectues = 0;

        shooter.setShooterTargetRPM(vitesseShooter);  // Démarre immédiatement
        state = TirState.SHOOTER_SPINUP;
    }

    public TirState getState() {
        return state;
    }
}
