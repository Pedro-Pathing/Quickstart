package org.firstinspires.ftc.teamcode.pedroPathing.logique;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.AfficheurLeft;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.AfficheurRight;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.AngleShooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Indexeur;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.ServoTireur;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.SpinTurret;


public class TireurManagerTeleop {

    // --- Modules contrôlés ---
    private final Shooter shooter;
    private boolean tirEnCours = false;
    private final SpinTurret tourelle;
    private final AngleShooter ServoAngleShoot;
    private final ServoTireur servoTireur;
    private final Indexeur indexeur;
    private final Intake intake;

    private final AfficheurRight afficheurRight;

    // --- Machine à états ---
    public enum TirState {
        IDLE,
        SHOOTER_SPINUP,
        TURRET_POSITION,
        ANGLE_POSITION,
        SERVO_PUSH,
        SERVO_RETRACT,
        INDEX_ADVANCE,
        WAIT_AFTER_INDEX,
        AVANCE1TIR,
        AFTERWAIT_INDEX
    }

    private TirState state = TirState.IDLE;
    private final ElapsedTime timer = new ElapsedTime();
    private int tirsEffectues = 0;

    private int shotsRemaining = 0;

    private double Min_shooterRPM = 4000;
    private double TargetFlyWheelRPM = 4700;

    private double shootermaxspintime = 2;

    // --- Cibles dynamiques ---
    private double angleCibleShooter = 0;
    private double vitesseCibleShooter = 0;

    public TireurManagerTeleop(Shooter shooter,
                               SpinTurret tourelle,
                               AngleShooter ServoAngleShoot,
                               ServoTireur servoTireur,
                               Indexeur indexeur, Intake intake, AfficheurRight afficheurRight) {

        this.shooter = shooter;
        this.tourelle = tourelle;
        this.ServoAngleShoot = ServoAngleShoot;
        this.servoTireur = servoTireur;
        this.indexeur = indexeur;
        this.intake = intake;
        this.afficheurRight = afficheurRight;
    }

    public void update() {

        boolean tirActif = (state != TirState.IDLE);
        if (tirActif) {
            afficheurRight.setClignoteVert();
        } else {
            afficheurRight.setIdle();
        }

        afficheurRight.update();

        switch (state) {

            case IDLE:
                break;

            // --- 4) Pousser la balle ---

            case AVANCE1TIR:
                afficheurRight.setRouge();
                shooter.setShooterTargetRPM(vitesseCibleShooter);
                if (!indexeur.isHomingDone()) {
                    indexeur.lancerHoming();
                    return;
                }
                if (indexeur.isHomingDone()) {
                    indexeur.avancerPourTir();
                    timer.reset();
                    state = TirState.WAIT_AFTER_INDEX;
                }

                break;

            case ANGLE_POSITION:
                shooter.setShooterTargetRPM(vitesseCibleShooter);
                ServoAngleShoot.setAngle(angleCibleShooter);
                afficheurRight.setJaune();

                if (ServoAngleShoot.isAtAngle(angleCibleShooter)) {
                    timer.reset();
                    state = TirState.SHOOTER_SPINUP;
                }
                break;

            case SHOOTER_SPINUP:
                shooter.setShooterTargetRPM(vitesseCibleShooter);
                double toleranceVelocityMax = 1.05 * vitesseCibleShooter;
                double toleranceVelocityMin = 0.95 * vitesseCibleShooter;
                if ((shooter.getShooterVelocityRPM() > toleranceVelocityMin) && (shooter.getShooterVelocityRPM() < toleranceVelocityMax)){;
                    timer.reset();
                    state = TirState.SERVO_PUSH;
                };
                break;

            case SERVO_PUSH:
                afficheurRight.setVert();
                if (!indexeur.isHomingDone()) {
                    indexeur.lancerHoming();
                    return;
                }
                if (indexeur.isHomingDone()) {
                    servoTireur.push();
                    if (timer.milliseconds() > 300) {
                        timer.reset();

                        indexeur.decrementerBalle();
                        state = TirState.SERVO_RETRACT;
                    }
                }
                break;

            // --- 5) Rétracter le servo ---
            case SERVO_RETRACT:
                servoTireur.retract();
                if (timer.milliseconds() > 300) {
                    timer.reset();
                    shotsRemaining--; // retrait d'un tir
                    tirsEffectues++;
                    state = TirState.INDEX_ADVANCE;// Tir réellement terminé ici
                }
                break;

            // --- 6) Attendre fin rotation indexeur ---
            case INDEX_ADVANCE:
                if (shotsRemaining == 0) {
                    afficheurRight.setIdle();
                    shooter.setShooterTargetRPM(0);
                    intake.repriseApresTir();
                    state = TirState.IDLE;

                }
                if (!(shotsRemaining == 0)){
                    indexeur.avancerPourTir();
                    timer.reset();
                    state = TirState.WAIT_AFTER_INDEX;
                }
                break;

            // --- 7) Petite pause avant tir suivant ---
            case WAIT_AFTER_INDEX:
                if (timer.milliseconds() > 190){
                     state = TirState.AFTERWAIT_INDEX;
                    }
                break;

            case AFTERWAIT_INDEX:

                if (indexeur.isRotationTerminee()){
                    timer.reset();
                    state = TirState.ANGLE_POSITION;
                }
                break;
        }
    }

    // --- Lancer un tir automatique ---
    public void startTirManuel3Tirs(double angleShooter, double vitesseShooter) {
        intake.arretPourTir();
        tirEnCours = true;
        shotsRemaining = 3;
        tirsEffectues = 0;
        this.angleCibleShooter = angleShooter;
        this.vitesseCibleShooter = vitesseShooter;
        shooter.setShooterTargetRPM(vitesseShooter);
        timer.reset();
        state = TirState.ANGLE_POSITION;
    }

    public void startTirManuel1tir(double angleShooter, double vitesseShooter) {
        intake.arretPourTir();
        tirEnCours = true;
        shotsRemaining = 1;
        this.angleCibleShooter = angleShooter;
        this.vitesseCibleShooter = vitesseShooter;
        //this.vitesseCibleShooter = vitesseShooter;
        tirsEffectues = 0;
        shooter.setShooterTargetRPM(vitesseShooter);  // Démarre immédiatement
        timer.reset();
        state = TirState.AVANCE1TIR;


        //if ((shooter.getShooterVelocityRPM() >= vitesseShooter) && (indexeur.isHomingDone())) {
        //    state = TirState.SERVO_PUSH;
        //    timer.reset();
        //}
    }

    public TirState getState() {
        return state;
    }

    public boolean isBusy(){
        return state != TirState.IDLE;
    }

    public boolean isTirEnCours() {
        return tirEnCours; }
}
