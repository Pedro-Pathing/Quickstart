package org.firstinspires.ftc.teamcode.pedroPathing.logique;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.SpinTurret;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.AngleShooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Indexeur;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.ServoTireur;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.AfficheurRight;


public class TireurManager {

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
        WAIT_AFTER_INDEX
    }

    private TirState state = TirState.IDLE;
    private final ElapsedTime timer = new ElapsedTime();
    private int tirsEffectues = 0;

    private int shotsRemaining = 0;

    private double Min_shooterRPM = 4000;
    private double TargetFlyWheelRPM = 4700;

    private double shootermaxspintime = 2;

    // --- Cibles dynamiques ---
    private double angleCibleTourelle = 0;
    private double angleCibleShooter = 0;
    private double vitesseCibleShooter = 0;

    public TireurManager(Shooter shooter,
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

            // --- 1) Shooter spin-up ---
            case SHOOTER_SPINUP:
                if (shotsRemaining >0) {
                    intake.disableRamassage();
                    tourelle.allerVersAngle(angleCibleTourelle);
                    if (!indexeur.isHomingDone()) {
                        indexeur.lancerHoming();
                        return;
                    }
                    if ((shooter.getShooterVelocityRPM() >= vitesseCibleShooter) && (indexeur.isHomingDone())) {
                        state = TirState.TURRET_POSITION;
                        timer.reset();
                    }
                    else {state = TirState.IDLE;}
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
                ServoAngleShoot.setAngle(angleCibleShooter);
                state = TirState.SERVO_PUSH;
                timer.reset();
                //if (angleShooter.isAtAngle(angleCibleShooter)) {
                    //timer.reset();
                    //state = TirState.SERVO_PUSH;
                //}
                break;

            // --- 4) Pousser la balle ---
            case SERVO_PUSH:
                tourelle.stopTourelle();
                servoTireur.push();

                if (timer.milliseconds() > 300) {
                    timer.reset();
                    state = TirState.SERVO_RETRACT;
                }
                break;

            // --- 5) Rétracter le servo ---
            case SERVO_RETRACT:
                servoTireur.retract();

                if (timer.milliseconds() > 300) {

                    shotsRemaining--; // retrait d'un tir
                    tirsEffectues++;   // Tir réellement terminé ici


                    if (shotsRemaining == 0) {
                        shooter.setShooterTargetRPM(0);
                        intake.repriseApresTir();
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
                    tourelle.allerVersAngle(angleCibleTourelle);
                    if (tourelle.isAtAngle(angleCibleTourelle)) {
                        timer.reset();
                        state = TirState.SERVO_PUSH;
                    }
                    //tirEnCours = false;
                    //intake.enableRamassage();


                }
                break;
        }
    }

    // --- Lancer un tir automatique ---
    public void startTirAuto(double angleTourelle, double angleShooter, double vitesseShooter) {
        intake.arretPourTir();
        tirEnCours = true;
        shotsRemaining = 3;
        this.angleCibleTourelle = angleTourelle;
        this.angleCibleShooter = angleShooter;
        this.vitesseCibleShooter = vitesseShooter;

        tirsEffectues = 0;

        shooter.setShooterTargetRPM(vitesseShooter);  // Démarre immédiatement
        state = TirState.SHOOTER_SPINUP;
    }

    public void startTirAutoIndividuel(double angleTourelle, double angleShooter, double vitesseShooter) {
        intake.arretPourTir();
        tirEnCours = true;
        shotsRemaining = 1;
        this.angleCibleTourelle = angleTourelle;
        this.angleCibleShooter = angleShooter;
        this.vitesseCibleShooter = vitesseShooter;

        tirsEffectues = 0;

        shooter.setShooterTargetRPM(vitesseShooter);  // Démarre immédiatement


        state = TirState.SHOOTER_SPINUP;
    }

    public void startTirManuel() {
        intake.arretPourTir();
        tirEnCours = true;
        shotsRemaining = 1;
        //this.vitesseCibleShooter = vitesseShooter;

        tirsEffectues = 0;

        //shooter.setShooterTargetRPM(vitesseShooter);  // Démarre immédiatement

        state = TirState.SERVO_PUSH;
        timer.reset();

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
