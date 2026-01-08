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
        AFTERWAIT_INDEX,
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
                shooter.setShooterTargetRPM(0);
                break;

            // --- 1) Shooter spin-up ---
            case SHOOTER_SPINUP:
                intake.arretPourTir();
                if (shotsRemaining >0) {
                    shooter.setShooterTargetRPM(vitesseCibleShooter);
                    intake.disableRamassage();
                    tourelle.allerVersAngle(angleCibleTourelle);
                    if (!indexeur.isHomingDone()) {
                        indexeur.lancerHoming();
                        return;
                    }
                    if (indexeur.isHomingDone()) {
                        state = TirState.TURRET_POSITION;
                        timer.reset();
                    }
                }
                else { state = TirState.IDLE;
                    shooter.setShooterTargetRPM(0);}
                break;

            // --- 2) Positionnement tourelle ---
            case TURRET_POSITION:
                shooter.setShooterTargetRPM(vitesseCibleShooter);
                tourelle.allerVersAngle(angleCibleTourelle);

                if (tourelle.isAtAngle(angleCibleTourelle)) {
                    timer.reset();
                    state = TirState.ANGLE_POSITION;
                }
                break;

            // --- 3) Positionnement angle shooter ---
            case ANGLE_POSITION:
                shooter.setShooterTargetRPM(vitesseCibleShooter);
                ServoAngleShoot.setAngle(angleCibleShooter);
                //timer.reset();

                if (ServoAngleShoot.isAtAngle(angleCibleShooter)) {
                    timer.reset();
                    state = TirState.SERVO_PUSH;
                }
                break;

            // --- 4) Pousser la balle ---
            case SERVO_PUSH:
                shooter.setShooterTargetRPM(vitesseCibleShooter);
                double toleranceVelocityMax = 1.06 * vitesseCibleShooter;
                double toleranceVelocityMin = 0.94 * vitesseCibleShooter;
                if ((shooter.getShooterVelocityRPM() > toleranceVelocityMin) && (shooter.getShooterVelocityRPM() < toleranceVelocityMax)){;
                    servoTireur.push();
                    timer.reset();
                    state = TirState.SERVO_RETRACT;
                    indexeur.decrementerBalle();
                };

                if (timer.milliseconds() > 1000) {
                    timer.reset();
                    state = TirState.SERVO_RETRACT;
                }
                break;

            // --- 5) Rétracter le servo ---
            case SERVO_RETRACT:
                if (timer.milliseconds() > 300) {
                    servoTireur.retract();
                    timer.reset();
                    shotsRemaining--; // retrait d'un tir
                    tirsEffectues++;   // Tir réellement terminé ici
                    state = TirState.INDEX_ADVANCE;
                }
                break;

            // --- 6) Attendre fin rotation indexeur ---
            case INDEX_ADVANCE:
                if ((timer.milliseconds() > 300) && shotsRemaining == 0) {
                    shooter.setShooterTargetRPM(0);
                    intake.repriseApresTir();
                    timer.reset();
                    state = TirState.IDLE;

                }
                if (!(shotsRemaining == 0) && (timer.milliseconds() > 300)) {
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
                if (indexeur.isRotationTerminee()) {
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

        shooter.setShooterTargetRPM(vitesseCibleShooter);  // Démarre immédiatement
        state = TirState.SHOOTER_SPINUP;
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
