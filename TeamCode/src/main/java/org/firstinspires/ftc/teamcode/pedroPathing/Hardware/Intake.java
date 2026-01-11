package org.firstinspires.ftc.teamcode.pedroPathing.Hardware;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.*;
import java.util.ArrayList;
import java.util.Random;
public class Intake {

    private DcMotorEx IntakeBall;
    private AfficheurLeft afficheurLeft;
    private boolean ramassageEnabled = true;
    private ElapsedTime statetimer = new ElapsedTime();
    int seuilcapteurmm = 50;

    private enum Intakeetat {
        IDLE,
        RAMASSAGE,
        EJECTION,
        ARRET_POUR_TIR
    }
    private Intakeetat intakeState = Intakeetat.IDLE;

    private double intake_reverse = -1000;

    private double intake_fast = 1000;


    private final double MINRPM = 0;

    // --- Variables télémétrie / détection
    private double rpm = 0;
    private float lumIndexeur = 0;
    private boolean ralentissement = false;
    private int score = 0;

    private final int tempsblocage = 200;
    private final double CHUTE_RPM = 0.85;
    private final float SEUIL_LUM_INDEXEUR = 0.0700f;
    private final float LUM_Indexeursansballes = 0.0577f;

    private Indexeur indexeur;
    private NormalizedColorSensor ColorIndexeur, ColorIntake;
    private Rev2mDistanceSensor distSensorIndexeur;

    private static final double TICKS_PER_REV_6000 = 145.1;  // ancien valeur 28

    public void init(@NonNull HardwareMap hwMap) {

        IntakeBall = hwMap.get(DcMotorEx.class, "IntakeBall");
        IntakeBall.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IntakeBall.setDirection(DcMotor.Direction.REVERSE);
        intakeState = Intakeetat.IDLE;

        ColorIndexeur = hwMap.get(NormalizedColorSensor.class, "ColorIndexeur");
        distSensorIndexeur = hwMap.get(Rev2mDistanceSensor.class, "DistSensorIndexeur");
    }

    public Intake(Indexeur indexeur, AfficheurLeft afficheurLeft) {
        this.indexeur = indexeur;
        this.afficheurLeft = afficheurLeft;}

    public void update() {

        int ballcomptage = indexeur.getBalles();
        boolean intakeEnMarche = (intakeState == Intakeetat.RAMASSAGE);
        if (intakeEnMarche) {
            afficheurLeft.setClignoteOrange();
        } else {
            afficheurLeft.setIdle();
        }

        afficheurLeft.update();



        switch (intakeState) {

            case IDLE:
                setIntakeBallTargetRPM(0);
                if (ballcomptage == 0) {
                    statetimer.reset();
                    intakeState = Intakeetat.RAMASSAGE;
                }
                break;
            case ARRET_POUR_TIR:
                setIntakeBallTargetRPM(0);  // moteur arrêté
                // pas de ramassage, pas de changement d’état
                break;


            case RAMASSAGE:

                //
                setIntakeBallTargetRPM(intake_fast);

                // --- Mise à jour des variables ---
                rpm = IntakeBall.getVelocity() * 60 / TICKS_PER_REV_6000;
                lumIndexeur = ColorIndexeur.getNormalizedColors().alpha;

                ralentissement = rpm < intake_fast * CHUTE_RPM;
                boolean indexeurLumiere = detectCapteurIndexeur();
                boolean indexeurdistance= balleDetecteeDistSensor();

                if (ballcomptage == 3) {
                    intakeState = Intakeetat.IDLE;
                    break;
                }

                // --- Fusion des signaux ---
                score = 0;
                //if (ralentissement) score++;
                if (indexeurLumiere) score++;
                if (indexeurdistance) score++;

                if (score >= 1) {
                    indexeur.setEtat(Indexeur.Indexeuretat.AVANCERAPIDEAMASSAGE);
                    statetimer.reset();
                    //IntakeBall.setVelocity(0);
                }

                // --- Bourrage ---
                //if (rpm <= MINRPM && statetimer.milliseconds() > tempsblocage) {
                //    statetimer.reset();
                //    intakeState = Intakeetat.EJECTION;
                //}
                break;

            case EJECTION:
                setIntakeBallTargetRPM(intake_reverse);
                if (statetimer.milliseconds() > 400) {
                    //if (ballcomptage == 3) {
                    //    intakeState = Intakeetat.IDLE;
                    //} else {
                    statetimer.reset();
                    intakeState = Intakeetat.RAMASSAGE;
                    }
                break;
        }
    }

    public void setIntakeBallTargetRPM(double targetRPM) {
        double targetTicksPerSec = (targetRPM * TICKS_PER_REV_6000) / 60.0;
        IntakeBall.setVelocity(targetTicksPerSec);
    }


    private boolean detectCapteurIndexeur() {
        return lumIndexeur > SEUIL_LUM_INDEXEUR;
    }


    public boolean balleDetecteeDistSensor() {
        int count = 0;
        for (int i = 0; i < 2; i++) {
            double d1 = distSensorIndexeur.getDistance(DistanceUnit.MM);
            boolean capteur1Detecte = Double.isFinite(d1) && d1 > 5 && d1 < seuilcapteurmm;
            if (capteur1Detecte) count++;
        }
        return count >= 1; // au moins une lecture valide
    }

    // --- GETTERS ---
    public double getRPM() { return rpm; }

    public float getLumIndexeur() { return ColorIndexeur.getNormalizedColors().alpha; }
    public boolean getRalentissement() { return ralentissement; }
    public int getScore() { return score; }
    public double getCapteurDistance() { return distSensorIndexeur.getDistance(DistanceUnit.MM);}
    public boolean getBalleDetectee() { return score >= 2; }
    public boolean isIdle() { return intakeState == Intakeetat.IDLE; }



    public void enableRamassage() {
        ramassageEnabled = true;
    }

    public void disableRamassage() {
        ramassageEnabled = false;
    }
    public void arretPourTir() {
        intakeState = Intakeetat.ARRET_POUR_TIR; }

    public void repriseApresTir() {
        intakeState = Intakeetat.IDLE; }
    public Intakeetat getEtat() {
        return intakeState;
    }

    public void setetatramasage(){
        intakeState = Intakeetat.RAMASSAGE;
    }
    public void setetatIDLE(){
        intakeState = Intakeetat.IDLE;
    }
    public void setetatEJECTION(){
        statetimer.reset();
        intakeState = Intakeetat.EJECTION;
    }
}

