package org.firstinspires.ftc.teamcode.pedroPathing.Hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.Rev9AxisImuOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.time.Month;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.*;
import java.util.ArrayList;
import java.util.Random;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Indexeur {
    private boolean indexeurActif = false;
    private DcMotorEx indexeur;
    private DigitalChannel magSensorAv;
    private Rev2mDistanceSensor distSensorIndexeur;
    private Rev2mDistanceSensor distSensorIndexeur2;
    private NormalizedColorSensor ColorLeft;
    private NormalizedColorSensor ColorRight;
    double hue;
    private static final double TICKS_PER_REV_43 = 3895.9; // GoBilda 5203 43 tours

    private int vitessehomingindexeurRPM = 10;
    private int vitesserapideindexeurRPM =25;
    private static final int COMPARTIMENTS = 3;
    private static final double TICKS_PAR_COMPARTIMENT = TICKS_PER_REV_43 / COMPARTIMENTS;
    private int compartimentActuel = 0;

    int ballComptage = 0;
    int MAX_BALLS =3;
    int MIN_BALLS=0;


    // Variable interne pour mémoriser l'état précédent
    private boolean lastBallDetected = false;
    private boolean homingDone = false;
    private boolean marcheForceeIndexeur = false;
    private int consecutiveDetections = 0;
    private static final int NB_LECTURES = 5;

    private enum Indexeuretat {
        IDLE,
        RECHERCHEPALE,

        AVANCERAPIDETIR,
        AVANCERAPIDEAMASSAGE,
        BOURRAGE,
        STABILISATION,
        PRETPOURTIR,

        HOMING
    }

    private Indexeuretat IndexeurState;
    private ElapsedTime timeretat = new ElapsedTime();
    private ElapsedTime indexeurtimer = new ElapsedTime();
    private int SEUIL_MMDETECTION = 5 //seuil detection capteur distance

    public void init(@NonNull HardwareMap hwMap) {

        indexeur = hwMap.get(DcMotorEx.class, "Indexeur");
        indexeur.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        indexeur.setDirection(DcMotor.Direction.REVERSE);

        magSensorAv = hwMap.get(DigitalChannel.class, "magSensorAv");
        magSensorAv.setMode(DigitalChannel.Mode.INPUT);

        // ColorLeft = hwMap.get(ColorSensor.class, "ColorLeft");
        ColorLeft = hwMap.get(NormalizedColorSensor.class, "ColorLeft");
        ColorRight = hwMap.get(NormalizedColorSensor.class, "ColorRight");

    }


    public void update (){
        switch (IndexeurState) {
            case IDLE:
                setindexeurTargetRPM(0.00);
                if (balleDetectee()) {
                    Indexeuretat = IndexeurState.AVANCERAPIDEAMASSAGE;}
                break;
            case RECHERCHEPALE:
                if (!detectionpale() && ballComptage < MAX_BALLS) {
                    setindexeurTargetRPM(6);
                } else if (detectionpale()) {
                    setindexeurTargetRPM(0.0);
                    Indexeuretat = IndexeurState.IDLE;
                }
               if (timeretat.milliseconds()>500) {
            }
                break;

            case AVANCERAPIDEAMASSAGE:
                avancerIndexeurRapide();
                ballComptage++;
                ballComptage = Math.min(ballComptage, MAX_BALLS);

                break;

            case AVANCERAPIDETIR:
                avancerIndexeurRapide();
                ballComptage--;
                ballComptage = Math.max(ballComptage, 0); //on empeche le decompte d'aller sous zero
                break;

            case BOURRAGE:
                break;

            case STABILISATION:
                break;

            case PRETPOURTIR:
                break;

            case HOMING:
                timeretat.reset();
                homingIndexeur();
                IndexeurState =  Indexeuretat.IDLE;

                break;



        }
    }
    public double getindexeurVelocityRPM() {
        double ticksPerSec = indexeur.getVelocity();
        return (ticksPerSec * 60) / TICKS_PER_REV_43;
    }

    public void setindexeurTargetRPM(double targetRPM) {
        // Conversion RPM -> ticks/sec
        double targetTicksPerSec = (targetRPM * TICKS_PER_REV_43) / 60.0;
        indexeur.setVelocity(targetTicksPerSec);
    }

    public String detectBallColor() {
        // Lecture capteur gauche
        NormalizedRGBA colorsLeft = ColorLeft.getNormalizedColors();
        float hueLeft = JavaUtil.colorToHue(colorsLeft.toColor());

        // Lecture capteur droit
        NormalizedRGBA colorsRight = ColorRight.getNormalizedColors();
        float hueRight = JavaUtil.colorToHue(colorsRight.toColor());

        // Détection individuelle
        String couleurLeft = detectHueColor(hueLeft);
        String couleurRight = detectHueColor(hueRight);

        // Fusion des résultats
        if (couleurLeft.equals("Vert") || couleurRight.equals("Vert")) {
            return "Vert";
        } else if (couleurLeft.equals("Violet") || couleurRight.equals("Violet")) {
            return "Violet";
        } else {
            return "Inconnue"; // indexeur, trou, réverbération, etc.
        }
    }

    // Fonction auxiliaire : convertit Hue en Vert/Violet/Inconnue
    private String detectHueColor(float hue) {
        if (hue >= 90 && hue <= 150) {
            return "Vert";
        } else if (hue >= 230 && hue <= 330) {
            return "Violet";
        } else {
            return "Inconnue";
        }
    }

    // Méthode de homing à appeler au démarrage
    public void homingIndexeur() {
        if (!homingDone) { // lancer le moteur doucement pour chercher l’aimant
            indexeur.setPower(0.25); // si l’aimant est détecté (via detectionpale)
            if (detectionpale()) {
                indexeur.setPower(0.0); // arrêt
                indexeur.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                indexeur.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // mode normal
                homingDone = true;
            }
            // Sécurité : si ça tourne trop longtemps
            if (timeretat.seconds() > 3.0) {
                    indexeur.setPower(0);
                    homingDone = true;
                }
            }


    }

    public int getBalles() {
        return ballComptage}

    // Fast advance by one compartment using RUN_TO_POSITION
    public void avancerIndexeurRapide() {
        if (!homingDone) return;
        compartimentActuel = (compartimentActuel + 1);
        int target = (int) (compartimentActuel * TICKS_PAR_COMPARTIMENT);
        indexeur.setTargetPosition(target);
        indexeur.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Power ceiling controls max speed in
        indexeur.setPower(0.5); // augmente si besoin, attention au couple }
        int erreur = Math.abs(indexeur.getCurrentPosition() - target);
        if (erreur < 15) { // tolérance de 15 ticks // Stop net
            indexeur.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            indexeur.setPower(0);}
        }

        public void reculerIndexeurbourrage() {
            int positionbourrage = indexeur.getCurrentPosition();
            int target = (int) ((positionbourrage) * 0.95);
            indexeur.setTargetPosition(target);
            indexeur.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Power ceiling controls max speed in
            indexeur.setPower(0.2); // augmente si besoin, attention au couple, pas besoin de mettre puissance negative car calcul auto }
            int erreur = Math.abs(target - indexeur.getCurrentPosition());
            if (erreur < 15) { // tolérance de 15 ticks // Stop net
                indexeur.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                indexeur.setPower(0);
            }
        }



    public boolean avanceTerminee() {
        boolean blocage = indexeur.isBusy() && Math.abs(indexeur.getVelocity()) < 10;
        boolean fini = !indexeur.isBusy() ||
                // le moteur pense avoir atteint sa cible
                Math.abs(indexeur.getCurrentPosition() - indexeur.getTargetPosition()) <= indexeur.getTargetPositionTolerance() || // proche de la cible
                indexeurtimer.seconds() > 1.0; // sécurité : timeout 1s
        if (fini) {
            indexeur.setPower(0.0);
            indexeur.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        return fini;
    }

    public boolean detectionpale() {
        return !magSensorAv.getState();
    }
    public boolean isindexeurBusy() {
        return indexeur.isBusy(); }
    /** * Getter pour accéder directement au moteur si besoin */
    public DcMotor getindexeurMotor() {
        return indexeur;}
    private static final int TOLERANCE_TIR = 15;
    public boolean indexeurPretPourTir() {
        if (Indexeur == null)
            return false;
        boolean fini = !indexeur.isBusy();
        int erreur = Math.abs(indexeur.getTargetPosition() - indexeur.getCurrentPosition());
        boolean dansTol = erreur < TOLERANCE_TIR;
        return fini && dansTol; }

    public boolean balleDetectee() {
        int count = 0;
        for (int i = 0; i < NB_LECTURES; i++) {
            //double d2 = distSensorIndexeur2.getDistance(DistanceUnit.MM);
            double d1 = distSensorIndexeur.getDistance(DistanceUnit.MM);
            // Vérification des deux capteurs
            boolean capteur1Detecte = (d1 > 5 && d1 < 500 && d1 < SEUIL_MMDETECTION);
            //boolean capteur2Detecte = (d2 > 5 && d2 < 500);
            // Si au moins un capteur détecte la balle
            if (capteur1Detecte) { count++; } }
        // Majorité des lectures doivent confirmer la présence
        return count >= (NB_LECTURES / 2 + 1); }
}



}

