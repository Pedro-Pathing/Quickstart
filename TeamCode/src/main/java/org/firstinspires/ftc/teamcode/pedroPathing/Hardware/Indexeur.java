package org.firstinspires.ftc.teamcode.pedroPathing.Hardware;
package org.firstinspires.ftc.teamcode.hardware;

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
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
    private static final int COMPARTIMENTS = 3;
    private static final double TICKS_PAR_COMPARTIMENT = TICKS_PER_REV_43 / COMPARTIMENTS;
    private int compartimentActuel = 0;

    // Variable interne pour mémoriser l'état précédent
    private boolean lastBallDetected = false;
    private boolean homingDone = false;
    private boolean marcheForceeIndexeur = false;
    private int consecutiveDetections = 0;
    private static final int NB_LECTURES = 5;

    private enum Indexeuretat {
        IDLE,
        RECHERCHEPALE,
        ATTENTEBALLE,
        IMPLUSION,
        AVANCERAPIDE,
        BOURRAGE,
        STABILISATION,
        HOMING,
    }

    private Indexeur etat  = Indexeuretat.IDLE;
    private ElapsedTime timeretat = new ElapsedTime();
    private ElapsedTime indexeurtimer = new ElapsedTime();

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

    public void setEtat(Indexeuretat nouvelEtat){
        etat = nouvelEtat;
        timeretat.reset();
    }

    public void update (){
        switch (etat) {
            case IDLE:

                break;
            case RECHERCHEPALE:
                break;
            case ATTENTEBALLE:
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
            indexeur.setPower(0.35); // si l’aimant est détecté (via detectionpale)
            if (detectionpale()) {
                indexeur.setPower(0.0); // arrêt
                indexeur.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                indexeur.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // mode normal
                homingDone = true;
            }
        }
    }

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

}

