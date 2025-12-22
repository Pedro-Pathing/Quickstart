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
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Indexeur {

    private boolean indexeurActif = false;
    private DcMotorEx Indexeur;
    private Servo ServoTir;
    private NormalizedColorSensor ColorLeft;
    private NormalizedColorSensor ColorRight;
    private  DigitalChannel magSensorAv;
    private Rev2mDistanceSensor distSensorIndexeur;
    private Rev2mDistanceSensor distSensorIndexeur2;
    double hue;
    private static final double TICKS_PER_REV_43 = 3895.9; // GoBilda 5203 43 tours
    private static final int COMPARTIMENTS = 3;
    private static final double TICKS_PAR_COMPARTIMENT = TICKS_PER_REV_43 / COMPARTIMENTS;
    private int compartimentActuel = 0;

    private ElapsedTime timerIndexeur = new ElapsedTime();
    // Variable interne pour mémoriser l'état précédent
    private boolean lastBallDetected = false;;

    private ElapsedTime timerServo = new ElapsedTime();
    private ElapsedTime indexeurTimer = new ElapsedTime();
    private boolean homingDone = false;

    private boolean marcheForceeIndexeur = false;
    private int consecutiveDetections = 0;
    private static final int NB_LECTURES = 5;
    private static final int SEUIL_MM = 50;
    public void init(@NonNull HardwareMap hwMap) {
        // ColorLeft = hwMap.get(ColorSensor.class, "ColorLeft");
        ColorLeft = hwMap.get(NormalizedColorSensor.class, "ColorLeft");
        ColorRight = hwMap.get(NormalizedColorSensor.class, "ColorRight");

        // Récupération du capteur magnétique arriere depuis la config (avant pour charger balle, arriere pour tir)
        //magSensorAr = hwMap.get(DigitalChannel.class, "magSensorAr");
        magSensorAv = hwMap.get(DigitalChannel.class, "magSensorAv");

        //Capteur Detection entrée d'une balle dans indexeur
        // Capteur de distance REV 2m
        distSensorIndexeur = hwMap.get(Rev2mDistanceSensor.class, "DistSensorIndexeur");

        // Définir le mode en entrée
        //magSensorAr.setMode(DigitalChannel.Mode.INPUT);

        Indexeur = hwMap.get(DcMotorEx.class, "Indexeur");
        Indexeur.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Indexeur.setDirection(DcMotor.Direction.REVERSE);
    }
    public double getIndexeurVelocityRPM() {
            double ticksPerSec = Indexeur.getVelocity();
            return (ticksPerSec * 60) / TICKS_PER_REV_43;
        }

    public void setIndexeurTargetRPM(double targetRPM) {
        // Conversion RPM -> ticks/sec
        double targetTicksPerSec = (targetRPM * TICKS_PER_REV_43) / 60.0;
        Indexeur.setVelocity(targetTicksPerSec);
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
    public void startAvance() {
        indexeurTimer.reset();
        avancerIndexeurRapide(); }

    // Fast advance by one compartment using RUN_TO_POSITION
    public void avancerIndexeurRapide() {
        if (!homingDone) return;
        compartimentActuel = (compartimentActuel + 1);
        int target = (int) (compartimentActuel * TICKS_PAR_COMPARTIMENT);
        Indexeur.setTargetPosition(target);
        Indexeur.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Power ceiling controls max speed in
        Indexeur.setPower(0.5); // augmente si besoin, attention au couple }
        int erreur = Math.abs(Indexeur.getCurrentPosition() - target);
        if (erreur < 15) { // tolérance de 15 ticks // Stop net
            Indexeur.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Indexeur.setPower(0);
        }
// Méthode de homing à appeler au démarrage
        public void homingIndexeur() {
            if (!homingDone) { // lancer le moteur doucement pour chercher l’aimant
                Indexeur.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Indexeur.setPower(0.35); // si l’aimant est détecté (via detectionpale)
                if (detectionpale()) {
                    Indexeur.setPower(0.0); // arrêt
                    Indexeur.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    Indexeur.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // mode normal
                    homingDone = true; } }}

        public boolean detectionpale() {
            return !magSensorAv.getState(); }

}
