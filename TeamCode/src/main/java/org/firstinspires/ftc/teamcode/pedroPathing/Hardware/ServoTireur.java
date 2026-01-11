package org.firstinspires.ftc.teamcode.pedroPathing.Hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Hardware.Indexeur;
import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.*;
import java.util.ArrayList;
import java.util.Random;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;


public class ServoTireur  {
    private Servo ServoTir;
    private Indexeur indexeur;
    private double tirpositionretour = 0.77; // position basse 079 gobilda
    private double tirpositionhaute = 0.35; // position haute 031 gobila speed

    private ElapsedTime timeretat = new ElapsedTime();
    private boolean tirServoEnCours = false;

    private boolean pauseTir = false;


    private enum Tireuretat {
        IDLE, // servo en position basse
        TirPositionhaute // Servo en position haute pour lancer balle

    }

    private Tireuretat tireuretat = Tireuretat.IDLE;

    public void init(@NonNull HardwareMap hwMap) {
        ServoTir= hwMap.get(Servo.class, "ServoTir");
        ServoTir.setPosition(tirpositionretour);


    }
    public ServoTireur(Indexeur indexeur) { this.indexeur = indexeur; }
    public void update() {
        int ballComptage = indexeur.getBalles();

        switch (tireuretat) {
            case IDLE:
                tirerAvecServo(tirpositionretour);
                break;

            case TirPositionhaute:
                tirerAvecServo(tirpositionhaute);
                break;
        }
    }
    public void tirerAvecServo(double servoposition) {
        ServoTir.setPosition(servoposition);   // monter ou descendre le servo
        tirServoEnCours = true;      // activer le flag
    }
    // --- Monter le servo pour tirer ---
    public void push() {
        ServoTir.setPosition(tirpositionhaute);
    }
    // --- Redescendre le servo ---
    public void retract() {
        ServoTir.setPosition(tirpositionretour);
    }
    // --- Vérifier si le servo est bien monté ---
    public boolean isPushDone() {
        return Math.abs(ServoTir.getPosition() - tirpositionhaute) < 0.02;
    }
    // --- Vérifier si le servo est bien redescendu ---
    public boolean isRetractDone() {
        return Math.abs(ServoTir.getPosition() - tirpositionretour) < 0.02;
    }
    public double getPosition() { return ServoTir.getPosition(); }
}


