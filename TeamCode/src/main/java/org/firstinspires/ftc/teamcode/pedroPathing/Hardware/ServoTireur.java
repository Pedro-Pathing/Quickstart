package org.firstinspires.ftc.teamcode.pedroPathing.Hardware;
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


public class ServoTireur {
    private Servo ServoTir;
    private ElapsedTime timeretat = new ElapsedTime();
    private boolean tirServoEnCours = false;

    private boolean pauseTir = false;

    private double tirpositionretour = 0.00;

    private double tirpositionhaute = 0.6;

    private enum Tireuretat {
        IDLE, // servo en position basse
        TirPositionhaute // Servo en position haute pour lancer balle

    }
    private Tireuretat tireuretat;
    public void init(@NonNull HardwareMap hwMap) {
        ServoTir= hwMap.get(Servo.class, "ServoTir");
        ServoTir.setPosition(tirpositionretour);

    }

    public void update() {
        int ballcomptage = indexeur.getBalles();

        switch (tireuretat) {
            case IDLE:
                ServoTir.setPosition(tirpositionretour);
                break;

            case TirPositionhaute:
                ServoTir.setPosition(tirpositionhaute);
        break;

        public void tirerAvecServo(double servoposition) {
        ServoTir.setPosition(servoposition);   // monter ou descendre le servo
        tirServoEnCours = true;      // activer le flag
        timerServo.reset();          // démarrer le timer

}


