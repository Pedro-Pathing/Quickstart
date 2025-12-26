package org.firstinspires.ftc.teamcode.pedroPathing.Hardware;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AfficheurLeft {
    private Servo RGBLeft;
    private double vert = 0.500;
    private double bleu = 0.611;
    private double violet = 0.722;
    private double blanc = 1.0;
    private double jaune = 0.388;
    private double rouge  = 0.277;
    private double orange = 0.333;
    private enum afficheurrightetat {
        IDLE,
        Vert, // PositionCentrale
        Bleu,
        Jaune,
        Rouge,
        Orange,
        Violet,
        Blanc

    }
    private afficheurrightetat Afficheurrightetat = afficheurrightetat.IDLE;

    public void init(@NonNull HardwareMap hwMap) {
        RGBLeft = hwMap.get(Servo.class, "RGBLeft");
    }
    public void update() {

        switch (Afficheurrightetat) {
            case IDLE:
                RGBLeft(0.0);
                break;
            case Vert:
                RGBLeft(vert);
                break;
            case Rouge:
                RGBLeft(rouge);
                break;
            case Jaune:
                RGBLeft(jaune);
                break;
            case Bleu:
                RGBLeft(bleu);
                break;
            case Violet:
                RGBLeft(violet);
                break;
            case Orange:
                RGBLeft(orange);
                break;
            case Blanc:
                RGBLeft(blanc);
                break;
        }
    }
    public void RGBLeft (double lumiereleft) {
        RGBLeft.setPosition(lumiereleft);
    }
}
