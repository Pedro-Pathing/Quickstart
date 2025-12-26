package org.firstinspires.ftc.teamcode.pedroPathing.Hardware;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AfficheurRight {
    private Servo RGBRight;
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
        RGBRight = hwMap.get(Servo.class, "RGBLeft");
    }
    public void update() {

        switch (Afficheurrightetat) {
            case IDLE:
                RGBRight(0.0);
                break;
            case Vert:
                RGBRight(vert);
                break;
            case Rouge:
                RGBRight(rouge);
                break;
            case Jaune:
                RGBRight(jaune);
                break;
            case Bleu:
                RGBRight(bleu);
                break;
            case Violet:
                RGBRight(violet);
                break;
            case Orange:
                RGBRight(orange);
                break;
            case Blanc:
                RGBRight(blanc);
                break;
        }
    }
    public void RGBRight (double lumiereleft) {
        RGBRight.setPosition(lumiereleft);
    }
}
