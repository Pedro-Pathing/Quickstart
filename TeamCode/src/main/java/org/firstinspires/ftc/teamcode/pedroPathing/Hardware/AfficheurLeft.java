package org.firstinspires.ftc.teamcode.pedroPathing.Hardware;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AfficheurLeft {

    private Servo RGBLeft;
    private ElapsedTime timer = new ElapsedTime();

    // Positions des couleurs
    private final double vert = 0.500;
    private final double bleu = 0.611;
    private final double violet = 0.722;
    private final double blanc = 1.0;
    private final double jaune = 0.388;
    private final double rouge = 0.277;
    private final double orange = 0.333;

    // États possibles
    private enum Etat {
        IDLE,
        Vert,
        Bleu,
        Jaune,
        Rouge,
        Orange,
        Violet,
        Blanc,
        ClignoteOrange,
    }

    private Etat etat = Etat.IDLE;

    public void init(@NonNull HardwareMap hwMap) {
        RGBLeft = hwMap.get(Servo.class, "RGBLeft");
    }

    public void update() {
        switch (etat) {
            case IDLE:
                RGBLeft(0.0);
                break;
            case Vert:
                RGBLeft(vert);
                break;
            case Bleu:
                RGBLeft(bleu);
                break;
            case Jaune:
                RGBLeft(jaune);
                break;
            case Rouge:
                RGBLeft(rouge);
                break;
            case Orange:
                RGBLeft(orange);
                break;
            case Violet:
                RGBLeft(violet);
                break;
            case Blanc:
                RGBLeft(blanc);
                break;
            case ClignoteOrange:
                if (timer.milliseconds() > 300) {
                    if (RGBLeft.getPosition() == orange)
                        setPosition(0.0);
                    else
                        setPosition(orange);
                    timer.reset();
                }
                break;
        }
    }

    // Méthodes publiques pour changer la couleur
    public void setIdle() {
        etat = Etat.IDLE;
    }

    public void setVert() {
        etat = Etat.Vert;
    }

    public void setBleu() {
        etat = Etat.Bleu;
    }

    public void setJaune() {
        etat = Etat.Jaune;
    }

    public void setRouge() {
        etat = Etat.Rouge;
    }

    public void setOrange() {
        etat = Etat.Orange;
    }

    public void setViolet() {
        etat = Etat.Violet;
    }

    public void setBlanc() {
        etat = Etat.Blanc;
    }

    private void RGBLeft(double pos) {
        RGBLeft.setPosition(pos);
    }

    //Mode clignotant pour l’intake
    public void setClignoteOrange() {
        etat = Etat.ClignoteOrange;
        timer.reset();
    }

    private void setPosition(double pos) {
        RGBLeft.setPosition(pos);
    }
}

