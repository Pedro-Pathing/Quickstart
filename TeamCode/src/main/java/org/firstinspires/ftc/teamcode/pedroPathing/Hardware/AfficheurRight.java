package org.firstinspires.ftc.teamcode.pedroPathing.Hardware;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AfficheurRight {

    private Servo RGBRight;
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
        ClignoteVert
    }

    private Etat etat = Etat.IDLE;

    public void init(@NonNull HardwareMap hwMap) {
        RGBRight = hwMap.get(Servo.class, "RGBRight");
        timer.reset();
    }

    public void update() {
        switch (etat) {

            case IDLE:
                setPosition(0.0);
                break;

            case Vert:   setPosition(vert); break;
            case Bleu:   setPosition(bleu); break;
            case Jaune:  setPosition(jaune); break;
            case Rouge:  setPosition(rouge); break;
            case Orange: setPosition(orange); break;
            case Violet: setPosition(violet); break;
            case Blanc:  setPosition(blanc); break;

            case ClignoteVert:
                if (timer.milliseconds() > 300) {
                    if (RGBRight.getPosition() == vert)
                        setPosition(0.0);
                    else
                        setPosition(vert);

                    timer.reset();
                }
                break;
        }
    }

    // Méthodes publiques pour changer la couleur
    public void setIdle()  { etat = Etat.IDLE; }
    public void setVert()  { etat = Etat.Vert; }
    public void setBleu()  { etat = Etat.Bleu; }
    public void setJaune() { etat = Etat.Jaune; }
    public void setRouge() { etat = Etat.Rouge; }
    public void setOrange(){ etat = Etat.Orange; }
    public void setViolet(){ etat = Etat.Violet; }
    public void setBlanc() { etat = Etat.Blanc; }

    // Mode clignotant vert pour le tir
    public void setClignoteVert() {
        etat = Etat.ClignoteVert;
        timer.reset();
    }

    public void setPosition(double pos) {
        RGBRight.setPosition(pos);
    }
}
