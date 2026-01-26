package org.firstinspires.ftc.teamcode.pedroPathing.Hardware;
import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AngleShooter {
    private Servo AngleShoot;

    private double positionInitiale;
    private double Positionlow = 0.12;
    private double Positionmedium = 0.30;

    private double PositionInterMdHaut = 0.40;
    private double Positionhaute = 0.52;

    public void init(@NonNull HardwareMap hwMap) {
        AngleShoot = hwMap.get(Servo.class, "AngleShoot");
        AngleShoot.setDirection(Servo.Direction.REVERSE);
        // Position de départ sécurisée
        positionInitiale = Positionlow;
        AngleShoot.setPosition(positionInitiale);

    }

    private enum AngleShooteretat {

        IDLE,  //Repos
        TirProche,

        TirMoyenneDistance,
        TirLoin,

    }
    private AngleShooteretat angleshooteretat = AngleShooteretat.IDLE;
    private ElapsedTime timeretat = new ElapsedTime();

    public void update() {

        switch (angleshooteretat) {
            case IDLE:
                break;

            case TirProche:
                angleShoot(Positionlow);
                break;

            case TirMoyenneDistance:
                angleShoot(Positionmedium);
                break;
            case TirLoin:
                angleShoot(Positionhaute);
                break;
        }
    }
    public void angleShoot(double angle) {
        AngleShoot.setPosition(angle);
    }

    // Methode publique utilisable par Tireur Manager
    public void setAngle(double pos)
    { AngleShoot.setPosition(pos); }
    public boolean isAtAngle(double pos)
    { return Math.abs(AngleShoot.getPosition() - pos) < 0.02; }


}
