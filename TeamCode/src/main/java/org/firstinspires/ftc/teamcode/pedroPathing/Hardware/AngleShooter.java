package org.firstinspires.ftc.teamcode.pedroPathing.Hardware;
import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AngleShooter {
    private Servo AngleShoot;
    private double Positioninit = 0.0;
    private double Positionlow = 0.1;
    private double Positionmedium = 0.3;
    private double Positionhaute = 0.6;

    public void init(@NonNull HardwareMap hwMap) {
    AngleShoot = hwMap.get(Servo.class, "AngleShoot");
        AngleShoot.setPosition(0.00);}

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
                angleShoot(Positioninit);
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

}
