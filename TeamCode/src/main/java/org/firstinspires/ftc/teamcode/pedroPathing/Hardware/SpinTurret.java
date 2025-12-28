package org.firstinspires.ftc.teamcode.pedroPathing.Hardware;
import androidx.annotation.NonNull;
import com.qualcomm.hardware.rev.Rev9AxisImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.CRServo;

public class SpinTurret {
    private CRServo SpinTourelle;
    private double powerarrettourelle = 0.0;
    private double powertournertourelle =1.0;
    public IMU imuTourelle;

    private enum spintourelleetat {
        IDLE,
        CentrageZeroTourelle, // PositionCentrale
        TournerJoystick,
        TournerAutoCamera,
        TournerAutoBlueSansCamera,
        TournerAutoRedSansCamera,

    }
    private spintourelleetat Spintourelleetat = spintourelleetat.IDLE;

    public void init(@NonNull HardwareMap hwMap) {
        SpinTourelle = hwMap.get(CRServo.class, "SpinTourelle");
        //SpinTourelle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Mettre le moteur en mode BRAKE
        //SpinTourelle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //SpinTourelle.setPower(0);

        imuTourelle = hwMap.get(IMU.class, "imuTourelle");
        IMU.Parameters parameters = new IMU.Parameters(
                new Rev9AxisImuOrientationOnRobot(
                        Rev9AxisImuOrientationOnRobot.LogoFacingDirection.UP,
                        Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection.LEFT)
        );
        imuTourelle.initialize(parameters);
        imuTourelle.resetYaw();

    }
    public void update() {

        switch (Spintourelleetat) {
            case IDLE:
                SpinTourelle.setPower(powerarrettourelle);
                break;

            case CentrageZeroTourelle:
                if (lectureangletourelle() > 2) {
                    SpinTourelle.setPower(-powertournertourelle);
                }
               if (lectureangletourelle() < -2){
                   SpinTourelle.setPower(powertournertourelle);
               }
               if (lectureangletourelle() <= 2 && lectureangletourelle() >= -2){
                           SpinTourelle.setPower(powerarrettourelle);
               }
               break;
            case TournerJoystick:
                //rotationtourelle();
                break;


        }
    }
    // Méthode pour lecture de l'angle gyro de la tourelle et pour faire tourner la tourelle avec un angle max
    public double lectureangletourelle() {
        double angletourelle = imuTourelle.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return angletourelle; // en degrés;
    }
    public void rotationtourelle(double power) {
        // Limitation de la puissance maximale
        double maxPower = 1; // valeur maximale autorisée
        power = Math.max(-maxPower, Math.min(power, maxPower)); // clamp entre -1 et +1

        double angletourelle = lectureangletourelle();
        if ((angletourelle >= 100 && power > 0) || (angletourelle <= -100 && power < 0)) {
            SpinTourelle.setPower(0);
        } else {
            SpinTourelle.setPower(power);
        }
    }

    public void allerVersAngle(double angleCible) {
        double angleActuel = lectureangletourelle();
        double erreur = angleCible - angleActuel;

        // Petit PID proportionnel simple
        double kP = 0.01; // à ajuster selon notre robot
        double power = kP * erreur;

        // Limite la puissance (1 sur servo pour aller plus vite à revoir)
        power = Math.max(-1, Math.min(power, 1));

        rotationtourelle(power);
    }

    // Connaitre l'angle de la tourelle
    public boolean isAtAngle(double angleCible) {
        return Math.abs(lectureangletourelle() - angleCible) < 2.0;
    }



}
