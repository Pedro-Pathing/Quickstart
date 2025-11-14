package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
public class Shooter {

    public final int shooterCloseRPM = 1000;
    public final int shooterFarRPM = 1375;
    public final int autonShooterFarRPM = 1375;
    public final int shooterMidRPM = 1125;
    public final int shooterHumanRPM = -1200;
    public final int shooterOffRPM = 0;
    public final int autoClose = 800;

    private int currentRPM = 0;

    public DcMotorEx shooterMotor;
    public RevBlinkinLedDriver shooterLight;

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooterLight = hardwareMap.get(RevBlinkinLedDriver.class, "shooterLight");
    }

    public void startCloseShoot() {
        currentRPM = shooterCloseRPM;
        shooterMotor.setVelocity(shooterCloseRPM); // converting RPM to ticks per second
    }
    public void startAutoCloseShoot() {
        currentRPM = autoClose;
        shooterMotor.setVelocity(autoClose); // converting RPM to ticks per second
    }

    public void startAutonFarShoot() {
        currentRPM = autonShooterFarRPM;
        shooterMotor.setVelocity(autonShooterFarRPM);
    }

    public void startFarShoot() {
        currentRPM = shooterFarRPM;
        shooterMotor.setVelocity(shooterFarRPM); // converting RPM to ticks per second
    }

    public void startMidShoot() {
        currentRPM = shooterMidRPM;
        shooterMotor.setVelocity(shooterMidRPM); // converting RPM to ticks per second
    }

    public void startHumanIntake() {
        currentRPM = shooterHumanRPM;
        shooterMotor.setVelocity(shooterHumanRPM); // converting RPM to ticks per second
    }

    public void stopShoot() {
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // converting RPM to ticks per second
        currentRPM = shooterOffRPM;
        shooterMotor.setVelocity(shooterOffRPM); // converting RPM to ticks per second
        shooterLight.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
    }

    public boolean reachedSpeed() {
        if (shooterMotor.getVelocity() >= currentRPM) {
            return true;
        } else {
            return false;
        }
    }
    public boolean reachMidSpeed () {
        if (shooterMotor.getVelocity() >= shooterMidRPM) {
            return true;
        } else {
            return false;
        }
    }

    public void shooterLightUpdate() {
        if(currentRPM == 0){
            return;
        }
        if (reachedSpeed()) {
            shooterLight.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }
        else {
            shooterLight.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
    }



    public void startReverseShoot() {
        shooterMotor.setVelocity(-shooterCloseRPM);
    }

    public void stopFlyWheel() {
        shooterMotor.setVelocity(0);
    }
}