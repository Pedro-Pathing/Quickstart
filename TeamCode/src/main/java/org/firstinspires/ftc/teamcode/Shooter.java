package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Shooter {

    public final int shooterCloseRPM = 900;
    public final int shooterFarRPM = 1500;

    public DcMotorEx shooterMotor;
    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void startCloseShoot() {
        shooterMotor.setVelocity(shooterCloseRPM); // converting RPM to ticks per second
    }
    public void startFarShoot() {
        shooterMotor.setVelocity(shooterFarRPM);
    }

    public boolean reachCloseSpeed () {
        if (shooterMotor.getVelocity() >= shooterCloseRPM) {
            return true;
        } else {
            return false;
        }
    }

    public boolean reachFarSpeed () {
        if (shooterMotor.getVelocity() >= shooterFarRPM) {
            return true;
        } else {
            return false;
        }
    }

    public void startReverseShoot() {
        shooterMotor.setVelocity(-shooterCloseRPM);
    }

    public void stopFlyWheel() {
        shooterMotor.setVelocity(0);
    }
}
