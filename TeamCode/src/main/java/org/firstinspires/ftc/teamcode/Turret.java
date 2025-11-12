package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    public CRServo turretCR;
    private static final double TURRET_POWER = 0.45;

    private static final double TURRET_STOP = 0.0;


    public Turret (HardwareMap hardwareMap) {
        turretCR = hardwareMap.get(CRServo.class, "turretServo");
        turretCR.setPower(0.0); // start stopped
    }

    public void goLeft() {
        turretCR.setPower(-TURRET_POWER); // rotate left
    }

    public void goRight() {
        turretCR.setPower(TURRET_POWER); // rotate right
    }

    public void stopTurret() {
        turretCR.setPower(TURRET_STOP); // stop turret
    }

    public void setTurretPower(double power) {
        turretCR.setPower(power);
    }
}
