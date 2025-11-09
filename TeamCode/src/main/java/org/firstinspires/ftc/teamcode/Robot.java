package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Shooter;

public class Robot {
    public DcMotorEx shooterMotor;
    public CRServo turretCR;
    public Intake intake;

    public Shooter shooter;
public Robot (HardwareMap hardwareMap) {
    intake = new Intake(hardwareMap);
    shooter = new Shooter(hardwareMap);
    turretCR = hardwareMap.get(CRServo.class, "turretServo");
    turretCR.setPower(0.0); // start stopped
    }

}
