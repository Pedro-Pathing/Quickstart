package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.Turret;

public class Robot {
    public DcMotorEx shooterMotor;
    public Intake intake;
    public Shooter shooter;
    public Turret turret;


public Robot (HardwareMap hardwareMap) {
    intake = new Intake(hardwareMap);
    shooter = new Shooter(hardwareMap);
    turret = new Turret(hardwareMap);
    }

}
