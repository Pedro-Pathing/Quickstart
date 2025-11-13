package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.Turret;

public class Robot {
    public Intake intake;
    public Shooter shooter;
    public Turret turret;
    public HardwareMap hardwareMap;

    public Telemetry telemetry;

    public Robot (HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
    }
}
