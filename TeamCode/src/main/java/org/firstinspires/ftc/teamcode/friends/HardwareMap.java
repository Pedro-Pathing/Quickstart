package org.firstinspires.ftc.teamcode.friends;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class HardwareMap {
    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;

    //public DcMotor shooterMotor1;
    //public DcMotor shooterMotor2;
    public DcMotor intakeMotor;

    public HardwareMap(com.qualcomm.robotcore.hardware.HardwareMap hardwaremap) {

        frontRightMotor = hardwaremap.get(DcMotor.class, "FRM");
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftMotor = hardwaremap.get(DcMotor.class, "FLM");
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor = hardwaremap.get(DcMotor.class, "BRM");
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor = hardwaremap.get(DcMotor.class, "BLM");
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //shooterMotor1 = hardwaremap.get(DcMotor.class, "Shooter1");
        //shooterMotor2 = hardwaremap.get(DcMotor.class, "Shooter2");
        intakeMotor = hardwaremap.get(DcMotor.class, "Intake");
    }
}
