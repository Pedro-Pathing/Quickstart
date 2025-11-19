package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
//lens, odo, gobuilda pinpoint
public class Hardware {
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotor frontRight;
    public DcMotor backRight;
    public DcMotor intake;
    public DcMotor mid1;
    public DcMotor mid2;
    public DcMotor outtake;
    public HuskyLens lens;

    public void init(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, "LF");
        backLeft = hardwareMap.get(DcMotor.class, "LB");
        frontRight = hardwareMap.get(DcMotor.class, "RF");
        backRight = hardwareMap.get(DcMotor.class, "RB");

        intake = hardwareMap.get(DcMotor.class, "intake");

        mid1 = hardwareMap.get(DcMotor.class, "mid1");
        mid2 = hardwareMap.get(DcMotor.class, "mid2");

        outtake = hardwareMap.get(DcMotor.class, "flywheel");

        lens = hardwareMap.get(HuskyLens.class, "lens");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        mid1.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

}
