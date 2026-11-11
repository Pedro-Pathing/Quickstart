package org.firstinspires.ftc.teamcode.Mech;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Chassi {

    private DcMotor leftFront, leftBack, rightFront, rightBack;

    public void init(HardwareMap hardwareMap){

        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void drive(double x, double y, double rx){

        rx = -rx;
        x = -x;

        rx = Math.copySign(rx*rx*rx*rx, rx);

        double Median = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(rx), 1);

        double lfp = (y + x + rx) / Median;
        double lbp = (y - x + rx) / Median;
        double rfp = (y - x - rx) / Median;
        double rbp = (y + x - rx) / Median;


        leftFront.setPower(lfp); leftBack.setPower(lbp);
        rightFront.setPower(rfp); rightBack.setPower(rbp);

    }

}
