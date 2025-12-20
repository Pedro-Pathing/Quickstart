package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Roti {
    DcMotor fl,fr,br,bl;
    public void Init(HardwareMap hardwareMap){
        fl = hardwareMap.get(DcMotor.class,"fl");
        fr = hardwareMap.get(DcMotor.class,"fr");
        bl = hardwareMap.get(DcMotor.class,"bl");
        br = hardwareMap.get(DcMotor.class,"br");
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public void Movement(Gamepad gamepad1){
        float forward,strafe,rotate;
        float vFL,vFR,vBL,vBR,denominator;
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        vFL = forward + strafe + rotate;
        vBL = forward - strafe + rotate;
        vFR = forward + strafe - rotate;
        vBR = forward - strafe - rotate;
        denominator = Math.max(vFL,Math.max(vBL,Math.max(vFR,vBR)));
        if(denominator<1)
            denominator = 1;
        fl.setPower(vFL/denominator);
        bl.setPower(vBL/denominator);
        fr.setPower(vFR/denominator);
        br.setPower(vBR/denominator);
    }

}