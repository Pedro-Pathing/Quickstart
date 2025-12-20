package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    DcMotor FrM, BM;
    public void Init(HardwareMap hardwareMap){
        FrM = hardwareMap.get(DcMotor.class, "FrM");
        BM = hardwareMap.get(DcMotor.class, "BM");
    }
    public void take(Gamepad gamepad1){
        if (gamepad1.xWasPressed()){
            FrM.setPower(0.75);
        }
    }
    public void climb(Gamepad gamepad1){
        if (gamepad1.circleWasPressed()){
            BM.setPower(0.75);
        }
    }
}
