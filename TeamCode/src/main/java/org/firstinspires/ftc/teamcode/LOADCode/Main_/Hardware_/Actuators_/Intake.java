package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Generic_.CRServoClass;
import org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Generic_.DcMotorExClass;

public class Intake {
    private final OpMode opMode;

    private final DcMotorExClass intake;
    private final CRServoClass belt;

    public Intake(OpMode opmode){
        opMode = opmode;
        this.intake = new DcMotorExClass();
        this.belt = new CRServoClass();
    }

    public void init(){
        intake.init(opMode, "intake");

    }
}
