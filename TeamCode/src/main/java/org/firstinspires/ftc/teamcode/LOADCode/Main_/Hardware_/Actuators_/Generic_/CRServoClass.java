package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Generic_;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class CRServoClass{
    private CRServo servo;

    public void init(@NonNull OpMode opmode, String servoName){
        servo = opmode.hardwareMap.get(CRServo.class, servoName);
    }

    /**
     * @param power The power to set the servo to. Must be a value between -1 and 1.
     */
    public void setPower(double power){
        servo.setPower(power);
    }

    /**
     * @param direction The direction to set the servo to.
     */
    public void setDirection(DcMotorSimple.Direction direction){
        servo.setDirection(direction);
    }

    /**
     * @return The power the servo has been set to.
     */
    public double getPower(){
        return servo.getPower();
    }
}