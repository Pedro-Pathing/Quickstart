package org.firstinspires.ftc.teamcode.LOADCode.Main_.Hardware_.Actuators_.Generic_;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoClass {
    private Servo servo;

    public void init(@NonNull OpMode opmode, String servoName){
        servo = opmode.hardwareMap.get(Servo.class, servoName);
    }

    /**
     * @param angle The angle to set the servo to. Must be a value between 0 and 1,
     *              representing the endpoints of it's movement.
     */
    public void setAngle(double angle){
        servo.setPosition(angle);
    }

    /**
     * @param direction The direction to set the servo to.
     */
    public void setDirection(Servo.Direction direction){
        servo.setDirection(direction);
    }

    /**
     * @return A value between 0 and 1 that the servo has been set to.
     */
    public double getAngle(){
        return servo.getPosition();
    }
}