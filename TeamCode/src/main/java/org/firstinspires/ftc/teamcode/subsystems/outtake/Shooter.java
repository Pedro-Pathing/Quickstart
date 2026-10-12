package org.firstinspires.ftc.teamcode.subsystems.outtake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.util.Utility;

public class Shooter implements Subsystem {

    //---------------- Hardware ----------------
    private final DcMotorEx flywheelLeft;
    private final DcMotorEx flywheelRight;
    private final Servo hood;

    //---------------- Software ----------------
    private Utility util;

    private double flywheelPow = 0.0;
    private double hoodPos = 0.0;
    private boolean hoodChanged = false;

    private final double hoodStart = 0.5;
    //TODO find hood limits
    private final double hoodMin = 0.0;
    private final double hoodMax = 1.0;

    //---------------- Constructor ----------------
    public Shooter(HardwareMap map) {
        flywheelLeft = map.get(DcMotorEx.class, "fly_left");
        flywheelRight = map.get(DcMotorEx.class, "fly_right");
        flywheelLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelRight.setDirection(DcMotorSimple.Direction.FORWARD);
        hood = map.get(Servo.class, "hood");
    }

    //---------------- Core Methods ----------------
    private void setFlywheel(double pow){
        flywheelLeft.setPower(pow);
        flywheelRight.setPower(pow);
    }

    private void setHood(double pos){
        hood.setPosition(util.clamp(pos, hoodMin, hoodMax));
    }


    //---------------- Methods ----------------
    public void setFlywheelPow(double pow){
        flywheelPow = pow;
    }

    public void setHoodPos(double pos){
        hoodChanged = true;
        hoodPos = pos;
    }

    //---------------- Interface Methods ----------------
    @Override
    public void init(){
        setHoodPos(hoodStart);
    }

    @Override
    public void update(){
        setFlywheel(flywheelPow);

        if (hoodChanged){
            setHood(hoodPos);
            hoodChanged = false;
        }
    }
}
