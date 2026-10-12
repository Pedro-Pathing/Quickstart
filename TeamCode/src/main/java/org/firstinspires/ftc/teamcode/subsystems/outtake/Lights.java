package org.firstinspires.ftc.teamcode.subsystems.outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

public class Lights implements Subsystem {

    //---------------- Hardware ----------------
    private final Servo lightLeft;
    private final Servo lightRight;

    //---------------- Software ----------------
    private double lightLeftPos = 0;
    private boolean lightLeftChanged = false;
    private double lightRightPos = 0;
    private boolean lightRightChanged = false;

    //---------------- Constructor ----------------
    public Lights(HardwareMap map) {
        lightLeft = map.get(Servo.class, "light_left");
        lightRight = map.get(Servo.class, "light_right");
    }

    //---------------- Core Methods ----------------
    private void setLightLeft(double pos){
        lightLeft.setPosition(pos);
    }

    private void setLightRight(double pos){
        lightRight.setPosition(pos);
    }

    //---------------- Methods ----------------
    public void setLightLeftPos(double pos){
        lightLeftChanged = true;
        lightLeftPos = pos;
    }

    public void setLightRightPos(double pos){
        lightRightChanged = true;
        lightRightPos = pos;
    }

    public void setLights(double pos){
        setLightLeftPos(pos);
        setLightRightPos(pos);
    }

    //---------------- Interface Methods ----------------
    @Override
    public void init(){}

    @Override
    public void update(){
        if (lightLeftChanged){
            setLightLeft(lightLeftPos);
            lightLeftChanged = false;
        }
        if (lightRightChanged){
            setLightRight(lightRightPos);
            lightRightChanged = false;
        }
    }
}
