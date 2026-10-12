package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.util.Utility;

public class Ramp implements Subsystem {

    //---------------- Hardware ----------------
    private final Servo rampLeft;
    private final Servo rampRight;
    private final Utility util;


    //---------------- Software ----------------
    private final double rightRampOffset = 0.024;
    private final double rampStart = 0.5;
    private boolean negativeOffset;
    private double rampPos = rampStart;
    private boolean rampChanged = false;
    //TODO find ramp limits
    private final double rampMinPos = 0;
    private final double rampMaxPos = 1;

    //---------------- Constructor ----------------
    public Ramp(HardwareMap map) {
        rampLeft = map.get(Servo.class, "ramp_left");
        rampRight = map.get(Servo.class, "ramp_right");
        rampLeft.setDirection(Servo.Direction.REVERSE);
        util = new Utility();

        negativeOffset = rightRampOffset < 0;
    }

    //---------------- Core Methods ----------------
    public void setRamp(double pos){
        pos = util.clamp(pos, (negativeOffset ? rampMinPos-rightRampOffset : rampMinPos), (negativeOffset ? rampMaxPos : rampMaxPos-rightRampOffset));
        rampLeft.setPosition(pos);
        rampRight.setPosition(pos+rightRampOffset);
    }


    //---------------- Methods ----------------
    public void setRampPos(double pos){
        rampChanged = true;
        rampPos = pos;
    }

    //---------------- Interface Methods ----------------
    @Override
    public void init(){
        setRampPos(rampStart);
    }

    @Override
    public void update(){
        if (rampChanged){
            setRamp(rampPos);
            rampChanged = false;
        }
    }
}
