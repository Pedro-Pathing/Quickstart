package org.firstinspires.ftc.teamcode.subsystems.outtake;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.util.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.util.Utility;

public class Turret implements Subsystem {

    //---------------- Hardware ----------------
    private final Servo turretLeft;
    private final Servo turretRight;
    private final AnalogInput turretAnalog;
    private final AbsoluteAnalogEncoder turretEnc;
    private final Utility util;

    //---------------- Software ----------------
    private final double rightTurretOffset = -0.008;
    private boolean negativeOffset;
    private final double turretPwmMinUs = 500; // are these the right limits?
    private final double turretPwmMaxUs = 2500;
    //TODO find turret limits
    private final double turretMinPos = 0;
    private final double turretMaxPos = 1;

    //---------------- Constructor ----------------
    public Turret(HardwareMap map) {
        turretLeft = map.get(Servo.class, "turret_left");
        turretRight = map.get(Servo.class, "turret_right");
        turretAnalog = map.get(AnalogInput.class, "turret_left_analog");
        turretEnc = new AbsoluteAnalogEncoder(turretAnalog, 3.3, 0, 1);
        util = new Utility();
        PwmControl.PwmRange pwmRange = new PwmControl.PwmRange(turretPwmMinUs, turretPwmMaxUs);
        ((PwmControl) turretLeft).setPwmRange(pwmRange);
        ((PwmControl) turretRight).setPwmRange(pwmRange);

        negativeOffset = rightTurretOffset < 0;
    }

    //---------------- Core Methods ----------------
    public void setTurret(double pos){
        pos = util.clamp(pos, (negativeOffset ? turretMinPos-rightTurretOffset : turretMinPos), (negativeOffset ? turretMaxPos : turretMaxPos-rightTurretOffset));
        turretLeft.setPosition(pos);
        turretRight.setPosition(pos+rightTurretOffset);
    }

    //---------------- Methods ----------------


    //---------------- Interface Methods ----------------
    @Override
    public void init(){
        
    }

    @Override
    public void update(){

    }

}
