package org.firstinspires.ftc.teamcode.subsystems.outtake;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
    //TODO find turret limits
    private final double turretMinDeg = 0;
    private final double turretMaxDeg = 360;

    //---------------- Constructor ----------------
    public Turret(HardwareMap map) {
        turretLeft = map.get(Servo.class, "turret_left");
        turretRight = map.get(Servo.class, "turret_right");
        turretAnalog = map.get(AnalogInput.class, "turret_left_analog");
        turretEnc = new AbsoluteAnalogEncoder(turretAnalog, 3.3, 0, 1);
        util = new Utility();
    }

    //---------------- Core Methods ----------------


    //---------------- Methods ----------------


    //---------------- Interface Methods ----------------
    @Override
    public void init(){}

    @Override
    public void update(){}

}
