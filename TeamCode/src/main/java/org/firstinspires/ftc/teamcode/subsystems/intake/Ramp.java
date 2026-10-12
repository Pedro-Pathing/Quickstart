package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

public class Ramp implements Subsystem {

    //---------------- Hardware ----------------
    private final Servo rampLeft;
    private final Servo rampRight;


    //---------------- Software ----------------
    //TODO find turret limits
    private final double turretMinDeg = 0;
    private final double turretMaxDeg = 360;

    //---------------- Constructor ----------------
    public Ramp(HardwareMap map) {
        rampLeft = map.get(Servo.class, "ramp_left");
        rampRight = map.get(Servo.class, "ramp_right");
    }

    //---------------- Core Methods ----------------


    //---------------- Methods ----------------


    //---------------- Interface Methods ----------------
    @Override
    public void init(){}

    @Override
    public void update(){}
}
