package org.firstinspires.ftc.teamcode.subsystems.outtake;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Subsystem;

public class Outtake implements Subsystem {

    //---------------- Subsystems ----------------
    private final Shooter shooter;
    private final Turret turret;
    private final Lights lights;


    //---------------- Software ----------------


    //---------------- Constructor ----------------
    public Outtake(HardwareMap map) {
        shooter = new Shooter(map);
        turret = new Turret(map);
        lights = new Lights(map);
    }

    //---------------- Methods ----------------



    //---------------- Interface Methods ----------------
    @Override
    public void init(){
        shooter.init();
        turret.init();
        lights.init();
    }

    @Override
    public void update(){
        shooter.update();
        turret.update();
        lights.update();
    }

}
