package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.outtake.Shooter;

public class Intake implements Subsystem {

    //---------------- Subsystems ----------------
    public final Ramp ramp;
    public final Spinner spinner;
    public final Transfer transfer;

    //---------------- Software ----------------


    //---------------- Constructor ----------------
    public Intake(HardwareMap map) {
        ramp = new Ramp(map);
        spinner = new Spinner(map);
        transfer = new Transfer(map);
    }

    //---------------- Methods ----------------


    //---------------- Interface Methods ----------------
    @Override
    public void init(){
        ramp.init();
        spinner.init();
        transfer.init();
    }

    @Override
    public void update(){
        ramp.update();
        spinner.update();
        transfer.update();
    }
}
