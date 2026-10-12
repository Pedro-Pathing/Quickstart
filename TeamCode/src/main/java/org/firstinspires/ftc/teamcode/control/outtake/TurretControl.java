package org.firstinspires.ftc.teamcode.control.outtake;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.Control;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.outtake.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.outtake.Turret;

public class TurretControl implements Control {

    //---------------- Software ----------------
    Turret turret;
    Gamepad gp1;
    Gamepad gp2;
    Robot robot;

    //---------------- Constructor ----------------
    public TurretControl(Outtake outtake, Gamepad gp1, Gamepad gp2){
        this.turret = outtake.turret;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public TurretControl(Robot robot, Gamepad gp1, Gamepad gp2) {
        this.robot = robot;
        this.turret = robot.outtake.turret;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    //---------------- Methods ----------------

    //---------------- Interface Methods ----------------
    @Override
    public void update(){

    }

    @Override
    public void addTelemetry(Telemetry telemetry){

    }
}
