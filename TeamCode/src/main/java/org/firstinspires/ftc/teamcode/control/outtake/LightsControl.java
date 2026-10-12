package org.firstinspires.ftc.teamcode.control.outtake;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.Control;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.outtake.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.outtake.Lights;

public class LightsControl implements Control {

    //---------------- Software ----------------
    Lights lights;
    Gamepad gp1;
    Gamepad gp2;
    Robot robot;

    //---------------- Constructor ----------------
    public LightsControl(Outtake outtake, Gamepad gp1, Gamepad gp2){
        this.lights = outtake.lights;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public LightsControl(Robot robot, Gamepad gp1, Gamepad gp2) {
        this.robot = robot;
        this.lights = robot.outtake.lights;
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
