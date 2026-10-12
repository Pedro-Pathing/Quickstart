package org.firstinspires.ftc.teamcode.control.intake;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.Control;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.Ramp;

public class RampControl implements Control {

    //---------------- Software ----------------
    Ramp ramp;
    Gamepad gp1;
    Gamepad gp2;
    Robot robot;

    //---------------- Constructor ----------------
    public RampControl(Intake intake, Gamepad gp1, Gamepad gp2){
        this.ramp = intake.ramp;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public RampControl(Robot robot, Gamepad gp1, Gamepad gp2) {
        this.robot = robot;
        this.ramp = robot.intake.ramp;
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
