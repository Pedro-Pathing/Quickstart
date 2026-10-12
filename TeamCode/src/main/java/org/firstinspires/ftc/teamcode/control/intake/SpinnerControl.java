package org.firstinspires.ftc.teamcode.control.intake;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.Control;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.Spinner;

public class SpinnerControl implements Control {

    //---------------- Software ----------------
    Spinner spinner;
    Gamepad gp1;
    Gamepad gp2;
    Robot robot;

    //---------------- Constructor ----------------
    public SpinnerControl(Intake intake, Gamepad gp1, Gamepad gp2){
        this.spinner = intake.spinner;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public SpinnerControl(Robot robot, Gamepad gp1, Gamepad gp2) {
        this.robot = robot;
        this.spinner = robot.intake.spinner;
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
