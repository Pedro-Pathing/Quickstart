package org.firstinspires.ftc.teamcode.control.intake;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.Control;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;

public class IntakeControl implements Control {

    //---------------- Software ----------------
    Intake intake;
    Gamepad gp1;
    Gamepad gp2;
    Robot robot;

    //---------------- Constructor ----------------
    public IntakeControl(Intake intake, Gamepad gp1, Gamepad gp2){
        this.intake = intake;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public IntakeControl(Robot robot, Gamepad gp1, Gamepad gp2) {
        this.robot = robot;
        this.intake = robot.intake;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    //---------------- Methods ----------------

    //---------------- Interface Methods ----------------
    @Override
    public void update() {

    }

    @Override
    public void addTelemetry(Telemetry t) {

    }
}
