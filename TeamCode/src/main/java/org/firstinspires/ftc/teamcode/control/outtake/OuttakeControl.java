package org.firstinspires.ftc.teamcode.control.outtake;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.Control;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.outtake.Outtake;

public class OuttakeControl implements Control {

    //---------------- Software ----------------
    Outtake outtake;
    Gamepad gp1;
    Gamepad gp2;
    Robot robot;

    //---------------- Constructor ----------------
    public OuttakeControl(Outtake outtake, Gamepad gp1, Gamepad gp2){
        this.outtake = outtake;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public OuttakeControl(Robot robot, Gamepad gp1, Gamepad gp2) {
        this.robot = robot;
        this.outtake = robot.outtake;
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
