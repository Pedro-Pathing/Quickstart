package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.EdgeDetector;

public class VisionControl implements Control {
    Vision vision;
    Gamepad gp1;
    Gamepad gp2;
    Robot robot;

    private final EdgeDetector snapButton;

    public VisionControl(Vision vision, Gamepad gp1, Gamepad gp2)
    {
        this.vision = vision;
        this.gp1 = gp1;
        this.gp2 = gp2;

        this.snapButton = new EdgeDetector(vision::takeSnapshot);
    }

    public VisionControl(Robot robot, Gamepad gp1, Gamepad gp2)
    {
        this.robot = robot;
        this.vision = robot.vision;
        this.gp1 = gp1;
        this.gp2 = gp2;

        this.snapButton = new EdgeDetector(vision::takeSnapshot);
    }


    @Override
    public void update() {
        snapButton.update(gp1.a);
    }

    @Override
    public void addTelemetry(Telemetry t) {
        t.addData("Network: ", vision.getConnectionStatus());
        t.addData("Snapshots Saved ", vision.getSnapshotCount());
        t.addData("Has Target?", vision.hasTarget());
        t.addData("Object(s) Tx:", vision.getTx());
        t.addData("Objects(s) Ty: ", vision.getTy());
        t.addData("Object(s) Area:", vision.getArea());
    }
}
