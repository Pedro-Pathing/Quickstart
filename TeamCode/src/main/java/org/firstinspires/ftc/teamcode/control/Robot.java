package org.firstinspires.ftc.teamcode.control;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Robot {
    public final HardwareMap hardwareMap;
    public final Telemetry telemetry;
    public final Flywheel flywheel;

    public Robot(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
        flywheel = new Flywheel(this);
        periodic();
    }
    public void periodic() {
        flywheel.periodic();
    }
}
