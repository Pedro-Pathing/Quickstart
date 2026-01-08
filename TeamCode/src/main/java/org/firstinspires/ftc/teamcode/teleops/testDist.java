package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.DistToRPM;
import org.firstinspires.ftc.teamcode.util.DistToRPM.*;
@TeleOp

public class testDist extends OpMode {
    double dist, rpm;
    @Override
    public void init() {

    }
    @Override
    public void start() {

    }
    @Override
    public void loop() {
        dist = gamepad1.left_stick_y * 100;
        rpm = DistToRPM.CalcRPM(dist);
        telemetry.addData("dist", dist);
        telemetry.addData("RPM", rpm);
        telemetry.update();
    }
}
