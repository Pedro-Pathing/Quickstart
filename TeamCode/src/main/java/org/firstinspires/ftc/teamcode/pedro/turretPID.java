package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.pedro.mechanisms.robot;

@TeleOp
public class turretPID extends OpMode {
    robot robot = new robot();
    int selector = 0;
    double P = 0;
    double I = 0;
    double D = 0;
    double position = 0;
    double speed = 1;
    double range = 0;
    int run = 1;

    public double changeValue(double howMuch) {
        double variableName = 0;
        if(gamepad1.dpadRightWasReleased()) {
            variableName += howMuch;
        } else if(gamepad1.dpadLeftWasReleased()) {
            variableName -= howMuch;
        }
        return variableName;
    }

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        if(gamepad1.dpadUpWasReleased()) {
            selector += 1;
        } else if(gamepad1.dpadDownWasReleased()) {
            selector -= 1;
        }
        switch(selector) {
            case 0:
                P += changeValue(0.1);
                telemetry.addData("P" , P);
                break;
            case 1:
                I += changeValue(0.5);
                telemetry.addData("I" , I);
                break;
            case 2:
                D += changeValue(0.5);
                telemetry.addData("D", D);
                break;
            case 3:
                position += changeValue(10);
                telemetry.addData("position", position);
                break;
            case 4:
                telemetry.addData("speed", speed);
                break;
            case 5:
                range += changeValue(1);
                telemetry.addData("range", range);
                break;
            default:
                telemetry.addData("Use the Dpad up and down to make within range of 0-4", selector);
        }
        if(gamepad1.aWasReleased()) {
            run *= -1;
        }
        if(run == -1) {
            robot.runToPositionPID(robot.getTurret(), position, speed, range, new PIDFCoefficients(P, I, D, 0));
        } else {
            robot.getTurret().setPower(0);
        }
        telemetry.addData("position", robot.getTurret().getCurrentPosition());
        telemetry.update();
    }
}
