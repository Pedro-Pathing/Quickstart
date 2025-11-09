package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.util.Hardware;

public class servoControl {
    private Hardware robot;

    public servoControl(Hardware robot) {
        this.robot = robot;
    }

    public void servoPower(double power) {
        robot.ballServo1.setPower(power);
        robot.ballServo2.setPower(power);
    }



    public void servoStop() {
        robot.ballServo1.setPower(0);
        robot.ballServo2.setPower(0);
    }
}
