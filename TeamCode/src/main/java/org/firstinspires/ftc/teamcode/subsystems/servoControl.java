package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.util.Hardware;

public class servoControl {
    private Hardware robot;

    public servoControl(Hardware robot) {
        this.robot = robot;
    }

    public void servoPower(double power) {
        robot.mid1.setPower(power);
        robot.mid2.setPower(power);
    }



    public void servoStop() {
        robot.mid1.setPower(0);
        robot.mid2.setPower(0);
    }
}
