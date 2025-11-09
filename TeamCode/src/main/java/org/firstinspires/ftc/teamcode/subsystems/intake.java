package org.firstinspires.ftc.teamcode.subsystems;



import org.firstinspires.ftc.teamcode.util.Hardware;

public class intake {
    private Hardware robot;

    public intake(Hardware robot) {
        this.robot = robot;
    }
    public void intakePower(double power) {
        robot.intake.setPower(power);
    }


    public void intakeStop() {
        robot.intake.setPower(0);
    }
}
