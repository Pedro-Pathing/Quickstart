package org.firstinspires.ftc.teamcode.CommandBase.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Global.Robot;

public class Intake extends SubsystemBase {
   Robot robot = Robot.getInstance();


    public void startIntake() {
        robot.intakeMotor.set(1);
    }

    public void startOuttake() {
        robot.intakeMotor.set(0);
    }



}
