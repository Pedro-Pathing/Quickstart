package org.firstinspires.ftc.teamcode.CommandBase.Subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;


import org.firstinspires.ftc.teamcode.Global.Robot;

import Util.MecanumDrive;

public class Drive extends SubsystemBase {
    Robot robot = Robot.getInstance();
    MecanumDrive mecanumDrive;

    double botHeading = 0;
    double offsetAngle = 0;

    public Drive() {
        mecanumDrive = new MecanumDrive(
                robot.frontLeftMotor,
                robot.frontRightMotor,
                robot.backLeftMotor,
                robot.backRightMotor
        );
    }

    public void driveFieldCentric(double speedY, double speedX, double rotationX, double botHeading) {
        mecanumDrive.driveFieldCentric(
            speedY, speedX, rotationX, botHeading
        );
    }

    public double getBotHeading() {
        botHeading = robot.imu.getAngularOrientation().firstAngle;
        return -botHeading - offsetAngle;
    }

    public void updateBotHeading() {
        offsetAngle = -robot.imu.getAngularOrientation().firstAngle;
    }

}
