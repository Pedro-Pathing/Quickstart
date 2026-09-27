package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;

import dev.nextftc.hardware.actuators.NextMotor;
import dev.nextftc.robot.Mechanism;
import dev.nextftc.robot.drive.DriveCommands;

public class Drivetrain implements Mechanism {
    public final NextMotor frontLeft = new NextMotor("frontLeftDriveMotor");
    public final NextMotor frontRight = new NextMotor("frontRightDriveMotor");
    public final NextMotor backLeft = new NextMotor("backLeftDriveMotor");
    public final NextMotor backRight = new NextMotor("backRightDriveMotor");

    //robot centric
    public void startDrive(Gamepad gamepad) {
        frontLeft.setDirection(NextMotor.Direction.REVERSE);
        backLeft.setDirection(NextMotor.Direction.REVERSE);
        DriveCommands.mecanumDrive(
                frontLeft, frontRight, backLeft, backRight, gamepad).schedule();
    }

    //field centric
    public void startDrive(Gamepad gamepad, Follower follower) {
        frontLeft.setDirection(NextMotor.Direction.REVERSE);
        backLeft.setDirection(NextMotor.Direction.REVERSE);
        DriveCommands.mecanumDriveFieldCentric(
                frontLeft, frontRight, backLeft, backRight, gamepad,
                () -> follower.pose().heading()).schedule();
    }
}