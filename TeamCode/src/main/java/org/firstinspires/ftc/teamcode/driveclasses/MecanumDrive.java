package org.firstinspires.ftc.teamcode.driveclasses;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MecanumDrive {
    DcMotor rightFrontDrive;
    DcMotor leftFrontDrive;
    DcMotor rightBackDrive;
    DcMotor leftBackDrive;

    public MecanumDrive (DcMotor rightFrontDrive, DcMotor leftFrontDrive, DcMotor rightBackDrive, DcMotor leftBackDrive) {
        this.rightFrontDrive = rightFrontDrive;
        this.leftFrontDrive = leftFrontDrive;
        this.rightBackDrive = rightBackDrive;
        this.leftBackDrive = leftBackDrive;
    }

    public void drive(double forward, double strafe, double turn) {
        double[] drivePowers = MecanumKinematics.mecanumKinematics(forward, strafe, turn);
        rightFrontDrive.setPower(drivePowers[0]);
        leftFrontDrive.setPower(drivePowers[1]);
        rightBackDrive.setPower(drivePowers[2]);
        leftBackDrive.setPower(drivePowers[3]);
    }

}
