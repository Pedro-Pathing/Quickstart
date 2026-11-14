package org.firstinspires.ftc.teamcode.driveclasses;

import com.qualcomm.robotcore.hardware.DcMotor;

public class TankDrive {

    DcMotor rightDrive;
    DcMotor leftDrive;
    public TankDrive (DcMotor rightDrive, DcMotor leftDrive) {
        this.rightDrive = rightDrive;
        this.leftDrive = leftDrive;
    }

    public void drive(double forward, double turn) {
        double[] drivePowers = TankKinematics.tankKinematics(forward, turn);
        rightDrive.setPower(drivePowers[0]);
        leftDrive.setPower(drivePowers[1]);
    }
}