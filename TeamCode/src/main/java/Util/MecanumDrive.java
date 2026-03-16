package Util;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class MecanumDrive {

    private final MotorEx frontLeft, frontRight, backLeft, backRight;
    double rotX, rotY, denominator, flP, blP, frP, brP;

    public MecanumDrive(MotorEx frontLeft, MotorEx frontRight, MotorEx backLeft, MotorEx backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
    }



    public void driveFieldCentric(double speedY, double speedX, double rotationX, double botHeading) {
        rotX = speedX * Math.cos(botHeading) - speedY * Math.sin(botHeading);
        rotY = speedX * Math.sin(botHeading) + speedY * Math.cos(botHeading);

        denominator = Math.max(Math.abs(speedY) + Math.abs(speedX) + Math.abs(rotationX), 1);

        //Front Left Wheel Power
        flP = (rotY + rotX + rotationX) / denominator;
        //Back Left Wheel Power
        blP = (rotY - rotX + rotationX) / denominator;
        //Front Right Wheel Power
        frP = (rotY - rotX - rotationX) / denominator;
        //Back Right Wheel Power
        brP = (rotY + rotX - rotationX) / denominator;

        frontLeft.set(flP);
        backLeft.set(blP);
        frontRight.set(frP);
        backRight.set(brP);
    }

}
