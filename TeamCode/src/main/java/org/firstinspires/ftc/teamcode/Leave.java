package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Time Based Movement Demo")
public class Leave extends LinearOpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft  = hardwareMap.dcMotor.get("front_left_motor");
        frontRight = hardwareMap.dcMotor.get("front_right_motor");
        backLeft   = hardwareMap.dcMotor.get("back_left_motor");
        backRight  = hardwareMap.dcMotor.get("back_right_motor");

        // Reverse these if your robot drives backwards
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        // EXAMPLE MOVES:
        driveForward(0.5, 300);  // speed, time in ms
    }

    // ---------------------------
    // MOVEMENT FUNCTIONS
    // ---------------------------

    // Drive Forward
    public void driveForward(double power, long timeMs) {
        setAllMotors(power, power, power, power);
        sleep(timeMs);
        stopMotors();
    }

    // Drive Backward
    public void driveBackward(double power, long timeMs) {
        setAllMotors(-power, -power, -power, -power);
        sleep(timeMs);
        stopMotors();
    }

    // Strafe Right
    public void strafeRight(double power, long timeMs) {
        setAllMotors(power, -power, -power, power);
        sleep(timeMs);
        stopMotors();
    }

    // Strafe Left
    public void strafeLeft(double power, long timeMs) {
        setAllMotors(-power, power, power, -power);
        sleep(timeMs);
        stopMotors();
    }

    // Turn Right (Pivot)
    public void turnRight(double power, long timeMs) {
        setAllMotors(power, -power, power, -power);
        sleep(timeMs);
        stopMotors();
    }

    // Turn Left (Pivot)
    public void turnLeft(double power, long timeMs) {
        setAllMotors(-power, power, -power, power);
        sleep(timeMs);
        stopMotors();
    }

    // Helper: set all motors
    private void setAllMotors(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    // Helper: stop motors
    private void stopMotors() {
        setAllMotors(0, 0, 0, 0);
    }
}
