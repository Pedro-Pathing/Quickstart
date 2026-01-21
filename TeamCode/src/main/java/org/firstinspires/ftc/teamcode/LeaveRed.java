package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Time Based Robot With Intake & Ramp")
public class LeaveRed extends LinearOpMode {

    DcMotor fl, fr, bl, br;
    DcMotor intake;
    DcMotor rampMotor1, rampMotor2;

    @Override
    public void runOpMode() throws InterruptedException {

        // DRIVE MOTORS
        fl = hardwareMap.dcMotor.get("front_left_motor");
        bl = hardwareMap.dcMotor.get("back_left_motor");
        fr = hardwareMap.dcMotor.get("front_right_motor");
        br = hardwareMap.dcMotor.get("back_right_motor");

        // INTAKE + RAMP MOTORS

        // MOTOR DIRECTIONS
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        // -----------------------
        // EXAMPLE AUTONOMOUS STEPS
        // -----------------------
        driveForward(50, 300);
        wait(10000);
        strafeRight(50, 50);
    }

    // ============================
    // MOVEMENT METHODS
    // ============================

    public void driveForward(double power, long timeMs) {
        setDrive(power, power, power, power);
        sleep(timeMs);
        stopDrive();
    }

    public void driveBackward(double power, long timeMs) {
        setDrive(-power, -power, -power, -power);
        sleep(timeMs);
        stopDrive();
    }

    public void strafeRight(double power, long timeMs) {
        setDrive(power, -power, -power, power);
        sleep(timeMs);
        stopDrive();
    }

    public void strafeLeft(double power, long timeMs) {
        setDrive(-power, power, power, -power);
        sleep(timeMs);
        stopDrive();
    }

    public void turnRight(double power, long timeMs) {
        setDrive(power, -power, power, -power);
        sleep(timeMs);
        stopDrive();
    }

    public void turnLeft(double power, long timeMs) {
        setDrive(-power, power, -power, power);
        sleep(timeMs);
        stopDrive();
    }

    // ============================
    // INTAKE & RAMP METHODS
    // ============================

    public void runIntake(double power, long timeMs) {
        intake.setPower(power);
        sleep(timeMs);
        intake.setPower(0);
    }

    public void runRamp(double power, long timeMs) {
        rampMotor1.setPower(power);
        rampMotor2.setPower(power);
        sleep(timeMs);
        rampMotor1.setPower(0);
        rampMotor2.setPower(0);
    }

    // ============================
    // HELPER FUNCTIONS
    // ============================

    private void setDrive(double flP, double frP, double blP, double brP) {
        fl.setPower(flP);
        fr.setPower(frP);
        bl.setPower(blP);
        br.setPower(brP);
    }

    private void stopDrive() {
        setDrive(0, 0, 0, 0);
    }

    private void stopAllMotors() {
        stopDrive();
        intake.setPower(0);
        rampMotor1.setPower(0);
        rampMotor2.setPower(0);
    }
}
