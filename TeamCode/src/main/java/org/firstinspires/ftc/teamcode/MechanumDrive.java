package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MechanumDrive{


    public MechanumDrive(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
    }

    public static MechanumDrive calculatePowers(double gamepad1_left_stick_y, double gamepad1_left_stick_x, double gamepad1_right_stick_x) {
    double max;

    // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
    double axial = -gamepad1_left_stick_y;  // Note: pushing stick forward gives negative value
    double lateral = gamepad1_left_stick_x;
    double yaw = gamepad1_right_stick_x;

    // Set up a variable for each drive wheel to save the power level for telemetry.
    double frontLeftPower = axial + lateral + yaw;
    double frontRightPower = axial - lateral - yaw;
    double backLeftPower = axial - lateral + yaw;
    double backRightPower = axial + lateral - yaw;

    // Normalize the values so no wheel power exceeds 100%
    // This ensures that the robot maintains the desired motion.
    max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
    max = Math.max(max, Math.abs(backLeftPower));
    max = Math.max(max, Math.abs(backRightPower));

    if (max > 1.0) {
        frontLeftPower /= max;
        frontRightPower /= max;
        backLeftPower /= max;
        backRightPower /= max;
    }

    // Show the elapsed game time and wheel power.
    return new MechanumDrive(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }
}