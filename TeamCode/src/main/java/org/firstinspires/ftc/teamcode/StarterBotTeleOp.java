/*   MIT License
 *   Copyright (c) [2026] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config("LauncherTuning")
@TeleOp(name = "Mec BioBuzz StarterBot Teleop", group = "StarterBot")
public class StarterBotTeleOp extends OpMode {

    // Launcher velocity PIDF gains. Must be non-zero or the launcher won't spin.
    public static double KP = 20;
    public static double KI = 0.0;
    public static double KD = 0.1;
    public static double KF = 12.5;

    // Dashboard telemetry declared at the class level so all methods can use it
    FtcDashboard dashboard = FtcDashboard.getInstance();
    org.firstinspires.ftc.robotcore.external.Telemetry dashboardTelemetry = dashboard.getTelemetry();

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftIntakeServo = null;
    private CRServo rightIntakeServo = null;
    private CRServo windmillServo = null;

    /*
     * Launcher velocities in encoder ticks per second (28 ticks/rev).
     * The windmill (transfer) only runs once the launcher is above MIN.
     */
    public final int LAUNCHER_TARGET_VELOCITY = 1200; //2678 RPM
    public final int LAUNCHER_MIN_VELOCITY = 1150; //2571 RPM

    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    double intakePower;

    @Override
    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "front_left");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right");
        leftBackDrive = hardwareMap.get(DcMotor.class, "back_left");
        rightBackDrive = hardwareMap.get(DcMotor.class, "back_right");
        intake = hardwareMap.get(DcMotor.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        windmillServo = hardwareMap.get(CRServo.class, "transfer");
        leftIntakeServo = hardwareMap.get(CRServo.class, "left_flower");
        rightIntakeServo = hardwareMap.get(CRServo.class, "right_flower");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
        windmillServo.setPower(0);

        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        windmillServo.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(KP, KI, KD, KF));

        if (gamepad1.dpadUpWasPressed())    KD += 0.01;
        if (gamepad1.dpadDownWasPressed())  KD -= 0.01;
        if (gamepad1.dpadRightWasPressed()) KI += 0.25;
        if (gamepad1.dpadLeftWasPressed())  KI -= 0.25;

        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        intakePower = gamepad1.right_trigger - gamepad1.left_trigger;

        launch();

        intake.setPower(intakePower);
        leftIntakeServo.setPower(intakePower);
        rightIntakeServo.setPower(intakePower);

        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftFrontPower, rightFrontPower);
        telemetry.addData("Triggers", "left (%.2f), right (%.2f)", gamepad1.left_trigger, gamepad1.right_trigger);
    }

    @Override
    public void stop() {
    }

    void mecanumDrive(double forward, double strafe, double rotate) {
        leftFrontPower = forward + strafe + rotate;
        rightFrontPower = forward - strafe - rotate;
        leftBackPower = forward - strafe + rotate;
        rightBackPower = forward + strafe - rotate;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    void launch() {

        boolean shoot = gamepad1.left_bumper;

        if (shoot) {
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else {
            launcher.setVelocity(0);
        }

        // Only feed once the launcher is up to speed.
        if (shoot && launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
            windmillServo.setPower(1);
            intakePower += 0.5;
        } else {
            windmillServo.setPower(0);
        }

        dashboardTelemetry.addData("Left bumper", shoot);
        dashboardTelemetry.addData("Launcher vel", "%.0f / %d", launcher.getVelocity(), LAUNCHER_TARGET_VELOCITY);
        dashboardTelemetry.addData("Target Velo", LAUNCHER_TARGET_VELOCITY);
        dashboardTelemetry.addData("Current Velo", launcher.getVelocity());
        dashboardTelemetry.addData("KP", KP);
        dashboardTelemetry.addData("KI", KI);
        dashboardTelemetry.addData("KD", KD);
        dashboardTelemetry.addData("KF", KF);

        telemetry.addData("Left bumper", shoot);
        telemetry.addData("Launcher vel", "%.0f / %d", launcher.getVelocity(), LAUNCHER_TARGET_VELOCITY);
        telemetry.addData("Target Velo", LAUNCHER_TARGET_VELOCITY);
        telemetry.addData("Current Velo", launcher.getVelocity());
        telemetry.addData("KP", KP);
        telemetry.addData("KI", KI);
        telemetry.addData("KD", KD);
        telemetry.addData("KF", KF);

        dashboardTelemetry.update();
    }
}