package org.firstinspires.ftc.teamcode;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Field Centric TeleOp", group = "Teleop")
public class FieldCentricTeleOp extends LinearOpMode {

    private static final String FL_NAME = "lf";
    private static final String BL_NAME = "lr";
    private static final String FR_NAME = "rf";
    private static final String BR_NAME = "rr";

    private static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    private static final RevHubOrientationOnRobot.UsbFacingDirection USB_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;

    @Override
    public void runOpMode() {
        Scheduler.reset();

        DcMotor frontLeft = hardwareMap.get(DcMotor.class, FL_NAME);
        DcMotor backLeft = hardwareMap.get(DcMotor.class, BL_NAME);
        DcMotor frontRight = hardwareMap.get(DcMotor.class, FR_NAME);
        DcMotor backRight = hardwareMap.get(DcMotor.class, BR_NAME);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{frontLeft, backLeft, frontRight, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(LOGO_DIR, USB_DIR)));

        Command drive = Command.build()
                .setExecute(() -> {
                    if (gamepad1.options) {
                        imu.resetYaw();
                    }

                    double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                    double axial = -gamepad1.left_stick_y;
                    double lateral = gamepad1.left_stick_x;
                    double yaw = gamepad1.right_stick_x;

                    double rotX = lateral * Math.cos(-heading) - axial * Math.sin(-heading);
                    double rotY = lateral * Math.sin(-heading) + axial * Math.cos(-heading);

                    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(yaw), 1.0);

                    frontLeft.setPower((rotY + rotX + yaw) / denominator);
                    backLeft.setPower((rotY - rotX + yaw) / denominator);
                    frontRight.setPower((rotY - rotX - yaw) / denominator);
                    backRight.setPower((rotY + rotX - yaw) / denominator);
                })
                .setDone(() -> false)
                .setEnd(wasInterrupted -> {
                    frontLeft.setPower(0);
                    backLeft.setPower(0);
                    frontRight.setPower(0);
                    backRight.setPower(0);
                })
                .requiring(frontLeft);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        drive.schedule();

        while (opModeIsActive()) {
            Scheduler.execute();
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
            telemetry.update();
        }
    }
}
