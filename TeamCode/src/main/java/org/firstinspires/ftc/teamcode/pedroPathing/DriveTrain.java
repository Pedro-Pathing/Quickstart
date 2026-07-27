package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/* JADX INFO: loaded from: classes7.dex */
public class DriveTrain {
    public final DcMotor backLeftMotor;
    public final DcMotor backRightMotor;
    public final DcMotor frontLeftMotor;
    public final DcMotor frontRightMotor;
    private final IMU imu;
    private final LinearOpMode linearOpMode;
    private int counter = 0;
    private double ROTATE_SPEED_ADJUSTER = 1.0d;
    private double DRIVE_AND_STRAFE_SPEED_ADJUSTER = 1.0d;
    private final SimplePIDController headingController = new SimplePIDController(15.0d, LynxServoController.apiPositionFirst, 60.0d);
    private final ElapsedTime headingTimer = new ElapsedTime();
    private boolean headingHoldActive = false;
    private double headingTargetDeg = LynxServoController.apiPositionFirst;

    public DriveTrain(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        this.frontLeftMotor = (DcMotor) linearOpMode.hardwareMap.dcMotor.get("frontLeftMotor");
        this.frontRightMotor = (DcMotor) linearOpMode.hardwareMap.dcMotor.get("frontRightMotor");
        this.backLeftMotor = (DcMotor) linearOpMode.hardwareMap.dcMotor.get("backLeftMotor");
        this.backRightMotor = (DcMotor) linearOpMode.hardwareMap.dcMotor.get("backRightMotor");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        this.imu = (IMU) linearOpMode.hardwareMap.get(IMU.class, "imu");
        this.imu.initialize(new IMU.Parameters(orientation));
    }

    public void configureMotorModes() {
        this.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.imu.resetYaw();
    }

    public double getHeading() {
        return this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void driveFieldCentric(double forward, double strafe, double rotation, double botHeading) {
        double rotX = (Math.cos(-botHeading) * strafe) - (Math.sin(-botHeading) * forward);
        double rotY = (Math.sin(-botHeading) * strafe) + (Math.cos(-botHeading) * forward);
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotation), 1.0d);
        double fl = ((rotY + rotX) + rotation) / denominator;
        double bl = ((rotY - rotX) + rotation) / denominator;
        double fr = ((rotY - rotX) - rotation) / denominator;
        double br = ((rotY + rotX) - rotation) / denominator;
        this.frontLeftMotor.setPower(fl / this.DRIVE_AND_STRAFE_SPEED_ADJUSTER);
        this.backLeftMotor.setPower(bl / this.DRIVE_AND_STRAFE_SPEED_ADJUSTER);
        this.frontRightMotor.setPower(fr / this.DRIVE_AND_STRAFE_SPEED_ADJUSTER);
        this.backRightMotor.setPower(br / this.DRIVE_AND_STRAFE_SPEED_ADJUSTER);
    }

    static class Powers {
        private final double backLeftPower;
        private final double backRightPower;
        private final double frontLeftPower;
        private final double frontRightPower;

        public Powers(double fl, double fr, double bl, double br) {
            this.frontLeftPower = fl;
            this.frontRightPower = fr;
            this.backLeftPower = bl;
            this.backRightPower = br;
        }

        public double getFrontLeftPower() {
            return this.frontLeftPower;
        }

        public double getFrontRightPower() {
            return this.frontRightPower;
        }

        public double getBackLeftPower() {
            return this.backLeftPower;
        }

        public double getBackRightPower() {
            return this.backRightPower;
        }
    }

    static class Rotation {
        private final double rotX;
        private final double rotY;

        public Rotation(double rotX, double rotY) {
            this.rotX = rotX;
            this.rotY = rotY;
        }

        public double getRotX() {
            return this.rotX;
        }

        public double getRotY() {
            return this.rotY;
        }
    }

    public void resetYaw() {
        if (this.linearOpMode.gamepad1.start || this.linearOpMode.gamepad1.back) {
            this.imu.resetYaw();
            this.counter++;
        }
    }

    public void adjustTurnSpeed() {
        if (this.linearOpMode.gamepad1.right_bumper) {
            this.ROTATE_SPEED_ADJUSTER = 3.0d;
            this.DRIVE_AND_STRAFE_SPEED_ADJUSTER = 3.0d;
        } else if (this.linearOpMode.gamepad1.left_bumper) {
            this.ROTATE_SPEED_ADJUSTER = 1.0d;
            this.DRIVE_AND_STRAFE_SPEED_ADJUSTER = 1.0d;
        }
    }

    public void displayTelemetry() {
        this.linearOpMode.telemetry.addData("Heading (Deg)", Double.valueOf(Math.toDegrees(getHeading())));
        this.linearOpMode.telemetry.addData("Front Left Pwr", Double.valueOf(this.frontLeftMotor.getPower()));
    }

    public void setMovePower(double power) {
        this.frontLeftMotor.setPower(power);
        this.frontRightMotor.setPower(power);
        this.backLeftMotor.setPower(power);
        this.backRightMotor.setPower(power);
    }
}
