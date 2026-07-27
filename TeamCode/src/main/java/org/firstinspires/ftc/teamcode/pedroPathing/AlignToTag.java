package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/* JADX INFO: loaded from: classes7.dex */
@Autonomous(name = "Tag Align Auto")
@Disabled
public class AlignToTag extends LinearOpMode {
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    private DecodeAuto decodeAuto;
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    IMU imu;
    GoBildaPinpointDriver odo;
    AprilTagProcessor tagProcessor;
    int counter = 0;
    boolean PPG = false;
    boolean PGP = false;
    boolean GPP = false;
    boolean loopFinished = true;

    @Override // com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
    public void runOpMode() throws InterruptedException {
        initAuto();
        waitForStart();
        if (isStopRequested()) {
            return;
        }
        waitForStart();
        while (!isStopRequested()) {
            this.loopFinished = true;
            AprilTagDetection tag = getLatestTag();
            this.decodeAuto.AlignToTag(tag);
        }
    }

    private void driveToPos(double targetX, double targetY) {
        this.odo.update();
        boolean telemAdded = true;
        while (opModeIsActive() && (Math.abs(targetX + this.odo.getPosX(DistanceUnit.MM)) > 30.0d || Math.abs(targetY - this.odo.getPosY(DistanceUnit.MM)) > 30.0d)) {
            this.odo.update();
            double x = (targetX + this.odo.getPosX(DistanceUnit.MM)) * 0.001d;
            double y = (targetY - this.odo.getPosY(DistanceUnit.MM)) * (-0.001d);
            double botHeading = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotY = (Math.cos(-botHeading) * y) - (Math.sin(-botHeading) * x);
            double rotX = (Math.sin(-botHeading) * y) + (Math.cos(-botHeading) * x);
            if (!telemAdded) {
                this.telemetry.addData("x: ", Double.valueOf(x));
                this.telemetry.addData("y: ", Double.valueOf(y));
                this.telemetry.addData("rotX: ", Double.valueOf(rotX));
                this.telemetry.addData("rotY: ", Double.valueOf(rotY));
                this.telemetry.update();
                telemAdded = true;
            }
            if (Math.abs(rotX) < 0.15d) {
                rotX = Math.signum(rotX) * 0.15d;
            }
            if (Math.abs(rotY) < 0.15d) {
                rotY = Math.signum(rotY) * 0.15d;
            }
            double denominator = Math.max(Math.abs(y) + Math.abs(x), 1.0d);
            double frontLeftPower = (rotX + rotY) / denominator;
            boolean telemAdded2 = telemAdded;
            double backLeftPower = (rotX - rotY) / denominator;
            double frontRightPower = (rotX - rotY) / denominator;
            double backRightPower = (rotX + rotY) / denominator;
            this.frontLeftMotor.setPower(frontLeftPower);
            this.backLeftMotor.setPower(backLeftPower);
            this.frontRightMotor.setPower(frontRightPower);
            this.backRightMotor.setPower(backRightPower);
            telemAdded = telemAdded2;
        }
        this.frontLeftMotor.setPower(LynxServoController.apiPositionFirst);
        this.backLeftMotor.setPower(LynxServoController.apiPositionFirst);
        this.frontRightMotor.setPower(LynxServoController.apiPositionFirst);
        this.backRightMotor.setPower(LynxServoController.apiPositionFirst);
    }

    private void AlignToTag(AprilTagDetection tag) {
        double error = tag.ftcPose.yaw;
        while (true) {
            if (!opModeIsActive() || Math.abs(error) <= 1.0d) {
                break;
            }
            this.odo.update();
            AprilTagDetection currentTag = getLatestTag();
            if (currentTag == null) {
                this.telemetry.addLine("Tag lost — stopping alignment.");
                break;
            }
            error = currentTag.ftcPose.yaw;
            double drivePower = error / 50.0d;
            if (drivePower > LynxServoController.apiPositionFirst) {
                drivePower = Math.max(drivePower, 0.35d);
            } else if (drivePower < LynxServoController.apiPositionFirst) {
                drivePower = Math.min(drivePower, -0.35d);
            }
            this.frontLeftMotor.setPower(-drivePower);
            this.backLeftMotor.setPower(-drivePower);
            this.frontRightMotor.setPower(drivePower);
            this.backRightMotor.setPower(drivePower);
            this.telemetry.addData("Y:", Double.valueOf(this.odo.getPosY(DistanceUnit.MM)));
            this.telemetry.addData("X:", Double.valueOf(-this.odo.getPosX(DistanceUnit.MM)));
            this.telemetry.addData("Tag Yaw", Double.valueOf(error));
            this.telemetry.addData("Drive Power", Double.valueOf(drivePower));
            this.telemetry.update();
        }
        this.frontLeftMotor.setPower(LynxServoController.apiPositionFirst);
        this.backLeftMotor.setPower(LynxServoController.apiPositionFirst);
        this.frontRightMotor.setPower(LynxServoController.apiPositionFirst);
        this.backRightMotor.setPower(LynxServoController.apiPositionFirst);
    }

    private AprilTagDetection getLatestTag() {
        if (this.tagProcessor.getDetections().size() > 0) {
            AprilTagDetection aprilTagDetection = this.tagProcessor.getDetections().get(0);
            return aprilTagDetection;
        }
        return null;
    }

    private void gyroTurnToAngle(double turnAngle) {
        this.imu.resetYaw();
        double error = turnAngle;
        while (opModeIsActive() && (error > 1.0d || error < -1.0d)) {
            this.odo.update();
            this.telemetry.addData("X: ", Double.valueOf(-this.odo.getPosX(DistanceUnit.MM)));
            this.telemetry.addData("Y: ", Double.valueOf(this.odo.getPosY(DistanceUnit.MM)));
            this.telemetry.addData("Heading IMU: ", Double.valueOf(this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
            this.telemetry.update();
            double driveMotorsPower = error / 200.0d;
            if ((driveMotorsPower >= 0.2d || driveMotorsPower <= LynxServoController.apiPositionFirst) && driveMotorsPower > -0.2d && driveMotorsPower < LynxServoController.apiPositionFirst) {
            }
            double driveMotorsPower2 = error / 50.0d;
            if (driveMotorsPower2 < 0.35d && driveMotorsPower2 > LynxServoController.apiPositionFirst) {
                driveMotorsPower2 = 0.35d;
            } else if (driveMotorsPower2 > -0.35d && driveMotorsPower2 < LynxServoController.apiPositionFirst) {
                driveMotorsPower2 = -0.35d;
            }
            this.frontLeftMotor.setPower(-driveMotorsPower2);
            this.backLeftMotor.setPower(-driveMotorsPower2);
            this.frontRightMotor.setPower(driveMotorsPower2);
            this.backRightMotor.setPower(driveMotorsPower2);
            double currentHeadingAngle = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            error = turnAngle - currentHeadingAngle;
        }
        this.frontLeftMotor.setPower(LynxServoController.apiPositionFirst);
        this.backLeftMotor.setPower(LynxServoController.apiPositionFirst);
        this.frontRightMotor.setPower(LynxServoController.apiPositionFirst);
        this.backRightMotor.setPower(LynxServoController.apiPositionFirst);
    }

    private void initAuto() {
        this.decodeAuto = new DecodeAuto(this);
        this.odo = (GoBildaPinpointDriver) this.hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        this.odo.setOffsets(65.0d, 142.0d, DistanceUnit.MM);
        this.odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        this.odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        this.odo.resetPosAndIMU();
        this.odo.recalibrateIMU();
        this.frontLeftMotor = (DcMotor) this.hardwareMap.get(DcMotor.class, "frontLeftMotor");
        this.backLeftMotor = (DcMotor) this.hardwareMap.get(DcMotor.class, "backLeftMotor");
        this.frontRightMotor = (DcMotor) this.hardwareMap.get(DcMotor.class, "frontRightMotor");
        this.backRightMotor = (DcMotor) this.hardwareMap.get(DcMotor.class, "backRightMotor");
        this.frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.imu = (IMU) this.hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        this.imu.initialize(parameters);
        this.imu.resetYaw();
        ElapsedTime timer = new ElapsedTime();
        if (timer.seconds() >= 1.0d) {
            this.counter++;
            timer.reset();
            this.telemetry.addData("Counter:", Integer.valueOf(this.counter));
            this.telemetry.update();
        }
    }
}
