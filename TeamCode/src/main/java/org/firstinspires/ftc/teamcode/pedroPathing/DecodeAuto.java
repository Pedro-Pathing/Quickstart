package org.firstinspires.ftc.teamcode.pedroPathing;

import android.os.SystemClock;

import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Iterator;
import java.util.List;
import java.util.Locale;

/* JADX INFO: loaded from: classes7.dex */
public class DecodeAuto {
    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    private DcMotor containerMotor;
    float counter;
    private Servo flapServo;
    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    IMU imu;
    private DcMotor intakeMotor;
    private double launchFactor;
    private double launchVelocity;
    private final LinearOpMode linearOpMode;
    boolean loopState = true;
    com.qualcomm.hardware.gobilda.GoBildaPinpointDriver odo;
    private final DcMotorEx outtakeMotor;
    AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;

    public DecodeAuto(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        this.frontLeftMotor = (DcMotor) linearOpMode.hardwareMap.get(DcMotor.class, "frontLeftMotor");
        this.frontRightMotor = (DcMotor) linearOpMode.hardwareMap.get(DcMotor.class, "frontRightMotor");
        this.backLeftMotor = (DcMotor) linearOpMode.hardwareMap.get(DcMotor.class, "backLeftMotor");
        this.backRightMotor = (DcMotor) linearOpMode.hardwareMap.get(DcMotor.class, "backRightMotor");
        this.outtakeMotor = (DcMotorEx) linearOpMode.hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        this.intakeMotor = (DcMotor) linearOpMode.hardwareMap.get(DcMotor.class, "intakeMotor");
        this.containerMotor = (DcMotor) linearOpMode.hardwareMap.get(DcMotor.class, "containerMotor");
        this.flapServo = (Servo) linearOpMode.hardwareMap.get(Servo.class, "flapServo");
//        MotorConfigurationType config = this.outtakeMotor.getMotorType().m174clone();
//        config.setAchieveableMaxRPMFraction(1.0d);
//        this.outtakeMotor.setMotorType(config);
        this.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.containerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.outtakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.containerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.flapServo.setDirection(Servo.Direction.FORWARD);
        this.outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(85.0d, LynxServoController.apiPositionFirst, 26.0d, 12.536939130434783d);
        this.outtakeMotor.setVelocityPIDFCoefficients(85.0d, LynxServoController.apiPositionFirst, 26.0d, 12.536939130434783d);
        this.outtakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        this.launchVelocity = 1255.0d;
        this.imu = (IMU) linearOpMode.hardwareMap.get(IMU.class, "imu");
        this.odo = (com.qualcomm.hardware.gobilda.GoBildaPinpointDriver) linearOpMode.hardwareMap.get(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.class, "odo");
    }

    private void initAprilTag() {
        this.aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        this.visionPortal = VisionPortal.easyCreateWithDefaults((CameraName) this.linearOpMode.hardwareMap.get(WebcamName.class, "Webcam"), this.aprilTag);
    }

    private void stopMotors() {
        this.frontLeftMotor.setPower(LynxServoController.apiPositionFirst);
        this.backLeftMotor.setPower(LynxServoController.apiPositionFirst);
        this.frontRightMotor.setPower(LynxServoController.apiPositionFirst);
        this.backRightMotor.setPower(LynxServoController.apiPositionFirst);
    }

    public void alignToAprilTag(int desiredTagId) {
        AprilTagDetection tag = null;
        while (opModeIsActive()) {
            if (!this.tagProcessor.getDetections().isEmpty()) {
                Iterator<AprilTagDetection> it = this.tagProcessor.getDetections().iterator();
                while (true) {
                    if (!it.hasNext()) {
                        break;
                    }
                    AprilTagDetection t = it.next();
                    if (t.id == desiredTagId) {
                        tag = t;
                        break;
                    }
                }
            }
            if (tag == null) {
                this.linearOpMode.telemetry.addLine("Searching for Tag " + desiredTagId + "...");
                this.linearOpMode.telemetry.update();
            } else {
                return;
            }
        }
    }

    public void alignToAprilTagPixels(int desiredTagId) {
        this.aprilTag.getDetections();
        initAprilTag();
        for (AprilTagDetection detection : this.tagProcessor.getDetections()) {
            double error = detection.center.y;
            while (opModeIsActive() && Math.abs(error) > 1.0d) {
                error = detection.center.y;
                double power = error / 45.0d;
                double power2 = power > LynxServoController.apiPositionFirst ? Math.max(power, 0.22d) : Math.min(power, -0.22d);
                this.frontLeftMotor.setPower(-power2);
                this.backLeftMotor.setPower(-power2);
                this.frontRightMotor.setPower(power2);
                this.backRightMotor.setPower(power2);
                this.linearOpMode.telemetry.addData("Tag ID", Integer.valueOf(detection.id));
                this.linearOpMode.telemetry.addData("Yaw Error", Double.valueOf(error));
                this.linearOpMode.telemetry.addData("Turn Power", Double.valueOf(power2));
                this.linearOpMode.telemetry.update();
            }
        }
        stopMotors();
    }

    public void setTagProcessor(AprilTagProcessor processor) {
        this.tagProcessor = processor;
    }

    private AprilTagDetection getLatestTagById(int id) {
        for (AprilTagDetection t : this.tagProcessor.getDetections()) {
            if (t.id == id) {
                return t;
            }
        }
        return null;
    }

    private AprilTagDetection getLatestTag() {
        if (this.tagProcessor.getDetections().size() > 0) {
            AprilTagDetection aprilTagDetection = this.tagProcessor.getDetections().get(0);
            return aprilTagDetection;
        }
        return null;
    }

    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = this.aprilTag.getDetections();
        this.linearOpMode.telemetry.addData("# AprilTags Detected", Integer.valueOf(currentDetections.size()));
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                this.linearOpMode.telemetry.addLine(String.format("\n==== (ID %d) %s", Integer.valueOf(detection.id), detection.metadata.name));
                this.linearOpMode.telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", Double.valueOf(detection.ftcPose.x), Double.valueOf(detection.ftcPose.y), Double.valueOf(detection.ftcPose.z)));
                this.linearOpMode.telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", Double.valueOf(detection.ftcPose.pitch), Double.valueOf(detection.ftcPose.roll), Double.valueOf(detection.ftcPose.yaw)));
                this.linearOpMode.telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", Double.valueOf(detection.ftcPose.range), Double.valueOf(detection.ftcPose.bearing), Double.valueOf(detection.ftcPose.elevation)));
                this.linearOpMode.telemetry.update();
            } else {
                this.linearOpMode.telemetry.addLine(String.format("\n==== (ID %d) Unknown", Integer.valueOf(detection.id)));
                this.linearOpMode.telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", Double.valueOf(detection.center.x), Double.valueOf(detection.center.y)));
                this.linearOpMode.telemetry.addLine(String.format("Yaw", Double.valueOf(detection.ftcPose.yaw)));
            }
        }
    }

    public void OuttakeSystemFar(boolean run) {
        if (run) {
            this.outtakeMotor.setVelocity(1255.0d);
        } else {
            this.outtakeMotor.setVelocity(LynxServoController.apiPositionFirst);
        }
    }

    public void OuttakeSystemNear(boolean run) {
        if (run) {
            this.outtakeMotor.setVelocity(1130.0d);
        } else {
            this.outtakeMotor.setVelocity(LynxServoController.apiPositionFirst);
        }
    }

    public void shootingSystem(float shootArtifact, float rejectArtifact) {
        if (shootArtifact > 0.0f) {
            this.outtakeMotor.setVelocity(this.launchVelocity * ((double) shootArtifact));
        } else if (rejectArtifact > 0.0f) {
            this.outtakeMotor.setVelocity((-this.launchVelocity) * ((double) rejectArtifact));
        } else {
            this.outtakeMotor.setVelocity(LynxServoController.apiPositionFirst);
        }
    }

    public void shootAutoArtifactDouble() {
        while (opModeIsActive()) {
            OuttakeSystemFar(true);
            SystemClock.sleep(1000L);
            AutoflapSystem(true);
            SystemClock.sleep(370L);
            AutoflapSystem(false);
            SystemClock.sleep(370L);
            intakeSystemAuto(true, false);
            SystemClock.sleep(1000L);
            AutoflapSystem(true);
            SystemClock.sleep(370L);
            AutoflapSystem(false);
            SystemClock.sleep(370L);
        }
    }

    public void setShootState(boolean state) {
        this.loopState = state;
    }

    public void shootAutoArtifactFar(double targetVelocity, double margin) {
        while (Math.abs(this.outtakeMotor.getVelocity() - targetVelocity) > margin) {
            try {
                this.linearOpMode.telemetry.addLine("Velocity in while 1" + this.outtakeMotor.getVelocity() + "...");
                this.linearOpMode.telemetry.update();
                if (!opModeIsActive()) {
                    OuttakeSystemFar(false);
                    return;
                }
            } catch (Exception e) {
                throw new RuntimeException(e);
            }
        }
        AutoflapSystem(true);
        SystemClock.sleep(800L);
        AutoflapSystem(false);
        SystemClock.sleep(800L);
        intakeSystemAuto(true, false);
        SystemClock.sleep(900L);
        while (Math.abs(this.outtakeMotor.getVelocity() - targetVelocity) > margin) {
            this.linearOpMode.telemetry.addLine("Velocity in while 2" + this.outtakeMotor.getVelocity() + "...");
            this.linearOpMode.telemetry.update();
            if (!opModeIsActive()) {
                OuttakeSystemFar(false);
                return;
            }
        }
        AutoflapSystem(true);
        SystemClock.sleep(800L);
        AutoflapSystem(false);
        SystemClock.sleep(800L);
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (Math.abs(this.outtakeMotor.getVelocity() - targetVelocity) > 4.0d + margin && timer.seconds() < 2.0d) {
            this.linearOpMode.telemetry.addLine("Velocity in while 3" + this.outtakeMotor.getVelocity() + "...");
            this.linearOpMode.telemetry.update();
            if (!opModeIsActive()) {
                OuttakeSystemFar(false);
                return;
            }
        }
        AutoflapSystem(true);
        SystemClock.sleep(800L);
        AutoflapSystem(false);
        SystemClock.sleep(800L);
        OuttakeSystemFar(false);
        this.loopState = false;
    }

    public void shootAutoArtifactTriple() {
        boolean z;
        int shot;
        OuttakeSystemFar(true);
        boolean z2 = true;
        boolean z3 = false;
        if (!waitForOuttakeVelocity(1255.0d, 5.0d, 2000L)) {
            OuttakeSystemFar(false);
            return;
        }
        int shot2 = 0;
        while (shot2 < 3 && opModeIsActive()) {
            AutoflapSystem(z2);
            SystemClock.sleep(800L);
            AutoflapSystem(z3);
            SystemClock.sleep(800L);
            if (shot2 >= 2 || !opModeIsActive()) {
                z = z3;
                shot = shot2;
            } else {
                intakeSystemAuto(z2, z3);
                SystemClock.sleep(1L);
                intakeStop();
                SystemClock.sleep(1L);
                z = z3;
                shot = shot2;
                if (!waitForOuttakeVelocity(1255.0d, 5.0d, 2000L)) {
                    break;
                }
            }
            shot2 = shot + 1;
            z3 = z;
            z2 = true;
        }
        z = z3;
        OuttakeSystemFar(z);
        intakeStop();
    }

    public void shootAutoArtifactNear() {
        try {
            OuttakeSystemNear(true);
            while (Math.abs(this.outtakeMotor.getVelocity() - 1130.0d) > 8.0d) {
                if (!opModeIsActive()) {
                    OuttakeSystemFar(false);
                    return;
                }
            }
            AutoflapSystem(true);
            SystemClock.sleep(800L);
            AutoflapSystem(false);
            SystemClock.sleep(800L);
            intakeSystemAuto(true, false);
            SystemClock.sleep(900L);
            while (Math.abs(this.outtakeMotor.getVelocity() - 1255.0d) > 8.0d) {
                if (!opModeIsActive()) {
                    OuttakeSystemFar(false);
                    return;
                }
            }
            AutoflapSystem(true);
            SystemClock.sleep(800L);
            AutoflapSystem(false);
            SystemClock.sleep(800L);
            while (Math.abs(this.outtakeMotor.getVelocity() - 1255.0d) > 8.0d) {
                if (!opModeIsActive()) {
                    OuttakeSystemFar(false);
                    return;
                }
            }
            AutoflapSystem(true);
            SystemClock.sleep(800L);
            AutoflapSystem(false);
            SystemClock.sleep(800L);
            OuttakeSystemFar(false);
            this.loopState = false;
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    public void shootAutoArtifactSingle() {
        OuttakeSystemFar(true);
        SystemClock.sleep(3000L);
        AutoflapSystem(true);
        SystemClock.sleep(870L);
        this.flapServo.setPosition(0.475d);
    }

    public void intakeSystemAuto(boolean intakeArtifact, boolean rejectArtifact) {
        if (intakeArtifact) {
            this.intakeMotor.setPower(1.0d);
            this.containerMotor.setPower(0.8d);
        } else if (rejectArtifact) {
            this.intakeMotor.setPower(-1.0d);
            this.containerMotor.setPower(-1.0d);
        } else {
            this.intakeMotor.setPower(LynxServoController.apiPositionFirst);
            this.containerMotor.setPower(LynxServoController.apiPositionFirst);
        }
    }

    public void intakeRun() {
        this.intakeMotor.setPower(1.0d);
        this.containerMotor.setPower(0.8d);
    }

    public void intakeStop() {
        this.intakeMotor.setPower(LynxServoController.apiPositionFirst);
        this.containerMotor.setPower(LynxServoController.apiPositionFirst);
    }

    public void displayTelemetryAuto() {
        this.linearOpMode.telemetry.addData("x: ", Double.valueOf(this.odo.getPosX(DistanceUnit.MM)));
        this.linearOpMode.telemetry.addData("y: ", Double.valueOf(this.odo.getPosY(DistanceUnit.MM)));
    }

    public final boolean opModeIsActive() {
        boolean isActive = !this.linearOpMode.isStopRequested() && this.linearOpMode.isStarted();
        if (isActive) {
            this.linearOpMode.idle();
        }
        return isActive;
    }

    public void driveToPos(double targetX, double targetY) {
        this.odo.update();
        boolean telemAdded = false;
        while (opModeIsActive() && (Math.abs(targetX + this.odo.getPosX(DistanceUnit.MM)) > 30.0d || Math.abs(targetY - this.odo.getPosY(DistanceUnit.MM)) > 30.0d)) {
            this.odo.update();
            double x = (targetX + this.odo.getPosX(DistanceUnit.MM)) * 0.001d;
            double y = (targetY - this.odo.getPosY(DistanceUnit.MM)) * 0.001d;
            double botHeading = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotY = (Math.cos(-botHeading) * y) - (Math.sin(-botHeading) * x);
            double rotX = (Math.sin(-botHeading) * y) + (Math.cos(-botHeading) * x);
            if (!telemAdded) {
                this.linearOpMode.telemetry.addData("x: ", Double.valueOf(x));
                this.linearOpMode.telemetry.addData("y: ", Double.valueOf(y));
                this.linearOpMode.telemetry.addData("rotX: ", Double.valueOf(rotX));
                this.linearOpMode.telemetry.addData("rotY: ", Double.valueOf(rotY));
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
        this.odo.resetPosAndIMU();
    }

    public void AutoflapSystem(boolean flapUp) {
        if (flapUp) {
            this.flapServo.setPosition(0.6844d);
        } else {
            this.flapServo.setPosition(0.475d);
        }
    }

    public void autoShootArtifactsFar() {
        AutoflapSystem(true);
    }

    public void AlignToTag(AprilTagDetection tag) {
        double error = tag.ftcPose.yaw;
        while (true) {
            if (!opModeIsActive() || Math.abs(error) <= 1.0d) {
                break;
            }
            this.odo.update();
            AprilTagDetection currentTag = getLatestTag();
            if (currentTag == null) {
                this.linearOpMode.telemetry.addLine("Tag lost — stopping alignment.");
                break;
            }
            error = currentTag.ftcPose.yaw;
            double drivePower = Math.signum(error);
            if (drivePower > LynxServoController.apiPositionFirst) {
                drivePower = Math.max(drivePower, 0.35d);
            } else if (drivePower < LynxServoController.apiPositionFirst) {
                drivePower = Math.min(drivePower, -0.35d);
            }
            this.frontLeftMotor.setPower(-drivePower);
            this.backLeftMotor.setPower(-drivePower);
            this.frontRightMotor.setPower(drivePower);
            this.backRightMotor.setPower(drivePower);
            this.linearOpMode.telemetry.addData("Y:", Double.valueOf(this.odo.getPosY(DistanceUnit.MM)));
            this.linearOpMode.telemetry.addData("X:", Double.valueOf(-this.odo.getPosX(DistanceUnit.MM)));
            this.linearOpMode.telemetry.addData("Tag Yaw", Double.valueOf(error));
            this.linearOpMode.telemetry.addData("Drive Power", Double.valueOf(drivePower));
            this.linearOpMode.telemetry.update();
        }
        this.frontLeftMotor.setPower(LynxServoController.apiPositionFirst);
        this.backLeftMotor.setPower(LynxServoController.apiPositionFirst);
        this.frontRightMotor.setPower(LynxServoController.apiPositionFirst);
        this.backRightMotor.setPower(LynxServoController.apiPositionFirst);
    }

    private boolean waitForOuttakeVelocity(double targetVelocity, double tolerance, long timeoutMs) {
        long startTime = System.currentTimeMillis();
        while (Math.abs(targetVelocity - this.outtakeMotor.getVelocity()) > tolerance && System.currentTimeMillis() - startTime > timeoutMs) {
            if (!opModeIsActive()) {
                OuttakeSystemFar(false);
                return true;
            }
        }
        return true;
    }

    public void PinpointYBlue(double target, double speed) {
        this.odo.setPosY(LynxServoController.apiPositionFirst, DistanceUnit.MM);
        double margin = target + this.odo.getPosY(DistanceUnit.MM);
        while (opModeIsActive() && Math.abs(margin) > 100.0d) {
            this.odo.update();
            Pose2D pos = this.odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", Double.valueOf(pos.getX(DistanceUnit.MM)), Double.valueOf(pos.getY(DistanceUnit.MM)), Double.valueOf(pos.getHeading(AngleUnit.DEGREES)));
            this.linearOpMode.telemetry.addData("Position", data);
            this.linearOpMode.telemetry.addData("Status", this.odo.getDeviceStatus());
            double direction = Math.signum(margin);
            double power = 0.5d * direction;
            double current = this.odo.getPosY(DistanceUnit.MM);
            margin = target + current;
            this.linearOpMode.telemetry.addData("Margin", Double.valueOf(margin));
            this.linearOpMode.telemetry.update();
            this.frontLeftMotor.setPower(power);
            this.backLeftMotor.setPower(power);
            this.frontRightMotor.setPower(power);
            this.backRightMotor.setPower(power);
        }
        this.frontLeftMotor.setPower(LynxServoController.apiPositionFirst);
        this.backLeftMotor.setPower(LynxServoController.apiPositionFirst);
        this.frontRightMotor.setPower(LynxServoController.apiPositionFirst);
        this.backRightMotor.setPower(LynxServoController.apiPositionFirst);
    }

    public void gyroTurnToAngle(double turnAngle) {
        this.imu.resetYaw();
        double error = turnAngle;
        while (opModeIsActive() && (error > 1.0d || error < -1.0d)) {
            this.odo.update();
            this.linearOpMode.telemetry.addData("X: ", Double.valueOf(-this.odo.getPosX(DistanceUnit.MM)));
            this.linearOpMode.telemetry.addData("Y: ", Double.valueOf(this.odo.getPosY(DistanceUnit.MM)));
            this.linearOpMode.telemetry.addData("Heading IMU: ", Double.valueOf(this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
            this.linearOpMode.telemetry.update();
            double driveMotorsPower = error / 180.0d;
            if ((driveMotorsPower >= 0.2d || driveMotorsPower <= LynxServoController.apiPositionFirst) && driveMotorsPower > -0.2d && driveMotorsPower < LynxServoController.apiPositionFirst) {
            }
            double driveMotorsPower2 = error / 50.0d;
            if (driveMotorsPower2 < 0.35d && driveMotorsPower2 > LynxServoController.apiPositionFirst) {
                driveMotorsPower2 = 0.35d;
            } else if (driveMotorsPower2 > -0.35d && driveMotorsPower2 < LynxServoController.apiPositionFirst) {
                driveMotorsPower2 = -0.35d;
            }
            this.frontLeftMotor.setPower((-driveMotorsPower2) / 2.0d);
            this.backLeftMotor.setPower((-driveMotorsPower2) / 2.0d);
            this.frontRightMotor.setPower(driveMotorsPower2 / 2.0d);
            this.backRightMotor.setPower(driveMotorsPower2 / 2.0d);
            double currentHeadingAngle = this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            error = turnAngle - currentHeadingAngle;
        }
        this.frontLeftMotor.setPower(LynxServoController.apiPositionFirst);
        this.backLeftMotor.setPower(LynxServoController.apiPositionFirst);
        this.frontRightMotor.setPower(LynxServoController.apiPositionFirst);
        this.backRightMotor.setPower(LynxServoController.apiPositionFirst);
    }

    public void stopAllMotors() {
        this.frontLeftMotor.setPower(LynxServoController.apiPositionFirst);
        this.backLeftMotor.setPower(LynxServoController.apiPositionFirst);
        this.frontRightMotor.setPower(LynxServoController.apiPositionFirst);
        this.backRightMotor.setPower(LynxServoController.apiPositionFirst);
        this.outtakeMotor.setPower(LynxServoController.apiPositionFirst);
        this.intakeMotor.setPower(LynxServoController.apiPositionFirst);
        this.containerMotor.setPower(LynxServoController.apiPositionFirst);
    }
}
