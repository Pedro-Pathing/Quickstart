package org.firstinspires.ftc.teamcode.pedroPathing;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/* JADX INFO: loaded from: classes7.dex */
public class ArtifactHandlingSystem {
    private double autoLaunchVelocity;
    private ColorDetection colorDetection;
    private final DcMotor containerMotor;
    private final Servo flapServo;
    private final DcMotor intakeMotor;
    private double launchVelocity;
    private final LinearOpMode linearOpMode;
    private final DcMotorEx outtakeMotor;
    private boolean lastSwitchState = false;
    private boolean flapDown = true;
    private boolean autoFeeding = false;
    private String intakeAutoStatus = "idle";
    private boolean intakeStoppedForCapacity = false;
//    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public ArtifactHandlingSystem(LinearOpMode linearOpMode) {
        this.outtakeMotor = (DcMotorEx) linearOpMode.hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        this.intakeMotor = (DcMotor) linearOpMode.hardwareMap.dcMotor.get("intakeMotor");
        this.containerMotor = (DcMotor) linearOpMode.hardwareMap.dcMotor.get("containerMotor");
        this.flapServo = (Servo) linearOpMode.hardwareMap.servo.get("flapServo");
        this.linearOpMode = linearOpMode;
        this.colorDetection = new ColorDetection(linearOpMode);
    }

    public void configureMotorModes() {
//        MotorConfigurationType config = this.outtakeMotor.getMotorType().m174clone();
//        config.setAchieveableMaxRPMFraction(1.0d);
//        this.outtakeMotor.setMotorType(config);
        this.outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.containerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.outtakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.containerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.flapServo.setDirection(Servo.Direction.FORWARD);
        this.launchVelocity = 1315.0d;
        this.autoLaunchVelocity = 1250.0d;
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(60.0d, LynxServoController.apiPositionFirst, 26.5d, 13.961591304347825d);
        this.outtakeMotor.setVelocityPIDFCoefficients(60.0d, LynxServoController.apiPositionFirst, 26.5d, 13.961591304347825d);
        this.outtakeMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    public void intakeSystem(boolean intakeArtifact, boolean rejectArtifact, int artifactCount) {
        this.intakeStoppedForCapacity = false;
        if (artifactCount >= 3 && intakeArtifact) {
            this.intakeMotor.setPower(LynxServoController.apiPositionFirst);
            //this.containerMotor.setPower(LynxServoController.apiPositionFirst);
            this.intakeAutoStatus = "stopped: full (>=cap)";
            this.intakeStoppedForCapacity = true;
            return;
        }
        if (intakeArtifact) {
            this.intakeMotor.setPower(1.0d);
            //this.containerMotor.setPower(1.0d);
            this.intakeAutoStatus = "intake: forward";
        } else if (rejectArtifact) {
            this.intakeMotor.setPower(-1.0d);
            //this.containerMotor.setPower(-1.0d);
            this.intakeAutoStatus = "intake: reverse";
        } else {
            this.intakeMotor.setPower(LynxServoController.apiPositionFirst);
            //this.containerMotor.setPower(LynxServoController.apiPositionFirst);
            this.intakeAutoStatus = "intake: idle";
        }
    }

    public void flapSystem(boolean flapUp) {
        if (flapUp) {
            this.flapServo.setPosition(0.6844d);
            this.flapDown = false;
        } else {
            this.flapServo.setPosition(0.475d);
            this.flapDown = true;
        }
    }

    public void manageIntakeWithAutoFeed(boolean intakeArtifact, boolean rejectArtifact, boolean shooterActive, boolean artifactAtBack, int artifactCount) {
        if (intakeArtifact || rejectArtifact) {
            this.autoFeeding = false;
            this.intakeAutoStatus = intakeArtifact ? "manual: intake" : "manual: reject";
            intakeSystem(intakeArtifact, rejectArtifact, artifactCount);
        } else if (shooterActive && this.flapDown && !artifactAtBack) {
            this.autoFeeding = true;
            this.intakeAutoStatus = "auto: feeding (shoot)";
            intakeSystem(true, false, artifactCount);
        } else if (this.autoFeeding) {
            intakeSystem(false, false, artifactCount);
            this.autoFeeding = false;
            this.intakeAutoStatus = "auto: stopped";
        } else {
            intakeSystem(false, false, artifactCount);
            this.intakeAutoStatus = "idle";
        }
    }

    public void shootAutoArtifact() {
        Thread ArtifactShoot = new Thread(new Runnable() { // from class: org.firstinspires.ftc.teamcode.ArtifactHandlingSystem$$ExternalSyntheticLambda0
            @Override // java.lang.Runnable
            public final void run() {
//                this.f$0.m187lambda$shootAutoArtifact$0$orgfirstinspiresftcteamcodeArtifactHandlingSystem();
            }
        });
        ArtifactShoot.start();
    }

    /* JADX INFO: renamed from: lambda$shootAutoArtifact$0$org-firstinspires-ftc-teamcode-ArtifactHandlingSystem, reason: not valid java name */
    /* synthetic */ void m187lambda$shootAutoArtifact$0$orgfirstinspiresftcteamcodeArtifactHandlingSystem() {
        long endTime = System.currentTimeMillis() + 4000;
        shootingSystem(1.0f, 0.0f);
        while (System.currentTimeMillis() < endTime) {
            //this.containerMotor.setPower(1.0d);
        }
        shootingSystem(0.0f, 0.0f);
    }

    public void shootingSystem(float shootArtifact, float rejectArtifact) {
        if (shootArtifact > 0.1d) {
            this.outtakeMotor.setVelocity(this.launchVelocity);
            this.colorDetection.velocityIndicatorStart(this.launchVelocity);
        } else if (rejectArtifact > 0.1d) {
            this.outtakeMotor.setVelocity(-500.0d);
        } else {
            this.outtakeMotor.setVelocity(LynxServoController.apiPositionFirst);
        }
    }

    public void updateLaunchVelocityForRange(double rangeInches, double tagId) {
        if (Double.isNaN(rangeInches) || rangeInches <= LynxServoController.apiPositionFirst) {
            return;
        }
        if (rangeInches >= 45.0d && rangeInches <= 65.0d) {
            if (tagId == 20.0d) {
                this.launchVelocity = 1315.0d;
                this.autoLaunchVelocity = 1250.0d;
                TeleOpConstants.CAMERA_OFFSET_X_IN_RUNTIME = 1.968503937007874d;
                return;
            } else {
                this.launchVelocity = 1315.0d;
                this.autoLaunchVelocity = 1250.0d;
                TeleOpConstants.CAMERA_OFFSET_X_IN_RUNTIME = 1.968503937007874d;
                return;
            }
        }
        if (rangeInches >= 40.0d && rangeInches <= 44.99d) {
            if (tagId == 20.0d) {
                this.launchVelocity = 1255.0d;
                this.autoLaunchVelocity = 1250.0d;
                TeleOpConstants.CAMERA_OFFSET_X_IN_RUNTIME = LynxServoController.apiPositionFirst;
                return;
            } else {
                this.launchVelocity = 1255.0d;
                this.autoLaunchVelocity = 50.0d;
                TeleOpConstants.CAMERA_OFFSET_X_IN_RUNTIME = LynxServoController.apiPositionFirst;
                return;
            }
        }
        if (rangeInches < 30.0d || rangeInches > 39.99d) {
            if (tagId == 20.0d) {
                this.launchVelocity = 1135.0d;
                this.autoLaunchVelocity = 50.0d;
                TeleOpConstants.CAMERA_OFFSET_X_IN_RUNTIME = LynxServoController.apiPositionFirst;
                return;
            } else {
                this.launchVelocity = 1135.0d;
                this.autoLaunchVelocity = 50.0d;
                TeleOpConstants.CAMERA_OFFSET_X_IN_RUNTIME = LynxServoController.apiPositionFirst;
                return;
            }
        }
        if (tagId == 20.0d) {
            this.launchVelocity = 1220.0d;
            this.autoLaunchVelocity = 50.0d;
            TeleOpConstants.CAMERA_OFFSET_X_IN_RUNTIME = LynxServoController.apiPositionFirst;
        } else {
            this.launchVelocity = 1220.0d;
            this.autoLaunchVelocity = 50.0d;
            TeleOpConstants.CAMERA_OFFSET_X_IN_RUNTIME = LynxServoController.apiPositionFirst;
        }
    }

    private void waitForMotor() {
        while (this.outtakeMotor.getVelocity() > this.autoLaunchVelocity - 20.0d && this.outtakeMotor.getVelocity() < this.autoLaunchVelocity + 20.0d && this.linearOpMode.opModeIsActive()) {
            this.linearOpMode.telemetry.addData("Outtake Velocity", Double.valueOf(this.outtakeMotor.getVelocity()));
            this.linearOpMode.telemetry.addData("Outtake Target Velocity", Integer.valueOf(this.outtakeMotor.getTargetPosition()));
            this.linearOpMode.telemetry.addData("Launch Velocity", Double.valueOf(this.autoLaunchVelocity));
            boolean z = true;
            this.linearOpMode.telemetry.addData("Velocity -", Boolean.valueOf(this.outtakeMotor.getVelocity() > this.autoLaunchVelocity - 20.0d));
            this.linearOpMode.telemetry.addData("Velocity +", Boolean.valueOf(this.outtakeMotor.getVelocity() < this.autoLaunchVelocity + 20.0d));
            Telemetry telemetry = this.linearOpMode.telemetry;
            if (this.outtakeMotor.getVelocity() <= this.autoLaunchVelocity - 20.0d || this.outtakeMotor.getVelocity() >= this.autoLaunchVelocity + 20.0d) {
                z = false;
            }
            telemetry.addData("Velocity T/F", Boolean.valueOf(z));
            this.linearOpMode.telemetry.update();
            this.linearOpMode.sleep(10L);
        }
    }

    public void adjustShootingFactor(boolean increase, boolean decrease) {
        if (increase) {
            this.launchVelocity += 10.0d;
        } else if (decrease) {
            this.launchVelocity -= 10.0d;
        }
        this.launchVelocity = Math.max(LynxServoController.apiPositionFirst, Math.min(this.launchVelocity, 2300.0d));
    }

    public void switchShootingFactor(boolean switch_f) {
        if (switch_f && !this.lastSwitchState) {
            this.launchVelocity = this.launchVelocity == 1135.0d ? 1255.0d : 1135.0d;
            this.autoLaunchVelocity = this.autoLaunchVelocity == 50.0d ? 1250.0d : 50.0d;
        }
        this.lastSwitchState = switch_f;
    }

    public boolean isFlapDown() {
        return this.flapDown;
    }

    public double getLaunchVelocity() {
        return this.launchVelocity;
    }

    public double getActualVelocity() {
        return this.outtakeMotor.getVelocity();
    }

    public void checkMotorHealth() {
        if (this.outtakeMotor.isOverCurrent()) {
            this.linearOpMode.telemetry.addData("WARNING", "Shooter motor over current!");
            this.outtakeMotor.setVelocity(LynxServoController.apiPositionFirst);
        }
    }

    public void displayTelemetry() {
        this.linearOpMode.telemetry.addData("Outtake Target Velocity", "%.0f ticks/sec", Double.valueOf(this.launchVelocity));
        this.linearOpMode.telemetry.addData("Outtake AUTO Target Velocity", "%.0f ticks/sec", Double.valueOf(this.autoLaunchVelocity));
        this.linearOpMode.telemetry.addData("Outtake Actual Velocity", "%.0f ticks/sec", Double.valueOf(this.outtakeMotor.getVelocity()));
        this.linearOpMode.telemetry.addData("Velocity Error", "%.0f ticks/sec", Double.valueOf(this.launchVelocity - this.outtakeMotor.getVelocity()));
        this.linearOpMode.telemetry.addData("AUTO Velocity Error", "%.0f ticks/sec", Double.valueOf(this.autoLaunchVelocity - this.outtakeMotor.getVelocity()));
        this.linearOpMode.telemetry.addData("Outtake Current Draw", "%.2f A", Double.valueOf(this.outtakeMotor.getCurrent(CurrentUnit.AMPS)));
        this.linearOpMode.telemetry.addData("Outtake Position", Integer.valueOf(this.outtakeMotor.getCurrentPosition()));
        this.linearOpMode.telemetry.addData("Velocity %", "%.1f%%", Double.valueOf((this.outtakeMotor.getVelocity() / 2300.0d) * 100.0d));
        this.linearOpMode.telemetry.addData("Motor Enabled", Boolean.valueOf(this.outtakeMotor.isMotorEnabled()));
        this.linearOpMode.telemetry.addData("Motor PIDF", this.outtakeMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        this.linearOpMode.telemetry.addData("Intake Motor Power", Double.valueOf(this.intakeMotor.getPower()));
        this.linearOpMode.telemetry.addData("Container Motor Power", Double.valueOf(this.containerMotor.getPower()));
        this.linearOpMode.telemetry.addData("Flap Servo Position", Double.valueOf(this.flapServo.getPosition()));
        this.linearOpMode.telemetry.addData("Intake Auto Status", this.intakeAutoStatus);
        this.linearOpMode.telemetry.addData("Intake Auto-Stopped (Full)", Boolean.valueOf(this.intakeStoppedForCapacity));
//        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("targetVelocity", Double.valueOf(this.launchVelocity));
//        packet.put("autoTargetVelocity", Double.valueOf(this.autoLaunchVelocity));
//        packet.put("actualVelocity", Double.valueOf(this.outtakeMotor.getVelocity()));
//        packet.put("velocityError", Double.valueOf(this.launchVelocity - this.outtakeMotor.getVelocity()));
//        packet.put("autoVelocityError", Double.valueOf(this.autoLaunchVelocity - this.outtakeMotor.getVelocity()));
//        packet.put("intakePower", Double.valueOf(this.intakeMotor.getPower()));
//        packet.put("containerPower", Double.valueOf(this.containerMotor.getPower()));
//        packet.put("flapPosition", Double.valueOf(this.flapServo.getPosition()));
//        packet.put("intakeAutoStatus", this.intakeAutoStatus);
//        packet.put("intakeAutoStopped", Boolean.valueOf(this.intakeStoppedForCapacity));
//        this.dashboard.sendTelemetryPacket(packet);
    }
}
