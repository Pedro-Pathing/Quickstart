package org.firstinspires.ftc.teamcode.pedroPathing;

import android.graphics.Color;
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

/* JADX INFO: loaded from: classes7.dex */
public class ColorDetection {
    private final ColorSensor backColorSensor;
    private final ColorSensor frontLeftColorSensor;
    private final ColorSensor frontRightColorSensor;
    private final LinearOpMode linearOpMode;
    private final ColorSensor middleColorSensor;
    private final Servo outtakeIndicator;
    private final DcMotorEx outtakeMotor;
    private final Servo rgbIndicator;
    private RobotControls robotControls;
    private boolean celebrateOn = false;
    private boolean previousCelebrate = false;
    private int g_artifactCount = 0;
//    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public ColorDetection(LinearOpMode linearOpMode) {
        this.frontLeftColorSensor = (ColorSensor) linearOpMode.hardwareMap.get(ColorSensor.class, "frontLeftColorSensor");
        this.frontRightColorSensor = (ColorSensor) linearOpMode.hardwareMap.get(ColorSensor.class, "frontRightColorSensor");
        this.middleColorSensor = (ColorSensor) linearOpMode.hardwareMap.get(ColorSensor.class, "middleColorSensor");
        this.backColorSensor = (ColorSensor) linearOpMode.hardwareMap.get(ColorSensor.class, "backColorSensor");
        this.rgbIndicator = (Servo) linearOpMode.hardwareMap.get(Servo.class, "artifactIndicator");
        this.outtakeIndicator = (Servo) linearOpMode.hardwareMap.get(Servo.class, "outtakeIndicator");
        this.outtakeMotor = (DcMotorEx) linearOpMode.hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        this.linearOpMode = linearOpMode;
        this.robotControls = new RobotControls(linearOpMode);
    }

    public void celebrateToggle(boolean celebrate) throws InterruptedException {
        boolean wasCelebrateOn = this.celebrateOn;
        if (celebrate && !this.previousCelebrate) {
            this.celebrateOn = !this.celebrateOn;
        }
        this.previousCelebrate = celebrate;
        if (this.celebrateOn) {
            long currentTime = System.currentTimeMillis();
            double cyclePosition = (currentTime % 3500.0d) / 3500.0d;
            double triangleWave = cyclePosition < 0.5d ? 2.0d * cyclePosition : 2.0d - (cyclePosition * 2.0d);
            double rainbowColor = (0.43999999999999995d * triangleWave) + 0.28d;
            this.rgbIndicator.setPosition(rainbowColor);
            this.outtakeIndicator.setPosition(rainbowColor);
            return;
        }
        if (wasCelebrateOn) {
            this.rgbIndicator.setPosition(LynxServoController.apiPositionFirst);
            this.outtakeIndicator.setPosition(LynxServoController.apiPositionFirst);
        }
    }

    private float[] getHSV(ColorSensor colorSensor) {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();
        float[] hsv = new float[3];
        Color.RGBToHSV(red, green, blue, hsv);
        return hsv;
    }

    private int detectArtifact(ColorSensor colorSensor) {
        float[] hsv = getHSV(colorSensor);
        float hue = hsv[0];
        float value = hsv[2];
        if (value < 0.5d) {
            return 0;
        }
        if (hue <= 140.0f || hue >= 180.0f) {
            return (hue <= 181.0f || hue >= 260.0f) ? 0 : 1;
        }
        return 1;
    }

    public boolean isArtifactAtBack() {
        return detectArtifact(this.backColorSensor) == 1;
    }

    public int getArtifactCount() {
        int i = 1;
        if (detectArtifact(this.frontLeftColorSensor) != 1 && detectArtifact(this.frontRightColorSensor) != 1) {
            i = 0;
        }
        int artifactCount = i + detectArtifact(this.middleColorSensor) + detectArtifact(this.backColorSensor);
        this.g_artifactCount = artifactCount;
        return artifactCount;
    }

    public void setRGBIndicator() {
        if (this.celebrateOn) {
            return;
        }
        getArtifactCount();
    }

    public void velocityIndicatorStart(final double outtakeTargetVelocity) {
        if (this.celebrateOn) {
            return;
        }
        this.rgbIndicator.setPosition(0.28d);
        new Thread(new Runnable() { // from class: org.firstinspires.ftc.teamcode.ColorDetection$$ExternalSyntheticLambda0
            @Override // java.lang.Runnable
            public final void run() {
//                this.f$0.m188lambda$velocityIndicatorStart$0$orgfirstinspiresftcteamcodeColorDetection(outtakeTargetVelocity);
            }
        }).start();
    }

    /* JADX INFO: renamed from: lambda$velocityIndicatorStart$0$org-firstinspires-ftc-teamcode-ColorDetection, reason: not valid java name */
    /* synthetic */ void m188lambda$velocityIndicatorStart$0$orgfirstinspiresftcteamcodeColorDetection(double outtakeTargetVelocity) {
        while (this.linearOpMode.opModeIsActive() && this.robotControls.shootArtifact >= 0.1d) {
            if (this.outtakeMotor.getVelocity() >= outtakeTargetVelocity * 0.5d) {
                this.rgbIndicator.setPosition(0.333d);
            } else if (this.outtakeMotor.getVelocity() >= 0.985d * outtakeTargetVelocity) {
                this.rgbIndicator.setPosition(0.5d);
            }
            try {
                Thread.sleep(50L);
            } catch (InterruptedException e) {
            }
        }
        this.rgbIndicator.setPosition(0.28d);
    }

    public void setOuttakeIndicatorWithVelocity(double targetVelocity, double actualVelocity) {
        if (this.celebrateOn) {
            return;
        }
        if (targetVelocity == LynxServoController.apiPositionFirst) {
            this.outtakeIndicator.setPosition(LynxServoController.apiPositionFirst);
            return;
        }
        boolean artifactInBack = detectArtifact(this.backColorSensor) == 1;
        boolean reachedTargetVelocity = Math.abs(actualVelocity - targetVelocity) <= 15.0d;
        if (!artifactInBack || !reachedTargetVelocity) {
            this.outtakeIndicator.setPosition(LynxServoController.apiPositionFirst);
        } else {
            this.outtakeIndicator.setPosition(0.611d);
        }
    }

    private double getRGBIndicatorPosition() {
        return this.rgbIndicator.getPosition();
    }

    private boolean isCelebrateOn() {
        return this.celebrateOn;
    }

    public void displayTelemetry() {
        int FLred = this.frontLeftColorSensor.red();
        int FLgreen = this.frontLeftColorSensor.green();
        int FLblue = this.frontLeftColorSensor.blue();
        int FRred = this.frontRightColorSensor.red();
        int FRgreen = this.frontRightColorSensor.green();
        int FRblue = this.frontRightColorSensor.blue();
        int Mred = this.middleColorSensor.red();
        int Mgreen = this.middleColorSensor.green();
        int Mblue = this.middleColorSensor.blue();
        int Bred = this.backColorSensor.red();
        int Bgreen = this.backColorSensor.green();
        int Bblue = this.backColorSensor.blue();
        float[] FLhsv = getHSV(this.frontLeftColorSensor);
        float[] FRhsv = getHSV(this.frontRightColorSensor);
        float[] Mhsv = getHSV(this.middleColorSensor);
        float[] Bhsv = getHSV(this.backColorSensor);
        this.linearOpMode.telemetry.addData("Detected Color FL", Integer.valueOf(detectArtifact(this.frontLeftColorSensor)));
        this.linearOpMode.telemetry.addData("Detected Color FR", Integer.valueOf(detectArtifact(this.frontRightColorSensor)));
        this.linearOpMode.telemetry.addData("Detected Color M", Integer.valueOf(detectArtifact(this.middleColorSensor)));
        this.linearOpMode.telemetry.addData("Detected Color B", Integer.valueOf(detectArtifact(this.backColorSensor)));
        this.linearOpMode.telemetry.addData("FLred", Integer.valueOf(FLred));
        this.linearOpMode.telemetry.addData("FLgreen", Integer.valueOf(FLgreen));
        this.linearOpMode.telemetry.addData("FLblue", Integer.valueOf(FLblue));
        this.linearOpMode.telemetry.addData("FRred", Integer.valueOf(FRred));
        this.linearOpMode.telemetry.addData("FRgreen", Integer.valueOf(FRgreen));
        this.linearOpMode.telemetry.addData("FRblue", Integer.valueOf(FRblue));
        this.linearOpMode.telemetry.addData("FLhue", Float.valueOf(FLhsv[0]));
        this.linearOpMode.telemetry.addData("FLsaturation", Float.valueOf(FLhsv[1]));
        this.linearOpMode.telemetry.addData("FLvalue", Float.valueOf(FLhsv[2]));
        this.linearOpMode.telemetry.addData("FRhue", Float.valueOf(FRhsv[0]));
        this.linearOpMode.telemetry.addData("FRsaturation", Float.valueOf(FRhsv[1]));
        this.linearOpMode.telemetry.addData("FRvalue", Float.valueOf(FRhsv[2]));
        this.linearOpMode.telemetry.addData("Mhue", Float.valueOf(Mhsv[0]));
        this.linearOpMode.telemetry.addData("Msaturation", Float.valueOf(Mhsv[1]));
        this.linearOpMode.telemetry.addData("Mvalue", Float.valueOf(Mhsv[2]));
        this.linearOpMode.telemetry.addData("Bhue", Float.valueOf(Bhsv[0]));
        this.linearOpMode.telemetry.addData("Bsaturation", Float.valueOf(Bhsv[1]));
        this.linearOpMode.telemetry.addData("Bvalue", Float.valueOf(Bhsv[2]));
        this.linearOpMode.telemetry.addData("Mred", Integer.valueOf(Mred));
        this.linearOpMode.telemetry.addData("Mgreen", Integer.valueOf(Mgreen));
        this.linearOpMode.telemetry.addData("Mblue", Integer.valueOf(Mblue));
        this.linearOpMode.telemetry.addData("Bred", Integer.valueOf(Bred));
        this.linearOpMode.telemetry.addData("Bgreen", Integer.valueOf(Bgreen));
        this.linearOpMode.telemetry.addData("Bblue", Integer.valueOf(Bblue));
        this.linearOpMode.telemetry.addData("Amount of Artifacts in Robot", Integer.valueOf(this.g_artifactCount));
        this.linearOpMode.telemetry.addData("RGB Position", Double.valueOf(getRGBIndicatorPosition()));
        this.linearOpMode.telemetry.addData("Celebrate Mode", Boolean.valueOf(isCelebrateOn()));
//        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("detectedColorFL", Integer.valueOf(detectArtifact(this.frontLeftColorSensor)));
//        packet.put("detectedColorFR", Integer.valueOf(detectArtifact(this.frontRightColorSensor)));
//        packet.put("detectedColorM", Integer.valueOf(detectArtifact(this.middleColorSensor)));
//        packet.put("detectedColorB", Integer.valueOf(detectArtifact(this.backColorSensor)));
//        packet.put("g_artifactCount", Integer.valueOf(this.g_artifactCount));
//        packet.put("rgbPosition", Double.valueOf(getRGBIndicatorPosition()));
//        packet.put("celebrateMode", Boolean.valueOf(isCelebrateOn()));
//        this.dashboard.sendTelemetryPacket(packet);
    }
}
