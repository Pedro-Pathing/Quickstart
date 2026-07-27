package org.firstinspires.ftc.teamcode.pedroPathing;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/* JADX INFO: loaded from: classes7.dex */
public class RobotControls {
    boolean DEBUG;
    boolean alignRobot;
    boolean celebrate;
//    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    boolean decreaseFactor;
    boolean flapArtifact;
    boolean increaseFactor;
    boolean intakeArtifact;
    private final LinearOpMode linearOpMode;
    float motorBrake;
    boolean rejectIntakeArtifact;
    float shootArtifact;
    boolean switchLaunchPower;

    public RobotControls(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
    }

    public void updateControls() {
        this.intakeArtifact = this.linearOpMode.gamepad2.a;
        this.flapArtifact = this.linearOpMode.gamepad2.b;
        this.switchLaunchPower = this.linearOpMode.gamepad2.x;
        this.rejectIntakeArtifact = this.linearOpMode.gamepad2.y;
        this.motorBrake = this.linearOpMode.gamepad2.left_trigger;
        this.shootArtifact = this.linearOpMode.gamepad2.right_trigger;
        this.celebrate = this.linearOpMode.gamepad2.back;
        this.increaseFactor = this.linearOpMode.gamepad2.dpad_up;
        this.decreaseFactor = this.linearOpMode.gamepad2.dpad_down;
        this.alignRobot = this.linearOpMode.gamepad1.b;
        this.DEBUG = this.linearOpMode.gamepad1.y;
    }

    public void displayTelemetry() {
        this.linearOpMode.telemetry.addData("Intake (A)", Boolean.valueOf(this.intakeArtifact));
        this.linearOpMode.telemetry.addData("Reject Intake (Y)", Boolean.valueOf(this.rejectIntakeArtifact));
        this.linearOpMode.telemetry.addData("Flap (B)", Boolean.valueOf(this.flapArtifact));
        this.linearOpMode.telemetry.addData("Shoot Trigger", Float.valueOf(this.shootArtifact));
        this.linearOpMode.telemetry.addData("Motor Brake Trigger", Float.valueOf(this.motorBrake));
        this.linearOpMode.telemetry.addData("Increase Velocity (DPad Up)", Boolean.valueOf(this.increaseFactor));
        this.linearOpMode.telemetry.addData("Decrease Velocity (DPad Down)", Boolean.valueOf(this.decreaseFactor));
        this.linearOpMode.telemetry.addData("Switch Launch Power (X)", Boolean.valueOf(this.switchLaunchPower));
        this.linearOpMode.telemetry.addData("Celebrate (Back)", Boolean.valueOf(this.celebrate));
        this.linearOpMode.telemetry.addData("Align Robot (gamepad1 B)", Boolean.valueOf(this.alignRobot));
//        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("intakeArtifact", Boolean.valueOf(this.intakeArtifact));
//        packet.put("rejectIntakeArtifact", Boolean.valueOf(this.rejectIntakeArtifact));
//        packet.put("flapArtifact", Boolean.valueOf(this.flapArtifact));
//        packet.put("shootTrigger", Float.valueOf(this.shootArtifact));
//        packet.put("motorBrake", Float.valueOf(this.motorBrake));
//        packet.put("increaseFactor", Boolean.valueOf(this.increaseFactor));
//        packet.put("decreaseFactor", Boolean.valueOf(this.decreaseFactor));
//        packet.put("switchLaunchPower", Boolean.valueOf(this.switchLaunchPower));
//        packet.put("celebrate", Boolean.valueOf(this.celebrate));
//        packet.put("alignRobot", Boolean.valueOf(this.alignRobot));
//        this.dashboard.sendTelemetryPacket(packet);
    }
}
