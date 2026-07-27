package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Locale;

/* JADX INFO: loaded from: classes7.dex */
public class Odometry {
    LinearOpMode linearOpMode;
    GoBildaPinpointDriver odo;
    double oldTime = LynxServoController.apiPositionFirst;
    double frequency = LynxServoController.apiPositionFirst;

    public Odometry(LinearOpMode linearOpMode) {
        this.odo = (GoBildaPinpointDriver) linearOpMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        this.linearOpMode = linearOpMode;
    }

    public void configureOdo() {
        this.odo.setOffsets(-84.0d, -168.0d, DistanceUnit.MM);
        this.odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        this.odo.setEncoderResolution(13.26291192d, DistanceUnit.MM);
        this.odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        this.odo.recalibrateIMU();
        this.odo.resetPosAndIMU();
        this.linearOpMode.telemetry.addData("Status", "Initialized");
        this.linearOpMode.telemetry.addData("X offset", Float.valueOf(this.odo.getXOffset(DistanceUnit.MM)));
        this.linearOpMode.telemetry.addData("Y offset", Float.valueOf(this.odo.getYOffset(DistanceUnit.MM)));
        this.linearOpMode.telemetry.addData("Device Version Number:", Integer.valueOf(this.odo.getDeviceVersion()));
        this.linearOpMode.telemetry.addData("Heading Scalar", Float.valueOf(this.odo.getYawScalar()));
        this.linearOpMode.telemetry.update();
        this.linearOpMode.resetRuntime();
    }

    public void updateOdo() {
        this.odo.update();
        if (this.linearOpMode.gamepad1.a) {
            this.odo.resetPosAndIMU();
        }
        if (this.linearOpMode.gamepad1.b) {
            this.odo.recalibrateIMU();
        }
        double newTime = this.linearOpMode.getRuntime();
        double loopTime = newTime - this.oldTime;
        double frequency = 1.0d / loopTime;
        this.oldTime = newTime;
        this.frequency = frequency;
    }

    public void displayTelemetry() {
        Pose2D pos = this.odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", Double.valueOf(pos.getX(DistanceUnit.MM)), Double.valueOf(pos.getY(DistanceUnit.MM)), Double.valueOf(pos.getHeading(AngleUnit.DEGREES)));
        this.linearOpMode.telemetry.addData("Position", data);
        String velocity = String.format(Locale.US, "{XVel: %.3f, YVel: %.3f, HVel: %.3f}", Double.valueOf(this.odo.getVelX(DistanceUnit.MM)), Double.valueOf(this.odo.getVelY(DistanceUnit.MM)), Double.valueOf(this.odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES)));
        this.linearOpMode.telemetry.addData("Velocity", velocity);
        this.linearOpMode.telemetry.addData("Status", this.odo.getDeviceStatus());
        this.linearOpMode.telemetry.addData("Pinpoint Frequency", Double.valueOf(this.odo.getFrequency()));
        this.linearOpMode.telemetry.addData("REV Hub Frequency: ", Double.valueOf(this.frequency));
        this.linearOpMode.telemetry.update();
    }
}
