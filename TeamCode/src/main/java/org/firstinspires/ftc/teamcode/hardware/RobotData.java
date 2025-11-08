package org.firstinspires.ftc.teamcode.hardware;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;

public class RobotData {

    public long loopTime = System.currentTimeMillis();
    public IntakeSubsystem.IntakeState intakeState = IntakeSubsystem.IntakeState.STOP;

    public ShooterSubsystem.StopState stopState = ShooterSubsystem.StopState.STOP;

    public ShooterSubsystem.ShootState shootState = ShooterSubsystem.ShootState.STOP;

    public double shootVelocity = 0;
    public double shootTargetVelocity = 0;

    public Pose currentPose = new Pose(0,0, Math.toRadians(0));

    public CameraSubsystem.Obelisk obelisk = CameraSubsystem.Obelisk.PPP;

    public String turretState = "MANUAL";
    public double turretAngleDeg = 0.0;

    public CameraSubsystem.ShootDistance shootDistance = CameraSubsystem.ShootDistance.OUTOFRANGE;


    private static final double TICKS_PER_REV = 1440.0;

    public void write(Telemetry telemetry) {

        telemetry.addData("LOOP TIME", System.currentTimeMillis() - loopTime);
        loopTime = System.currentTimeMillis();

        telemetry.addLine("");

        telemetry.addData("POSE", this.currentPose);
        telemetry.addData("BUSY", Robot.getInstance().follower.isBusy());
        telemetry.addLine(Constants.robotCentric ? "ROBOT CENTRIC" : "FIELD CENTRIC");

        telemetry.addLine("");

        telemetry.addData("ALLIANCE", Globals.ALLIANCE);


        double curRpm    = (shootVelocity / TICKS_PER_REV) * 60.0;
        double targetRpm = (shootTargetVelocity / TICKS_PER_REV) * 60.0;

        // Define a tolerance (either a constant or computed as a percent)
        double tolTicksPerSec = Math.max(75, 0.05 * Math.abs(shootTargetVelocity)); // 5% or 75 tps minimum
        boolean atSpeed = Math.abs(shootVelocity - shootTargetVelocity) <= tolTicksPerSec;

        telemetry.addData("SHOOT STATE", this.shootState);
        telemetry.addData("STOP STATE",  this.stopState);
        telemetry.addLine("");

        telemetry.addData("SHOOT v (tps)", "%.0f / %.0f", shootVelocity, shootTargetVelocity);
        telemetry.addData("SHOOT v (rpm)", "%.0f / %.0f", curRpm, targetRpm);
        telemetry.addData("AT SPEED", atSpeed ? "YES" : "NO");
        telemetry.addData("TOL (tps)", "%.0f", tolTicksPerSec);
        telemetry.addLine("");



        telemetry.addLine("");

        telemetry.addData("OBELISK",  this.obelisk);

        telemetry.addLine("");

        telemetry.addData("DIST", this.shootDistance);


        telemetry.update();


    }


}
