package org.firstinspires.ftc.teamcode.hardware;

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

    public Pose currentPose = new Pose(0,0, Math.toRadians(0));

    public CameraSubsystem.Obelisk obelisk = CameraSubsystem.Obelisk.PPP;

    public CameraSubsystem.ShootDistance shootDistance = CameraSubsystem.ShootDistance.OUTOFRANGE;


    public void write(Telemetry telemetry) {

        telemetry.addData("LOOP TIME", System.currentTimeMillis() - loopTime);
        loopTime = System.currentTimeMillis();

        telemetry.addLine();

        telemetry.addData("POSE", this.currentPose);
        telemetry.addData("BUSY", Robot.getInstance().follower.isBusy());
        telemetry.addLine(Constants.robotCentric ? "ROBOT CENTRIC" : "FIELD CENTRIC");

        telemetry.addLine();

        telemetry.addData("ALLIANCE", Globals.ALLIANCE);

        telemetry.addLine();

        telemetry.addData("OBELISK",  this.obelisk);

        telemetry.addLine();

        telemetry.addData("DIST", this.shootDistance);


        telemetry.update();


    }


}
