
package org.firstinspires.ftc.teamcode.pedroPathing;
import android.app.slice.SliceMetrics;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.Consumer;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class RunCustomPath3 extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState, limelightAngle; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    private static DcMotor rotate, intake, shooter;
    private static Servo kicker, pusher, hood;



    private Limelight3A limelight;

    public void defineMechanisms() {
        rotate  = hardwareMap.get(DcMotor.class, "rotate");
        intake  = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        hood = hardwareMap.get(Servo.class, "hood"); // TODO add the hood servo to the config
        kicker  = hardwareMap.get(Servo.class, "kicker");
        pusher  = hardwareMap.get(Servo.class, "pusher");

        limelightAngle = 23;
    }

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(64, 80, Math.toRadians(140)));
        defineMechanisms();
        changePath(0);

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);

        if (!follower.isBusy()) {
            return;
        }
    }


    public static class Paths {
        public PathChain loadpath;
        public PathChain Path2;

        public Paths(Follower follower) {
            loadpath = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(64.000, 80.000),
                                    new Pose(35.246, 62.217),
                                    new Pose(40.000, 90.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
                    .addParametricCallback(
                            0.8,
                            () -> intake.setPower(1.0)
                    )
                    .addParametricCallback(
                            1.0,
                            () -> intake.setPower(0.0)
                    )
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(40.000, 90.000),
                                    new Pose(66.903, 114.186),
                                    new Pose(64.000, 80.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()

                    .build();
        }


    }

    public void changePath(int new_path) {
        pathState = new_path;
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.loadpath, true);
                changePath(1);
                break;
            case 1:
                follower.followPath(paths.Path2);
                break;
        }
    }
}
