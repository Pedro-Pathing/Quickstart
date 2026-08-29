package org.firstinspires.ftc.teamcode.autonomous;
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



@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class SampleAutoPathing extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class



    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();



        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));



        paths = new Paths(follower); // Build paths



        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }



    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine



// Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }



    public static class Paths {
        public PathChain MainChain;



        public Paths(Follower follower) {
            MainChain = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(20.509, 120.639),
                                    new Pose(56.832, 83.276)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(142))
                    .addPath(
                            new BezierCurve(
                                    new Pose(12.062, 81.884),
                                    new Pose(33.628, 83.185),
                                    new Pose(63.194, 82.485)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .addPath(
                            new BezierCurve(
                                    new Pose(63.194, 82.485),
                                    new Pose(58.818, 52.194),
                                    new Pose(10.863, 58.585)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    new Pose(10.863, 58.585),
                                    new Pose(60.478, 84.228)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .addPath(
                            new BezierCurve(
                                    new Pose(60.478, 84.228),
                                    new Pose(77.501, 27.500),
                                    new Pose(10.902, 34.795)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    new Pose(10.902, 34.795),
                                    new Pose(59.650, 84.040)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .addPath(
                            new BezierLine(
                                    new Pose(59.650, 84.040),
                                    new Pose(59.830, 115.365)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }



    public int autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                follower.followPath(paths.MainChain);
                return 1;

            case 1:
                if (!follower.isBusy()) {
                    return 2;
                }
                break;
        }

        return pathState;
    }
}