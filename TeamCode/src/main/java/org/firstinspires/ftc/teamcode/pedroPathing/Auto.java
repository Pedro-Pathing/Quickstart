
package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;


@Autonomous(name = "27", group = "Autonomous")
@Configurable // Panels
public class Auto extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(108.793, 135.650, Math.toRadians(0)));

        paths = new Paths(follower); // Build paths
        pathState = 0; // Initialize state machine

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        // Start the first path when autonomous starts
        pathState = 1;
        follower.followPath(paths.Path1);
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
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(108.793, 135.650),
                                    new Pose(79.81532, 47.847),
                                    new Pose(134.240766, 58.30916552)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(0))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(134.240766, 58.30916552),

                                    new Pose(74.65937, 71.9015)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(74.65937, 71.9015),
                                    new Pose(109.44459, 42.82284),
                                    new Pose(137.912,63.6703)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(50))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(137.912, 63.231),

                                    new Pose(77.153, 75.650)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(67))

                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // State machine for autonomous path following
        switch (pathState) {
            case 0:
                // Waiting to start
                break;
            case 1:
                // Following Path1
                if (!follower.isBusy()) {
                    // Path1 completed, start Path2
                    pathState = 2;
                    follower.followPath(paths.Path2);
                }
                break;
            case 2:
                // Following Path2
                if (!follower.isBusy()) {
                    // Path2 completed, start Path3
                    pathState = 3;
                    follower.followPath(paths.Path3);
                }
                break;
            case 3:
                // Following Path3
                if (!follower.isBusy()) {
                    // Path3 completed, start Path4
                    pathState = 4;
                    follower.followPath(paths.Path4);
                }
                break;
            case 4:
                // Following Path4
                if (!follower.isBusy()) {
                    // Path4 completed, autonomous finished
                    pathState = 5;
                }
                break;
            case 5:
                // Autonomous finished
                break;
        }
        return pathState;
    }


}
