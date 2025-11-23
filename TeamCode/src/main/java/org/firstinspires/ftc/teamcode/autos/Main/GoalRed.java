package org.firstinspires.ftc.teamcode.autos.Main;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Hardware;

@Autonomous(name = "Red1", group = "Autonomous")
@Configurable // Panels
public class GoalRed extends OpMode {

    Hardware robot = new Hardware();
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Timer actionTimer; // Timer for delays

    @Override
    public void init() {
        robot.init(hardwareMap);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(122.426, 124.005, Math.toRadians(216)));

        paths = new Paths(follower); // Build paths
        actionTimer = new Timer(); // Initialize timer

        pathState = 0; // Initialize state

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
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Busy", follower.isBusy());
        panelsTelemetry.debug("Timer", actionTimer.getElapsedTimeSeconds());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(122.426, 124.005), new Pose(72.263, 71.562))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(216), Math.toRadians(45))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(72.263, 71.562), new Pose(101.730, 58.933))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Start following Path1
                follower.followPath(paths.Path1);
                pathState = 1;
                break;

            case 1: // Wait for Path1 to finish
                if (!follower.isBusy()) {
                    robot.outtake.setPower(1); // Start outtake
                    actionTimer.resetTimer();
                    pathState = 2;
                }
                break;

            case 2: // Wait 2 second delay
                if (actionTimer.getElapsedTimeSeconds() > 2) {
                    robot.mid1.setPower(1); // Set mid1 to 1
                    robot.mid2.setPower(1); // Set mid2 to 1
                    actionTimer.resetTimer(); // Reset timer for next delay
                    pathState = 3;
                }
                break;

            case 3: // Wait 1 second delay after mid motors
                if (actionTimer.getElapsedTimeSeconds() > 1) {
                    robot.intake.setPower(1); // Start intake at 1 power
                    actionTimer.resetTimer(); // Reset timer for intake delay
                    pathState = 4;
                }
                break;

            case 4: // Wait 1 second delay with intake running
                if (actionTimer.getElapsedTimeSeconds() > 1) {
                    pathState = 5;
                }
                break;

            case 5: // Start following Path2
                follower.followPath(paths.Path2);
                robot.intake.setPower(1); // Keep intake at 1 during Path2
                pathState = 6;
                break;

            case 6: // Wait for Path2 to finish
                if (!follower.isBusy()) {
                    pathState = 7;
                }
                break;

            case 7: // All paths complete
                // Add any end actions here (e.g., park, lower arm, etc.)
                break;
        }

        return pathState;
    }
}