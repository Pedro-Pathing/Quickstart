package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    private HardwareConfigAuto robot;
    private RobotFunctionsAuto robotFunctions;
    
    // Timer for shooting phases
    private ElapsedTime shootTimer = new ElapsedTime();
    private static final double SHOOT_TIME = 0.5; // seconds
    
    // Flag for shooter control - must be updated continuously for bang-bang
    private boolean shooterRunning = false;

    // Starting pose - MUST match the beginning of Path1!
    private final Pose startPose = new Pose(129.0, 108.0, Math.toRadians(-90));

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        paths = new Paths(follower); // Build paths
        pathState = 0;

        try {
            robot = new HardwareConfigAuto();
            robot.init(hardwareMap);
            robotFunctions = new RobotFunctionsAuto(robot);
            robotFunctions.setBlocker(true);
            telemetry.addLine("INITIALIZED SUCCESSFULLY");
        } catch (Exception e) {
            // Hardware init failed - paths will still work
            robot = null;
            robotFunctions = null;
            telemetry.addLine("THREW ERROR");
        }

        telemetry.update();
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        // Start following the first path when autonomous begins
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine
        
        // Continuously update shooter with bang-bang control (uses Limelight for target TPS)
        robotFunctions.controlShooter(shooterRunning);

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("Shooter Running", shooterRunning);
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(129.000, 108.000),
                                    new Pose(98.000, 110.000),
                                    new Pose(88.000, 83.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(88.000, 83.000), new Pose(122.346, 80.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(122.346, 80.000),
                                    new Pose(122.000, 75.000),
                                    new Pose(129.000, 72.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(129.000, 72.000), new Pose(88.000, 83.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-96))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(88.000, 83.000),
                                    new Pose(85.000, 56.000),
                                    new Pose(100.042, 59.332),
                                    new Pose(123.000, 57.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(123.000, 57.000), new Pose(88.000, 83.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-6), Math.toRadians(-95))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(88.000, 83.000),
                                    new Pose(82.000, 44.000),
                                    new Pose(94.629, 35.946),
                                    new Pose(84.000, 36.000),
                                    new Pose(123.000, 32.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(123.000, 32.000), new Pose(88.000, 83.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(88.000, 83.000), new Pose(120.000, 69.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
        }
    }

    /**
     * Sets the path state and starts following the corresponding path
     * @param state the path state to transition to
     */
    public void setPathState(int state) {
        pathState = state;
        switch (pathState) {
            case 0:
                // Start Path1, move turret to -145 DURING path
                robotFunctions.setTurretAngle(-80);
                shooterRunning = true;
                follower.followPath(paths.Path1, true);
                break;
            case 1:
                // End of Path1: start shooting with bang-bang control, start timer (turret already at -145)
                shooterRunning = true;
                shootTimer.reset();
                break;
            case 2:
                // Shooting done after Path1, start Path2 with intake ON
                robotFunctions.setBlocker(true);
                shooterRunning = false;
                follower.followPath(paths.Path2, true);
                break;
            case 3:
                // Path2 done, start Path3 (intake still on)
                robotFunctions.stopIntakeSystem();
                follower.followPath(paths.Path3, true);
                break;
            case 4:
                // End of Path3: start Path4, move turret to -45 DURING path
                robotFunctions.setTurretAngle(-30);
                follower.followPath(paths.Path4, true);
                break;
            case 5:
                // End of Path4: start shooting with bang-bang control, start timer (turret already at -45)
                shooterRunning = true;
                shootTimer.reset();
                break;
            case 6:
                // Shooting done after Path4, start Path5 with intake ON
                robotFunctions.setBlocker(true);
                shooterRunning = false;
                follower.followPath(paths.Path5, true);
                break;
            case 7:
                // Path5 done, start Path6, move turret to -45 DURING path
                robotFunctions.stopIntakeSystem();
                robotFunctions.setTurretAngle(-30);
                follower.followPath(paths.Path6, true);
                break;
            case 8:
                // End of Path6: start shooting with bang-bang control, start timer (turret already at -45)
                robotFunctions.stopIntakeSystem();
                shooterRunning = true;
                shootTimer.reset();
                break;
            case 9:
                // Shooting done after Path6, start Path7 with intake ON
                robotFunctions.setBlocker(true);
                shooterRunning = false;
                follower.followPath(paths.Path7, true);
                break;
            case 10:
                // Path7 done, start Path8, move turret to -145 DURING path
                robotFunctions.stopIntakeSystem();
                robotFunctions.setTurretAngle(-80);
                follower.followPath(paths.Path8, true);
                break;
            case 11:
                // End of Path8: start shooting with bang-bang control, start timer (turret already at -145)
                robotFunctions.stopIntakeSystem();
                shooterRunning = true;
                shootTimer.reset();
                break;
            case 12:
                // Shooting done after Path8, start Path9
                robotFunctions.setBlocker(true);
                shooterRunning = false;
                robotFunctions.stopIntakeSystem();
                follower.followPath(paths.Path9, true);
                break;
            case 13:
                // Autonomous complete
                shooterRunning = false;
                robotFunctions.stopIntakeSystem();
                break;
        }
    }

    /**
     * State machine for autonomous path following.
     * Checks if the current path is complete and transitions to the next state.
     */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Following Path1
                if (!follower.isBusy()) {
                    setPathState(1); // End of Path1 - shoot
                }
                break;
            case 1:
                // Shooting after Path1 - wait 1.5 seconds
                if(shootTimer.seconds() >= SHOOT_TIME + 0.75) {
                    setPathState(2); //done shooting
                } else if (shootTimer.seconds() >= SHOOT_TIME) {
                    robotFunctions.setBlocker(false); //open blocker after shooter is ramped up
                    robotFunctions.runIntakeSystem(HardwareConfigAuto.INTAKE_POWER);
                }
                break;
            case 2:
                // Following Path2 (intake on)
                if (!follower.isBusy()) {
                    setPathState(3); // Start Path3
                }
                break;
            case 3:
                // Following Path3 (intake on)
                if (!follower.isBusy()) {
                    setPathState(4); // End of Path3
                }
                break;
            case 4:
                // Following Path4
                if (!follower.isBusy()) {
                    setPathState(5); // End of Path4 - shoot
                }
                break;
            case 5:
                // Shooting after Path4 - wait 1.5 seconds
                if(shootTimer.seconds() >= SHOOT_TIME + 0.75) {
                    setPathState(6); //done shooting
                } else if (shootTimer.seconds() >= SHOOT_TIME) {
                    robotFunctions.setBlocker(false); //open blocker after shooter is ramped up
                    robotFunctions.runIntakeSystem(HardwareConfigAuto.INTAKE_POWER);
                }
                break;
            case 6:
                // Following Path5 (intake on)
                if (!follower.isBusy()) {
                    setPathState(7); // Start Path6
                }
                break;
            case 7:
                // Following Path6 (intake on)
                if (!follower.isBusy()) {
                    setPathState(8); // End of Path6 - shoot
                }
                break;
            case 8:
                // Shooting after Path6 - wait 1.5 seconds
                if(shootTimer.seconds() >= SHOOT_TIME + 0.75) {
                    setPathState(9); //done shooting
                } else if (shootTimer.seconds() >= SHOOT_TIME) {
                    robotFunctions.setBlocker(false); //open blocker after shooter is ramped up
                    robotFunctions.runIntakeSystem(HardwareConfigAuto.INTAKE_POWER);
                }
                break;
            case 9:
                // Following Path7 (intake on)
                if (!follower.isBusy()) {
                    setPathState(10); // Start Path8
                }
                break;
            case 10:
                // Following Path8 (intake on)
                if (!follower.isBusy()) {
                    setPathState(11); // End of Path8 - shoot
                }
                break;
            case 11:
                // Shooting after Path8 - wait 1.5 seconds
                if(shootTimer.seconds() >= SHOOT_TIME + 0.75) {
                    setPathState(12); //done shooting
                } else if (shootTimer.seconds() >= SHOOT_TIME) {
                    robotFunctions.setBlocker(false); //open blocker after shooter is ramped up
                    robotFunctions.runIntakeSystem(HardwareConfigAuto.INTAKE_POWER);
                }
                break;
            case 12:
                // Following Path9
                if (!follower.isBusy()) {
                    setPathState(13); // Autonomous complete
                }
                break;
            case 13:
                // Autonomous complete
                break;
        }
    }
}
