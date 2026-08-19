//package org.firstinspires.ftc.teamcode;
//
//// Pedro Pathing imports
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//
//// FTC imports
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//// Math
//import static java.lang.Math.toRadians;
//
//
//@Autonomous(name = "Sample Auto Pathing")
//public class SampleAutoPathing extends OpMode {
//
//    // ============================================================
//    // PEDRO PATHING
//    // ============================================================
//
//    private Follower follower;
//
//    // Timers
//    private Timer pathTimer;
//    private Timer opModeTimer;
//
//
//    // ============================================================
//    // STATE MACHINE
//    // ============================================================
//
//    public enum PathState {
//        DRIVE_START_POSITION_TO_SHOOT_POSITION,
//        SHOOT_PRELOAD,
//        SHOOT_POSITION_TO_END_POSITION
//    }
//
//    private PathState pathState;
//
//
//    // ============================================================
//    // POSES
//    // ============================================================
//
//    /*
//     * Replace these X/Y values with the coordinates from the
//     * Pedro Pathing visualizer used in the video.
//     *
//     * The video uses a 138 degree heading for the start/shoot
//     * poses and 90 degrees for the final pose.
//     */
//
//    private final Pose startPose = new Pose(
//            START_X,
//            START_Y,
//            Math.toRadians(138)
//    );
//
//    private final Pose shootPose = new Pose(
//            SHOOT_X,
//            SHOOT_Y,
//            Math.toRadians(138)
//    );
//
//    private final Pose endPose = new Pose(
//            END_X,
//            END_Y,
//            Math.toRadians(90)
//    );
//
//
//    // ============================================================
//    // PATH CHAINS
//    // ============================================================
//
//    private PathChain driveStartPositionToShootPosition;
//    private PathChain driveShootPositionToEndPosition;
//
//
//    // ============================================================
//    // BUILD PATHS
//    // ============================================================
//
//    public void buildPaths() {
//
//        // --------------------------------------------------------
//        // PATH 1:
//        // Start position -> Shoot position
//        // --------------------------------------------------------
//
//        driveStartPositionToShootPosition = follower
//                .pathBuilder()
//                .addPath(
//                        new BezierLine(
//                                startPose,
//                                shootPose
//                        )
//                )
//                .setLinearHeadingInterpolation(
//                        startPose.getHeading(),
//                        shootPose.getHeading()
//                )
//                .build();
//
//
//        // --------------------------------------------------------
//        // PATH 2:
//        // Shoot position -> End position
//        // --------------------------------------------------------
//
//        driveShootPositionToEndPosition = follower
//                .pathBuilder()
//                .addPath(
//                        new BezierLine(
//                                shootPose,
//                                endPose
//                        )
//                )
//                .setLinearHeadingInterpolation(
//                        shootPose.getHeading(),
//                        endPose.getHeading()
//                )
//                .build();
//    }
//
//
//    // ============================================================
//    // STATE MACHINE UPDATE
//    // ============================================================
//
//    public void statePathUpdate() {
//
//        switch (pathState) {
//
//            // ----------------------------------------------------
//            // STATE 1
//            // Drive from start -> shoot
//            // ----------------------------------------------------
//
//            case DRIVE_START_POSITION_TO_SHOOT_POSITION:
//
//                follower.followPath(
//                        driveStartPositionToShootPosition,
//                        true
//                );
//
//                setPathState(
//                        PathState.SHOOT_PRELOAD
//                );
//
//                break;
//
//
//            // ----------------------------------------------------
//            // STATE 2
//            // Shoot preload
//            // ----------------------------------------------------
//
//            case SHOOT_PRELOAD:
//
//                /*
//                 * Wait until the first path is finished AND
//                 * at least 5 seconds have elapsed.
//                 */
//
//                if (!follower.isBusy()
//                        && pathTimer.getElapsedTimeSeconds() > 5) {
//
//                    telemetry.addLine("Done Path 1");
//
//                    /*
//                     * In a real robot this is where your
//                     * shooting/flywheel logic would go.
//                     */
//
//                    follower.followPath(
//                            driveShootPositionToEndPosition,
//                            true
//                    );
//
//                    setPathState(
//                            PathState.SHOOT_POSITION_TO_END_POSITION
//                    );
//                }
//
//                break;
//
//
//            // ----------------------------------------------------
//            // STATE 3
//            // Drive from shoot -> end
//            // ----------------------------------------------------
//
//            case SHOOT_POSITION_TO_END_POSITION:
//
//                if (!follower.isBusy()) {
//
//                    telemetry.addLine("Done All Paths");
//                }
//
//                break;
//
//
//            // ----------------------------------------------------
//            // DEFAULT
//            // ----------------------------------------------------
//
//            default:
//
//                telemetry.addLine("No State Commanded");
//
//                break;
//        }
//    }
//
//
//    // ============================================================
//    // SET PATH STATE
//    // ============================================================
//
//    public void setPathState(PathState newState) {
//
//        pathState = newState;
//
//        // Reset timer whenever we enter a new state
//        pathTimer.resetTimer();
//    }
//
//
//    // ============================================================
//    // INIT
//    // ============================================================
//
//    @Override
//    public void init() {
//
//        // Initial state
//        pathState =
//                PathState.DRIVE_START_POSITION_TO_SHOOT_POSITION;
//
//        // Create timers
//        pathTimer = new Timer();
//        opModeTimer = new Timer();
//
//        // Reset op mode timer
//        opModeTimer.resetTimer();
//
//        // Create Pedro Pathing follower
//        follower = Constants.createFollower(hardwareMap);
//
//        // Build all paths before autonomous starts
//        buildPaths();
//
//        // Tell Pedro where the robot physically starts
//        follower.setStartingPose(startPose);
//    }
//
//
//    // ============================================================
//    // START
//    // ============================================================
//
//    @Override
//    public void start() {
//
//        // Reset op mode timer
//        opModeTimer.resetTimer();
//
//        // Start the state machine
//        setPathState(
//                PathState.DRIVE_START_POSITION_TO_SHOOT_POSITION
//        );
//    }
//
//
//    // ============================================================
//    // LOOP
//    // ============================================================
//
//    @Override
//    public void loop() {
//
//        // Pedro Pathing MUST be updated every loop
//        follower.update();
//
//        // Update our state machine
//        statePathUpdate();
//
//
//        // --------------------------------------------------------
//        // TELEMETRY
//        // --------------------------------------------------------
//
//        telemetry.addData(
//                "Path State",
//                pathState.toString()
//        );
//
//        telemetry.addData(
//                "X",
//                follower.getPose().getX()
//        );
//
//        telemetry.addData(
//                "Y",
//                follower.getPose().getY()
//        );
//
//        telemetry.addData(
//                "Heading",
//                follower.getPose().getHeading()
//        );
//
//        telemetry.addData(
//                "Path Time",
//                pathTimer.getElapsedTimeSeconds()
//        );
//
//        telemetry.update();
//    }
//}