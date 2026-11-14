package org.firstinspires.ftc.teamcode.opmodes.auton;

/**
 * BaseLinearAutonomous - state-sequence autonomous base class (ported from Decode_Pedro to Pedro Pathing 3).
 *
 * Same architecture as Decode_Pedro:
 * - Subclasses build a list of StateNodes in buildStateSequence(); the base class runs them in order
 * - Parameterized generic states (MOVE_TO_TARGET, MOVE_TO_TARGET_SLOW, WAIT) take a LaunchTarget / time
 * - Alliance-agnostic design using AllianceConfig (thin Blue/Red subclasses)
 * - Forced RETURN_HOME at the deadline so the robot finishes inside 30 seconds
 * - Final pose stored in PoseHolder for a seamless autonomous-to-teleop transition
 *
 * DECODE-specific launcher/kicker/intake states and hardware were not carried over.
 * Add this robot's mechanism states to AutonomousState and handle them in updateAutonomous().
 *
 * Subclasses must implement:
 * - buildStateSequence(): Define the sequence of states for the autonomous program
 * - getAllianceConfig(): Provide alliance-specific configuration (Blue/Red)
 */

import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;


import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.IntakeUtil;
import org.firstinspires.ftc.teamcode.mechanisms.LauncherUtil;
import org.firstinspires.ftc.teamcode.mechanisms.LiftUtil;
import org.firstinspires.ftc.teamcode.mechanisms.LimelightUtil;
import org.firstinspires.ftc.teamcode.mechanisms.ServoUtil;
import org.firstinspires.ftc.teamcode.opmodes.AllianceConfig;
import org.firstinspires.ftc.teamcode.opmodes.LaunchTarget;
import org.firstinspires.ftc.teamcode.opmodes.PoseHolder;
import org.firstinspires.ftc.teamcode.opmodes.RobotConstants;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public abstract class BaseLinearAutonomous extends LinearOpMode {
    // Hardware components
    protected Follower follower;                    // Main robot movement controller
    protected final PoseFactory poseFactory = PoseFactory.degrees();
    protected LauncherUtil launcher;
    protected IntakeUtil intake;
    protected ServoUtil servos;
    protected LiftUtil lift;
    protected LimelightUtil limelight;              // Limelight3A AprilTag camera (alliance goal cluster)
    // Alliance-specific configuration
    protected AllianceConfig allianceConfig;

    // Autonomous state tracking
    protected AutonomousState currentState = AutonomousState.COMPLETE;

    // Path following state tracking
    protected boolean pathStarted = false;          // Reset whenever a state advances
    protected boolean homeTurnDone = false;         // RETURN_HOME: true once the turn-in-place to the park heading is done
    protected Path homePath;                        // RETURN_HOME: park path, built once when RETURN_HOME starts
    protected Pose homeTurnPose;                    // RETURN_HOME: current x/y with the heading the park path starts with
    protected int liftStartPosition;                // LIFT_UP / LIFT_DOWN: encoder position when the move started
    protected ElapsedTime alignedTimer = new ElapsedTime();  // ALIGN_TO_GOAL: time tx has stayed within tolerance
    protected boolean aligned = false;              // ALIGN_TO_GOAL: tx currently within tolerance

    // Timing control
    protected ElapsedTime autonomousTimer = new ElapsedTime();  // Tracks total autonomous runtime (never reset)
    protected ElapsedTime stateTimer = new ElapsedTime();       // Tracks individual state duration (reset on state change)
    protected ElapsedTime pathTimer = new ElapsedTime();

    // Autonomous states
    protected enum AutonomousState {
        MOVE_TO_TARGET,                 // Parameterized: drive to "target" at autonomous speed
        MOVE_TO_TARGET_SLOW,            // Parameterized: drive to "target" at INTAKE_MOVEMENT_SPEED
        START_POLLEN_LAUNCHER,
        STOP_POLLEN_LAUNCHER,
        START_INTAKE,
        STOP_INTAKE,
        START_NECTAR_LAUNCHER,
        STOP_NECTAR_LAUNCHER,
        LIFT_UP,
        LIFT_DOWN,
        OPEN_POLLEN_SERVO,
        OPEN_NECTAR_SERVO,
        CLOSE_POLLEN_SERVO,
        CLOSE_NECTAR_SERVO,
        WAIT_FOR_TIP,                   // Wait for the goal tag cluster, then start the pollen launcher (go home at TIP_WAIT_GIVE_UP_MS)
        ALIGN_TO_GOAL,                  // Turn in place until the Limelight sees the alliance goal centered (tx ~ 0)
        WAIT,                           // Parameterized: pause for "ms" milliseconds
        RETURN_HOME,                    // Return to auton end position
        COMPLETE                        // Autonomous sequence complete
    }

    /**
     * StateNode - Represents a single state in the autonomous sequence with optional parameters
     */
    protected static class StateNode {
        private final AutonomousState state;
        private final Map<String, Object> parameters;

        public StateNode(AutonomousState state) {
            this.state = state;
            this.parameters = new HashMap<>();
        }

        public StateNode(AutonomousState state, Map<String, Object> parameters) {
            this.state = state;
            this.parameters = parameters != null ? parameters : new HashMap<>();
        }

        public AutonomousState getState() {
            return state;
        }

        public Map<String, Object> getParameters() {
            return parameters;
        }

        public Object getParameter(String key) {
            return parameters.get(key);
        }

        public String getParameterAsString(String key, String defaultValue) {
            Object value = parameters.get(key);
            return value != null ? value.toString() : defaultValue;
        }

        public Pose getParameterAsPose(String key) {
            return (Pose) parameters.get(key);
        }
    }

    // State sequence management
    protected List<StateNode> stateSequence = new ArrayList<>();
    protected int currentStateIndex = 0;

    /**
     * Subclasses must implement this method to build their state sequence
     * @return List of StateNode objects defining the autonomous sequence
     */
    protected abstract List<StateNode> buildStateSequence();

    /**
     * Abstract method that subclasses must implement to provide alliance-specific configuration
     * @return AllianceConfig object with alliance-specific constants
     */
    protected abstract AllianceConfig getAllianceConfig();

    /**
     * Path driven in RETURN_HOME. Built from the robot's current pose when RETURN_HOME starts; the robot
     * first turns in place to the path's starting heading, then follows it.
     *
     * Default: the autons launch from the init pose against a side wall and park further along the same
     * wall, so the robot must turn ~90 degrees close to the wall. Turning in place there swings the back
     * corners into the wall, so instead the path bows out toward the field center by
     * RobotConstants.PARK_WALL_CLEARANCE_IN (cubic curve with both control points pushed off the wall),
     * and the heading turns gradually from the start heading to the park heading along the way. The
     * turn happens mostly mid-path, where the robot is furthest from the wall. Override for a custom park.
     */
    protected Path park() {
        Pose start = follower.pose();
        Pose end = allianceConfig.getAutonEndPose();
        double startHrading = Math.toDegrees(start.heading());
        // Push the control points away from the nearest side wall (x = 0 for Red, x = 144 for Blue)
        double awayFromWall = start.y() < 15 && (startHrading > 87 && startHrading < 93) ? RobotConstants.PARK_WALL_CLEARANCE_IN : -RobotConstants.PARK_WALL_CLEARANCE_IN;
        Pose control1 = new Pose(start.x() , start.y()+awayFromWall, 0);  // Bezier control points; their headings are unused
        Pose control2 = new Pose(end.x() , end.y(), 0);

        return Paths.curve(start, control1, control2, end)
                .linear(start, end)
                .with(Constants.foresightConfig.maxPathSpeed.at(RobotConstants.TEST_PATH_SPEED));
    }

    /**
     * Overridable method for subclasses to configure the autonomous timeout
     * Default is 28 seconds (28000ms) to ensure return home before 30 second limit
     * @return Timeout in milliseconds
     */
    protected long getAutonomousTimeoutMs() {
        return RobotConstants.AUTONOMOUS_RETURN_HOME_DEADLINE_MS;
    }

    // ========== Helper Methods for Building State Sequences ==========

    /**
     * Helper method to create a simple StateNode without parameters
     */
    protected StateNode state(AutonomousState state) {
        return new StateNode(state);
    }

    /**
     * Helper method to create a StateNode with parameters
     */
    protected StateNode state(AutonomousState state, Map<String, Object> parameters) {
        return new StateNode(state, parameters);
    }

    /**
     * Helper method to create a MOVE_TO_TARGET StateNode
     */
    protected StateNode moveTo(LaunchTarget target) {
        Map<String, Object> params = new HashMap<>();
        params.put("target", target);
        return state(AutonomousState.MOVE_TO_TARGET, params);
    }

    /**
     * Helper method to create a MOVE_TO_TARGET_SLOW StateNode
     */
    protected StateNode moveSlowTo(LaunchTarget target) {
        Map<String, Object> params = new HashMap<>();
        params.put("target", target);
        return state(AutonomousState.MOVE_TO_TARGET_SLOW, params);
    }

    /**
     * Helper method to create an ALIGN_TO_GOAL StateNode (turn to face the alliance goal with the Limelight)
     */
    protected StateNode alignToGoal() {
        return state(AutonomousState.ALIGN_TO_GOAL);
    }

    /**
     * Helper method to create a WAIT StateNode
     */
    protected StateNode waitMs(long ms) {
        Map<String, Object> params = new HashMap<>();
        params.put("ms", ms);
        return state(AutonomousState.WAIT, params);
    }

    /**
     * Helper method to add a complete collection cycle to the state sequence
     * A collection cycle consists of:
     * 1. Move to intake start position
     * 2. Move slowly to intake end position
     *
     * @param sequence The list to add states to
     * @param intakeStartTarget LaunchTarget for intake start position
     * @param intakeEndTarget LaunchTarget for intake end position
     */
    protected void addCollectionCycle(List<StateNode> sequence,
                                      LaunchTarget intakeStartTarget,
                                      LaunchTarget intakeEndTarget) {
        sequence.add(moveTo(intakeStartTarget));
        sequence.add(moveSlowTo(intakeEndTarget));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // INITIALIZATION PHASE (equivalent to init())
        // Get alliance-specific configuration
        allianceConfig = getAllianceConfig();

        // Initialize hardware components
        initializeHardware();

        // Build state sequence from subclass
        stateSequence = buildStateSequence();
        if (stateSequence == null || stateSequence.isEmpty()) {
            throw new IllegalStateException("buildStateSequence() must return a non-empty list");
        }
        telemetry.addData("State Sequence", "Loaded (" + stateSequence.size() + " states)");

        // Display ready status
        telemetry.addData("Status", "Ready to start autonomous");
        telemetry.addData("Alliance", allianceConfig.getAllianceName());
        displayPose();
        // Wait for the game to start (driver presses PLAY)
        // Init loop (equivalent to init_loop()): keep odometry live so the start pose can be checked
        while (opModeInInit()) {
            follower.update();
            limelight.update();

            Pose pose = follower.pose();
            telemetry.addData("Status", "Ready to start autonomous");
            telemetry.addData("Alliance", allianceConfig.getAllianceName());
            telemetry.addData("State Sequence", "Loaded (" + stateSequence.size() + " states)");
            telemetry.addData("Pose", String.format("(%.1f, %.1f, %.1f°)",
                pose.x(), pose.y(), Math.toDegrees(pose.heading())));
            telemetry.addData("Limelight", limelight.getStatusString());
            telemetry.update();
        }

        // START PHASE (equivalent to start())
        if (isStopRequested()) return;

        // Start the autonomous sequence timer (tracks total runtime, never reset)
        autonomousTimer.reset();

        // Start the state timer for individual state tracking
        stateTimer.reset();

        // Initialize state sequence
        currentStateIndex = 0;
        currentState = stateSequence.get(0).getState();

        // MAIN LOOP PHASE (equivalent to loop())
        while (opModeIsActive() && currentState != AutonomousState.COMPLETE) {
            updateAutonomous();
            displayPose();
        }

        // Final cleanup (when COMPLETE or opMode stopped)
        stopAllMotors();

        // Stop the follower to prevent continued navigation
        if (follower != null) {
            follower.stop();

            // Store the final pose in PoseHolder for TeleOp to use
            Pose finalPose = follower.pose();
            PoseHolder.setCurrentPose(finalPose);
            telemetry.addData("Status", "Autonomous Complete!");
            telemetry.addData("Final Pose", String.format("(%.1f, %.1f, %.1f°)",
                finalPose.x(), finalPose.y(), Math.toDegrees(finalPose.heading())));
            telemetry.addData("Pose Stored", "Ready for TeleOp transition");
            telemetry.update();
        }
    }

    /**
     * Initialize hardware. Override (and call super) to add this robot's mechanisms.
     */
    protected void initializeHardware() {
        follower = Constants.create(hardwareMap);
        follower.setPose(allianceConfig.getInitPose());
        follower.update();

        launcher = new LauncherUtil();
        launcher.init(hardwareMap);

        intake = new IntakeUtil();
        intake.init(hardwareMap);

        servos = new ServoUtil();
        servos.init(hardwareMap);

        lift = new LiftUtil();
        lift.init(hardwareMap);

        // Track only this alliance's goal AprilTags (same setup as BaseLinearTeleOp)
        limelight = new LimelightUtil(hardwareMap);
        limelight.setTargetTagId(-1);
        limelight.setClusterMode(true);
        limelight.setTargetCluster(allianceConfig.getAllianceName() + " Goal",
                allianceConfig.getTargetAprilTagIds());
    }
    /**
     * Main state machine update - called every loop
     */
    protected void updateAutonomous() {
        // Check if we've exceeded the return home deadline (safety mechanism to ensure completion within 30 seconds)
        // NOTE: Using autonomousTimer which tracks total runtime and is never reset during the sequence
        if (autonomousTimer.milliseconds() >= getAutonomousTimeoutMs()) {
            // Only force return home if not already returning or complete
            if (currentState != AutonomousState.RETURN_HOME && currentState != AutonomousState.COMPLETE) {
                telemetry.addData("WARNING", String.format("Time deadline reached (%.1fs) - Forcing return home!",
                    autonomousTimer.milliseconds() / 1000.0));

                // Stop a lift move that was cut short, then reset path flag and transition to return home
                lift.stop();
                pathStarted = false;
                homeTurnDone = false;
                currentState = AutonomousState.RETURN_HOME;
                stateTimer.reset();  // Reset for the return home sequence
            }
        }

        // Cache fresh AprilTag data for this loop
        limelight.update();

        switch (currentState) {
            case ALIGN_TO_GOAL:
                handleAlignToGoal();
                break;
            case START_POLLEN_LAUNCHER:
                handlePollenStart();
                break;
            case START_NECTAR_LAUNCHER:
                handleNectarStart();
                break;
            case OPEN_POLLEN_SERVO:
                handlePollenServo();
                break;
            case OPEN_NECTAR_SERVO:
                handleNectarServo();
                break;
            case STOP_POLLEN_LAUNCHER:
                stopPollenLauncher();
                break;
            case STOP_NECTAR_LAUNCHER:
                stopNectarLauncher();
                break;
            case CLOSE_POLLEN_SERVO:
                closePollenServo();
                break;
            case CLOSE_NECTAR_SERVO:
                closeNectarServo();
                break;
            case RETURN_HOME:
                handleMoveToHomeState();
                break;
            case WAIT:
                handleWaitState();
                break;
            case LIFT_UP:
                handleLiftUp();
                break;
            case LIFT_DOWN:
                handleLiftDown();
                break;
            case START_INTAKE:
                handleStartIntake();
                break;
            case STOP_INTAKE:
                handleStopIntake();
                break;
            case WAIT_FOR_TIP:
                handleWaitForTip();
                break;
            case COMPLETE:
                break;
        }

        // Update follower and telemetry
        if (currentState != AutonomousState.COMPLETE) {
            follower.update();
            updateTelemetry();
        }
    }

    /**
     * Wait until the Limelight sees at least RobotConstants.TIP_CLUSTER_MIN_PERCENT of the goal cluster
     * (the alliance config's tag IDs - 32-35 for Red SF, 44-47 for Blue SF), then advance to the next state.
     * If nothing is seen by RobotConstants.TIP_WAIT_GIVE_UP_MS of auton time, skip the rest of the sequence
     * and go straight to RETURN_HOME.
     */
    protected void handleWaitForTip(){
        int percentFound = limelight.getClusterPercentFound();
        telemetry.addData("Status", String.format("Waiting for goal tags: %d%% visible (need %d%%) %s",
            percentFound, RobotConstants.TIP_CLUSTER_MIN_PERCENT, limelight.getClusterTagIds()));

        if (percentFound >= RobotConstants.TIP_CLUSTER_MIN_PERCENT) {
            advanceToNextState();
            return;  // don't let the give-up check below override the state we just moved to
        }

        if (autonomousTimer.milliseconds() >= RobotConstants.TIP_WAIT_GIVE_UP_MS) {
            telemetry.addData("WARNING", "Goal tags not seen in time - going home");
            pathStarted = false;
            homeTurnDone = false;
            currentState = AutonomousState.RETURN_HOME;
            stateTimer.reset();
        }
    }

    /**
     * Run the intake inward at RobotConstants.INTAKE_SPEED, then advance
     */
    protected void handleStartIntake(){
        intake.intake();
        telemetry.addData("Status", "Intake started at " + RobotConstants.INTAKE_SPEED + " ticks/sec");
        advanceToNextState();
    }

    /**
     * Stop the intake, then advance
     */
    protected void handleStopIntake(){
        intake.stop();
        telemetry.addData("Status", "Intake stopped");
        advanceToNextState();
    }


    /**
     * Raise the lift RobotConstants.LIFT_TICKS_ATON ticks from where it is when the state starts
     */
    protected void handleLiftUp(){
        handleLiftMove(true);
    }

    /**
     * Lower the lift RobotConstants.LIFT_TICKS_ATON ticks from where it is when the state starts
     */
    protected void handleLiftDown(){
        handleLiftMove(false);
    }

    /**
     * Runs the lift at LIFT_UP_SPEED / LIFT_DOWN_SPEED until it has moved RobotConstants.LIFT_TICKS_ATON ticks
     * (or RobotConstants.LIFT_TIMEOUT_MS passes), then stops it and advances
     */
    protected void handleLiftMove(boolean up){
        // First loop of this state: remember where the lift started and start it moving
        // (pathStarted is reset by advanceToNextState, so it doubles as the "move started" flag)
        if (!pathStarted) {
            liftStartPosition = lift.getPosition();
            if (up) {
                lift.up();
            } else {
                lift.down();
            }
            pathStarted = true;
        }

        int ticksMoved = Math.abs(lift.getPosition() - liftStartPosition);
        telemetry.addData("Status", String.format("Lift %s: %d / %d ticks", up ? "up" : "down",
            ticksMoved, RobotConstants.LIFT_TICKS_ATON));

        boolean timedOut = stateTimer.milliseconds() >= RobotConstants.LIFT_TIMEOUT_MS;
        if (timedOut) {
            telemetry.addData("WARNING", "Lift move timed out!");
        }

        if (ticksMoved >= RobotConstants.LIFT_TICKS_ATON || timedOut) {
            lift.stop();
            advanceToNextState();
        }
    }

    /**
     * Turn in place to face the alliance goal using the Limelight horizontal offset (tx).
     * Same proportional control as BaseLinearTeleOp.handleAlignToGoal(). Advances once tx has stayed within
     * RobotConstants.ALIGN_TOLERANCE_DEG for ALIGN_SETTLE_MS, or after ALIGN_TIMEOUT_MS (e.g. no tag in view),
     * then holds the current pose so the follower keeps the new heading.
     */
    protected void handleAlignToGoal() {
        boolean timedOut = stateTimer.milliseconds() >= RobotConstants.ALIGN_TIMEOUT_MS;
        double turnPower = 0.0;

        if (!limelight.hasTarget()) {
            aligned = false;
            telemetry.addData("Status", "Align: no goal tag in view");
        } else {
            double tx = limelight.getTx();
            boolean inTolerance = Math.abs(tx) <= RobotConstants.ALIGN_TOLERANCE_DEG;
            if (inTolerance && !aligned) {
                alignedTimer.reset();  // just entered tolerance - start the settle timer
            }
            aligned = inTolerance;
            if (!aligned) {
                // tx > 0 means the tag is to the right; negative turn power turns the robot right
                turnPower = -Math.signum(tx) * Math.min(RobotConstants.ALIGN_MAX_TURN_POWER,
                    Math.max(RobotConstants.ALIGN_MIN_TURN_POWER, Math.abs(tx) * RobotConstants.ALIGN_KP));
            }
            telemetry.addData("Status", String.format("Align: tx %.1f° %s", tx, aligned ? "ALIGNED" : "TURNING"));
        }

        boolean settled = aligned && alignedTimer.milliseconds() >= RobotConstants.ALIGN_SETTLE_MS;
        if (settled || timedOut) {
            if (timedOut && !settled) {
                telemetry.addData("WARNING", "Align to goal timed out!");
            }
            aligned = false;
            follower.hold(follower.pose());  // Keep the aligned heading while later states run
            advanceToNextState();
            return;
        }

        follower.manual(new DrivePowers(0, 0, turnPower));
    }

    protected void handleDelayedPollenStart(){

    }

    protected void handlePollenStart(){
        launcher.setPollenSpeed(allianceConfig.getPollenLaunchSpeed());
        telemetry.addData("Status", "Pollen Launcher started at " + allianceConfig.getPollenLaunchSpeed() + " ticks/sec");

        // Auto-advance to next state in sequence
        advanceToNextState();
    }

    protected void handleNectarStart(){
        launcher.setNectarSpeed(allianceConfig.getNectarLaunchSpeed());
        telemetry.addData("Status", "Nectar Launcher started at " + allianceConfig.getNectarLaunchSpeed() + " ticks/sec");

        advanceToNextState();
    }

    protected void handlePollenServo(){
        servos.openPollen();
        telemetry.addData("Status", "Pollen Servo Open");
        advanceToNextState();
    }

    protected void handleNectarServo(){
        servos.openNectar();
        telemetry.addData("Status", "Nectar Servo opened");
        advanceToNextState();
    }

    protected void stopPollenLauncher(){
        launcher.stopPollen();
        advanceToNextState();
    }

    protected void stopNectarLauncher(){
        launcher.stopNectar();
        advanceToNextState();
    }

    protected void closePollenServo(){
        servos.closePollen();
        telemetry.addData("Status", "Pollen Servo closed");
        advanceToNextState();
    }

    protected void closeNectarServo(){
        servos.closeNectar();
        telemetry.addData("Status", "Nectar Servo closed");
        advanceToNextState();
    }

    /**
     * Handle a parameterized move to the StateNode's "target"
     */
    protected void handleMoveToTargetState(double speed, long timeoutMs) {
        StateNode currentNode = getCurrentStateNode();
        LaunchTarget target = currentNode != null ? (LaunchTarget) currentNode.getParameter("target") : null;
        if (target == null) {
            telemetry.addData("ERROR", currentState + " missing target parameter - skipping");
            advanceToNextState();
            return;
        }

        Pose targetPose = allianceConfig.getTargetPose(target);

        // Start path following if not already started
        if (!pathStarted) {
            follower.follow(pathTo(targetPose, speed));
            pathStarted = true;
            pathTimer.reset();
            telemetry.addData("Status", "Starting path to " + target.name());
        }

        // Check for path timeout
        boolean pathTimedOut = pathTimer.milliseconds() >= timeoutMs;
        if (pathTimedOut) {
            telemetry.addData("WARNING", "Path to " + target.name() + " timed out!");
        }

        // Path complete AND at target pose, OR timeout
        if ((!follower.isBusy() && isAtTargetPose(targetPose)) || pathTimedOut) {
            telemetry.addData("Status", "Reached " + target.name());
            advanceToNextState();
        }
    }

    /**
     * Handle a parameterized pause of "ms" milliseconds
     */
    protected void handleWaitState() {
        StateNode currentNode = getCurrentStateNode();
        Object ms = currentNode != null ? currentNode.getParameter("ms") : null;
        long waitMs = ms instanceof Number ? ((Number) ms).longValue() : 0;

        telemetry.addData("Status", String.format("Waiting %.1fs", (waitMs - stateTimer.milliseconds()) / 1000.0));
        if (stateTimer.milliseconds() >= waitMs) {
            advanceToNextState();
        }
    }

    /**
     * Handle moving to home position
     */
    protected void handleMoveToHomeState() {
        Pose targetPose = allianceConfig.getAutonEndPose();

        // Step 1: turn in place to the heading the park path starts with (hold current x/y with that heading),
        // so the robot then drives along the path instead of strafing while it turns
        if (!homeTurnDone) {
            if (!pathStarted) {
                homePath = park();
                Pose currentPose = follower.pose();
                homeTurnPose = new Pose(currentPose.x(), currentPose.y(), homePath.heading(0));
                follower.hold(homeTurnPose);
                pathStarted = true;
                pathTimer.reset();
                telemetry.addData("Status", "Turning to park heading...");
            }

            if (isAtTargetHeading(homeTurnPose) || pathTimer.milliseconds() >= RobotConstants.TURN_TIMEOUT_MS) {
                homeTurnDone = true;
                pathStarted = false;  // Step 2 starts its own path
            }
            return;
        }

        // Step 2: drive to park. Start path following if not already started
        if (!pathStarted) {
            follower.follow(homePath);
            pathStarted = true;
            pathTimer.reset();
            telemetry.addData("Status", "Starting path to home position...");
        }

        // Check for path timeout
        boolean pathTimedOut = pathTimer.milliseconds() >= RobotConstants.PARK_PATH_TIMEOUT_MS;
        if (pathTimedOut) {
            telemetry.addData("WARNING", "Path to home timed out!");
        }

        // Check if we've reached the target (path complete AND at target pose, OR timeout)
        if ((!follower.isBusy() && isAtTargetPose(targetPose)) || pathTimedOut) {
            // Proceed directly to complete state
            currentState = AutonomousState.COMPLETE;
            stateTimer.reset();
            telemetry.addData("Status", "Reached home position - Complete...");
        }
    }



    /**
     * Straight-line path from the robot's current pose to targetPose.
     * Short paths turn to the target heading immediately; longer paths turn gradually along the way.
     */
    protected Path pathTo(Pose targetPose, double speed) {
        Pose currentPose = follower.pose();
        Path path = line(currentPose, targetPose);
        if (currentPose.distance(targetPose) < RobotConstants.SHORT_PATH_THRESHOLD_MM) {
            path = path.constant(targetPose);
        } else {
            path = path.linear(currentPose, targetPose);
        }
        return path.with(Constants.foresightConfig.maxPathSpeed.at(speed));
    }

    /**
     * Update telemetry display
     */
    protected void updateTelemetry() {
        Pose currentPose = follower.pose();

        double elapsedSeconds = autonomousTimer.milliseconds() / 1000.0;
        double remainingSeconds = (getAutonomousTimeoutMs() - autonomousTimer.milliseconds()) / 1000.0;

        telemetry.addData("Elapsed", String.format("%.1fs (home in %.1fs)", elapsedSeconds, Math.max(0, remainingSeconds)));
        telemetry.addData("State", String.format("%s (%d/%d)", currentState, currentStateIndex + 1, stateSequence.size()));
        telemetry.addData("Pose", String.format("(%.1f, %.1f, %.1f°)",
            currentPose.x(), currentPose.y(), Math.toDegrees(currentPose.heading())));
        telemetry.addData("Follower Status", follower.isBusy() ? "BUSY" : "IDLE");
        telemetry.addData("Limelight", limelight.getStatusString());
    }

    /**
     * Stop all mechanisms. Override to stop this robot's mechanisms.
     */
    protected void stopAllMotors() {
        launcher.stop();
        intake.stop();
        lift.stop();
        limelight.stop();
    }

    /**
     * Checks if the robot is at the specified target pose within tolerance
     */
    protected boolean isAtTargetPose(Pose targetPose) {
        return isAtTargetPosition(targetPose) && isAtTargetHeading(targetPose);
    }

    /**
     * Checks if the robot position (X, Y) is at target within tolerance
     */
    protected boolean isAtTargetPosition(Pose targetPose) {
        return follower.pose().distance(targetPose) <= RobotConstants.POSITION_TOLERANCE;
    }

    /**
     * Checks if the robot heading is at target within tolerance
     */
    protected boolean isAtTargetHeading(Pose targetPose) {
        double headingDifference = Math.abs(normalizeAngle(targetPose.heading() - follower.pose().heading()));
        return headingDifference <= RobotConstants.HEADING_TOLERANCE;
    }

    /**
     * Normalizes an angle to be within [-π, π] range
     */
    protected double normalizeAngle(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    /**
     * Helper method to advance to the next state in the sequence
     */
    protected void advanceToNextState() {
        if (stateSequence == null || stateSequence.isEmpty()) {
            return;
        }

        pathStarted = false;  // Next state starts its own path
        homeTurnDone = false;
        currentStateIndex++;
        if (currentStateIndex < stateSequence.size()) {
            currentState = stateSequence.get(currentStateIndex).getState();
            stateTimer.reset();
        } else {
            // Reached end of sequence
            currentState = AutonomousState.COMPLETE;
            stateTimer.reset();
        }
    }

    /**
     * Helper method to get the current state node from the sequence
     */
    protected StateNode getCurrentStateNode() {
        if (stateSequence == null || currentStateIndex >= stateSequence.size()) {
            return null;
        }
        return stateSequence.get(currentStateIndex);
    }

    private void displayPose() {
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }

}
