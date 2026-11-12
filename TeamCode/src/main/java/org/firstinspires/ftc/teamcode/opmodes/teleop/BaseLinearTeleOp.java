package org.firstinspires.ftc.teamcode.opmodes.teleop;

/**
 * BaseLinearTeleOp - LinearOpMode TeleOp base class for two-player control (ported from Decode_Pedro to Pedro Pathing 3).
 *
 * Same architecture as Decode_Pedro:
 * - Alliance-agnostic design using AllianceConfig (thin Blue/Red subclasses)
 * - Starting pose comes from PoseHolder (set by autonomous), falling back to the alliance home pose
 * - Main loop calls one handler per subsystem, then displays telemetry
 *
 * Mechanisms use the shared utils in org.firstinspires.ftc.teamcode.mechanisms:
 * - LauncherUtil for the two launcher motors
 * - IntakeUtil for the intake motor
 * - ServoUtil for the pollen and nectar gate servos
 * - LimelightUtil for AprilTag detection of the alliance goal tag
 *
 * GAMEPAD1 CONTROLS (Driver):
 * - Left stick: Forward/backward and strafe movement
 * - Right stick: Robot rotation
 * - Right trigger: Slow mode (RobotConstants.SLOW_MODE_MULTIPLIER)
 * - Left + right bumper (hold): Drive to the shooting position if inside the shooting zone (release to cancel)
 * - Back (press to toggle): Turn to face the alliance goal AprilTag cluster using the Limelight.
 *   Stays on, holding the goal heading, until Back is pressed again or the right stick is moved
 * - X button: Toggle pollen launcher (on at RobotConstants.NEAR_LAUNCH_SPEED, press again to stop);
 *   the pollen gate opens automatically once the launcher reaches speed
 * - B button: Toggle nectar launcher (on at RobotConstants.NEAR_LAUNCH_SPEED, press again to stop);
 *   the nectar gate opens automatically once the launcher reaches speed
 * - Right bumper: Add RobotConstants.LAUNCHER_SPEED_STEP to every running launcher
 * - Left bumper: Subtract RobotConstants.LAUNCHER_SPEED_STEP from every running launcher
 * - Dpad left: Toggle pollen gate servo (open, press again to close)
 * - Dpad right: Toggle nectar gate servo (open, press again to close)
 *
 * GAMEPAD2 CONTROLS (Operator):
 * - Y button: Toggle intake (press again to stop)
 * - A button: Toggle outtake (press again to stop)
 * - Right bumper: Add RobotConstants.INTAKE_SPEED_STEP to the intake/outtake if it is running
 * - Left bumper: Subtract RobotConstants.INTAKE_SPEED_STEP from the intake/outtake if it is running
 *
 * Subclasses must implement:
 * - getAllianceConfig(): Provide alliance-specific configuration (Blue/Red)
 */

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.IntakeUtil;
import org.firstinspires.ftc.teamcode.mechanisms.LauncherUtil;
import org.firstinspires.ftc.teamcode.mechanisms.LimelightUtil;
import org.firstinspires.ftc.teamcode.mechanisms.ServoUtil;
import org.firstinspires.ftc.teamcode.opmodes.AllianceConfig;
import org.firstinspires.ftc.teamcode.opmodes.PoseHolder;
import org.firstinspires.ftc.teamcode.opmodes.RobotConstants;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public abstract class BaseLinearTeleOp extends LinearOpMode {
    // Hardware components
    protected Follower follower;                    // Main robot movement controller
    protected LauncherUtil launcher;                // Two-motor launcher
    protected IntakeUtil intake;                    // Intake motor
    protected ServoUtil servos;                     // Pollen and nectar gate servos
    protected LimelightUtil limelight;              // Limelight3A AprilTag camera

    // Alliance-specific configuration
    protected AllianceConfig allianceConfig;

    // Control states
    protected boolean slowMode = false;             // Tracks whether slow mode is held
    protected DrivePowers lastDrivePowers = DrivePowers.zero();  // Last powers sent to the follower (for telemetry)
    protected boolean aligning = false;             // Tracks whether goal alignment is active (toggled by gamepad1 back)
    protected boolean aligned = false;              // Tracks whether the robot is facing the goal tag
    protected int speedSteps = 0;                   // Gamepad1 bumper launcher adjustment this loop: +1 faster, -1 slower, 0 none
    protected int intakeSpeedSteps = 0;             // Gamepad2 bumper intake adjustment this loop: +1 faster, -1 slower, 0 none
    protected boolean pollenGatePending = false;    // Pollen launcher started with X; open its gate once at speed
    protected boolean nectarGatePending = false;    // Nectar launcher started with B; open its gate once at speed
    protected boolean pollenAtSpeed = false;        // Pollen launcher reached speed; waiting out RobotConstants.GATE_DELAY_MS
    protected boolean nectarAtSpeed = false;        // Nectar launcher reached speed; waiting out RobotConstants.GATE_DELAY_MS
    protected ElapsedTime pollenGateTimer = new ElapsedTime();  // Time since the pollen launcher reached speed
    protected ElapsedTime nectarGateTimer = new ElapsedTime();  // Time since the nectar launcher reached speed
    protected boolean drivingToShootingPosition = false;  // Following the path to the shooting position (both bumpers held)

    /**
     * Abstract method that subclasses must implement to provide alliance-specific configuration
     * @return AllianceConfig object with alliance-specific constants
     */
    protected abstract AllianceConfig getAllianceConfig();

    @Override
    public void runOpMode() throws InterruptedException {
        // INITIALIZATION PHASE (equivalent to init())
        // Get alliance-specific configuration
        allianceConfig = getAllianceConfig();

        // Initialize hardware components
        initializeHardware();

        // Init loop (equivalent to init_loop()): keep odometry live so the start pose can be checked
        String poseSource = PoseHolder.getCurrentPose() != null ? "Autonomous" : "Alliance home";
        while (opModeInInit()) {
            follower.update();

            Pose pose = follower.pose();
            telemetry.addData("Alliance", allianceConfig.getAllianceName());
            telemetry.addData("Status", "Initialized - Waiting for start");
            telemetry.addData("Pose Source", poseSource);
            telemetry.addData("Pose", String.format("(%.1f, %.1f, %.1f°)",
                pose.x(), pose.y(), Math.toDegrees(pose.heading())));
            telemetry.update();
        }

        // START PHASE (equivalent to start())
        if (isStopRequested()) return;

        // MAIN LOOP PHASE (equivalent to loop())
        while (opModeIsActive()) {
            // Update the follower to get current robot position and handle movement
            follower.update();

            // Cache fresh AprilTag data for this loop
            limelight.update();

            // MANUAL DRIVE CONTROL (GAMEPAD1)
            handleManualControl();

            // BUMPER SPEED ADJUSTMENT - gamepad1 for launchers, gamepad2 for intake
            readSpeedAdjustButtons();

            // LAUNCHER CONTROL (GAMEPAD1)
            handleLauncherControl();

            // INTAKE CONTROL (GAMEPAD1)
            handleIntakeControl();

            // GATE SERVO CONTROL (GAMEPAD1 dpad left = pollen, dpad right = nectar)
            handlePollenServo();
            handleNectarServo();

            handleShootingPosition();

            // TELEMETRY DISPLAY
            displayTelemetry();
        }

        // Final cleanup when the opmode stops
        stopAllMotors();
    }

    /**
     * Initialize hardware. Override (and call super) to add this robot's mechanisms.
     */
    protected void initializeHardware() {
        launcher = new LauncherUtil();
        launcher.init(hardwareMap);

        intake = new IntakeUtil();
        intake.init(hardwareMap);

        servos = new ServoUtil();
        servos.init(hardwareMap);

        // Track only this alliance's goal AprilTag
        limelight = new LimelightUtil(hardwareMap);
        limelight.setTargetTagId(-1);
        limelight.setClusterMode(true);
        // Blue: RobotConstants.BLUE_GOAL_APRILTAG_IDS, Red: RobotConstants.RED_GOAL_APRILTAG_IDS
        limelight.setTargetCluster(allianceConfig.getAllianceName() + " Goal",
                allianceConfig.getTargetAprilTagIds());

        // Create the follower object using our robot's hardware configuration
        follower = Constants.create(hardwareMap);

        // Check if we have a pose from autonomous mode, otherwise use alliance config
        Pose startingPose = PoseHolder.getCurrentPose();
        if (startingPose != null) {
            // Use the pose from autonomous mode
            follower.setPose(startingPose);
            telemetry.addData("Status", "Using pose from autonomous mode");
            telemetry.addData("Starting Pose", String.format("(%.1f, %.1f, %.1f°)",
                startingPose.x(), startingPose.y(), Math.toDegrees(startingPose.heading())));
        } else {
            // Use the default alliance config pose
            follower.setPose(allianceConfig.getAllianceHomePose());
            telemetry.addData("Status", "Using default alliance config pose");
        }

        // Update the follower to get current robot position
        follower.update();
    }

    /**
     * Handles manual driver control with speed adjustment
     */
    protected void handleManualControl() {
        // Slow mode while right trigger is held
        slowMode = gamepad1.right_trigger > RobotConstants.TRIGGER_DEADBAND;
        double speedMultiplier = slowMode ? RobotConstants.SLOW_MODE_MULTIPLIER : 1.0;

        // Gamepad Y is negative when pushed up and Pedro strafe is positive to the left, so both are negated
        double forward = -applyDeadzone(gamepad1.left_stick_y) * speedMultiplier;   // Forward/backward movement
        double strafe = -applyDeadzone(gamepad1.left_stick_x) * speedMultiplier;    // Left/right strafe movement
        double turn = -applyDeadzone(gamepad1.right_stick_x) * speedMultiplier;     // Rotation

        // Back toggles goal align on/off; moving the right stick cancels it so the driver can take over
        if (gamepad1.backWasPressed()) {
            aligning = !aligning;
        }
        if (aligning && turn != 0) {
            aligning = false;
        }
        if (aligning) {
            // Limelight takes over rotation to face the goal
            turn = handleAlignToGoal(turn);
        } else {
            aligned = false;
        }

        // Path to the shooting position has control of the drivetrain - don't override it with the sticks
        if (drivingToShootingPosition) {
            return;
        }

        // Robot centric - robot moves relative to its current orientation
        lastDrivePowers = new DrivePowers(forward, strafe, turn);
        follower.manual(lastDrivePowers);
    }

    /**
     * Turns the robot to face the alliance goal AprilTag using the Limelight horizontal offset (tx).
     * Proportional control on tx, clamped to RobotConstants.ALIGN_MAX_TURN_POWER.
     *
     * @param driverTurn the driver's turn input, used when no tag is visible
     * @return turn power to send to the follower
     */
    protected double handleAlignToGoal(double driverTurn) {
        if (!limelight.hasTarget()) {
            // No tag in view - leave rotation to the driver so they can find it
            aligned = false;
            return driverTurn;
        }

        double tx = limelight.getTx();
        aligned = Math.abs(tx) <= RobotConstants.ALIGN_TOLERANCE_DEG;
        if (aligned) {
            return 0.0;
        }

        // tx > 0 means the tag is to the right; negative turn power turns the robot right
        double turnPower = Math.min(RobotConstants.ALIGN_MAX_TURN_POWER,
            Math.max(RobotConstants.ALIGN_MIN_TURN_POWER, Math.abs(tx) * RobotConstants.ALIGN_KP));
        return -Math.signum(tx) * turnPower;
    }

    /**
     * Reads the bumper edges once per loop - gamepad1 bumpers adjust the launchers, gamepad2 bumpers the intake
     */
    protected void readSpeedAdjustButtons() {
        speedSteps = readBumperSteps(gamepad1);
        intakeSpeedSteps = readBumperSteps(gamepad2);
    }

    /**
     * Right bumper = faster (+1), left bumper = slower (-1); both pressed together cancel out
     */
    protected int readBumperSteps(Gamepad gamepad) {
        int steps = 0;
        if (gamepad.rightBumperWasPressed()) {
            steps++;
        }
        if (gamepad.leftBumperWasPressed()) {
            steps--;
        }
        return steps;
    }

    /**
     * Handles launcher control
     */
    protected void handleLauncherControl() {
        // Read every edge each loop so a stale press is never picked up later
        boolean xPressed = gamepad1.xWasPressed();
        boolean bPressed = gamepad1.bWasPressed();

        int speedChange = speedSteps * RobotConstants.LAUNCHER_SPEED_STEP;

        // Pollen launcher: X toggles on (near speed) / off; bumpers adjust speed while it is on
        if (xPressed) {
            if (launcher.isPollenOn()) {
                launcher.stopPollen();
                intake.stop();
                servos.openPollen();
                pollenGatePending = false;
                pollenAtSpeed = false;
            } else {
                launcher.setPollenSpeed(RobotConstants.CENTER_POLLEN_LAUNCH_SPEED);
                pollenGatePending = true;
            }
        } else if (speedChange != 0 && launcher.isPollenOn()) {
            launcher.setPollenSpeed(adjustLauncherSpeed(launcher.getPollenTargetSpeed(), speedChange));
        }

        // Nectar launcher: B toggles on (near speed) / off; bumpers adjust speed while it is on
        if (bPressed) {
            if (launcher.isNectarOn()) {
                launcher.stopNectar();
                intake.stop();
                servos.openNectar();
                nectarGatePending = false;
                nectarAtSpeed = false;
            } else {
                launcher.setNectarSpeed(RobotConstants.CENTER_NECTAR_LAUNCH_SPEED);
                nectarGatePending = true;
            }
        } else if (speedChange != 0 && launcher.isNectarOn()) {
            launcher.setNectarSpeed(adjustLauncherSpeed(launcher.getNectarTargetSpeed(), speedChange));
        }

        // Open each gate RobotConstants.GATE_DELAY_MS after its launcher (started with X / B) first reaches speed.
        // Non-blocking: the timer is checked every loop so driving keeps working during the wait
        if (pollenGatePending && !pollenAtSpeed
                && isLauncherAtSpeed(launcher.getPollenTargetSpeed(), launcher.getPollenVelocity())) {
            pollenAtSpeed = true;
            pollenGateTimer.reset();
        }
        if (pollenAtSpeed && pollenGateTimer.milliseconds() >= RobotConstants.GATE_DELAY_MS) {
            servos.closePollen();
            intake.intake();
            pollenGatePending = false;
            pollenAtSpeed = false;
        }

        if (nectarGatePending && !nectarAtSpeed
                && isLauncherAtSpeed(launcher.getNectarTargetSpeed(), launcher.getNectarVelocity())) {
            nectarAtSpeed = true;
            nectarGateTimer.reset();
        }
        if (nectarAtSpeed && nectarGateTimer.milliseconds() >= RobotConstants.GATE_DELAY_MS) {
            servos.closeNectar();
            intake.intake();
            nectarGatePending = false;
            nectarAtSpeed = false;
        }
    }

    /**
     * Adds speedChange to a running launcher's target speed, kept between one step and RobotConstants.LAUNCHER_MAX_SPEED
     * (never reaches 0, so slowing down never turns the launcher off - use X / B for that)
     */
    protected int adjustLauncherSpeed(int currentSpeed, int speedChange) {
        int newSpeed = currentSpeed + speedChange;
        return Math.max(RobotConstants.LAUNCHER_SPEED_STEP, Math.min(RobotConstants.LAUNCHER_MAX_SPEED, newSpeed));
    }

    /**
     * Handles intake control
     */
    protected void handleIntakeControl() {
        // Read every edge each loop so a stale press is never picked up later
        boolean intakePressed = gamepad2.yWasPressed();
        boolean outtakePressed = gamepad2.aWasPressed();

        // Direction comes from the sign, since bumpers can move the speed off the default values
        int currentSpeed = intake.getTargetSpeed();

        // Gamepad2 Y toggles intake on/off; switches straight from outtake to intake if outtake is running
        if (intakePressed) {
            if (currentSpeed > 0) {
                intake.stop();
            } else {
                intake.intake();
            }
        } else if (outtakePressed) {
            // Gamepad2 A toggles outtake on/off; switches straight from intake to outtake if intake is running
            if (currentSpeed < 0) {
                intake.stop();
            } else {
                intake.outtake();
            }
        } else if (intakeSpeedSteps != 0 && intake.isOn()) {
            // Gamepad2 bumpers adjust speed while intake or outtake is running
            intake.setSpeed(adjustIntakeSpeed(currentSpeed, intakeSpeedSteps * RobotConstants.INTAKE_SPEED_STEP));
        }
    }

    /**
     * Gamepad1 dpad left toggles the pollen gate servo open/closed
     */
    protected void handlePollenServo() {
        if (gamepad1.dpadLeftWasPressed()) {
            servos.togglePollen();
        }
    }

    /**
     * Gamepad1 dpad right toggles the nectar gate servo open/closed
     */
    protected void handleNectarServo() {
        if (gamepad1.dpadRightWasPressed()) {
            servos.toggleNectar();
        }
    }

    /**
     * Both gamepad1 bumpers held inside the shooting zone (RobotConstants.SHOOTING_ZONE_*) drives the robot to
     * the shooting position (RobotConstants.SHOOTING_POSITION_*). The zone is only checked when the drive starts;
     * releasing either bumper cancels it and hands the drivetrain back to the sticks.
     */
    protected void handleShootingPosition(){
        boolean bothBumpers = gamepad1.left_bumper && gamepad1.right_bumper;
        if(bothBumpers)
            telemetry.addData("Shooting Position", follower.isBusy() ? "DRIVING" : "ARRIVED");
    }

    /**
     * Adds speedChange to the running intake's speed magnitude, keeping its direction (intake or outtake).
     * Kept between one step and RobotConstants.INTAKE_ADJUST_MAX_SPEED (never reaches 0, so slowing down
     * never turns the intake off - use gamepad2 Y / A for that)
     */
    protected int adjustIntakeSpeed(int currentSpeed, int speedChange) {
        int newMagnitude = Math.abs(currentSpeed) + speedChange;
        newMagnitude = Math.max(RobotConstants.INTAKE_SPEED_STEP, Math.min(RobotConstants.INTAKE_ADJUST_MAX_SPEED, newMagnitude));
        return currentSpeed < 0 ? -newMagnitude : newMagnitude;
    }

    /**
     * Displays telemetry information
     */
    protected void displayTelemetry() {
        Pose currentPose = follower.pose();
        telemetry.addData("Alliance", allianceConfig.getAllianceName());
        telemetry.addData("Current Position", String.format("(%.1f, %.1f)", currentPose.x(), currentPose.y()));
        telemetry.addData("Current Heading", String.format("%.1f°", normalizeAngleTo360(Math.toDegrees(currentPose.heading()))));
        telemetry.addData("Slow Mode", slowMode ? "ACTIVE" : "OFF");
        telemetry.addData("Drive Powers", String.format("fwd:%.2f str:%.2f turn:%.2f",
            lastDrivePowers.forward(), lastDrivePowers.strafe(), lastDrivePowers.turn()));
        telemetry.addData("Raw Sticks", String.format("LX:%.2f LY:%.2f RX:%.2f",
            gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x));
        telemetry.addData("Goal Align", !aligning ? "OFF" : (!limelight.hasTarget() ? "NO TAG" : (aligned ? "ALIGNED" : "TURNING")));

        // Launcher status (each motor is controlled separately)
        telemetry.addData("Pollen Launcher", launcherStatus(launcher.isPollenOn(),
            launcher.getPollenTargetSpeed(), launcher.getPollenVelocity()));
        telemetry.addData("Nectar Launcher", launcherStatus(launcher.isNectarOn(),
            launcher.getNectarTargetSpeed(), launcher.getNectarVelocity()));

        // Intake status
        int intakeSpeed = intake.getTargetSpeed();
        telemetry.addData("Intake", intakeSpeed == RobotConstants.INTAKE_STOP_SPEED ? "OFF" :
            String.format("%s (%d)", intakeSpeed < 0 ? "REVERSE" : "ON", intakeSpeed));
        telemetry.addData("Intake Velocity", String.format("%.0f ticks/sec", intake.getVelocity()));

        // Gate servo status
        telemetry.addData("Pollen Gate", servos.isPollenOpen() ? "OPEN" : "CLOSED");
        telemetry.addData("Nectar Gate", servos.isNectarOpen() ? "OPEN" : "CLOSED");

        // Limelight status
        telemetry.addData("Limelight", limelight.getStatusString());
        telemetry.addData("Cluster Mode", limelight.isClusterMode() ? "ON" : "OFF");
        telemetry.addData("Target Cluster", "%s [%s]",
            limelight.getTargetClusterName() == null ? "Any" : limelight.getTargetClusterName(),
            allianceConfig.getTargetAprilTagIds());
        if (limelight.hasTarget()) {
            if (limelight.isTrackingCluster()) {
                telemetry.addData("Cluster", limelight.getTrackedClusterName());
                telemetry.addData("Tags Found", "%d%% %s",
                    limelight.getClusterPercentFound(), limelight.getClusterTagIds());
            }
            telemetry.addData("Target ID", limelight.getTrackedTagId());
            telemetry.addData("Goal Distance", String.format("%.1f in", limelight.getDistanceToTargetInches()));
            telemetry.addData("Goal Height", String.format("%.1f in", limelight.getHeightToTargetInches()));
            telemetry.addData("Goal tx", String.format("%.1f°", limelight.getTx()));
            telemetry.addData("Target Area", String.format("%.2f", limelight.getTargetArea()));
        }

        telemetry.addData("Gamepad1 Controls", String.format("X=Pollen On(%d)/Off, B=Nectar On/Off, RB=+%d, LB=-%d, RT=Slow, LB+RB=Shoot Pos, Back=Align On/Off, DpadL=Pollen Gate, DpadR=Nectar Gate",
            RobotConstants.NEAR_LAUNCH_SPEED, RobotConstants.LAUNCHER_SPEED_STEP, RobotConstants.LAUNCHER_SPEED_STEP));
        telemetry.addData("Intake Controls", String.format("GP2 Y=Intake On/Off, GP2 A=Outtake On/Off, GP2 RB=+%d, GP2 LB=-%d",
            RobotConstants.INTAKE_SPEED_STEP, RobotConstants.INTAKE_SPEED_STEP));
        telemetry.addData("Gamepad2 Controls", "Y=Intake, A=Outtake, RB/LB=Intake Speed");
        telemetry.update();
    }

    /**
     * Telemetry text for one launcher motor, e.g. "ON (2080) AT SPEED - 2075 ticks/sec"
     */
    protected String launcherStatus(boolean on, int targetSpeed, double velocity) {
        if (!on) {
            return String.format("OFF - %.0f ticks/sec", velocity);
        }
        return String.format("ON (%d) %s - %.0f ticks/sec", targetSpeed,
            isLauncherAtSpeed(targetSpeed, velocity) ? "AT SPEED" : "WAITING", velocity);
    }

    /**
     * Checks if a launcher motor velocity is within tolerance of the target speed
     * (asymmetric: RobotConstants.LAUNCHER_VELOCITY_TOLERANCE_LOWER below, _UPPER above)
     */
    protected boolean isLauncherAtSpeed(int targetSpeed, double velocity) {
        double lowerBound = targetSpeed * (1.0 - RobotConstants.LAUNCHER_VELOCITY_TOLERANCE_LOWER);
        double upperBound = targetSpeed * (1.0 + RobotConstants.LAUNCHER_VELOCITY_TOLERANCE_UPPER);
        return velocity >= lowerBound && velocity <= upperBound;
    }

    /**
     * Stop all mechanisms and the drivetrain. Override (and call super) to stop this robot's mechanisms.
     */
    protected void stopAllMotors() {
        launcher.stop();
        intake.stop();
        limelight.stop();
        follower.stop();
    }

    /**
     * Zero out small joystick values
     */
    protected double applyDeadzone(double value) {
        return Math.abs(value) < RobotConstants.STICK_DEADZONE ? 0.0 : value;
    }

    /**
     * Normalizes an angle to be within [0°, 360°) range for display purposes
     */
    protected double normalizeAngleTo360(double angleDegrees) {
        while (angleDegrees < 0) {
            angleDegrees += 360;
        }
        while (angleDegrees >= 360) {
            angleDegrees -= 360;
        }
        return angleDegrees;
    }
}
