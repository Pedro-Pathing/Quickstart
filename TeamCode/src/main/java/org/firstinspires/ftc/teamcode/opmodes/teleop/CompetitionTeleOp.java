package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.DriveCommands;
import org.firstinspires.ftc.teamcode.commands.DriveMixer;
import org.firstinspires.ftc.teamcode.commands.LaunchCommands;
import org.firstinspires.ftc.teamcode.commands.Requirements;
import org.firstinspires.ftc.teamcode.commands.RobotCommands;
import org.firstinspires.ftc.teamcode.commands.ShotReadiness;
import org.firstinspires.ftc.teamcode.drive.CollectAssist;
import org.firstinspires.ftc.teamcode.drive.TeleOpDrive;
import org.firstinspires.ftc.teamcode.flywheel.Flywheel;
import org.firstinspires.ftc.teamcode.gates.Gates;
import org.firstinspires.ftc.teamcode.helpers.BulkCache;
import org.firstinspires.ftc.teamcode.helpers.MatchState;
import org.firstinspires.ftc.teamcode.helpers.SystemStatus;
import org.firstinspires.ftc.teamcode.intake.Intake;
import org.firstinspires.ftc.teamcode.launch.LaunchModel;
import org.firstinspires.ftc.teamcode.launch.RpmTable;
import org.firstinspires.ftc.teamcode.launch.ScoringPosition;
import org.firstinspires.ftc.teamcode.launch.ShotQuality;
import org.firstinspires.ftc.teamcode.leds.Leds;
import org.firstinspires.ftc.teamcode.vision.Alliance;
import org.firstinspires.ftc.teamcode.vision.HiveAim;
import org.firstinspires.ftc.teamcode.vision.HiveTarget;
import org.firstinspires.ftc.teamcode.vision.LimelightCamera;

/**
 * The competition TeleOp, for two drivers. Gamepad 1 is the driver, gamepad 2 the gunner.
 *
 * Driver:
 *   Left stick       Drive                       Right stick  Turn (cancels the gunner's aim)
 *   L1 / R1 held     Crawl / sprint (else walk)  Circle       Field- / robot-centric
 *   Cross            Auto-collect (again, or any stick, cancels); lowers the camera
 *   R2               Intake in on/off            L2 held      Intake out
 *   Options held     Re-zero the field heading   Triangle     Cancel everything
 * Gunner:
 *   R2 / L2          Launch Pollen / Nectar: queued until ready (again cancels)
 *   R1 held          Fire everything             Square       Auto-aim on/off
 *   D-pad up / down  Camera to shooting / collecting
 * During INIT, d-pad left / right on either pad picks red / blue (the last auto's alliance if one ran).
 *
 * The rules between the two (who wins, who waits) come from the Ivy commands' requirements and
 * priorities (commands.Requirements), plus the few in rules() below. More Than Driving Test is the
 * one-gamepad version.
 */
@TeleOp(name = "Competition TeleOp", group = "TeleOp")
@Configurable
public class CompetitionTeleOp extends OpMode {

    /** Where the robot starts when no auto has run since the app started. */
    public static Pose startingPose = new Pose(72, 10.44, Math.toRadians(90));
    public static Alliance alliance = Alliance.RED;
    /** How long Options must be held to re-zero the field heading. */
    public static double reZeroHoldSeconds = 0.5;
    /** A stick moved further than this cancels an auto-collect. */
    public static double stickCancel = 0.2;

    // Rumbles (ms)
    public static int queuedRumbleMs = 120;
    public static int lockedRumbleMs = 300;
    public static int cancelledRumbleMs = 700;
    public static int collectOverRumbleMs = 300;

    /** The SDK's Gamepad.LED_DURATION_CONTINUOUS. */
    private static final int LED_CONTINUOUS = -1;

    private final SystemStatus status = new SystemStatus();
    private BulkCache bulkCache;
    private Follower follower;  // null (and drive too) when a drive device is missing
    private TeleOpDrive drive;
    private DriveMixer mixer;
    private TelemetryManager telemetryM;

    private Flywheel pollenLauncher;
    private Flywheel nectarLauncher;
    private RpmTable pollenRpmTable;
    private RpmTable nectarRpmTable;
    private Gates gates;

    private LimelightCamera camera;
    private HiveTarget hiveTarget;
    private final HiveAim hiveAim = new HiveAim();
    private double aimHeading;
    private final ShotReadiness readiness = new ShotReadiness();

    private Intake intake;
    private CollectAssist collect;  // null when the drive is disabled

    private Leds leds;
    private final ScoringPosition scoringPosition = new ScoringPosition();
    private ShotQuality shotQuality;

    // Commands
    private Command manualTranslation, headingLock, collectCommand, intakeIn, intakeOut, cameraUp, cameraDown;
    private DriveCommands.Aim aim;
    private LaunchCommands.Launch pollenLaunch, nectarLaunch;
    private LaunchCommands.FireAll fireAll;

    /** Set while the gunner cancels their own aim or launch, so it doesn't rumble as "cancelled". */
    private boolean gunnerCancelling = false;
    private final ElapsedTime optionsHeld = new ElapsedTime();
    private boolean optionsWasDown = false, reZeroed = false;
    private String lastFeedback = "";

    @Override
    public void init() {
        Scheduler.reset();
        bulkCache = new BulkCache(hardwareMap);
        leds = new Leds(hardwareMap);
        if (MatchState.alliance() != null) alliance = MatchState.alliance();
        Pose start = MatchState.pose() != null ? MatchState.pose() : startingPose;
        follower = TeleOpDrive.createFollower(hardwareMap, start, status);
        drive = follower != null ? new TeleOpDrive(follower) : null;
        mixer = drive != null ? new DriveMixer(drive) : null;
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        nectarLauncher = new Flywheel(hardwareMap, "NectarFlywheel", DcMotorSimple.Direction.FORWARD);
        pollenLauncher = new Flywheel(hardwareMap, "PollenFlywheel", DcMotorSimple.Direction.REVERSE);
        gates = new Gates(hardwareMap);
        camera = new LimelightCamera(hardwareMap, alliance);
        intake = new Intake(hardwareMap);
        collect = follower != null ? new CollectAssist(follower, camera, intake) : null;

        status.check("Nectar launcher", nectarLauncher.missingDevices());
        status.check("Pollen launcher", pollenLauncher.missingDevices());
        status.check("Feed gates", gates.missingDevices());
        status.check("Vision", camera.missingDevices());
        status.check("Floor vision (collect)", camera.floorMissingDevices());
        status.check("Intake", intake.missingDevices());
        status.check("LEDs", leds.missingDevices());

        pollenRpmTable = new RpmTable(LaunchModel.Launcher.POLLEN);
        nectarRpmTable = new RpmTable(LaunchModel.Launcher.NECTAR);

        buildCommands();

        telemetry.addData("Status", "Initialized. D-pad left/right picks red/blue.");
        status.addTelemetry(telemetry);
        telemetry.update();
    }

    private void buildCommands() {
        LaunchCommands.Conditions conditions = new LaunchCommands.Conditions(camera, readiness,
                () -> collectCommand != null && collectCommand.isScheduled());
        LaunchCommands.Launcher pollen = new LaunchCommands.Launcher("Pollen", pollenLauncher,
                gates::feedPollen, gates::isPollenOpen, Requirements.POLLEN_GATE);
        LaunchCommands.Launcher nectar = new LaunchCommands.Launcher("Nectar", nectarLauncher,
                gates::feedNectar, gates::isNectarOpen, Requirements.NECTAR_GATE);
        pollenLaunch = new LaunchCommands.Launch(pollen, conditions, this::gunnerCommandEnded);
        nectarLaunch = new LaunchCommands.Launch(nectar, conditions, this::gunnerCommandEnded);
        fireAll = new LaunchCommands.FireAll(conditions, () -> gamepad2.right_bumper, pollen, nectar);

        intakeIn = RobotCommands.intakeIn(intake);
        intakeOut = RobotCommands.intakeOut(intake, () -> gamepad1.left_trigger > 0.5);
        cameraUp = RobotCommands.moveCamera(camera, LimelightCamera.Mode.TAGS);
        cameraDown = RobotCommands.moveCamera(camera, LimelightCamera.Mode.FLOOR);

        if (drive != null) {
            manualTranslation = DriveCommands.manualTranslation(drive, mixer);
            headingLock = DriveCommands.headingLock(drive, mixer);
            aim = new DriveCommands.Aim(drive, mixer, follower, hiveAim, () -> aimHeading, camera, this::gunnerCommandEnded);
        }
        if (collect != null) {
            collectCommand = RobotCommands.collect(collect, end -> feedback(gamepad1, collectOverRumbleMs, "Driver: collect over"));
        }
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed()) alliance = Alliance.RED;
        if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed()) alliance = Alliance.BLUE;
        double[] colour = allianceColour();
        gamepad1.setLedColor(colour[0], colour[1], colour[2], LED_CONTINUOUS);
        gamepad2.setLedColor(colour[0], colour[1], colour[2], LED_CONTINUOUS);
        telemetry.addData("Alliance", alliance + (MatchState.alliance() != null ? " (from the auto)" : ""));
        telemetry.addData("Start", MatchState.pose() != null ? "where the auto finished" : "the default pose");
        status.addTelemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void start() {
        bulkCache.clear();
        nectarLauncher.reset();
        pollenLauncher.reset();
        if (drive != null) {
            drive.start();
            Scheduler.schedule(manualTranslation, headingLock);
        }
    }

    @Override
    public void loop() {
        bulkCache.clear();
        camera.setAlliance(alliance);
        hiveTarget = camera.findTarget();

        Pose pose = follower != null ? follower.pose() : null;
        nectarLauncher.runToRPM(nectarRpmTable.rpmFor(hiveTarget, alliance, pose));
        pollenLauncher.runToRPM(pollenRpmTable.rpmFor(hiveTarget, alliance, pose));
        gates.update();

        if (follower != null) {
            follower.update();
            pose = follower.pose();
            shotQuality = scoringPosition.update(alliance, pose.x(), pose.y());
            leds.setState(shotQuality.ledState);
            // Every loop, so they have the robot's recent poses when an aim or collect starts
            aimHeading = hiveAim.update(alliance, hiveTarget, pose);
            collect.update(camera.findBalls(), alliance);
            drive.readDriver(gamepad1);
        }
        readiness.update(camera, hiveTarget);
        leds.update();

        rules();
        Scheduler.execute();
        if (mixer != null) mixer.flush();
        showLightBars();

        if (telemetryM.getShouldUpdateLines()) {
            showTelemetry();
        }
    }

    /** The buttons, and the rules the commands' priorities don't cover. */
    private void rules() {
        boolean collecting = collectCommand != null && collectCommand.isScheduled();

        // Driver
        if (collectCommand != null && gamepad1.crossWasPressed()) {
            if (collecting) {
                collectCommand.cancel();
            } else {
                // The driver's collect wins: it ends an aim (by priority) and drops queued launches
                cancelLaunches(false);
                Scheduler.schedule(collectCommand);
            }
        } else if (collecting && sticksMoved()) {
            collectCommand.cancel();
        }
        if (aim != null && aim.isScheduled() && drive.driverTurning()) {
            aim.cancel();  // the driver takes the heading back; the gunner feels it
        }
        if (gamepad1.rightTriggerWasPressed()) {
            if (intakeIn.isScheduled()) {
                intakeIn.cancel();
                raiseCameraIfFree();
            } else {
                Scheduler.schedule(intakeIn);
            }
        }
        if (gamepad1.leftTriggerWasPressed()) Scheduler.schedule(intakeOut);
        if (gamepad1.triangleWasPressed()) cancelEverything();
        reZeroHeading();

        // Gunner
        if (gamepad2.rightTriggerWasPressed()) toggleLaunch(pollenLaunch);
        if (gamepad2.leftTriggerWasPressed()) toggleLaunch(nectarLaunch);
        if (gamepad2.rightBumperWasPressed() && !fireAll.isScheduled()) Scheduler.schedule(fireAll);
        if (aim != null && gamepad2.squareWasPressed()) {
            if (aim.isScheduled()) {
                gunnerCancel(aim);
            } else {
                Scheduler.schedule(aim);
            }
        }
        if (gamepad2.dpadUpWasPressed()) Scheduler.schedule(cameraUp);  // ends a collect (higher priority)
        if (gamepad2.dpadDownWasPressed()) {
            // Lowering it cancels the gunner's own aim and launches
            if (aim != null && aim.isScheduled()) gunnerCancel(aim);
            cancelLaunches(true);
            Scheduler.schedule(cameraDown);
        }
    }

    private void toggleLaunch(LaunchCommands.Launch launch) {
        if (launch.isScheduled()) {
            gunnerCancel(launch);
        } else {
            Scheduler.schedule(launch);
            feedback(gamepad2, queuedRumbleMs, "Gunner: launch queued");
        }
    }

    /** Cancel queued single launches and fire-everything. Quietly if it's the gunner's doing. */
    private void cancelLaunches(boolean byGunner) {
        for (Command launch : new Command[]{pollenLaunch, nectarLaunch}) {
            if (launch.isScheduled()) {
                if (byGunner) gunnerCancel(launch); else launch.cancel();
            }
        }
        if (fireAll.isScheduled()) fireAll.cancel();
    }

    private void gunnerCancel(Command command) {
        gunnerCancelling = true;
        command.cancel();
        gunnerCancelling = false;
    }

    /** An aim or launch ended: the gunner feels it if someone else cancelled it. */
    private void gunnerCommandEnded(EndCondition end) {
        if (end == EndCondition.INTERRUPTED && !gunnerCancelling) {
            feedback(gamepad2, cancelledRumbleMs, "Gunner: cancelled");
        }
    }

    private void cancelEverything() {
        if (collectCommand != null && collectCommand.isScheduled()) collectCommand.cancel();
        if (aim != null && aim.isScheduled()) aim.cancel();
        cancelLaunches(false);
        if (intakeIn.isScheduled()) intakeIn.cancel();
        if (intakeOut.isScheduled()) intakeOut.cancel();
        intake.stop();
        raiseCameraIfFree();
    }

    /** Nothing needs the floor view: put the camera up, so the flywheels get the tags' distance. */
    private void raiseCameraIfFree() {
        boolean collecting = collectCommand != null && collectCommand.isScheduled();
        if (!collecting && !cameraDown.isScheduled()) camera.setMode(LimelightCamera.Mode.TAGS);
    }

    private void reZeroHeading() {
        if (drive == null) return;
        if (gamepad1.options) {
            if (!optionsWasDown) {
                optionsHeld.reset();
                reZeroed = false;
            }
            if (!reZeroed && optionsHeld.seconds() >= reZeroHoldSeconds) {
                drive.resetFieldHeading();
                reZeroed = true;
                feedback(gamepad1, queuedRumbleMs, "Driver: field heading re-zeroed");
            }
        }
        optionsWasDown = gamepad1.options;
    }

    private boolean sticksMoved() {
        return Math.abs(gamepad1.left_stick_x) > stickCancel || Math.abs(gamepad1.left_stick_y) > stickCancel
                || Math.abs(gamepad1.right_stick_x) > stickCancel;
    }

    private void feedback(Gamepad gamepad, int rumbleMs, String what) {
        gamepad.rumble(rumbleMs);
        lastFeedback = what;
    }

    // Light bars: the gunner's shows the aim and launches, the driver's the collect and intake
    private boolean wasLocked = false;

    private void showLightBars() {
        boolean locked = aim != null && aim.isLocked();
        if (locked && !wasLocked) feedback(gamepad2, lockedRumbleMs, "Gunner: aim locked");
        wasLocked = locked;

        double[] gunner;
        if (locked) gunner = new double[]{0, 1, 0};
        else if (aim != null && aim.isScheduled()) gunner = new double[]{0, 0.4, 1};
        else if (pollenLaunch.isWaiting() || nectarLaunch.isWaiting() || fireAll.isScheduled()) gunner = new double[]{1, 0.6, 0};
        else gunner = dim(allianceColour());
        gamepad2.setLedColor(gunner[0], gunner[1], gunner[2], LED_CONTINUOUS);

        double[] driver;
        if (collectCommand != null && collectCommand.isScheduled()) driver = new double[]{0.6, 0, 1};
        else if (intakeOut.isScheduled()) driver = new double[]{1, 0.3, 0};
        else if (intakeIn.isScheduled()) driver = new double[]{0, 1, 0.6};
        else driver = dim(allianceColour());
        gamepad1.setLedColor(driver[0], driver[1], driver[2], LED_CONTINUOUS);
    }

    private double[] allianceColour() {
        return alliance == Alliance.RED ? new double[]{1, 0, 0} : new double[]{0, 0, 1};
    }

    private static double[] dim(double[] colour) {
        return new double[]{colour[0] * 0.3, colour[1] * 0.3, colour[2] * 0.3};
    }

    @Override
    public void stop() {
        Scheduler.reset();
        intake.stop();
        camera.stop();
    }

    private void showTelemetry() {
        status.addTelemetry(telemetryM);
        telemetryM.addData("Alliance", alliance);
        telemetryM.addData("Driver", collectCommand != null && collectCommand.isScheduled() ? "COLLECTING"
                : intakeOut.isScheduled() ? "INTAKE OUT" : intakeIn.isScheduled() ? "INTAKE IN" : "DRIVING");
        telemetryM.addData("Gunner", aim != null && aim.isLocked() ? "AIM LOCKED" : aim != null && aim.isScheduled() ? "AIMING" : "IDLE");
        if (aim != null && aim.isScheduled()) {
            double error = Math.toDegrees(com.pedropathing.utils.Angle.normalizeSigned(aimHeading - follower.pose().heading()));
            telemetryM.addData("Aim", String.format("%s, %.1f deg off, %.2f rad/s",
                    hiveAim.isFromVision() ? "camera" : "odometry", error, follower.twist().omega));
        }
        telemetryM.addData("Pollen Launch", launchState(pollenLaunch));
        telemetryM.addData("Nectar Launch", launchState(nectarLaunch));
        telemetryM.addData("Fire All", fireAll.isScheduled() ? "FIRING (" + fireAll.feeds() + " fed)" : "OFF");
        telemetryM.addData("Camera", camera.getMode() + (camera.isSettled() ? "" : " (moving)"));
        telemetryM.addData("Shot Distance", !readiness.isReady() ? "waiting" : readiness.fromTags() ? "tags" : "odometry");
        telemetryM.addData("Last Feedback", lastFeedback);
        telemetryM.addData("Shot Position", shotQuality);
        telemetryM.addData("Tags Seen", hiveTarget != null ? hiveTarget.tagCount : 0);
        if (collect != null) collect.addTelemetry(telemetryM);
        if (drive != null) drive.addTelemetry(telemetryM);
        telemetryM.addData("pollen.ready", pollenLauncher.isReady());
        telemetryM.addData("nectar.ready", nectarLauncher.isReady());
        telemetryM.update(telemetry);
    }

    private static String launchState(LaunchCommands.Launch launch) {
        return launch.isWaiting() ? "QUEUED" : launch.isScheduled() ? "FIRING" : "IDLE";
    }
}
