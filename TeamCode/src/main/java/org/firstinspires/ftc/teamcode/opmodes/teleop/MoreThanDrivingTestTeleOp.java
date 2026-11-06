package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.CollectAssist;
import org.firstinspires.ftc.teamcode.drive.TeleOpDrive;
import org.firstinspires.ftc.teamcode.flywheel.Flywheel;
import org.firstinspires.ftc.teamcode.gates.Gates;
import org.firstinspires.ftc.teamcode.helpers.BulkCache;
import org.firstinspires.ftc.teamcode.helpers.SystemStatus;
import org.firstinspires.ftc.teamcode.intake.Intake;
import org.firstinspires.ftc.teamcode.launch.LaunchModel;
import org.firstinspires.ftc.teamcode.launch.RpmTable;
import org.firstinspires.ftc.teamcode.launch.ScoringPosition;
import org.firstinspires.ftc.teamcode.launch.ShotQuality;
import org.firstinspires.ftc.teamcode.leds.Leds;
import org.firstinspires.ftc.teamcode.leds.RobotState;
import org.firstinspires.ftc.teamcode.vision.Alliance;
import org.firstinspires.ftc.teamcode.vision.HiveAim;
import org.firstinspires.ftc.teamcode.vision.HiveTarget;
import org.firstinspires.ftc.teamcode.vision.LimelightCamera;

/**
 * Driving plus launching. Drive controls are TeleOpDrive's (heading lock, circle for
 * robot/field-centric, bumpers for slow and sprint), and:
 *
 * Square          Auto-aim at our HIVE until the right stick is turned
 * Cross           Collect the nearest Pollen or Nectar of ours (CollectAssist); press again, or move
 *                 a stick, to cancel
 * Right trigger   Feed a Pollen (only when its flywheel is at speed)
 * Left trigger    Feed a Nectar (only when its flywheel is at speed)
 *
 * The flywheels always run, at the RPM for the camera's distance to the CELL, or the default speed
 * when no tags are in view. The LEDs show how good the robot's position is for scoring.
 *
 * Any part whose devices are missing is disabled and listed on telemetry, and the rest carries on:
 * e.g. with no Limelight the flywheels use the default speed and auto-aim uses odometry only.
 */
@TeleOp(name = "More Than Driving Test", group = "TeleOp")
@Configurable
public class MoreThanDrivingTestTeleOp extends OpMode {

    public static Pose startingPose = new Pose(72, 10.44, Math.toRadians(90));
    /** Which HIVE to aim at. */
    public static Alliance alliance = Alliance.RED;

    /** While a launcher has a fault, the LEDs flash LAUNCHER_FAULT this often, for FAULT_FLASH_SECONDS. */
    private static final double FAULT_FLASH_INTERVAL_SECONDS = 2.0;
    private static final double FAULT_FLASH_SECONDS = 0.3;

    private final SystemStatus status = new SystemStatus();
    private BulkCache bulkCache;
    private Follower follower;  // null (and drive too) when a drive device is missing
    private TeleOpDrive drive;
    private TelemetryManager telemetryM;

    private Flywheel pollenLauncher;
    private Flywheel nectarLauncher;
    // RPM against distance for each launcher, worked out with the flight model at INIT
    private RpmTable pollenRpmTable;
    private RpmTable nectarRpmTable;
    private Gates gates;

    private LimelightCamera camera;
    private HiveTarget hiveTarget;
    private final HiveAim hiveAim = new HiveAim();

    private Intake intake;
    private CollectAssist collect;  // null when the drive is disabled
    private boolean wasCollecting = false;

    private Leds leds;
    private final ElapsedTime faultFlashTimer = new ElapsedTime();
    private final ScoringPosition scoringPosition = new ScoringPosition();

    @Override
    public void init() {
        bulkCache = new BulkCache(hardwareMap);
        leds = new Leds(hardwareMap);  // shows yellow: ready to drive
        follower = TeleOpDrive.createFollower(hardwareMap, startingPose, status);
        drive = follower != null ? new TeleOpDrive(follower) : null;

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

        // Takes about a second on the Control Hub
        pollenRpmTable = new RpmTable(LaunchModel.Launcher.POLLEN);
        nectarRpmTable = new RpmTable(LaunchModel.Launcher.NECTAR);

        telemetry.addData("Status", "Initialized. Circle toggles robot/field-centric.");
        status.addTelemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void start() {
        bulkCache.clear();
        if (drive != null) drive.start();
        nectarLauncher.reset();
        pollenLauncher.reset();
    }

    @Override
    public void loop() {
        bulkCache.clear();

        camera.setAlliance(alliance);
        hiveTarget = camera.findTarget();

        // Flywheel speeds from the distance to the CELL, or the default speed when no tags are in view
        Pose pose = follower != null ? follower.pose() : null;
        nectarLauncher.runToRPM(nectarRpmTable.rpmFor(hiveTarget, alliance, pose));
        pollenLauncher.runToRPM(pollenRpmTable.rpmFor(hiveTarget, alliance, pose));
        gates.update();

        // How good this spot is for scoring both balls, shown on the LEDs (needs odometry)
        ShotQuality shotQuality = null;
        if (follower != null) {
            follower.update();
            shotQuality = scoringPosition.update(alliance, follower.pose().x(), follower.pose().y());
            leds.setState(shotQuality.ledState);
        }
        if (hasLauncherFault() && faultFlashTimer.seconds() > FAULT_FLASH_INTERVAL_SECONDS) {
            leds.flash(RobotState.LAUNCHER_FAULT, FAULT_FLASH_SECONDS);
            faultFlashTimer.reset();
        }
        leds.update();

        // Only open a gate when its flywheel is at speed; presses while it isn't are ignored
        if (gamepad1.rightTriggerWasPressed() && pollenLauncher.isReady()) {
            gates.feedPollen();
            pollenLauncher.shotFired();
        }
        if (gamepad1.leftTriggerWasPressed() && nectarLauncher.isReady()) {
            gates.feedNectar();
            nectarLauncher.shotFired();
        }

        // Cross collects the nearest ball of ours; cross again or moving a stick cancels
        if (collect != null) {
            if (gamepad1.crossWasPressed()) {
                if (collect.isActive()) collect.cancel(); else collect.start();
            } else if (collect.isActive() && sticksMoved()) {
                collect.cancel();
            }
            collect.update(camera.findBalls(), alliance);
        }
        boolean collecting = collect != null && collect.isActive();

        if (drive != null && !collecting) {
            // After a collect, hold the heading it left the robot at
            if (wasCollecting) drive.start();
            if (gamepad1.squareWasPressed()) {
                hiveAim.reset();
                drive.startAiming();
            }
        }
        // Every loop, so it has the robot's recent poses when an aim starts
        double aimHeading = follower != null ? hiveAim.update(alliance, hiveTarget, follower.pose()) : 0.0;
        if (drive != null && !collecting) {
            drive.update(gamepad1, () -> aimHeading);
        }
        wasCollecting = collecting;

        // Telemetry is only built when Panels is due to send it, to keep the loop fast
        if (telemetryM.getShouldUpdateLines()) {
            showTelemetry(shotQuality);
        }
    }

    @Override
    public void stop() {
        camera.stop();
    }

    private boolean sticksMoved() {
        return Math.abs(gamepad1.left_stick_x) > 0.2 || Math.abs(gamepad1.left_stick_y) > 0.2
                || Math.abs(gamepad1.right_stick_x) > 0.2;
    }

    private boolean hasLauncherFault() {
        return pollenLauncher.getHealth() != Flywheel.Health.OK || nectarLauncher.getHealth() != Flywheel.Health.OK;
    }

    private void showTelemetry(ShotQuality shotQuality) {
        status.addTelemetry(telemetryM);
        if (pollenLauncher.getHealth() != Flywheel.Health.OK) {
            telemetryM.addLine("Pollen launcher FAULT: " + pollenLauncher.faultMessage());
        }
        if (nectarLauncher.getHealth() != Flywheel.Health.OK) {
            telemetryM.addLine("Nectar launcher FAULT: " + nectarLauncher.faultMessage());
        }
        telemetryM.addData("Alliance", alliance);
        if (drive != null) {
            telemetryM.addData("Auto Aim", !drive.isAiming() ? "Off" : (hiveAim.isFromVision() ? "Vision" : "Odometry"));
            telemetryM.addData("Shot Position", shotQuality);
            telemetryM.addData("Shot Chance (%)", 100 * scoringPosition.lastChance());
        }
        telemetryM.addData("Tags Seen", hiveTarget != null ? hiveTarget.tagCount : 0);
        if (hiveTarget != null) {
            telemetryM.addData("Target CELL", hiveTarget.cell);
            telemetryM.addData("Target Distance (m)", hiveTarget.horizontalDistance());
            telemetryM.addData("Target Bearing", Math.toDegrees(hiveTarget.bearing()));
        }
        if (collect != null) collect.addTelemetry(telemetryM);
        if (drive != null) drive.addTelemetry(telemetryM);

        telemetryM.addData("nectar.targetRPM", nectarLauncher.targetRPM());
        telemetryM.addData("nectar.filteredRPM", nectarLauncher.filteredRPM());
        telemetryM.addData("nectar.power", nectarLauncher.power());
        telemetryM.addData("nectar.ready", nectarLauncher.isReady());
        telemetryM.addData("pollen.targetRPM", pollenLauncher.targetRPM());
        telemetryM.addData("pollen.filteredRPM", pollenLauncher.filteredRPM());
        telemetryM.addData("pollen.power", pollenLauncher.power());
        telemetryM.addData("pollen.ready", pollenLauncher.isReady());
        telemetryM.update(telemetry);
    }
}
