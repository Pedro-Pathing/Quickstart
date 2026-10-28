package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.TeleOpDrive;
import org.firstinspires.ftc.teamcode.flywheel.Flywheel;
import org.firstinspires.ftc.teamcode.gates.Gates;
import org.firstinspires.ftc.teamcode.helpers.BulkCache;
import org.firstinspires.ftc.teamcode.helpers.SystemStatus;
import org.firstinspires.ftc.teamcode.launch.LaunchModel;
import org.firstinspires.ftc.teamcode.launch.RpmTable;
import org.firstinspires.ftc.teamcode.launch.ScoringPosition;
import org.firstinspires.ftc.teamcode.launch.ShotQuality;
import org.firstinspires.ftc.teamcode.leds.Leds;
import org.firstinspires.ftc.teamcode.vision.Alliance;
import org.firstinspires.ftc.teamcode.vision.HiveCells;
import org.firstinspires.ftc.teamcode.vision.HiveTarget;
import org.firstinspires.ftc.teamcode.vision.LimelightCamera;

/**
 * Driving plus launching. Drive controls are TeleOpDrive's (heading lock, circle for
 * robot/field-centric, bumpers for slow and sprint), and:
 *
 * Square          Auto-aim at our HIVE until the right stick is turned
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

    /** Launch speed used when the camera can't see our tags. */
    private static final double DEFAULT_LAUNCH_SPEED_MPS = 6.0;

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
    /** Whether this auto-aim has had a heading from the camera yet. */
    private boolean aimFromVision = false;

    private Leds leds;
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

        status.check("Nectar launcher", nectarLauncher.missingDevices());
        status.check("Pollen launcher", pollenLauncher.missingDevices());
        status.check("Feed gates", gates.missingDevices());
        status.check("Vision", camera.missingDevices());
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
        double defaultRPM = LaunchModel.rpmForSpeed(DEFAULT_LAUNCH_SPEED_MPS);
        nectarLauncher.runToRPM(hiveTarget != null ? nectarRpmTable.rpmFor(hiveTarget) : defaultRPM);
        pollenLauncher.runToRPM(hiveTarget != null ? pollenRpmTable.rpmFor(hiveTarget) : defaultRPM);
        gates.update();

        // How good this spot is for scoring both balls, shown on the LEDs (needs odometry)
        ShotQuality shotQuality = null;
        if (follower != null) {
            follower.update();
            shotQuality = scoringPosition.update(alliance, follower.pose().x(), follower.pose().y());
            leds.setState(shotQuality.ledState);
        }
        leds.update();

        // Only open a gate when its flywheel is at speed; presses while it isn't are ignored
        if (gamepad1.rightTriggerWasPressed() && pollenLauncher.isReady()) {
            gates.feedPollen();
        }
        if (gamepad1.leftTriggerWasPressed() && nectarLauncher.isReady()) {
            gates.feedNectar();
        }

        if (drive != null) {
            if (gamepad1.squareWasPressed()) {
                aimFromVision = false;
                drive.startAiming();
            }
            drive.update(gamepad1, this::aimHeading);
        }

        // Telemetry is only built when Panels is due to send it, to keep the loop fast
        if (telemetryM.getShouldUpdateLines()) {
            showTelemetry(shotQuality);
        }
    }

    @Override
    public void stop() {
        camera.stop();
    }

    /** The heading auto-aim holds this loop. */
    private double aimHeading() {
        if (hiveTarget != null) {
            // Tags in view: point straight at the CELL's open face
            aimFromVision = true;
            return hiveTarget.targetHeading(follower.pose().heading());
        }
        if (!aimFromVision) {
            // No tags yet: point roughly at our nearest CELL using odometry
            return HiveCells.headingToNearestCell(alliance, follower.pose().x(), follower.pose().y());
        }
        // Once the camera has given a heading, keep it if the tags drop out rather than going back to the rough one
        return drive.getTargetHeading();
    }

    private void showTelemetry(ShotQuality shotQuality) {
        status.addTelemetry(telemetryM);
        telemetryM.addData("Alliance", alliance);
        if (drive != null) {
            telemetryM.addData("Auto Aim", !drive.isAiming() ? "Off" : (aimFromVision ? "Vision" : "Odometry"));
            telemetryM.addData("Shot Position", shotQuality);
            telemetryM.addData("Shot Chance (%)", 100 * scoringPosition.lastChance());
        }
        telemetryM.addData("Tags Seen", hiveTarget != null ? hiveTarget.tagCount : 0);
        if (hiveTarget != null) {
            telemetryM.addData("Target CELL", hiveTarget.cell);
            telemetryM.addData("Target Distance (m)", hiveTarget.horizontalDistance());
            telemetryM.addData("Target Bearing", Math.toDegrees(hiveTarget.bearing()));
        }
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
