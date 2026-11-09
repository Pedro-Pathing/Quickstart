package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.repeat;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.flywheel.Flywheel;
import org.firstinspires.ftc.teamcode.gates.Gates;
import org.firstinspires.ftc.teamcode.helpers.BulkCache;
import org.firstinspires.ftc.teamcode.helpers.MatchState;
import org.firstinspires.ftc.teamcode.launch.LaunchModel;
import org.firstinspires.ftc.teamcode.launch.RpmTable;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.vision.Alliance;
import org.firstinspires.ftc.teamcode.vision.HiveAim;
import org.firstinspires.ftc.teamcode.vision.HiveTarget;
import org.firstinspires.ftc.teamcode.vision.LimelightCamera;

/**
 * Drives path 1, then aims at our HIVE (HiveAim, as the TeleOp's auto-aim does) while holding the
 * end of the path, and fires pollenShots Pollen. Each shot waits until the Pollen flywheel is at
 * speed and the heading has settled on the aim.
 *
 * The Pollen flywheel runs from the start: at the default speed until the camera sees our tags, then
 * at the RPM for the distance to the CELL.
 */
@Autonomous(name = "Test Auto", group = "Autonomous")
@Configurable
public class TestAuto extends LinearOpMode {

    /** Which HIVE to aim at. */
    public static Alliance alliance = Alliance.RED;
    public static int pollenShots = 4;
    /** After each feed, wait this long before the next shot can fire, so the ball clears and the gate closes. */
    public static double shotIntervalMs = 500;

    private Follower follower;
    private BulkCache bulkCache;
    private Flywheel pollenLauncher;
    private RpmTable pollenRpmTable;
    private Gates gates;
    private LimelightCamera camera;
    private HiveTarget hiveTarget;
    private final HiveAim hiveAim = new HiveAim();

    /** Whether the robot is holding the end of path 1 and aiming at the HIVE. */
    private boolean aiming = false;
    private double aimHeading = 0.0;
    private int shotsFired = 0;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(8.3, 23, 180);
    private final Pose path1 = poseFactory.of(57.1444, 10.3322, 90);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                lazy(() -> follow(follower, path1())),
                instant(this::startAiming),
                repeat(sequential(
                        waitUntil(this::readyToFire),
                        instant(this::firePollen),
                        waitMs(shotIntervalMs)
                ), pollenShots)
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        bulkCache = new BulkCache(hardwareMap);
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        pollenLauncher = new Flywheel(hardwareMap, "PollenFlywheel", DcMotorSimple.Direction.REVERSE);
        gates = new Gates(hardwareMap);
        camera = new LimelightCamera(hardwareMap, alliance);
        // Takes about a second on the Control Hub
        pollenRpmTable = new RpmTable(LaunchModel.Launcher.POLLEN);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        bulkCache.clear();
        pollenLauncher.reset();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            bulkCache.clear();

            camera.setAlliance(alliance);
            hiveTarget = camera.findTarget();
            pollenLauncher.runToRPM(pollenRpmTable.rpmFor(hiveTarget, alliance, follower.pose()));
            gates.update();

            follower.update();
            Scheduler.execute();
            updateAim();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            if (aiming) {
                telemetry.addData("Aim", hiveAim.isFromVision() ? "Vision" : "Odometry");
                telemetry.addData("Aim error (deg)", Math.toDegrees(headingError()));
                telemetry.addData("Shots fired", "%d of %d", shotsFired, pollenShots);
            }
            telemetry.addData("Pollen RPM", "%.0f / %.0f", pollenLauncher.filteredRPM(), pollenLauncher.targetRPM());
            telemetry.addData("Pollen ready", pollenLauncher.isReady());

            telemetry.update();
        }

        MatchState.save(alliance, follower.pose());
        camera.stop();
    }

    /**
     * Path 1 from where the robot actually is when it starts (not exactly where it was placed) to the
     * planned end of path 1. Built when the step starts, not when the routine is.
     */
    public Path path1() {
        Pose from = follower.pose();
        return Paths.line(from, path1).linear(from, path1);
    }

    private void startAiming() {
        hiveAim.reset();
        aiming = true;
        updateAim();
    }

    /**
     * This loop's aim heading (every loop, so the aim has the robot's recent poses), and while aiming,
     * hold the end of path 1 turned to it.
     */
    private void updateAim() {
        aimHeading = hiveAim.update(alliance, hiveTarget, follower.pose());
        if (aiming) follower.hold(new Pose(path1.x(), path1.y(), aimHeading));
    }

    private boolean readyToFire() {
        return pollenLauncher.isReady()
                && HiveAim.isLocked(headingError(), follower.twist().omega);
    }

    private void firePollen() {
        gates.feedPollen();
        pollenLauncher.shotFired();
        shotsFired++;
    }

    /** Aim heading minus the robot's heading, wrapped to -pi..pi. */
    private double headingError() {
        double error = aimHeading - follower.pose().heading();
        return Math.atan2(Math.sin(error), Math.cos(error));
    }
}
