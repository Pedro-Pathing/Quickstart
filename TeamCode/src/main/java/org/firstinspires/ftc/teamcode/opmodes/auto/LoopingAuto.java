package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import com.pedropathing.ivy.groups.Groups;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.repeat;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.flywheel.Flywheel;
import org.firstinspires.ftc.teamcode.gates.Gates;
import org.firstinspires.ftc.teamcode.helpers.BulkCache;
import org.firstinspires.ftc.teamcode.launch.LaunchModel;
import org.firstinspires.ftc.teamcode.launch.RpmTable;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.vision.Alliance;
import org.firstinspires.ftc.teamcode.vision.HiveAim;
import org.firstinspires.ftc.teamcode.vision.HiveTarget;
import org.firstinspires.ftc.teamcode.vision.LimelightCamera;

/**
 * An Autonomous that keeps playing until stopped, for exercising the sim with four robots and no
 * drivers (the multiplayer server's stress test). Each alliance station's OpMode subclasses it with
 * its own start and its own quarter of the field.
 *
 * It drives to its shooting spot and fires the preloads (the robot whose CELL starts face down,
 * Red 2's and Blue 2's, waits partnerDelayMs first, for its partner's volley to tip the HIVE and
 * turn that CELL up: red's audience CELL and blue's scoring CELL start up). Then it takes the four
 * Pollen from the FLOWER in its quarter: in front first, pushing against it, backing off and
 * pushing again. It goes back and fires them, then loops: drive the collecting loop through its
 * quarter (the intake leading), back to the shooting spot and fire a volley. A volley aims at our
 * HIVE (HiveAim, as TestAuto does) and feeds both launchers together volleyFeeds times, each time
 * both flywheels are at speed, or until shootTimeoutMs: a feed with nothing of that kind stored does
 * nothing. Every drive has a timeout, so a robot pushed off its path by another carries on with the
 * next step.
 *
 * Starts follow G304: touching the perimeter wall, on our own side, clear of the LOADING ZONE and the
 * FLOWERS. Pedro field frame, inches: +x towards blue, +y away from the audience. The robot's back
 * is 8.9 in from its centre, its sides 8.9 in.
 */
public abstract class LoopingAuto extends LinearOpMode {

    public static int volleyFeeds = 4;
    public static double shootTimeoutMs = 6000;
    public static double driveTimeoutMs = 7000;
    public static double aimToleranceDegrees = 2.0;
    public static double settleRate = 0.3;
    public static double shotIntervalMs = 300;  // a gate's open for Gates.feedMs (250)
    public static double partnerDelayMs = 5000;
    // The FLOWER: stop flowerApproachInches short, push in to flowerPushInches past flowerPose() for
    // flowerPushMs, back off flowerBackOffInches, and push again
    public static double flowerApproachInches = 16;
    public static double flowerPushInches = 1.5;
    public static double flowerBackOffInches = 5;
    public static double flowerPushMs = 2500;
    public static double flowerBackOffMs = 700;


    protected static final PoseFactory poseFactory = PoseFactory.degrees();

    private Follower follower;
    private BulkCache bulkCache;
    private Flywheel pollenLauncher;
    private Flywheel nectarLauncher;
    private RpmTable pollenRpmTable;
    private RpmTable nectarRpmTable;
    private Gates gates;
    private LimelightCamera camera;
    private HiveTarget hiveTarget;
    private final HiveAim hiveAim = new HiveAim();

    private boolean aiming = false;
    private double aimHeading = 0.0;
    private int feeds = 0;
    private int loops = 0;

    protected abstract Alliance alliance();

    /** Where the robot is placed, per G304. */
    protected abstract Pose startPose();

    /** Where it stops to shoot: inside the 90% line of red_score_heatmap.png (or blue's turned about the centre). */
    protected abstract Pose shootingSpot();

    /** The collecting loop's waypoints, after the shooting spot and before returning to it. */
    protected abstract Pose[] collectingLoop();

    /** The robot's pose with its intake against the FLOWER in its quarter, facing it. */
    protected abstract Pose flowerPose();

    /** Whether it waits partnerDelayMs before its first volley, for its partner's to tip the HIVE. */
    protected boolean waitsForPartner() {
        return false;
    }

    public Command autoRoutine() {
        return sequential(
                drive(Paths.line(startPose(), shootingSpot()).linear(startPose(), shootingSpot())),
                waitMs(waitsForPartner() ? partnerDelayMs : 0),
                shoot(),
                collectFlower(),
                drive(Paths.line(flowerPose(), shootingSpot()).linear(flowerPose(), shootingSpot())),
                shoot(),
                Groups.loop(sequential(
                        drive(collectingPath()),
                        drive(returnPath()),
                        shoot(),
                        instant(() -> loops++)
                ))
        );
    }

    /** To the FLOWER, then push in, back off and push in again: its Pollen drop one at a time. */
    private Command collectFlower() {
        Pose approach = along(flowerPose(), -flowerApproachInches);
        Pose in = along(flowerPose(), flowerPushInches);
        Pose out = along(flowerPose(), -flowerBackOffInches);
        return sequential(
                drive(Paths.line(shootingSpot(), approach).linear(shootingSpot(), approach)),
                race(follow(follower, Paths.line(approach, in).linear(approach, in)), waitMs(flowerPushMs)),
                race(follow(follower, Paths.line(in, out).linear(in, out)), waitMs(flowerBackOffMs)),
                race(follow(follower, Paths.line(out, in).linear(out, in)), waitMs(flowerPushMs))
        );
    }

    /** The pose moved inches along its heading. */
    private static Pose along(Pose pose, double inches) {
        return new Pose(pose.x() + inches * Math.cos(pose.heading()),
                pose.y() + inches * Math.sin(pose.heading()), pose.heading());
    }

    private Command drive(Path path) {
        return sequential(
                instant(() -> aiming = false),
                race(follow(follower, path), waitMs(driveTimeoutMs))
        );
    }

    private Command shoot() {
        return sequential(
                instant(this::startAiming),
                race(
                        repeat(sequential(
                                waitUntil(this::readyToFire),
                                instant(this::fire),
                                waitMs(shotIntervalMs)
                        ), volleyFeeds),
                        waitMs(shootTimeoutMs)
                )
        );
    }

    /** From the shooting spot through the collecting loop, the intake leading. */
    private Path collectingPath() {
        Pose[] loop = collectingLoop();
        Pose[] points = new Pose[loop.length + 1];
        points[0] = shootingSpot();
        System.arraycopy(loop, 0, points, 1, loop.length);
        return Paths.through(points).tangent();
    }

    /** From the end of the collecting loop back to the shooting spot. */
    private Path returnPath() {
        Pose[] loop = collectingLoop();
        Pose last = loop[loop.length - 1];
        return Paths.line(last, shootingSpot()).tangent();
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        bulkCache = new BulkCache(hardwareMap);
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose());
        follower.update();

        pollenLauncher = new Flywheel(hardwareMap, "PollenFlywheel", DcMotorSimple.Direction.REVERSE);
        nectarLauncher = new Flywheel(hardwareMap, "NectarFlywheel", DcMotorSimple.Direction.FORWARD);
        gates = new Gates(hardwareMap);
        camera = new LimelightCamera(hardwareMap, alliance());
        pollenRpmTable = new RpmTable(LaunchModel.Launcher.POLLEN);
        nectarRpmTable = new RpmTable(LaunchModel.Launcher.NECTAR);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        bulkCache.clear();
        pollenLauncher.reset();
        nectarLauncher.reset();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            bulkCache.clear();

            hiveTarget = camera.findTarget();
            pollenLauncher.runToRPM(pollenRpmTable.rpmFor(hiveTarget, alliance(), follower.pose()));
            nectarLauncher.runToRPM(nectarRpmTable.rpmFor(hiveTarget, alliance(), follower.pose()));
            gates.update();

            follower.update();
            Scheduler.execute();
            if (aiming) updateAim();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());
            telemetry.addData("Loops", loops);
            telemetry.addData("Feeds", feeds);
            if (aiming) {
                telemetry.addData("Aim", hiveAim.isFromVision() ? "Vision" : "Odometry");
                telemetry.addData("Aim error (deg)", Math.toDegrees(headingError()));
            }
            telemetry.update();
        }

        camera.stop();
    }

    private void startAiming() {
        hiveAim.reset();
        aiming = true;
        updateAim();
    }

    /** Hold the shooting spot, turned to this loop's aim heading. */
    private void updateAim() {
        aimHeading = hiveAim.update(alliance(), hiveTarget, follower.pose());
        follower.hold(new Pose(shootingSpot().x(), shootingSpot().y(), aimHeading));
    }

    private boolean readyToFire() {
        return pollenLauncher.isReady() && nectarLauncher.isReady()
                && Math.abs(headingError()) < Math.toRadians(aimToleranceDegrees)
                && Math.abs(follower.twist().omega) < settleRate;
    }

    private void fire() {
        gates.feedPollen();
        gates.feedNectar();
        pollenLauncher.shotFired();
        nectarLauncher.shotFired();
        feeds++;
    }

    /** Aim heading minus the robot's heading, wrapped to -pi..pi. */
    private double headingError() {
        double error = aimHeading - follower.pose().heading();
        return Math.atan2(Math.sin(error), Math.cos(error));
    }
}
