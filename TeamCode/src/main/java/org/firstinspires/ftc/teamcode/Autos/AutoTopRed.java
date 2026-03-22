package org.firstinspires.ftc.teamcode.Autos;
import static android.os.SystemClock.sleep;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;

@Autonomous
@Configurable // Panels
public class AutoTopRed extends OpMode {

    private DcMotor intake;
    private Servo gate, hold, flick;  // servos
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private ShooterSubsystem shooter;
    private Timer pathTimer;
    private Timer launchTimer;
    private Timer shootTimer;
    private double lastTime = 0.0;
    private boolean shooterActive = false;
    private int launchState3 = 0;
    private boolean isDone = false;
    private boolean isReset = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(130.21686746987953, 115.7590361445783, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths
        intake = hardwareMap.get(DcMotor.class, "intake");
        gate = hardwareMap.get(Servo.class, "gate");
        hold = hardwareMap.get(Servo.class, "hold");
        flick = hardwareMap.get(Servo.class,"flick");
        pathTimer = new Timer();
        launchTimer = new Timer();
// Shooter subsystem
        shooter = new ShooterSubsystem(hardwareMap);
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        hold.setPosition(0.3);
        flick.setPosition(1);
    }
    @Override
    public void start() {
        lastTime = getRuntime();
        shooter.getLimelight().start();
        shooter.getLimelight().pipelineSwitch(0); // 0 is for red tracking
    }
    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // launcher update
        if (shooterActive) {
            double currentTime = getRuntime();
            double dt = currentTime - lastTime;
            lastTime = currentTime;

            shooter.update(follower.getPose(), follower.getVelocity(), dt, telemetry);
        }

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());

        telemetry.update();
        panelsTelemetry.update(telemetry);
    }

    private void launch3balls() {  // we call this function every time you want to launch 3 balls
        switch (launchState3) {
            case 0:
                gate.setPosition(0.55);  //0.55 open 0.3 closed
                launchTimer.resetTimer();
                launchState3++;
                hold.setPosition(0.5);
                break;

            case 1:
                if (launchTimer.getElapsedTimeSeconds() > 1) {
                    intake.setPower(1);
                    launchTimer.resetTimer();
                    launchState3++;
                }
                break;
            case 2:
                intake.setPower(1);
                if (launchTimer.getElapsedTimeSeconds() > 1) {
                    flick.setPosition(0.6);  //0 is up
                    sleep(300);
                    flick.setPosition(1);
                    launchTimer.resetTimer();
                    isDone = true;
                }
                break;
        }
    }
    public static class Paths {
        public PathChain Starttoshoot1;
        public PathChain ShootPretopickup1;
        public PathChain Pickup1toshoot2;
        public PathChain Shoot3togate;
        public PathChain Gatetopickup2;
        public PathChain Pickup2tointake;
        public PathChain intaketoshoot3;
        public PathChain Shoot3topark;

        public Paths(Follower follower) {
            Starttoshoot1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(130.217, 115.759),
                                    new Pose(111.759, 100.735),
                                    new Pose(89.928, 92.096)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))

                    .build();

            ShootPretopickup1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(89.928, 92.096),
                                    new Pose(76.886, 55.488),
                                    new Pose(127.639, 59.398)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Pickup1toshoot2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(127.639, 59.398),
                                    new Pose(117.102, 51.102),
                                    new Pose(98.681, 79.645),
                                    new Pose(89.892, 92.060)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Shoot3togate = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(89.892, 92.060),
                                    new Pose(84.398, 82.681),
                                    new Pose(126.482, 68.988)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))

                    .build();

            Gatetopickup2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(126.482, 68.988),
                                    new Pose(97.488, 69.283),
                                    new Pose(94.976, 85.867)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))

                    .build();

            Pickup2tointake = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(94.976, 85.867),

                                    new Pose(125.898, 84.735)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            intaketoshoot3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(125.898, 84.735),
                                    new Pose(101.524, 79.651),
                                    new Pose(89.928, 92.084)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Shoot3topark = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(89.928, 92.084),
                                    new Pose(83.054, 99.373),
                                    new Pose(87.265, 113.482)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();
        }
    }
/* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:

                shooterActive = true;

                follower.followPath(paths.Starttoshoot1, true);
                setPathState(1);
                pathTimer.resetTimer();
                break;

            case 1:

                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    launch3balls();
                }

                if (isDone) {
                    isDone = false;
                    launchState3 = 0;
                    gate.setPosition(0.3);
                    hold.setPosition(0.5);
                    follower.followPath(paths.ShootPretopickup1, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (isReset == false && !follower.isBusy()) {
                    pathTimer.resetTimer();
                    isReset = true;
                }
                if (!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 0.8) {
                        hold.setPosition(0.3);
                        follower.followPath(paths.Pickup1toshoot2, true);
                        intake.setPower(0);
                        setPathState(5);
                        pathTimer.resetTimer();
                    }
                }
                break;

            case 5:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1) {
                    launch3balls();
                }

                if (isDone) {
                    isDone = false;
                    isReset = false;
                    launchState3 = 0;
                    gate.setPosition(0.3);
                    hold.setPosition(0.5);
                    follower.followPath(paths.Shoot3togate, true);
                    setPathState(6);
                    pathTimer.resetTimer();
                }
                break;

            case 6:
                follower.followPath(paths.Gatetopickup2, true);
                setPathState(7);
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Pickup2tointake);
                    setPathState(8);
                }
                break;

            case 8:
                if (isReset == false && !follower.isBusy()){
                    pathTimer.resetTimer();
                    isReset = true;
                }
                if (!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                        hold.setPosition(0.3);
                        follower.followPath(paths.intaketoshoot3, true);
                        intake.setPower(0);
                        pathTimer.resetTimer();
                        setPathState(9);
                    }
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    launch3balls();
                }


                if (isDone) {
                    follower.followPath(paths.Shoot3topark);
                    isDone = false;
                    isReset = false;
                    launchState3 = 0;
                    gate.setPosition(0.3);
                    hold.setPosition(0.5);
                    setPathState(-1);
                }
                break;
        }
        return pathState;
    }
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}